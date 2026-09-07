'use client';
import { memo, useCallback, useEffect, useRef, useState } from 'react';
import { apiBase, projectToPreview } from '@/lib/api';

/**
 * Actual and predicted position drawn on the reference map itself.
 *
 * The basemap is the store's own preview.png, not OSM or a tile server. That
 * matters more than it looks: the aircraft is offline by definition -- the
 * whole point of the system is that GNSS and often connectivity are gone --
 * and a map view that needs the internet is a map view that is blank exactly
 * when it is needed. The preview also shares the store's geotransform, so a
 * position drawn here is drawn against the same pixels the matcher used.
 */
function MapView({ map, records, basemap }) {
  const canvas = useRef(null);
  const [img, setImg] = useState(null);
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    if (!map) {
      // Dropping the session drops the basemap with it. Keeping the old image
      // means the next session flashes the previous flight's map before its
      // own loads.
      setImg(null);
      setFailed(false);
      return;
    }
    const im = new Image();
    im.crossOrigin = 'anonymous';
    im.onload = () => { setImg(im); setFailed(false); };
    im.onerror = () => setFailed(true);
    // A session carries its own tile as a data URI, so a replay draws the real
    // map with no board and no network. Only fall back to asking the API when
    // there is no embedded one -- i.e. when we are live.
    im.src = basemap || (apiBase() + '/api/map.png');
  }, [map, basemap]);

  const draw = useCallback(() => {
    const cv = canvas.current;
    if (!cv) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const box = cv.getBoundingClientRect();
    cv.width = Math.max(1, Math.round(box.width * dpr));
    cv.height = Math.max(1, Math.round(box.height * dpr));
    const g = cv.getContext('2d');
    g.clearRect(0, 0, cv.width, cv.height);
    // Bail out AFTER clearing, never before: returning early left the previous
    // session's track painted under a header that already said "no map".
    if (!map) return;

    const W = cv.width, H = cv.height;
    g.fillStyle = '#1b232e';
    g.fillRect(0, 0, W, H);

    // Fit the tile into the panel WITHOUT distorting it. Stretching to the box
    // is not just ugly: it gives the map two different scales, and a confidence
    // circle drawn in metres then has no single radius that is correct in both
    // axes. Letterboxing keeps metres-per-pixel isotropic, which is what makes
    // the sigma ring below an honest shape rather than a decorative one.
    const mw = map.width > 0 ? map.width : (img ? img.width : W);
    const mh = map.height > 0 ? map.height : (img ? img.height : H);
    const fit = Math.min(W / mw, H / mh);
    const dw = mw * fit, dh = mh * fit;
    const ox = (W - dw) / 2, oy = (H - dh) / 2;

    if (img) {
      g.drawImage(img, ox, oy, dw, dh);
      g.fillStyle = 'rgba(0,0,0,0.30)';
      g.fillRect(ox, oy, dw, dh);
    }

    const P = (lat, lon) => {
      const p = projectToPreview(map.bounds_wgs84, lat, lon);
      return p ? { x: ox + p.x * dw, y: oy + p.y * dh } : null;
    };
    // Device pixels per metre on the ground. Isotropic, because of the fit above.
    const pxPerMetre = map.gsd_m_px ? dw / (mw * map.gsd_m_px) : null;

    const rows = records || [];
    const track = (key) => rows.map((r) => r[key]).filter(Boolean).map((p) => P(p.lat, p.lon)).filter(Boolean);

    const drawTrack = (pts, colour, width) => {
      if (pts.length < 2) return;
      g.strokeStyle = colour; g.lineWidth = width * dpr;
      g.lineJoin = 'round'; g.lineCap = 'round';
      g.beginPath();
      pts.forEach((p, i) => (i ? g.lineTo(p.x, p.y) : g.moveTo(p.x, p.y)));
      g.stroke();
    };

    const actual = track('actual_gps');
    const predicted = track('predicted_gps');
    drawTrack(actual, '#3fb950', 2.2);
    drawTrack(predicted, '#4c9aff', 1.8);

    // Error whiskers: one short segment per row, so a systematic offset is
    // visible as a comb rather than having to be read off a chart.
    g.strokeStyle = 'rgba(248,81,73,0.55)';
    g.lineWidth = 1 * dpr;
    rows.slice(-200).forEach((r) => {
      if (!r.actual_gps || !r.predicted_gps) return;
      const a = P(r.actual_gps.lat, r.actual_gps.lon);
      const b = P(r.predicted_gps.lat, r.predicted_gps.lon);
      if (!a || !b) return;
      g.beginPath(); g.moveTo(a.x, a.y); g.lineTo(b.x, b.y); g.stroke();
    });

    const last = rows[rows.length - 1];
    if (last) {
      const drawMarker = (p, colour, radius, sigma) => {
        if (!p) return;
        if (sigma && pxPerMetre) {
          // Confidence circle, in real metres, from the emitted covariance.
          // No dpr factor here: pxPerMetre is already device pixels per metre,
          // because dw is in device pixels. Multiplying by dpr as well drew
          // this ring at twice its true radius on every HiDPI screen --
          // overstating the uncertainty the project exists to state correctly.
          g.beginPath();
          g.arc(p.x, p.y, sigma * pxPerMetre, 0, Math.PI * 2);
          g.fillStyle = 'rgba(76,154,255,0.14)';
          g.strokeStyle = 'rgba(76,154,255,0.5)';
          g.lineWidth = 1 * dpr;
          g.fill(); g.stroke();
        }
        g.beginPath(); g.arc(p.x, p.y, radius * dpr, 0, Math.PI * 2);
        g.fillStyle = colour; g.fill();
        g.strokeStyle = 'rgba(0,0,0,0.6)'; g.lineWidth = 1.5 * dpr; g.stroke();
      };
      drawMarker(last.predicted_gps && P(last.predicted_gps.lat, last.predicted_gps.lon),
                 '#4c9aff', 5, last.sigma_m);
      drawMarker(last.actual_gps && P(last.actual_gps.lat, last.actual_gps.lon), '#3fb950', 4);
    }
  }, [img, map, records]);

  useEffect(() => { draw(); }, [draw]);

  // The canvas backing store is sized from the element's box, so a layout
  // change invalidates it. Without this the map stays at its old resolution --
  // stretched or clipped -- until the next record happens to arrive, which on a
  // finished session is never.
  useEffect(() => {
    const cv = canvas.current;
    if (!cv || typeof ResizeObserver === 'undefined') return undefined;
    const ro = new ResizeObserver(() => draw());
    ro.observe(cv);
    return () => ro.disconnect();
  }, [draw]);

  return (
    <div className="panel">
      <header>
        <h2>Position</h2>
        <span className="note" style={{ marginLeft: 'auto', fontFamily: 'var(--mono)' }}>
          {map ? `${map.gsd_m_px?.toFixed(3)} m/px · EPSG:${map.epsg}` : 'no map'}
        </span>
      </header>
      <div className="body flush">
        <div className="mapwrap">
          <canvas ref={canvas} />
          {!map && <div className="mapempty">waiting for a map packet from the data layer</div>}
          {map && failed && <div className="mapempty">this store has no preview.png &mdash; rebuild it</div>}
          {map && (
            <div className="maplegend">
              <span><i style={{ background: 'var(--actual)' }} />actual</span>
              <span><i style={{ background: 'var(--predicted)' }} />predicted</span>
              <span><i style={{ background: 'var(--bad)' }} />error</span>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export default memo(MapView);
