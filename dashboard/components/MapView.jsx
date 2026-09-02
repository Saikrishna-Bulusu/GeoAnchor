'use client';
import { useEffect, useRef, useState } from 'react';
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
export default function MapView({ map, records, follow = true }) {
  const canvas = useRef(null);
  const [img, setImg] = useState(null);
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    if (!map) return;
    const im = new Image();
    im.crossOrigin = 'anonymous';
    im.onload = () => { setImg(im); setFailed(false); };
    im.onerror = () => setFailed(true);
    im.src = apiBase() + '/api/map.png';
  }, [map]);

  useEffect(() => {
    const cv = canvas.current;
    if (!cv || !map) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const box = cv.getBoundingClientRect();
    cv.width = Math.max(1, Math.round(box.width * dpr));
    cv.height = Math.max(1, Math.round(box.height * dpr));
    const g = cv.getContext('2d');
    g.clearRect(0, 0, cv.width, cv.height);

    const W = cv.width, H = cv.height;
    if (img) {
      g.drawImage(img, 0, 0, W, H);
      g.fillStyle = 'rgba(0,0,0,0.30)';
      g.fillRect(0, 0, W, H);
    } else {
      g.fillStyle = '#1b232e';
      g.fillRect(0, 0, W, H);
    }

    const P = (lat, lon) => {
      const p = projectToPreview(map.bounds_wgs84, lat, lon);
      return p ? { x: p.x * W, y: p.y * H } : null;
    };

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
        if (sigma && map.gsd_m_px) {
          // Confidence circle, in real metres, from the emitted covariance.
          const mPerPxX = (map.width * map.gsd_m_px) / W;
          g.beginPath();
          g.arc(p.x, p.y, (sigma / mPerPxX) * dpr, 0, Math.PI * 2);
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
