'use client';
import { useCallback, useEffect, useRef, useState } from 'react';
import { apiBase } from '@/lib/api';
import * as viz from '@/lib/viz';
import { BANDS, toRows } from '@/lib/thresholds';

/**
 * Drop-in replacement for MapView.jsx. Same props (map, records, basemap) plus
 * the interaction the mockup adds; everything extra is optional, so
 *
 *   <TrackMap map={state.map} records={records} basemap={state.basemap} />
 *
 * still works. Pan, zoom, follow, a yaw arrow for the vehicle, the track
 * coloured on `metric`, and time-stamped breadcrumbs on rejected/red fixes.
 */
export default function TrackMap({
  map, records, basemap, metric = 'ms', cursor = null, hover = null, sel = null,
  follow = false, show, bands = BANDS, onHover, onPick, className,
}) {
  const canvas = useRef(null);
  const tf = useRef(null);
  const pan = useRef(null);
  const [img, setImg] = useState(null);
  const [view, setView] = useState({ cx: 0.5, cy: 0.5, zoom: 1 });
  const rows = toRows(records);
  const at = cursor == null ? rows.length - 1 : cursor;

  useEffect(() => {
    if (!map) { setImg(null); return; }
    const im = new Image();
    im.crossOrigin = 'anonymous';
    im.onload = () => setImg(im);
    im.src = basemap || (apiBase() + '/api/map.png');
  }, [map, basemap]);

  const draw = useCallback(() => {
    if (!canvas.current || !map) return;
    tf.current = viz.drawMap(canvas.current, {
      img, map, bands, records: rows, metric, cursor: at, hover, sel, view,
      show: show || { actual: true, predicted: true, sigma: true, rejects: true },
      live: false,
    });
  }, [img, map, rows, metric, at, hover, sel, view, show, bands]);

  useEffect(() => { draw(); }, [draw]);
  useEffect(() => {
    if (!canvas.current || typeof ResizeObserver === 'undefined') return undefined;
    const ro = new ResizeObserver(() => draw());
    ro.observe(canvas.current);
    return () => ro.disconnect();
  }, [draw]);

  // Recentre on the vehicle while following.
  //
  // Two things here are load-bearing. `rows` is rebuilt by toRows() on every
  // render, so depending on it re-runs this effect every render; and setView
  // causes a render. Together that is an unbounded loop the moment `follow` is
  // true. So depend on the projected numbers, which are stable when the
  // vehicle has not moved, and return the previous object unchanged when the
  // centre would not actually move -- React bails out of a state update that
  // returns the identical reference, which is what breaks the cycle.
  const here = rows[at];
  const cx0 = here ? viz.project(viz.boundsOf(map), here.la, here.lo).x : null;
  const cy0 = here ? viz.project(viz.boundsOf(map), here.la, here.lo).y : null;
  useEffect(() => {
    if (!follow || !map || cx0 == null) return;
    setView((v) => (v.cx === cx0 && v.cy === cy0 ? v : { ...v, cx: cx0, cy: cy0 }));
  }, [follow, map, cx0, cy0]);

  const dpr = () => Math.min(typeof window === 'undefined' ? 1 : window.devicePixelRatio || 1, 2);

  return (
    <div className={className} style={{ position: 'relative', width: '100%', height: '100%' }}>
      <canvas
        ref={canvas}
        style={{ position: 'absolute', inset: 0, width: '100%', height: '100%', cursor: 'crosshair', touchAction: 'none' }}
        onWheel={(e) => { e.preventDefault(); setView((v) => ({ ...v, zoom: Math.max(0.6, Math.min(14, v.zoom * (e.deltaY < 0 ? 1.16 : 1 / 1.16))) })); }}
        onPointerDown={(e) => { e.currentTarget.setPointerCapture(e.pointerId); pan.current = { x: e.clientX, y: e.clientY, view, moved: false }; }}
        onPointerMove={(e) => {
          if (!tf.current) return;
          if (pan.current) {
            const dx = e.clientX - pan.current.x, dy = e.clientY - pan.current.y;
            if (Math.abs(dx) + Math.abs(dy) > 3) pan.current.moved = true;
            setView({ ...pan.current.view,
              cx: pan.current.view.cx - (dx * dpr()) / tf.current.dw,
              cy: pan.current.view.cy - (dy * dpr()) / tf.current.dh });
            return;
          }
          if (!onHover) return;
          const box = e.currentTarget.getBoundingClientRect();
          onHover(viz.pickRecord(tf.current, rows, (e.clientX - box.left) * dpr(), (e.clientY - box.top) * dpr()));
        }}
        onPointerUp={() => {
          if (pan.current && !pan.current.moved && hover != null && onPick) onPick(hover);
          pan.current = null;
        }}
        onPointerLeave={() => { pan.current = null; if (onHover) onHover(null); }}
      />
    </div>
  );
}

/** Frame the flight; pass whole=true for the entire reference tile. */
export function fitView(canvasEl, map, records, whole) {
  return viz.fitView(canvasEl, map, toRows(records), whole);
}
