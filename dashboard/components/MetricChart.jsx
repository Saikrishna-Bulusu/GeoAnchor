'use client';
import { useCallback, useEffect, useRef } from 'react';
import * as viz from '@/lib/viz';
import { BANDS, toRows } from '@/lib/thresholds';

/**
 * One banded metric over the flight. Replaces a <Chart> from Charts.jsx:
 *
 *   <MetricChart records={records} metric="ms" />
 *
 * metric is a key of BANDS ('ms' | 'e' | 'inl' | 'loss' | 'alt'). The threshold
 * zones, the dashed rules and the per-segment colour all come from BANDS, so
 * this chart and the map agree by construction. domain is [i0, i1] over the
 * record array — pass it to zoom the time axis; the y-axis rescales to it.
 */
export default function MetricChart({
  records, metric = 'ms', domain = null, hover = null, sel = null, cursor = null,
  big = false, height = 196, bands = BANDS, onHover, onSelect, onZoom,
}) {
  const canvas = useRef(null);
  const drag = useRef(null);
  const rows = toRows(records);

  const draw = useCallback(() => {
    if (!canvas.current) return;
    viz.drawChart(canvas.current, { records: rows, metric, bands, hover, sel, cursor, domain, big });
  }, [rows, metric, bands, hover, sel, cursor, domain, big]);

  useEffect(() => { draw(); }, [draw]);
  useEffect(() => {
    if (!canvas.current || typeof ResizeObserver === 'undefined') return undefined;
    const ro = new ResizeObserver(() => draw());
    ro.observe(canvas.current);
    return () => ro.disconnect();
  }, [draw]);

  const idx = (e) => viz.pickIndex(e.currentTarget, rows, e.clientX, domain, big);

  return (
    <div style={{ position: 'relative', height }}>
      <canvas
        ref={canvas}
        style={{ position: 'absolute', inset: 0, width: '100%', height: '100%', cursor: 'col-resize', touchAction: 'none' }}
        onPointerDown={(e) => { e.currentTarget.setPointerCapture(e.pointerId); drag.current = idx(e); }}
        onPointerMove={(e) => {
          const i = idx(e);
          if (onHover) onHover(i);
          if (drag.current != null && onSelect) onSelect([Math.min(drag.current, i), Math.max(drag.current, i)]);
        }}
        onPointerUp={() => { drag.current = null; }}
        onPointerLeave={() => { drag.current = null; if (onHover) onHover(null); }}
        onWheel={(e) => {
          if (!onZoom) return;
          e.preventDefault();
          const d0 = domain ? domain[0] : 0, d1 = domain ? domain[1] : rows.length - 1;
          const at = idx(e), span = d1 - d0;
          const next = Math.max(6, Math.min(rows.length - 1, Math.round(span * (e.deltaY < 0 ? 0.72 : 1 / 0.72))));
          if (next >= rows.length - 1) { onZoom(null); return; }
          const f = span > 0 ? (at - d0) / span : 0.5;
          const n0 = Math.max(0, Math.min(rows.length - 1 - next, Math.round(at - next * f)));
          onZoom([n0, n0 + next]);
        }}
      />
    </div>
  );
}
