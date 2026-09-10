// Copied verbatim from the mockup so the Next.js build and the design stay in
// step. Framework-agnostic: pure canvas drawing + scoring, no React.
// Drawing + scoring helpers shared by the User and Admin pages.
// Pure functions over canvas 2d contexts; no DOM layout, no styling decisions
// beyond the palette below, which mirrors globals.css.

export const COL = {
  good: '#3fb950', warn: '#d29922', bad: '#f85149',
  // `predicted` is the design's steel accent, not a generic UI blue -- it is
  // the same #7fa6cc the buttons and the selected states use, so the predicted
  // track reads as "ours" against the green of the actual GPS.
  actual: '#3fb950', predicted: '#7fa6cc',
  ink: '#e6edf3', dim: '#93a1b1', faint: '#64748b',
  line: '#26303d', panel: '#151b24', panel2: '#1b232e', bg: '#0d1117',
  steel: '#7fa6cc',
};

/** good | warn | bad for one metric value, from BANDS in session-data.js. */
export function band(metric, v, B) {
  const b = B[metric];
  if (v == null || !Number.isFinite(v)) return 'none';
  if (metric === 'alt') {
    const [lo, hi] = b.envelope;
    return v < lo || v > hi ? 'bad' : 'good';
  }
  if (b.invert) return v < b.warn ? 'bad' : v < b.good ? 'warn' : 'good';
  return v <= b.good ? 'good' : v <= b.warn ? 'warn' : 'bad';
}

export const bandColour = (k) => (k === 'none' ? COL.faint : COL[k]);

/**
 * The map packet's WGS84 extent, as [west, south, east, north].
 *
 * The live API publishes this as `bounds_wgs84`; the mockup's session-data.js
 * called it `bounds`. Reading the wrong one yields `undefined`, and `project`
 * then fails on the destructure with "undefined is not iterable" -- from
 * inside a canvas draw, so the message names neither the key nor the map.
 * Every reader goes through here so there is one place that knows both names.
 */
export const boundsOf = (map) => (map && (map.bounds_wgs84 || map.bounds)) || null;

/** Normalised 0..1 position inside the reference tile. */
export function project(bounds, lat, lon) {
  if (!bounds) return { x: 0.5, y: 0.5 };
  const [w, s, e, n] = bounds;
  return { x: (lon - w) / (e - w), y: (n - lat) / (n - s) };
}

export function bearing(a, b) {
  const toR = Math.PI / 180;
  const dl = (b.lo - a.lo) * toR, la1 = a.la * toR, la2 = b.la * toR;
  const y = Math.sin(dl) * Math.cos(la2);
  const x = Math.cos(la1) * Math.sin(la2) - Math.sin(la1) * Math.cos(la2) * Math.cos(dl);
  return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

/** Yaw for record i, from the attitude-rectified track. */
export function yawAt(recs, i) {
  const a = recs[Math.max(0, i - 1)], b = recs[Math.min(recs.length - 1, i + 1)];
  if (a === b) return 90;
  return bearing(a, b);
}

const pct = (rows, q) => {
  if (!rows.length) return null;
  const s = [...rows].sort((x, y) => x - y);
  const p = (s.length - 1) * q, lo = Math.floor(p), hi = Math.ceil(p);
  return lo === hi ? s[lo] : s[lo] + (s[hi] - s[lo]) * (p - lo);
};
const fin = (xs) => xs.filter((v) => typeof v === 'number' && Number.isFinite(v));

/** Same rules as Stats.jsx: medians and percentiles, never a mean or an RMSE. */
export function summarise(recs) {
  const errs = fin(recs.map((r) => r.e)), lat = fin(recs.map((r) => r.ms));
  const losses = fin(recs.map((r) => r.loss)), inl = fin(recs.map((r) => r.inl));
  const accepted = recs.filter((r) => r.ok).length;
  return {
    n: recs.length, accepted, scored: errs.length,
    acceptRate: recs.length ? accepted / recs.length : null,
    median: pct(errs, 0.5), p90: pct(errs, 0.9), p99: pct(errs, 0.99),
    within10: errs.length ? errs.filter((e) => e <= 10).length / errs.length : null,
    medLoss: pct(losses, 0.5), medLatency: pct(lat, 0.5), p95Latency: pct(lat, 0.95),
    medInliers: pct(inl, 0.5), minInliers: inl.length ? Math.min(...inl) : null,
    overBudget: lat.filter((v) => v > 250).length,
  };
}

export const fmt = (v, d = 2) => (v == null || !Number.isFinite(v) ? '--' : v.toFixed(d));

/**
 * Which gate a fix is failing, named the way the layer names it.
 * The strings are the code descriptions from geoanchor/codes.py, filled in
 * with this fix's own numbers.
 */
export function faultsOf(r, B, cfg) {
  if (!r) return [];
  const out = [];
  if (r.ms > B.ms.warn) out.push({ layer: 'Processing layer', code: 'PLE-09', kind: 'bad',
    text: `the fix landed ${Math.round(r.ms)} ms after capture — ArduPilot compensates only to ${cfg.latency_budget_ms} ms, so a fix this late is fused at the wrong time` });
  else if (r.ms > B.ms.good) out.push({ layer: 'Processing layer', code: 'PLE-09', kind: 'warn',
    text: `${Math.round(r.ms)} ms over the ${cfg.latency_budget_ms} ms budget` });
  if (r.inl != null && r.inl < cfg.inlier_gate) out.push({ layer: 'Processing layer', code: 'PLE-07', kind: 'bad',
    text: `${r.inl} inliers against a gate of ${cfg.inlier_gate} — the fix was rejected` });
  else if (r.inl != null && r.inl < cfg.min_quality_inliers) out.push({ layer: 'Output layer', code: 'OLE-06', kind: 'warn',
    text: `${r.inl} inliers is under the ${cfg.min_quality_inliers} a closed loop needs — logged, not sent` });
  if (r.e != null && r.e > cfg.alarm_error_m) out.push({ layer: 'Output layer', code: 'OLE-04', kind: 'bad',
    text: `${r.e.toFixed(1)} m from the actual GPS, past the ${cfg.alarm_error_m} m alarm threshold` });
  if (r.alt != null && (r.alt < cfg.agl_min_m || r.alt > cfg.agl_max_m)) out.push({ layer: 'Data layer', code: 'DLE-12', kind: 'bad',
    text: `${r.alt.toFixed(1)} m AGL is outside the ${cfg.agl_min_m}–${cfg.agl_max_m} m envelope — the fix is marked unreliable` });
  return out;
}

/**
 * Trust verdict over the last five fixes. A single amber fix should not flip a
 * pilot's decision, and a single good one should not clear a bad run: the
 * window has to be clean before this says trust.
 */
export function verdict(recs, upto, B, cfg) {
  const win = recs.slice(Math.max(0, upto - 4), upto + 1);
  if (!win.length) return { state: 'none', label: 'no fixes', detail: '', faults: [], n: 0 };
  const faults = [];
  win.forEach((r) => faultsOf(r, B, cfg).forEach((f) => faults.push(f)));
  const bad = faults.filter((f) => f.kind === 'bad');
  const warn = faults.filter((f) => f.kind === 'warn');
  const rejected = win.filter((r) => !r.ok).length;
  const n = win.length;
  // the window is short at takeoff, so agreement has to follow it
  const many = n === 1 ? 'the last fix is' : `${n} of the last ${n} fixes are`;
  if (bad.length || rejected) return {
    state: 'bad', label: 'do not trust', n,
    detail: `${bad.length ? bad[0].text : (rejected === 1 && n === 1 ? 'the last fix was rejected' : `${rejected} of the last ${n} fixes were rejected`)}`,
    faults,
  };
  if (warn.length) return {
    state: 'warn', label: 'caution', n,
    detail: n === 1
      ? `the last fix is usable but not closed-loop grade — ${warn[0].text}`
      : `${warn.length} of the last ${n} fixes ${warn.length === 1 ? 'is' : 'are'} usable but not closed-loop grade — ${warn[0].text}`,
    faults,
  };
  return {
    state: 'good', label: 'trust the fix', n,
    detail: `${many} inside every gate — under ${cfg.latency_budget_ms} ms, over ${cfg.min_quality_inliers} inliers, inside ${B.e.good} m`,
    faults,
  };
}

// ---------------------------------------------------------------- map -----

function sizeCanvas(cv) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const box = cv.getBoundingClientRect();
  const w = Math.max(1, Math.round(box.width * dpr)), h = Math.max(1, Math.round(box.height * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  return { g: cv.getContext('2d'), W: w, H: h, dpr };
}

/**
 * opts: { img, map, bands, records, cursor, hover, sel, metric, view:{cx,cy,zoom},
 *         show:{actual,predicted,error,sigma,rejects} }
 * Returns the transform so hit-testing can reuse it.
 */
export function drawMap(cv, o) {
  const { g, W, H, dpr } = sizeCanvas(cv);
  g.clearRect(0, 0, W, H);
  g.fillStyle = COL.panel2; g.fillRect(0, 0, W, H);

  const map = o.map, recs = o.records || [];
  const fit = Math.min(W / map.width, H / map.height);
  const s = fit * o.view.zoom;
  const dw = map.width * s, dh = map.height * s;
  const ox = W / 2 - o.view.cx * dw, oy = H / 2 - o.view.cy * dh;

  if (o.img) {
    g.imageSmoothingQuality = 'high';
    g.drawImage(o.img, ox, oy, dw, dh);
    g.fillStyle = 'rgba(6,10,16,0.42)'; g.fillRect(ox, oy, dw, dh);
  }
  // graticule, so pan and zoom read as movement over a georeferenced sheet
  g.strokeStyle = 'rgba(127,166,204,0.13)'; g.lineWidth = 1 * dpr;
  for (let i = 0; i <= 8; i++) {
    const x = ox + (dw * i) / 8, y = oy + (dh * i) / 8;
    g.beginPath(); g.moveTo(x, oy); g.lineTo(x, oy + dh); g.stroke();
    g.beginPath(); g.moveTo(ox, y); g.lineTo(ox + dw, y); g.stroke();
  }
  g.strokeStyle = 'rgba(127,166,204,0.5)'; g.lineWidth = 1 * dpr;
  g.strokeRect(ox, oy, dw, dh);

  const P = (r) => { const p = project(boundsOf(map), r.la, r.lo); return { x: ox + p.x * dw, y: oy + p.y * dh }; };
  const Pp = (r) => (r.pla == null ? null : (() => { const p = project(boundsOf(map), r.pla, r.plo); return { x: ox + p.x * dw, y: oy + p.y * dh }; })());
  const pxPerMetre = s / map.gsd_m_px;
  // live paints the flight as it happens; replay shows the whole track with
  // the vehicle parked wherever the scrub left it.
  const upto = o.live === false ? recs : recs.slice(0, (o.cursor ?? recs.length - 1) + 1);
  const show = o.show || {};

  // predicted track, thin and dashed under the coloured one.
  //
  // A rejected fix has NO predicted position -- 533 of 2515 in a typical
  // replay, and they fall mid-track, not only at the start. The old loop chose
  // moveTo vs lineTo on the array index `i` rather than on whether a point had
  // actually been drawn yet, and skipped the empty ones with a bare `return`.
  // So the path ran straight from the fix before a gap to the fix after it: a
  // long dashed line across the map through positions the pipeline never
  // predicted. Break the subpath at every gap instead.
  if (show.predicted && upto.length > 1) {
    g.save(); g.setLineDash([5 * dpr, 4 * dpr]);
    // COL.predicted, not a second copy of it. This was hardcoded to the old
    // #4c9aff and so kept drawing the bright blue after the palette moved to
    // the design's steel -- the one line on the map that ignored the theme.
    g.strokeStyle = COL.predicted; g.globalAlpha = 0.75;
    g.lineWidth = 1.4 * dpr;
    g.beginPath();
    let open = false;
    for (const r of upto) {
      const p = Pp(r);
      if (!p) { open = false; continue; }
      if (open) g.lineTo(p.x, p.y); else { g.moveTo(p.x, p.y); open = true; }
    }
    g.stroke(); g.restore();
  }

  // the coloured track: one segment per pair, banded on the selected metric
  if (show.actual !== false) {
    g.lineCap = 'round'; g.lineJoin = 'round';
    for (let i = 1; i < upto.length; i++) {
      const a = P(upto[i - 1]), b = P(upto[i]);
      const k = band(o.metric, upto[i][o.metric], o.bands);
      const inSel = o.sel && i >= o.sel[0] && i <= o.sel[1];
      if (inSel) {
        g.strokeStyle = 'rgba(255,255,255,0.85)'; g.lineWidth = 8 * dpr;
        g.beginPath(); g.moveTo(a.x, a.y); g.lineTo(b.x, b.y); g.stroke();
      }
      g.strokeStyle = bandColour(k); g.lineWidth = (inSel ? 4.5 : 3.2) * dpr;
      g.beginPath(); g.moveTo(a.x, a.y); g.lineTo(b.x, b.y); g.stroke();
    }
  }

  // error whiskers
  if (show.error) {
    g.strokeStyle = 'rgba(248,81,73,0.6)'; g.lineWidth = 1.2 * dpr;
    upto.forEach((r) => {
      const a = P(r), b = Pp(r); if (!b) return;
      g.beginPath(); g.moveTo(a.x, a.y); g.lineTo(b.x, b.y); g.stroke();
    });
  }

  // breadcrumbs: a time stamp only where a fix was rejected or fell in the red
  // band, so the trail annotates the trouble and nothing else
  if (show.rejects !== false) {
    g.font = `${10 * dpr}px ui-monospace, Menlo, monospace`;
    g.textAlign = 'left';
    const t0 = recs.length ? recs[0].t : 0;
    upto.forEach((r) => {
      const red = band(o.metric, r[o.metric], o.bands) === 'bad';
      if (r.ok && !red) return;
      const p = P(r), q = 4.5 * dpr;
      g.strokeStyle = COL.bad; g.lineWidth = 1.6 * dpr;
      g.strokeRect(p.x - q, p.y - q, q * 2, q * 2);
      const label = `${(r.t - t0).toFixed(1)}s`;
      const wTxt = g.measureText(label).width;
      g.fillStyle = 'rgba(13,17,23,0.82)';
      g.fillRect(p.x + 8 * dpr, p.y - 14 * dpr, wTxt + 8 * dpr, 15 * dpr);
      g.fillStyle = COL.bad;
      g.fillText(label, p.x + 12 * dpr, p.y - 3 * dpr);
    });
  }

  if (o.hover != null && recs[o.hover]) {
    const p = P(recs[o.hover]);
    g.strokeStyle = 'rgba(255,255,255,0.9)'; g.lineWidth = 1.5 * dpr;
    g.beginPath(); g.arc(p.x, p.y, 9 * dpr, 0, Math.PI * 2); g.stroke();
    g.beginPath(); g.moveTo(p.x - 14 * dpr, p.y); g.lineTo(p.x - 4 * dpr, p.y);
    g.moveTo(p.x + 4 * dpr, p.y); g.lineTo(p.x + 14 * dpr, p.y);
    g.moveTo(p.x, p.y - 14 * dpr); g.lineTo(p.x, p.y - 4 * dpr);
    g.moveTo(p.x, p.y + 4 * dpr); g.lineTo(p.x, p.y + 14 * dpr); g.stroke();
  }

  // the vehicle: an arrow pointing along yaw
  const ci = Math.min(o.cursor ?? recs.length - 1, recs.length - 1);
  const cur = recs[ci];
  if (cur) {
    const p = P(cur);
    const yaw = yawAt(recs, ci);
    const k = band(o.metric, cur[o.metric], o.bands);
    const col = bandColour(k);

    if (show.sigma && cur.sig) {
      g.beginPath(); g.arc(p.x, p.y, cur.sig * pxPerMetre, 0, Math.PI * 2);
      g.fillStyle = 'rgba(76,154,255,0.12)'; g.strokeStyle = 'rgba(76,154,255,0.55)';
      g.lineWidth = 1 * dpr; g.fill(); g.stroke();
    }
    // heading ray
    g.save(); g.translate(p.x, p.y); g.rotate((yaw - 90) * Math.PI / 180);
    g.setLineDash([4 * dpr, 4 * dpr]);
    g.strokeStyle = 'rgba(255,255,255,0.45)'; g.lineWidth = 1 * dpr;
    g.beginPath(); g.moveTo(16 * dpr, 0); g.lineTo(64 * dpr, 0); g.stroke();
    g.setLineDash([]);
    // arrow body
    const R = 13 * dpr;
    g.beginPath();
    g.moveTo(R, 0); g.lineTo(-R * 0.78, R * 0.72);
    g.lineTo(-R * 0.36, 0); g.lineTo(-R * 0.78, -R * 0.72);
    g.closePath();
    g.fillStyle = col; g.fill();
    g.strokeStyle = '#0d1117'; g.lineWidth = 2 * dpr; g.stroke();
    g.strokeStyle = 'rgba(255,255,255,0.92)'; g.lineWidth = 1 * dpr; g.stroke();
    g.restore();
  }

  // scale bar
  const target = 90 * dpr;
  const metres = target / pxPerMetre;
  const nice = [10, 20, 25, 50, 100, 200, 250, 500, 1000].reduce((a, b) => (Math.abs(b - metres) < Math.abs(a - metres) ? b : a), 10);
  const barPx = nice * pxPerMetre;
  const bx = 14 * dpr, by = H - 16 * dpr;
  g.strokeStyle = 'rgba(255,255,255,0.85)'; g.lineWidth = 1.5 * dpr;
  g.beginPath(); g.moveTo(bx, by); g.lineTo(bx + barPx, by);
  g.moveTo(bx, by - 5 * dpr); g.lineTo(bx, by + 3 * dpr);
  g.moveTo(bx + barPx, by - 5 * dpr); g.lineTo(bx + barPx, by + 3 * dpr); g.stroke();
  g.fillStyle = 'rgba(255,255,255,0.85)';
  g.font = `${10.5 * dpr}px ui-monospace, Menlo, monospace`;
  g.fillText(`${nice} m`, bx + 3 * dpr, by - 8 * dpr);

  // north arrow — the reference tile is north-up, so this is a fixed rose
  const nx = bx + 10 * dpr, ny = by - 40 * dpr;
  g.beginPath();
  g.moveTo(nx, ny - 16 * dpr); g.lineTo(nx + 5 * dpr, ny + 4 * dpr);
  g.lineTo(nx, ny); g.lineTo(nx - 5 * dpr, ny + 4 * dpr);
  g.closePath();
  g.fillStyle = 'rgba(255,255,255,0.88)'; g.fill();
  g.strokeStyle = 'rgba(13,17,23,0.8)'; g.lineWidth = 1 * dpr; g.stroke();
  g.fillStyle = 'rgba(255,255,255,0.88)';
  g.font = `${10 * dpr}px ui-monospace, Menlo, monospace`;
  g.textAlign = 'center';
  g.fillText('N', nx, ny + 15 * dpr);
  g.textAlign = 'start';

  return { ox, oy, dw, dh, s, P };
}

/**
 * Frame the whole flight, but never leave the panel showing bare ground: the
 * zoom is raised until the reference tile covers the canvas, so the map reads
 * as a map and not as a picture with margins.
 */
export function fitView(cv, map, recs, whole) {
  if (whole) return { cx: 0.5, cy: 0.5, zoom: 1 };
  const box = cv.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const W = Math.max(1, box.width * dpr), H = Math.max(1, box.height * dpr);
  const fit = Math.min(W / map.width, H / map.height);
  const cover = Math.max(W / (fit * map.width), H / (fit * map.height));
  const ps = recs.map((r) => project(boundsOf(map), r.la, r.lo));
  const xs = ps.map((p) => p.x), ys = ps.map((p) => p.y);
  const w = Math.max(0.03, Math.max(...xs) - Math.min(...xs));
  const h = Math.max(0.03, Math.max(...ys) - Math.min(...ys));
  const track = 0.82 / Math.max(w, h);
  return {
    cx: (Math.min(...xs) + Math.max(...xs)) / 2,
    cy: (Math.min(...ys) + Math.max(...ys)) / 2,
    zoom: Math.min(14, Math.max(cover, Math.min(6, track))),
  };
}

/** Nearest record index to a canvas-space point. */export function pickRecord(tf, recs, x, y, limit) {
  let best = null, bd = Infinity;
  const n = limit == null ? recs.length : limit + 1;
  for (let i = 0; i < n; i++) {
    const p = tf.P(recs[i]);
    const d = (p.x - x) ** 2 + (p.y - y) ** 2;
    if (d < bd) { bd = d; best = i; }
  }
  return bd < (34 * (window.devicePixelRatio || 1)) ** 2 ? best : null;
}

// -------------------------------------------------------------- chart -----

/**
 * opts: { records, metric, bands, hover, sel, cursor, live }
 * Threshold zones are painted as background bands, then the series is drawn
 * one coloured segment at a time so a stretch over budget is red on the chart
 * and red on the map for the same fixes.
 */
export function drawChart(cv, o) {
  const { g, W, H, dpr } = sizeCanvas(cv);
  g.clearRect(0, 0, W, H);
  const recs = o.records || [];
  const B = o.bands[o.metric];
  const F = o.big ? 1.4 : 1;
  const PL = 46 * F * dpr, PR = 10 * dpr, PT = 10 * dpr, PB = 22 * F * dpr;
  const w = W - PL - PR, h = H - PT - PB;
  // x-domain: the chart can be zoomed to a stretch of the flight while the
  // map keeps the whole track.
  const d0 = o.domain ? Math.max(0, o.domain[0]) : 0;
  const d1 = o.domain ? Math.min(recs.length - 1, o.domain[1]) : recs.length - 1;
  const win = recs.slice(d0, d1 + 1);
  const vals = win.map((r) => r[o.metric]).filter((v) => Number.isFinite(v));
  if (!vals.length || w <= 0 || h <= 0) return null;

  // The axis is scaled to the data and to the thresholds it is actually near.
  // Reaching for the red edge every time squashes a healthy series into the
  // bottom eighth of the plot and paints four fifths of it amber.
  const dLo = Math.min(...vals), dHi = Math.max(...vals);
  let lo, hi, edges;
  if (o.metric === 'alt') {
    edges = B.envelope;
    lo = Math.min(dLo, B.envelope[0]) - 6;
    hi = Math.max(dHi, B.envelope[1]) + 6;
  } else if (B.invert) {
    edges = dLo < B.good ? [B.warn, B.good] : [B.good];
    lo = 0;
    hi = Math.max(dHi * 1.08, B.good * 1.5);
  } else {
    edges = dHi > B.good ? [B.good, B.warn] : [B.good];
    lo = 0;
    hi = Math.max(dHi * 1.1, B.good * 1.15);
  }
  const span = Math.max(1, d1 - d0);
  const X = (i) => PL + ((i - d0) / span) * w;
  const Y = (v) => PT + h - ((v - lo) / (hi - lo)) * h;

  // threshold zones
  const zone = (a, b, col) => {
    const y0 = Y(Math.min(b, hi)), y1 = Y(Math.max(a, lo));
    g.fillStyle = col; g.fillRect(PL, Math.min(y0, y1), w, Math.abs(y1 - y0));
  };
  if (o.metric === 'alt') {
    zone(lo, B.envelope[0], 'rgba(248,81,73,0.09)');
    zone(B.envelope[0], B.envelope[1], 'rgba(63,185,80,0.06)');
    zone(B.envelope[1], hi, 'rgba(248,81,73,0.09)');
  } else if (B.invert) {
    zone(lo, B.warn, 'rgba(248,81,73,0.10)');
    zone(B.warn, B.good, 'rgba(210,153,34,0.09)');
    zone(B.good, hi, 'rgba(63,185,80,0.055)');
  } else {
    zone(lo, B.good, 'rgba(63,185,80,0.055)');
    zone(B.good, B.warn, 'rgba(210,153,34,0.09)');
    zone(B.warn, hi, 'rgba(248,81,73,0.10)');
  }

  // frame + threshold rules
  g.strokeStyle = COL.line; g.lineWidth = 1 * dpr;
  g.strokeRect(PL, PT, w, h);
  g.font = `${9.5 * F * dpr}px ui-monospace, Menlo, monospace`;
  g.save(); g.setLineDash([5 * dpr, 4 * dpr]);
  edges.forEach((e) => {
    g.strokeStyle = e === B.good ? COL.warn : COL.bad;
    if (o.metric === 'alt') g.strokeStyle = COL.bad;
    g.beginPath(); g.moveTo(PL, Y(e)); g.lineTo(PL + w, Y(e)); g.stroke();
    g.fillStyle = g.strokeStyle;
    g.fillText(String(e), PL + 4 * dpr, Y(e) - 4 * dpr);
  });
  g.restore();

  // selection
  if (o.sel) {
    g.fillStyle = 'rgba(255,255,255,0.09)';
    g.fillRect(X(o.sel[0]), PT, Math.max(2 * dpr, X(o.sel[1]) - X(o.sel[0])), h);
  }

  // series, banded
  const last = o.live && o.cursor != null ? Math.min(o.cursor, d1) : d1;
  g.lineCap = 'round';
  for (let i = d0 + 1; i <= last; i++) {
    const a = recs[i - 1][o.metric], b = recs[i][o.metric];
    if (!Number.isFinite(a) || !Number.isFinite(b)) continue;
    g.strokeStyle = bandColour(band(o.metric, b, o.bands));
    g.lineWidth = (o.big ? 2.6 : 2) * dpr;
    g.beginPath(); g.moveTo(X(i - 1), Y(a)); g.lineTo(X(i), Y(b)); g.stroke();
  }
  // rejected fixes marked on the series
  for (let i = d0; i <= last; i++) {
    const r = recs[i];
    if (r.ok || !Number.isFinite(r[o.metric])) continue;
    g.strokeStyle = COL.bad; g.lineWidth = 1.4 * dpr;
    g.strokeRect(X(i) - 2.6 * dpr, Y(r[o.metric]) - 2.6 * dpr, 5.2 * dpr, 5.2 * dpr);
  }
  // one dot per fix once the window is short enough to read them
  if (span <= 40) {
    for (let i = d0; i <= last; i++) {
      const v = recs[i][o.metric];
      if (!Number.isFinite(v)) continue;
      g.beginPath(); g.arc(X(i), Y(v), (o.big ? 3.2 : 2.4) * dpr, 0, Math.PI * 2);
      g.fillStyle = bandColour(band(o.metric, v, o.bands)); g.fill();
    }
  }

  // axes text
  g.fillStyle = COL.faint;
  g.font = `${10 * F * dpr}px ui-monospace, Menlo, monospace`;
  g.textAlign = 'right';
  [hi, (hi + lo) / 2, lo].forEach((v) => g.fillText(v >= 100 ? v.toFixed(0) : v.toFixed(1), PL - 6 * dpr, Y(v) + 3.5 * dpr));
  g.textAlign = 'left';
  const stepEvery = Math.max(1, Math.round(span / (o.big ? 12 : 6)));
  const t0 = recs.length ? recs[0].t : 0;
  for (let i = d0; i <= d1; i += stepEvery) {
    // elapsed seconds, not the step number: a gap in the series then reads as
    // time lost rather than as records missing
    g.fillText(`${(recs[i].t - t0).toFixed(1)}s`, X(i) - 8 * dpr, H - 7 * dpr);
  }
  g.textAlign = 'start';

  // cursor + hover
  if (o.live && o.cursor != null && recs[o.cursor] && o.cursor >= d0 && o.cursor <= d1) {
    g.strokeStyle = 'rgba(230,237,243,0.35)'; g.lineWidth = 1 * dpr;
    g.beginPath(); g.moveTo(X(o.cursor), PT); g.lineTo(X(o.cursor), PT + h); g.stroke();
  }
  if (o.hover != null && recs[o.hover] && o.hover >= d0 && o.hover <= d1 && Number.isFinite(recs[o.hover][o.metric])) {
    const x = X(o.hover), y = Y(recs[o.hover][o.metric]);
    g.strokeStyle = 'rgba(255,255,255,0.5)'; g.lineWidth = 1 * dpr;
    g.beginPath(); g.moveTo(x, PT); g.lineTo(x, PT + h); g.stroke();
    g.beginPath(); g.arc(x, y, 4.5 * dpr, 0, Math.PI * 2);
    g.fillStyle = bandColour(band(o.metric, recs[o.hover][o.metric], o.bands)); g.fill();
    g.strokeStyle = '#0d1117'; g.lineWidth = 1.5 * dpr; g.stroke();
  }
  return { PL, PT, w, h, X };
}

/** Chart x -> record index, honouring the zoom domain. */
export function pickIndex(cv, recs, clientX, domain, big) {
  const box = cv.getBoundingClientRect();
  const d0 = domain ? Math.max(0, domain[0]) : 0;
  const d1 = domain ? Math.min(recs.length - 1, domain[1]) : recs.length - 1;
  const PL = 46 * (big ? 1.4 : 1), PR = 10;
  const w = box.width - PL - PR;
  const f = (clientX - box.left - PL) / Math.max(1, w);
  return Math.max(d0, Math.min(d1, Math.round(d0 + f * (d1 - d0))));
}
