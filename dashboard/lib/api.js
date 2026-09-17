/**
 * One module decides where data comes from.
 *
 * live    a WebSocket to the API on the board. Everything streams.
 * replay  a session.json, dropped in or fetched. No sockets, no board.
 *
 * The panels do not know which mode they are in; they receive the same shapes
 * either way. That is what lets the Vercel build and the on-board build be the
 * same bundle.
 */

export function apiBase() {
  const env = process.env.NEXT_PUBLIC_GEOANCHOR_API;
  if (env) return env.replace(/\/$/, '');
  if (typeof window === 'undefined') return '';
  return window.location.origin;
}

export function wsUrl() {
  const base = apiBase();
  if (!base) return null;
  return base.replace(/^http/, 'ws') + '/ws';
}

export async function getJSON(path) {
  const r = await fetch(apiBase() + path, { cache: 'no-store' });
  if (!r.ok) throw new Error(`${path}: ${r.status}`);
  return r.json();
}

export async function postControl(layer, body) {
  const r = await fetch(apiBase() + '/api/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ layer, ...body }),
  });
  if (!r.ok) throw new Error(await r.text());
  return r.json();
}

/**
 * Upload a reference map, the way an operator does on the ground before a
 * flight. Multipart, not JSON: a GeoTIFF is tens of megabytes and base64 in a
 * JSON body would inflate it by a third for no reason.
 *
 * The response carries the georeference the server read back, INCLUDING a
 * `warning` when there is no projected CRS. That case matters more than it
 * looks: a plain image with a .tif extension loads without error and then
 * matches against nothing, which is a fault this project has already shipped
 * once.
 */
export async function uploadMap(file) {
  const form = new FormData();
  form.append('file', file);
  const r = await fetch(apiBase() + '/api/map', { method: 'POST', body: form });
  if (!r.ok) throw new Error(await r.text());
  return r.json();
}

/** lat/lon to a fraction of the map image, from the store's WGS84 bounds. */
export function projectToPreview(bounds, lat, lon) {
  if (!bounds) return null;
  const [w, s, e, n] = bounds;
  return { x: (lon - w) / (e - w), y: (n - lat) / (n - s) };
}

export function fmt(v, nd = 2, dash = '--') {
  if (v === null || v === undefined || Number.isNaN(v)) return dash;
  return typeof v === 'number' ? v.toFixed(nd) : String(v);
}

export function downloadJSON(obj, name) {
  const blob = new Blob([JSON.stringify(obj, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = name; a.click();
  URL.revokeObjectURL(url);
}
