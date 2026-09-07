'use client';
import { useCallback, useEffect, useRef, useState } from 'react';
import { apiBase, wsUrl } from './api';

const EMPTY = {
  layers: {}, logs: [], records: [], fixes: [], map: null, frame: null,
  board: null, config: null, basemap: null, codes: null, replay: false,
  // Seconds to add to this browser's clock to get the board's. Every "is this
  // layer still alive?" test compares a board timestamp against a local one, so
  // without this the answer is only as good as the agreement between two
  // machines' clocks. That agreement is not something to assume here: the
  // Xavier is reached over a point-to-point USB-C link with no NTP, and a
  // Jetson without a charged RTC cell boots believing it is 1970. A few
  // seconds of skew is enough to paint every layer permanently dead, or -- far
  // worse -- permanently alive after it has stopped.
  clockOffset: 0,
};

/**
 * Live telemetry over the WebSocket, with replay as a first-class alternative.
 *
 * Reconnects on its own with a backoff. A dropped socket is normal -- the board
 * gets rebooted, the laptop sleeps -- and it must never need a page refresh in
 * the middle of a flight.
 */
export function useTelemetry(mode) {
  const [state, setState] = useState(EMPTY);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  const sock = useRef(null);
  const retry = useRef(0);
  const timer = useRef(null);

  const loadSession = useCallback((doc) => {
    // A session file is meant to be the whole record, openable with no board
    // and no network. Everything the live dashboard gets off the bus is in
    // here, so unpack all of it rather than only the records.
    const codes = doc.codes || [];
    const byCode = new Map(codes.map((c) => [c.code, c]));

    // Per-layer logs are stored grouped; the panels want one stream. The kind
    // and description are added by the API from the registry when live, so in
    // replay they have to come from the registry carried in the file.
    const logs = Object.entries(doc.logs || {})
      .flatMap(([layer, v]) => (v.rows || []).map((r) => ({
        ...r,
        layer: r.layer || layer,
        kind: r.kind || byCode.get(r.code)?.kind || 'step',
        description: r.description || byCode.get(r.code)?.description || '',
      })))
      .sort((a, b) => (a.t_unix || 0) - (b.t_unix || 0));

    // The live shape is { status, live }; a recorded layer is neither live nor
    // dead, so `live` is left false and the cards render a recorded state.
    const layers = Object.fromEntries(
      Object.entries(doc.layers || {}).map(([id, status]) => [id, { status, live: false }]),
    );

    setState({
      ...EMPTY,
      records: doc.records || [],
      summary: doc.summary || null,
      header: doc.header || null,
      config: doc.header?.config || null,
      board: doc.header?.board || null,
      map: doc.map || null,
      basemap: doc.basemap || null,
      codes,
      logs,
      layers,
      replay: true,
    });
  }, []);

  // Switching modes has to drop what the other mode left on screen. The socket
  // effect below only stops listening; it does not clear, and live records that
  // survive into replay make the "open a session" panel think one is already
  // loaded.
  const reset = useCallback(() => {
    setState(EMPTY);
    setError(null);
  }, []);

  useEffect(() => {
    if (mode !== 'live') return undefined;
    let closed = false;

    const connect = () => {
      const url = wsUrl();
      if (!url) return;
      let ws;
      try { ws = new WebSocket(url); } catch (e) { setError(String(e)); return; }
      sock.current = ws;

      ws.onopen = () => { retry.current = 0; setConnected(true); setError(null); };
      ws.onerror = () => setError(`cannot reach ${apiBase()}`);
      ws.onclose = () => {
        setConnected(false);
        if (closed) return;
        // Backoff to 10 s. A board that is off should not be hammered.
        const wait = Math.min(10000, 500 * 2 ** retry.current++);
        timer.current = setTimeout(connect, wait);
      };
      ws.onmessage = (ev) => {
        let msg;
        try { msg = JSON.parse(ev.data); } catch { return; }
        setState((prev) => reduce(prev, msg));
      };
    };

    connect();
    return () => {
      closed = true;
      clearTimeout(timer.current);
      sock.current?.close();
    };
  }, [mode]);

  return { state, connected, error, loadSession, reset, setState };
}

function reduce(prev, msg) {
  const { type, data } = msg;
  switch (type) {
    case 'snapshot':
      return {
        ...EMPTY,
        layers: data.layers || {},
        logs: data.logs || [],
        records: data.records || [],
        fixes: data.fixes || [],
        map: data.map, frame: data.frame, board: data.board, config: data.config,
        // Measured once per connection, which is where it is most accurate:
        // the snapshot is the first thing sent after the socket opens, so the
        // round trip is one hop and the error is a few milliseconds against a
        // 4 s staleness threshold.
        clockOffset: typeof data.server_time === 'number'
          ? data.server_time - Date.now() / 1000
          : 0,
      };
    case 'log':
      return { ...prev, logs: [...prev.logs, data].slice(-400) };
    case 'status':
      return { ...prev, layers: { ...prev.layers, [data.layer]: { status: data, live: true } } };
    case 'fix':
      return { ...prev, fixes: [...prev.fixes, data].slice(-200) };
    case 'record':
      return { ...prev, records: [...prev.records, data].slice(-5000) };
    case 'frame':
      return { ...prev, frame: data };
    case 'map':
      return { ...prev, map: data };
    default:
      return prev;
  }
}
