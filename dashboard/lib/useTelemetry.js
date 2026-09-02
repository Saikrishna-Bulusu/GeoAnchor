'use client';
import { useCallback, useEffect, useRef, useState } from 'react';
import { apiBase, wsUrl } from './api';

const EMPTY = { layers: {}, logs: [], records: [], fixes: [], map: null, frame: null, board: null, config: null };

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
    setState({
      ...EMPTY,
      records: doc.records || [],
      summary: doc.summary || null,
      header: doc.header || null,
      config: doc.header?.config || null,
      board: doc.header?.board || null,
      replay: true,
    });
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

  return { state, connected, error, loadSession, setState };
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
