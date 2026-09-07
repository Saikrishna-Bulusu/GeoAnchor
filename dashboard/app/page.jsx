'use client';
import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import CameraView from '@/components/CameraView';
import Charts from '@/components/Charts';
import ControlBar from '@/components/ControlBar';
import LayerCards from '@/components/LayerCards';
import MapView from '@/components/MapView';
import RecordTable from '@/components/RecordTable';
import Stats from '@/components/Stats';
import StepProgress from '@/components/StepProgress';
import { apiBase, downloadJSON, getJSON } from '@/lib/api';
import { useTelemetry } from '@/lib/useTelemetry';

const DEFAULT_MODE = process.env.NEXT_PUBLIC_GEOANCHOR_MODE || 'live';

export default function Page() {
  const [mode, setMode] = useState(DEFAULT_MODE);
  const { state, connected, error, loadSession, reset } = useTelemetry(mode);
  const [methods, setMethods] = useState(null);
  const [dragging, setDragging] = useState(false);
  const [runs, setRuns] = useState([]);
  const fileInput = useRef(null);

  useEffect(() => {
    if (mode !== 'live' || !connected) return;
    getJSON('/api/methods').then(setMethods).catch(() => {});
    getJSON('/api/runs').then(setRuns).catch(() => {});
  }, [mode, connected]);

  // Staleness is a function of elapsed time, not of arriving messages, so
  // something has to re-render when nothing is happening. Without this a layer
  // that dies while the others are also quiet keeps its last colour: the panel
  // is only repainted by the very traffic whose absence it is meant to report.
  const [, tick] = useState(0);
  useEffect(() => {
    if (mode !== 'live') return undefined;
    const h = setInterval(() => tick((n) => n + 1), 1000);
    return () => clearInterval(h);
  }, [mode]);

  const records = state.records || [];
  const budget = state.config?.processing_layer?.latency_budget_ms ?? 250;
  const synthetic = state.header?.synthetic_from_reference
    || String(state.config?.data_layer?.feed?.path || '').includes('demo/flight');

  const openFile = useCallback(async (file) => {
    try {
      loadSession(JSON.parse(await file.text()));
      setMode('replay');
    } catch (e) {
      alert(`not a GeoAnchor session file: ${e.message}`);
    }
  }, [loadSession]);

  // Clearing before the switch is the whole fix: `records` outliving a
  // live -> replay switch left the panel below unrendered, so replay mode
  // showed live data and offered no way to open a file. Going the other way it
  // drops the replayed session rather than leaving it up until the first
  // snapshot lands.
  const toggleMode = useCallback(() => {
    reset();
    setMode((m) => (m === 'live' ? 'replay' : 'live'));
  }, [reset]);

  const onDrop = useCallback((e) => {
    e.preventDefault(); setDragging(false);
    const f = e.dataTransfer?.files?.[0];
    if (f) openFile(f);
  }, [openFile]);

  const exportNow = useCallback(() => {
    downloadJSON(
      { schema: 1, header: state.header || { exported_from: 'dashboard', config: state.config },
        summary: state.summary || null, records },
      `geoanchor_${new Date().toISOString().replace(/[:.]/g, '-')}.json`,
    );
  }, [records, state]);

  const status = useMemo(() => {
    if (mode === 'replay') return { tone: '', label: 'replay' };
    if (connected) return { tone: 'live', label: `live · ${apiBase().replace(/^https?:\/\//, '')}` };
    return { tone: 'stale', label: error ? 'no connection' : 'connecting' };
  }, [mode, connected, error]);

  return (
    <main className="shell" onDragOver={(e) => { e.preventDefault(); setDragging(true); }}
          onDragLeave={() => setDragging(false)} onDrop={onDrop}>
      <div className="topbar">
        <div className="brand">
          <h1>GeoAnchor</h1>
          <span>GNSS-denied absolute visual localization</span>
        </div>
        <span className="spacer" />
        <span className="mode"><span className={`dot ${status.tone}`} />{status.label}</span>
        {/* Only while a session is actually on screen. Clearing it without
            leaving replay mode is what re-renders the picker below, so this is
            the direct route to a second file -- the alternative is a round
            trip out to live and back, which needs a reachable board. */}
        {mode === 'replay' && state.replay && (
          <button className="btn" onClick={reset}>Load another</button>
        )}
        <button className="btn" onClick={toggleMode}>
          {mode === 'live' ? 'Replay a session' : 'Back to live'}
        </button>
        <button className="btn primary" onClick={exportNow} disabled={!records.length}>
          Export JSON
        </button>
      </div>

      {synthetic && (
        <div className="banner">
          <span>&#9888;</span>
          <div>
            <b>Synthetic feed.</b> These frames were cut from the same image used as the
            reference map, so the pipeline is matching a picture against itself. This session
            shows the three layers are wired correctly. It is not an accuracy result and no
            number from it belongs in a table.
          </div>
        </div>
      )}

      {mode === 'replay' && !state.replay && (
        <div className="panel" style={{ marginBottom: 14 }}>
          <header><h2>Replay</h2></header>
          <div className="body">
            <div className={`drop ${dragging ? 'over' : ''}`}
                 onClick={() => fileInput.current?.click()}
                 style={{ cursor: 'pointer' }}>
              Drop a <code>session.json</code> here, or click to choose one.
              <input ref={fileInput} type="file" accept="application/json" hidden
                     onChange={(e) => e.target.files?.[0] && openFile(e.target.files[0])} />
            </div>
            {runs.length > 0 && (
              <div style={{ marginTop: 12 }}>
                <p className="note">Sessions on the board:</p>
                <div className="btn-row" style={{ marginTop: 6 }}>
                  {runs.slice(0, 8).map((r) => (
                    <button key={r.name} className="btn"
                            onClick={() => getJSON(`/api/runs/${r.name}`).then(loadSession)}>
                      {r.name}
                    </button>
                  ))}
                </div>
              </div>
            )}
            <p className="note" style={{ marginTop: 14 }}>
              Replay mode needs no board. It is what the Vercel deployment runs, because a
              public host cannot reach a Jetson on your network &mdash; live telemetry stays
              on the LAN and only the exported file travels.
            </p>
          </div>
        </div>
      )}

      {/* Shown in replay too. The step codes are the operational record of a
          flight -- which layer got how far, which faults fired -- and a system
          flown GNSS- and internet-denied is reviewed almost entirely from the
          file afterwards, which is exactly when these used to disappear. */}
      {Object.keys(state.layers || {}).length > 0 && (
        <>
          <LayerCards layers={state.layers} logs={state.logs}
                      clockOffset={state.clockOffset} replay={state.replay} />
          <div style={{ height: 14 }} />
          <StepProgress layers={state.layers} codes={state.codes} />
          <div style={{ height: 14 }} />
        </>
      )}

      <Stats records={records} budgetMs={budget} />

      <div style={{ height: 14 }} />
      <div className="grid main">
        <MapView map={state.map} records={records} basemap={state.basemap} />
        <div className="grid" style={{ gap: 14 }}>
          {mode === 'live' && <CameraView frame={state.frame} />}
          {mode === 'live' && (
            <ControlBar layers={state.layers} methods={methods} disabled={!connected} />
          )}
          {mode === 'replay' && state.header && (
            <div className="panel">
              <header><h2>Session header</h2></header>
              <div className="body">
                <div className="kv">
                  <b>tag</b><span>{state.header.session_tag || '--'}</span>
                  <b>board</b><span>{state.board?.model || state.board?.arch || '--'}</span>
                  <b>method</b><span>{state.config?.processing_layer?.method || '--'}</span>
                  <b>loss</b><span>{state.header.loss || '--'}</span>
                  <b>loop</b><span>{state.config?.output_layer?.fc?.loop_mode || '--'}</span>
                  <b>records</b><span>{records.length}</span>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>

      <div style={{ height: 14 }} />
      <Charts records={records} budgetMs={budget} />

      <div style={{ height: 14 }} />
      <RecordTable records={records} />

      <p className="note" style={{ marginTop: 18 }}>
        Layers run as three independent processes. Killing one does not stop the others &mdash;
        its card simply stops reporting. This dashboard is an observer: it subscribes to the
        bus and sends operator commands, and nothing in the pipeline depends on it running.
      </p>
    </main>
  );
}
