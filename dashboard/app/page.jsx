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
  const { state, connected, error, loadSession } = useTelemetry(mode);
  const [methods, setMethods] = useState(null);
  const [dragging, setDragging] = useState(false);
  const [runs, setRuns] = useState([]);
  const fileInput = useRef(null);

  useEffect(() => {
    if (mode !== 'live' || !connected) return;
    getJSON('/api/methods').then(setMethods).catch(() => {});
    getJSON('/api/runs').then(setRuns).catch(() => {});
  }, [mode, connected]);

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
        <button className="btn" onClick={() => setMode(mode === 'live' ? 'replay' : 'live')}>
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

      {mode === 'replay' && !records.length && (
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

      {mode === 'live' && <LayerCards layers={state.layers} logs={state.logs} />}

      {mode === 'live' && <div style={{ height: 14 }} />}
      {mode === 'live' && <StepProgress layers={state.layers} />}

      <div style={{ height: 14 }} />
      <Stats records={records} budgetMs={budget} />

      <div style={{ height: 14 }} />
      <div className="grid main">
        <MapView map={state.map} records={records} />
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
