'use client';
import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import CameraView from '@/components/CameraView';
import ControlBar from '@/components/ControlBar';
import LayerCards from '@/components/LayerCards';
import MetricChart from '@/components/MetricChart';
import PipelineStrip from '@/components/PipelineStrip';
import RecordTable from '@/components/RecordTable';
import Stats from '@/components/Stats';
import StepProgress from '@/components/StepProgress';
import TrackMap from '@/components/TrackMap';
import { apiBase, downloadJSON, getJSON } from '@/lib/api';
import { useTelemetry } from '@/lib/useTelemetry';
import { BANDS, CFG } from '@/lib/thresholds';

const DEFAULT_MODE = process.env.NEXT_PUBLIC_GEOANCHOR_MODE || 'live';

// The four graphs the design specifies, in its order. `loss` is deliberately
// not among them: it is a function of `e` and sigma, so it earns a column in
// the table but not a quarter of the screen.
const GRAPHS = ['ms', 'e', 'inl', 'alt'];

// What the map can colour the track by. Same keys, same bands, so a red
// stretch of track and a red patch of graph mean the same thing.
const METRICS = [['ms', 'latency'], ['e', 'error'], ['inl', 'inliers'], ['alt', 'altitude']];

export default function Page() {
  const [mode, setMode] = useState(DEFAULT_MODE);
  const [role, setRole] = useState('user');
  const { state, connected, error, loadSession, reset } = useTelemetry(mode);
  const [methods, setMethods] = useState(null);
  const [dragging, setDragging] = useState(false);
  const [runs, setRuns] = useState([]);
  const [fleet, setFleet] = useState(null);
  const [device, setDevice] = useState(null);
  const fileInput = useRef(null);

  // Map + graph interaction, shared so that hovering a graph moves the map
  // cursor and vice versa. `cursor` is a committed pick, `hover` is transient,
  // `sel` is a dragged range and `domain` is the graph x-zoom.
  const [metric, setMetric] = useState('ms');
  const [cursor, setCursor] = useState(null);
  const [hover, setHover] = useState(null);
  const [sel, setSel] = useState(null);
  const [domain, setDomain] = useState(null);
  const [follow, setFollow] = useState(true);
  const [expanded, setExpanded] = useState(null);

  useEffect(() => {
    if (mode !== 'live' || !connected) return;
    getJSON('/api/methods').then(setMethods).catch(() => {});
    getJSON('/api/runs').then(setRuns).catch(() => {});
  }, [mode, connected]);

  // Other devices' sessions, out of the logs-repo clone. Fetched whenever the
  // replay panel is on screen rather than only when live, because the whole
  // point is reviewing a board's flight from a laptop that is not on the same
  // network as the board.
  useEffect(() => {
    if (mode !== 'replay') return;
    getJSON('/api/fleet').then(setFleet).catch(() => setFleet({ available: false }));
  }, [mode]);

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

  // The thresholds the graphs and the map colour against are the running
  // config's, not the file's defaults, whenever the board tells us what it is
  // using. A dashboard colouring against 250 ms while the board runs 400 is
  // worse than one that says nothing.
  const cfg = useMemo(() => ({
    ...CFG,
    latency_budget_ms: budget,
    inlier_gate: state.config?.processing_layer?.inlier_gate ?? CFG.inlier_gate,
    loop_mode: state.config?.output_layer?.fc?.loop_mode ?? 'off',
  }), [budget, state.config]);

  const bands = useMemo(
    () => ({ ...BANDS, ms: { ...BANDS.ms, good: budget, warn: budget * 2 } }),
    [budget],
  );

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
    setCursor(null); setSel(null); setDomain(null);
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

  const admin = role === 'admin';

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

        {/* Role is a view filter, never a permission. The Admin view adds the
            step codes, the record table and the camera; it unlocks nothing,
            because the control path is the API's and is identical either way. */}
        <div className="btn-row">
          {['user', 'admin'].map((r) => (
            <button key={r} className={`btn ${role === r ? 'primary' : ''}`}
                    onClick={() => setRole(r)}>{r}</button>
          ))}
        </div>

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
            {/* Every device's sessions, from the logs repo. A board that is
                powered off is still reviewable here, which is the point:
                its transcripts were pushed when it last had a network. */}
            {fleet?.available && fleet.runs?.length > 0 && (
              <div style={{ marginTop: 16, borderTop: '1px solid var(--line)', paddingTop: 14 }}>
                <p className="note">
                  Fleet &mdash; {fleet.runs.length} sessions from {fleet.devices.length} device
                  {fleet.devices.length === 1 ? '' : 's'}:
                </p>
                <div className="btn-row" style={{ margin: '8px 0' }}>
                  <button className={`btn ${device === null ? 'primary' : ''}`}
                          onClick={() => setDevice(null)}>all</button>
                  {fleet.devices.map((d) => (
                    <button key={d} className={`btn ${device === d ? 'primary' : ''}`}
                            onClick={() => setDevice(d)}>{d}</button>
                  ))}
                </div>
                <div className="btn-row">
                  {fleet.runs
                    .filter((r) => device === null || r.device === device)
                    .slice(0, 12)
                    .map((r) => (
                      <button key={`${r.device}/${r.name}`} className="btn"
                              onClick={() => getJSON(`/api/fleet/${r.device}/${r.name}`)
                                .then(loadSession)}>
                        <span style={{ color: 'var(--ink-faint)' }}>{r.device}</span>
                        {' / '}{r.name}
                      </button>
                    ))}
                </div>
              </div>
            )}

            <p className="note" style={{ marginTop: 14 }}>
              Replay mode needs no board. It is what the Vercel deployment runs, because a
              public host cannot reach a Jetson on your network &mdash; live telemetry stays
              on the LAN and only the exported file travels.
              {fleet && !fleet.available && (
                <> No <code>fleet/</code> clone here yet &mdash; run{' '}
                <code>bash scripts/sync_logs.sh</code> to pull other devices&rsquo; sessions.</>
              )}
            </p>
          </div>
        </div>
      )}

      {/* The User view gets the three-layer summary; the Admin view gets the
          per-layer cards and the full step-code progress underneath, which is
          the operational record of a flight and is reviewed almost entirely
          from the file afterwards. */}
      {Object.keys(state.layers || {}).length > 0 && (
        <>
          {admin ? (
            <>
              <LayerCards layers={state.layers} logs={state.logs}
                          clockOffset={state.clockOffset} replay={state.replay} />
              <div style={{ height: 14 }} />
              <StepProgress layers={state.layers} codes={state.codes} />
            </>
          ) : (
            <PipelineStrip layers={state.layers} record={records.at(-1)} config={cfg} />
          )}
          <div style={{ height: 14 }} />
        </>
      )}

      <Stats records={records} budgetMs={budget} />

      <div style={{ height: 14 }} />
      <div className="grid main">
        <div className="panel" style={{ display: 'flex', flexDirection: 'column' }}>
          <header style={{ gap: 10 }}>
            <h2>Track</h2>
            <span className="spacer" />
            <div className="btn-row">
              {METRICS.map(([k, label]) => (
                <button key={k} className={`btn ${metric === k ? 'primary' : ''}`}
                        onClick={() => setMetric(k)}>{label}</button>
              ))}
              <button className={`btn ${follow ? 'primary' : ''}`}
                      onClick={() => setFollow((f) => !f)}>follow</button>
            </div>
          </header>
          <div className="body flush" style={{ height: 460 }}>
            <TrackMap map={state.map} records={records} basemap={state.basemap}
                      metric={metric} cursor={cursor} hover={hover} sel={sel} bands={bands}
                      onHover={setHover} onPick={setCursor} follow={follow} />
          </div>
        </div>

        <div className="grid" style={{ gap: 14 }}>
          {mode === 'live' && admin && <CameraView frame={state.frame} />}
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
      <div className="panel">
        <header>
          <h2>Metrics</h2>
          <span className="spacer" />
          <span className="note" style={{ fontSize: 11 }}>
            drag to select &middot; wheel to zoom time
          </span>
          {domain && (
            <button className="btn" onClick={() => setDomain(null)}>reset zoom</button>
          )}
        </header>
        <div className="body">
          <div className="grid cols-2">
            {GRAPHS.map((m) => (
              <div key={m}>
                <div style={{ display: 'flex', alignItems: 'baseline', gap: 8, marginBottom: 4 }}>
                  <b style={{ fontSize: 12, textTransform: 'uppercase', letterSpacing: '.07em' }}>
                    {bands[m].label}
                  </b>
                  <span className="note" style={{ fontSize: 11 }}>{bands[m].why}</span>
                  <span className="spacer" />
                  <button className="btn" onClick={() => setExpanded(expanded === m ? null : m)}>
                    {expanded === m ? 'shrink' : 'expand'}
                  </button>
                </div>
                <MetricChart records={records} metric={m} domain={domain}
                             hover={hover} sel={sel} cursor={cursor} bands={bands}
                             big={expanded === m} height={expanded === m ? 420 : 196}
                             onHover={setHover} onSelect={setSel} onZoom={setDomain} />
              </div>
            ))}
          </div>
        </div>
      </div>

      {admin && (
        <>
          <div style={{ height: 14 }} />
          <RecordTable records={records} />
        </>
      )}

      <p className="note" style={{ marginTop: 18 }}>
        Layers run as three independent processes. Killing one does not stop the others &mdash;
        its card simply stops reporting. This dashboard is an observer: it subscribes to the
        bus and sends operator commands, and nothing in the pipeline depends on it running.
      </p>
    </main>
  );
}
