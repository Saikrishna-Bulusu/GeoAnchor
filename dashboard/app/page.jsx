'use client';
import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import CameraView from '@/components/CameraView';
import ControlBar from '@/components/ControlBar';
import LayerCards from '@/components/LayerCards';
import MetricChart from '@/components/MetricChart';
import PipelineStrip from '@/components/PipelineStrip';
import RecordTable from '@/components/RecordTable';
import StepProgress from '@/components/StepProgress';
import TrackMap, { fitView } from '@/components/TrackMap';
import { apiBase, downloadJSON, getJSON } from '@/lib/api';
import { useTelemetry } from '@/lib/useTelemetry';
import { BANDS, CFG, toRows } from '@/lib/thresholds';
import { band, bandColour, yawAt } from '@/lib/viz';

const DEFAULT_MODE = process.env.NEXT_PUBLIC_GEOANCHOR_MODE || 'live';

// The four graphs the design specifies, in its order. `loss` is deliberately
// not among them: it is a function of `e` and sigma, so it earns a column in
// the table but not a quarter of the screen.
const GRAPHS = ['ms', 'e', 'inl', 'alt'];
const METRICS = [['ms', 'latency'], ['e', 'error'], ['inl', 'inliers'], ['alt', 'altitude']];

const pct = (n, d) => (d ? `${Math.round((n / d) * 100)}%` : '--');
const quantile = (sorted, q) =>
  (sorted.length ? sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * q))] : null);

/** The registration marks are decorative and drawn in CSS; this carries the
 *  second pair, because a single element has only ::before and ::after. */
const Marks = () => <span className="marks" aria-hidden="true" />;

function Panel({ title, info, right, meta, children, className = '', style }) {
  const [open, setOpen] = useState(false);
  return (
    <section className={`panel ${className}`} style={style}>
      <Marks />
      <header>
        <h2>{title}</h2>
        {info && (
          <button className="info-btn" aria-expanded={open} title="what this panel means"
                  onClick={() => setOpen((v) => !v)}>i</button>
        )}
        {meta && <span className="meta">{meta}</span>}
        <span className="spacer" />
        {right}
      </header>
      {info && open && <p className="info-body">{info}</p>}
      {children}
    </section>
  );
}

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
  const mapCanvas = useRef(null);

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
  const [view, setView] = useState({ cx: 0.5, cy: 0.5, zoom: 1 });
  const [show, setShow] = useState({ predicted: true, error: true, sigma: true, rejects: true });

  useEffect(() => {
    if (mode !== 'live' || !connected) return;
    getJSON('/api/methods').then(setMethods).catch(() => {});
    getJSON('/api/runs').then(setRuns).catch(() => {});
  }, [mode, connected]);

  useEffect(() => {
    if (mode !== 'replay') return;
    getJSON('/api/fleet').then(setFleet).catch(() => setFleet({ available: false }));
  }, [mode]);

  // Staleness is a function of elapsed time, not of arriving messages, so
  // something has to re-render when nothing is happening.
  const [, tick] = useState(0);
  useEffect(() => {
    if (mode !== 'live') return undefined;
    const h = setInterval(() => tick((n) => n + 1), 1000);
    return () => clearInterval(h);
  }, [mode]);

  const records = state.records || [];
  const rows = useMemo(() => toRows(records), [records]);
  const budget = state.config?.processing_layer?.latency_budget_ms ?? 250;
  const synthetic = state.header?.synthetic_from_reference
    || String(state.config?.data_layer?.feed?.path || '').includes('demo/flight');

  const cfg = useMemo(() => ({
    ...CFG,
    latency_budget_ms: budget,
    inlier_gate: state.config?.processing_layer?.inlier_gate ?? CFG.inlier_gate,
    loop_mode: state.config?.output_layer?.fc?.loop_mode ?? 'off',
  }), [budget, state.config]);

  // The bands the map and the graphs colour against are the RUNNING config's,
  // not the file's defaults -- and so are the sentences describing them. The
  // Thresholds panel was reading BANDS.inl.why verbatim, so it said "the gate
  // of 25" while the board ran a gate of 8. A key that explains a number has
  // to move when the number does.
  const bands = useMemo(() => {
    const gate = state.config?.processing_layer?.inlier_gate ?? CFG.inlier_gate;
    const quality = Math.max(gate, CFG.min_quality_inliers);
    return {
      ...BANDS,
      ms: { ...BANDS.ms, good: budget, warn: budget * 2,
            why: `green under the ${budget} ms EKF3 delay budget; red past ${budget * 2} ms` },
      inl: { ...BANDS.inl, good: quality, warn: gate,
             why: `red below the gate of ${gate}; amber below the ${quality} needed for closed loop` },
    };
  }, [budget, state.config]);

  // Fix quality rescopes to a dragged selection, which is what makes the
  // graphs and this panel one instrument rather than two.
  const scoped = useMemo(() => {
    if (!sel || sel.length !== 2) return rows;
    const [a, b] = sel;
    return rows.filter((r) => r.s >= Math.min(a, b) && r.s <= Math.max(a, b));
  }, [rows, sel]);

  const quality = useMemo(() => {
    const lat = scoped.map((r) => r.ms).filter((v) => v != null).sort((a, b) => a - b);
    const err = scoped.filter((r) => r.ok && r.e != null).map((r) => r.e).sort((a, b) => a - b);
    const inl = scoped.map((r) => r.inl).filter((v) => v != null).sort((a, b) => a - b);
    const acc = scoped.filter((r) => r.ok).length;
    const over = lat.filter((v) => v > budget).length;
    const p95 = quantile(lat, 0.95);
    const med = quantile(err, 0.5);
    return [
      { k: 'p95 latency', v: p95 == null ? '--' : Math.round(p95), u: 'ms',
        tone: p95 == null ? '' : (p95 <= budget ? 'ok' : p95 <= budget * 2 ? 'warn' : 'bad'),
        note: `${over} of ${lat.length} over ${budget} ms` },
      { k: 'median error', v: med == null ? '--' : med.toFixed(2), u: 'm',
        tone: med == null ? '' : (med <= bands.e.good ? 'ok' : med <= bands.e.warn ? 'warn' : 'bad'),
        note: err.length
          ? `p90 ${quantile(err, 0.9).toFixed(2)} m · p99 ${quantile(err, 0.99).toFixed(2)} m`
          : 'no scored fixes' },
      { k: 'accepted', v: pct(acc, scoped.length), u: '',
        tone: scoped.length && acc / scoped.length >= 0.5 ? 'ok' : 'warn',
        note: `${acc} of ${scoped.length} fixes` },
      { k: 'median inliers', v: quantile(inl, 0.5) ?? '--', u: '',
        tone: '', note: inl.length
          ? `lowest ${inl[0]} · gate ${cfg.inlier_gate}` : 'no fixes yet' },
    ];
  }, [scoped, budget, bands, cfg.inlier_gate]);

  const openFile = useCallback(async (file) => {
    try {
      loadSession(JSON.parse(await file.text()));
      setMode('replay');
    } catch (e) {
      alert(`not a GeoAnchor session file: ${e.message}`);
    }
  }, [loadSession]);

  const toggleMode = useCallback((next) => {
    if (next === mode) return;
    reset();
    setCursor(null); setSel(null); setDomain(null);
    setMode(next);
  }, [reset, mode]);

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
    if (connected) return { tone: 'ok', label: `live · ${apiBase().replace(/^https?:\/\//, '')}` };
    return { tone: 'bad', label: error ? 'no connection' : 'connecting' };
  }, [mode, connected, error]);

  const admin = role === 'admin';
  const at = cursor == null ? rows.length - 1 : cursor;
  const here = rows[at];
  const t0 = rows[0]?.t ?? 0;
  // The map packet's keys are `gsd_m_px` and `epsg`; `crs` exists but is null
  // on a store built from a UTM GeoTIFF, so reading it first showed a bare
  // "32756" with no units and no EPSG prefix.
  const gsd = state.map?.gsd_m_px;
  const epsg = state.map?.epsg;
  const mapMeta = state.map
    ? [gsd ? `${Number(gsd).toFixed(3)} m/px` : null,
       epsg ? `EPSG:${epsg}` : state.map.crs].filter(Boolean).join(' · ')
    : '';

  const zoomBy = (k) => setView((v) => ({ ...v, zoom: Math.max(0.6, Math.min(14, v.zoom * k)) }));

  // Fit the TRACK, which is what the button says. It used to reset to
  // {cx: .5, cy: .5, zoom: 1} -- that is "fit the whole reference tile", the
  // `whole` branch of viz.fitView -- so on a flight covering a corner of a
  // 4112 px tile the button zoomed OUT and lost the track rather than framing
  // it. Following also fights it: the follow effect re-centres on the vehicle
  // every fix, so a fit while following is undone before it is seen. Turn it
  // off, the same way dragging the map does.
  const fitFlight = useCallback(() => {
    if (!mapCanvas.current || !state.map || !records.length) return;
    setFollow(false);
    setView(fitView(mapCanvas.current, state.map, records, false));
  }, [state.map, records]);

  const fitWhole = useCallback(() => {
    setFollow(false);
    setView({ cx: 0.5, cy: 0.5, zoom: 1 });
  }, []);

  // The legend spells out the same rule `band()` applies, in the same three
  // colours, so the key under the map and the colour on the track cannot drift.
  const bandLegend = (m) => {
    const b = bands[m];
    if (b.envelope) return [[`${b.envelope[0]}–${b.envelope[1]} m`, 'good'],
                            ['outside the envelope', 'bad']];
    if (b.invert) return [[`≥ ${b.good}`, 'good'], [`${b.warn}–${b.good}`, 'warn'],
                          [`< ${b.warn}`, 'bad']];
    return [[`≤ ${b.good}${b.unit}`, 'good'], [`${b.good}–${b.warn}${b.unit}`, 'warn'],
            [`> ${b.warn}${b.unit}`, 'bad']];
  };
  const readColour = (m, v) => bandColour(band(m, v, bands));
  const fmtMetric = (m, v) =>
    (v == null || !Number.isFinite(v) ? '--' : `${Number(v).toFixed(m === 'inl' ? 0 : 2)}${bands[m].unit}`);

  return (
    <main onDragOver={(e) => { e.preventDefault(); setDragging(true); }}
          onDragLeave={() => setDragging(false)} onDrop={onDrop}>

      {/* ── masthead ──────────────────────────────────────────────────── */}
      <div className="panel masthead" style={{ flexDirection: 'row' }}>
        <Marks />
        <div style={{ display: 'flex', alignItems: 'baseline', gap: 11, minWidth: 0 }}>
          <span className="brand">GeoAnchor</span>
          <span className="sub">GNSS-denied absolute visual localization</span>
        </div>
        <span className="spacer" />
        {/* Role is a view filter, never a permission. Admin adds the step
            codes, the record table and the camera; it unlocks nothing,
            because the control path is the API's and is identical either way. */}
        <div className="seg">
          {['user', 'admin'].map((r) => (
            <button key={r} className={role === r ? 'on' : ''} onClick={() => setRole(r)}>{r}</button>
          ))}
        </div>
        <div className="seg">
          <button className={mode === 'live' ? 'on' : ''} onClick={() => toggleMode('live')}>Live</button>
          <button className={mode === 'replay' ? 'on' : ''} onClick={() => toggleMode('replay')}>
            Replay a session
          </button>
        </div>
        <button className="btn" onClick={exportNow} disabled={!records.length}>Export</button>
        <div className="status"><span className={`dot ${status.tone}`} />{status.label}</div>
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

      {/* ── replay picker ─────────────────────────────────────────────── */}
      {mode === 'replay' && !state.replay && (
        <Panel title="Replay" meta={fleet?.available ? `${fleet.runs?.length || 0} fleet sessions` : ''}
               info={<>Replay needs no board. It reads the <code>session.json</code> an output
                 layer wrote &mdash; header, summary and one row per fix &mdash; so live telemetry
                 stays on the LAN and only the file travels. The basemap is the reference tile&rsquo;s
                 own preview, drawn against the same geotransform the matcher used.</>}>
          <div className="body">
            <div className={`drop ${dragging ? 'over' : ''}`}
                 onClick={() => fileInput.current?.click()}>
              <span className="t">Drop a session.json</span>
              <span className="h">or click to choose one</span>
              <input ref={fileInput} type="file" accept="application/json" hidden
                     onChange={(e) => e.target.files?.[0] && openFile(e.target.files[0])} />
            </div>

            {runs.length > 0 && (
              <div style={{ marginTop: 13 }}>
                <span className="kicker">On this board</span>
                <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', marginTop: 7 }}>
                  {runs.slice(0, 8).map((r) => (
                    <button key={r.name} className="btn"
                            onClick={() => getJSON(`/api/runs/${r.name}`).then(loadSession)}>
                      {r.name}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {/* Every device's sessions, from the logs-repo clone. A board that
                is powered off is still reviewable here, which is the point. */}
            {fleet?.available && fleet.runs?.length > 0 && (
              <div style={{ marginTop: 16, borderTop: '1px solid var(--line)', paddingTop: 13 }}>
                <span className="kicker">
                  Fleet &mdash; {fleet.devices.length} device{fleet.devices.length === 1 ? '' : 's'}
                </span>
                <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', margin: '7px 0 9px' }}>
                  <button className={`btn ${device === null ? 'on' : ''}`}
                          onClick={() => setDevice(null)}>all</button>
                  {fleet.devices.map((d) => (
                    <button key={d} className={`btn ${device === d ? 'on' : ''}`}
                            onClick={() => setDevice(d)}>{d}</button>
                  ))}
                </div>
                <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap' }}>
                  {fleet.runs.filter((r) => device === null || r.device === device)
                    .slice(0, 12).map((r) => (
                      <button key={`${r.device}/${r.name}`} className="btn"
                              onClick={() => getJSON(`/api/fleet/${r.device}/${r.name}`).then(loadSession)}>
                        <span style={{ color: 'var(--ink-faint)' }}>{r.device}</span>&nbsp;/&nbsp;{r.name}
                      </button>
                    ))}
                </div>
              </div>
            )}

            {fleet && !fleet.available && (
              <p className="note" style={{ marginTop: 13 }}>
                No <code>fleet/</code> clone here yet &mdash; run{' '}
                <code>bash scripts/sync_logs.sh</code> to pull other devices&rsquo; sessions.
              </p>
            )}
          </div>
        </Panel>
      )}

      {/* ── configuration ─────────────────────────────────────────────── */}
      {mode === 'live' && (
        <ControlBar layers={state.layers} methods={methods} disabled={!connected}
                    loopMode={cfg.loop_mode} Panel={Panel} />
      )}

      {/* ── pipeline ──────────────────────────────────────────────────── */}
      {Object.keys(state.layers || {}).length > 0 && (
        admin ? (
          <>
            <LayerCards layers={state.layers} logs={state.logs}
                        clockOffset={state.clockOffset} replay={state.replay} />
            <StepProgress layers={state.layers} codes={state.codes} />
          </>
        ) : (
          <PipelineStrip layers={state.layers} record={records.at(-1)} config={cfg} Panel={Panel} />
        )
      )}

      {/* ── position + camera ─────────────────────────────────────────── */}
      <div className="row">
        <Panel className="grow" title="Position" meta={mapMeta}
               info={<>Drag to pan, scroll to zoom, click a fix to park the cursor on it. The
                 arrow is the vehicle, pointing along yaw. The track is coloured by whatever
                 metric is selected below, on the thresholds in the panel to the right.
                 Breadcrumbs stamp the elapsed time on rejected and red fixes only.</>}
               right={(
                 <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap' }}>
                   <button className="btn icon" title="zoom out" onClick={() => zoomBy(1 / 1.3)}>&minus;</button>
                   <button className="btn icon" title="zoom in" onClick={() => zoomBy(1.3)}>+</button>
                   <button className="btn" onClick={fitFlight} disabled={!records.length}
                           title="frame the track">Fit flight</button>
                   <button className="btn" onClick={fitWhole} title="frame the whole reference tile">
                     Fit map
                   </button>
                   <button className={`btn ${follow ? 'on' : ''}`}
                           onClick={() => setFollow((f) => !f)}>Follow</button>
                   <button className="btn" onClick={() => setExpanded(expanded === 'map' ? null : 'map')}>
                     {expanded === 'map' ? 'Collapse' : 'Expand'}
                   </button>
                 </div>
               )}>
          {/* Heights are viewport-relative with a pixel floor, not a fixed
              460. A hard 460 px is most of a phone screen before the panel
              chrome, and it cannot be overridden from CSS because an inline
              style wins over a media query. */}
          <div className="mapbox" style={{ flex: 'none',
                 height: expanded === 'map' ? 'min(80vh, 700px)' : 'clamp(300px, 46vh, 460px)' }}>
            <TrackMap map={state.map} records={records} basemap={state.basemap}
                      metric={metric} cursor={cursor} hover={hover} sel={sel} bands={bands}
                      onHover={setHover} onPick={setCursor} follow={follow}
                      view={view} onView={setView} canvasRef={mapCanvas}
                      /* `error` was missing from this object, so the ERROR
                         toggle above the map set state that nothing read and
                         the whiskers never drew. All four keys go through. */
                      show={{ actual: true, predicted: show.predicted,
                              error: show.error, sigma: show.sigma,
                              rejects: show.rejects }} />

            {here && (
              <div className="overlay tl">
                <div className="cap">step {here.s} &nbsp; +{(here.t - t0).toFixed(1)}s</div>
                <div>{here.la?.toFixed(6)}, {here.lo?.toFixed(6)}</div>
                <div>alt {here.alt?.toFixed(1)} m AGL &nbsp; yaw {yawAt(rows, at).toFixed(0)}&deg;</div>
                <div style={{ color: readColour(metric, here[metric]) }}>
                  {bands[metric].label} {fmtMetric(metric, here[metric])}
                </div>
              </div>
            )}

            <div className="map-toggles">
              {[['predicted', 'Predicted'], ['error', 'Error'],
                ['sigma', 'Sigma ring'], ['rejects', 'Breadcrumbs']].map(([k, label]) => (
                <button key={k} className={`btn sm ${show[k] ? 'on' : ''}`}
                        onClick={() => setShow((s) => ({ ...s, [k]: !s[k] }))}>{label}</button>
              ))}
            </div>

            <div className="overlay br">
              <div className="cap">{bands[metric].label} on the track</div>
              <div className="legend">
                {bandLegend(metric).map(([text, tone]) => (
                  <span key={text}><i style={{ background: bandColour(tone) }} />{text}</span>
                ))}
              </div>
            </div>
          </div>

          <footer>
            <div style={{ display: 'flex', alignItems: 'center', gap: 10, flex: '1 1 auto', flexWrap: 'wrap' }}>
              <span className="kicker">Colour by</span>
              <div style={{ display: 'flex', gap: 5, flexWrap: 'wrap' }}>
                {METRICS.map(([k, label]) => (
                  <button key={k} className={`btn sm ${metric === k ? 'on' : ''}`}
                          onClick={() => setMetric(k)}>{label}</button>
                ))}
              </div>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: 10, flex: '1 1 240px', minWidth: 0 }}>
              <span className="meta" style={{ flex: 'none' }}>
                fix {rows.length ? at + 1 : 0} / {rows.length}
              </span>
              <input type="range" min={0} max={Math.max(0, rows.length - 1)} value={at}
                     onChange={(e) => setCursor(Number(e.target.value))}
                     style={{ flex: '1 1 120px', minWidth: 100 }} />
            </div>
          </footer>
        </Panel>

        {mode === 'live' && (
          <Panel className="side" title="Camera"
                 meta={state.frame?.w ? `${state.frame.w}x${state.frame.h} · seq ${state.frame.seq ?? '--'}` : 'no frame'}>
            {/* CameraView draws its own alt/yaw/capture-age overlay from the
                FRAME's telemetry, which is the right source here -- it is the
                attitude that frame was rectified with, not the cursor's. */}
            <div style={{ position: 'relative', height: 'clamp(200px, 30vh, 300px)',
                          background: '#000' }}>
              <CameraView frame={state.frame} bare />
            </div>
          </Panel>
        )}
      </div>

      {/* ── fix quality + thresholds ──────────────────────────────────── */}
      <div className="row">
        <Panel className="half" title="Fix quality"
               meta={sel ? `selection · ${scoped.length} fixes` : 'this flight so far'}>
          <div className="tiles">
            {quality.map((q) => (
              <div className="stat" key={q.k}>
                <div className="k">{q.k}</div>
                <div className={`v ${q.tone}`}>{q.v}{q.u && <u>{q.u}</u>}</div>
                <div className="note">{q.note}</div>
              </div>
            ))}
          </div>
        </Panel>

        <Panel className="third" title="Thresholds" meta="configs/system.yaml">
          <div className="thresholds">
            {GRAPHS.map((m) => (
              <div key={m}>
                <i className="swatch" style={{ background: bandColour('good') }} />
                <span className="t-label">{bands[m].label}</span>
                <span className="t-rule">{bands[m].why}</span>
              </div>
            ))}
          </div>
        </Panel>
      </div>

      {/* ── along the flight ──────────────────────────────────────────── */}
      <Panel title="Along the flight"
             info={<>Hover a graph to place that fix on the map. Drag across one to select a
               stretch &mdash; Fix quality rescopes to it. Scroll to zoom the time axis; the
               y-axis rescales to the window. Expand opens one graph full screen. The x-axis is
               elapsed seconds from the first fix, so a gap reads as time lost rather than as
               records missing.</>}
             right={(
               <>
                 {domain && (
                   <button className="btn sm on" onClick={() => setDomain(null)}>Reset zoom</button>
                 )}
                 {sel && <button className="btn sm" onClick={() => setSel(null)}>Clear selection</button>}
               </>
             )}>
        <div className="grid-1px">
          {GRAPHS.map((m) => (
            <div className="chart" key={m}>
              <div className="head">
                <button className={`title ${metric === m ? 'on' : ''}`} onClick={() => setMetric(m)}
                        title="colour the track by this">{bands[m].label}</button>
                <span className="spacer" />
                {here && here[m] != null && (
                  <span className="read" style={{ color: readColour(m, here[m]) }}>
                    {fmtMetric(m, here[m])}
                  </span>
                )}
                <button className="btn sm" onClick={() => setExpanded(expanded === m ? null : m)}>
                  Expand
                </button>
              </div>
              <div className="plot">
                <MetricChart records={records} metric={m} domain={domain}
                             hover={hover} sel={sel} cursor={cursor} bands={bands}
                             onHover={setHover} onSelect={setSel} onZoom={setDomain} />
              </div>
            </div>
          ))}
        </div>
      </Panel>

      {admin && <RecordTable records={records} />}

      <p className="foot">
        Layers run as three independent processes. Killing one does not stop the others &mdash;
        its card simply stops reporting. This dashboard is an observer: it subscribes to the
        bus and sends operator commands, and nothing in the pipeline depends on it running.
      </p>

      {/* ── one graph, full screen ────────────────────────────────────── */}
      {expanded && expanded !== 'map' && (
        <div className="fullscreen" onClick={(e) => e.target === e.currentTarget && setExpanded(null)}>
          <div className="fs-head">
            <span className="fs-title">{bands[expanded].label}</span>
            <span className="meta">{bands[expanded].why}</span>
            <span className="spacer" />
            <div style={{ display: 'flex', gap: 5, flexWrap: 'wrap' }}>
              {GRAPHS.map((m) => (
                <button key={m} className={`btn sm ${expanded === m ? 'on' : ''}`}
                        onClick={() => setExpanded(m)}>{bands[m].label}</button>
              ))}
            </div>
            <button className="btn" onClick={() => setExpanded(null)}>Close</button>
          </div>
          <div className="plot">
            <MetricChart records={records} metric={expanded} domain={domain}
                         hover={hover} sel={sel} cursor={cursor} bands={bands} big
                         onHover={setHover} onSelect={setSel} onZoom={setDomain} />
          </div>
        </div>
      )}
    </main>
  );
}
