'use client';
import { useEffect, useState } from 'react';
import { getJSON, postControl, uploadMap } from '@/lib/api';

const FIRMWARES = [
  { id: 'ardupilot', label: 'ArduPilot (EKF3)' },
  { id: 'px4', label: 'PX4 (EKF2)' },
];
const CONTROLLERS = ['pixhawk6c', 'pixhawk6x', 'cube-orange-plus', 'cubepilot-orange', 'matek-h743', 'sitl'];
const FEEDS = [
  { id: 'file', label: 'Video file (replay)' },
  { id: 'uvc', label: 'USB camera (live)' },
  { id: 'rtsp', label: 'Network stream' },
  { id: 'env80', label: 'AnyVisLoc env80 (dataset)' },
  { id: 'gz', label: 'Gazebo camera (simulation)' },
];
const LOOPS = [
  { id: 'off', label: 'Off — nothing sent' },
  { id: 'open', label: 'Open — send actual GPS' },
  { id: 'closed', label: 'Closed — send predicted GPS' },
];

export default function ControlBar({ layers, methods, disabled, loopMode, Panel }) {
  const [busy, setBusy] = useState(null);
  const [err, setErr] = useState(null);
  const [cov, setCov] = useState(null);       // /api/covariance
  const [mapInfo, setMapInfo] = useState(null);

  const loadCov = () => getJSON('/api/covariance').then(setCov).catch(() => setCov(null));
  useEffect(() => { loadCov(); }, [layers?.processing?.status?.config?.method]);

  // Two of these controls ask before they apply, and the design marks them
  // "confirms" on the label so it is visible BEFORE the click rather than
  // after. Feed interrupts the pipeline; loop mode decides what reaches the
  // vehicle. Everything else is a live setting and applies straight away.
  const [pending, setPending] = useState(null);

  const dl = layers?.data?.status?.config || {};
  const pl = layers?.processing?.status?.config || {};
  const ol = layers?.output?.status?.config || {};

  const send = async (layer, body) => {
    setBusy(layer); setErr(null);
    try { await postControl(layer, body); }
    catch (e) { setErr(String(e.message || e)); }
    finally { setBusy(null); }
  };

  // Two sources, two shapes, and they must be normalised or the greying-out
  // silently stops working. `/api/methods` returns the full survey --
  // {orb: {available, reason, label, blurb}} -- while the processing layer's
  // state carries `methods_available`, a plain {orb: true} boolean map. The
  // old code tested `available[m] === false` against whichever arrived, so
  // with the survey (an object, never === false) nothing was ever disabled.
  const raw = methods || pl.methods_available || {};
  const info = (m) => {
    const v = raw[m];
    if (v && typeof v === 'object') return v;
    return { available: v !== false, reason: '', label: m, blurb: '' };
  };
  const methodNames = Object.keys(raw).length
    ? Object.keys(raw)
    : ['orb', 'sift', 'akaze', 'xfeat_mnn', 'xfeat_lg',
       'edgepoint2_t32', 'edgepoint2_s32', 'edgepoint2_s64'];

  const loop = loopMode || ol.loop_mode || 'off';
  const Wrap = Panel || (({ title, right, children }) => (
    <div className="panel"><header><h2>{title}</h2><span className="spacer" />{right}</header>{children}</div>
  ));

  return (
    <Wrap
      title="Configuration"
      info={<>Each change sends one control message to the layer that owns it; nothing is
        written back to <code>configs/system.yaml</code>, so a restart returns to the
        file&rsquo;s known state. Feed and loop mode ask before they apply &mdash; one
        interrupts the pipeline, the other decides what reaches the vehicle.</>}
      right={(
        <>
          {err && <span className="meta" style={{ color: 'var(--bad)' }}>{err}</span>}
          <span className={`badge ${loop === 'closed' ? 'bad' : loop === 'open' ? 'accent' : ''}`}>
            loop {loop}
          </span>
        </>
      )}
    >
      <div className="body">
        <div className="controls">
          <div className="field">
            <label className="label" htmlFor="feed">
              Feed<span className="badge">confirms</span>
            </label>
            <select id="feed" disabled={disabled || busy === 'data'} value={dl.feed?.kind || 'file'}
                    onChange={(e) => setPending({
                      kind: 'feed', value: e.target.value,
                      text: `Reopen the feed as "${FEEDS.find((f) => f.id === e.target.value)?.label}". `
                          + 'The data layer closes the current source and re-opens; fixes stop '
                          + 'until it is live again.',
                      apply: () => send('data', { cmd: 'reopen_feed', feed: { type: e.target.value } }),
                    })}>
              {FEEDS.map((f) => <option key={f.id} value={f.id}>{f.label}</option>)}
            </select>
          </div>

          <div className="field">
            <label className="label" htmlFor="method">Method</label>
            <select id="method" disabled={disabled || busy === 'processing'} value={pl.method || 'xfeat_mnn'}
                    onChange={async (e) => {
                      // The reference store holds descriptors from one
                      // extractor, so changing the matcher changes the store.
                      // The data layer is told first; its store id hashes the
                      // method in, so a method already built comes back from
                      // cache and only a new one costs preprocessing time.
                      const m = e.target.value;
                      await send('data', { cmd: 'rebuild_map', method: m });
                      await send('processing', { cmd: 'set_method', method: m });
                    }}>
              {methodNames.map((m) => {
                const i = info(m);
                return (
                  <option key={m} value={m} disabled={!i.available}
                          title={i.available ? i.blurb : i.reason}>
                    {i.label || m}{i.available ? '' : ' (unavailable)'}
                  </option>
                );
              })}
            </select>
            {(() => {
              const i = info(pl.method || 'xfeat_mnn');
              const text = i.available ? i.blurb : i.reason;
              return text ? <span className="note">{text}</span> : null;
            })()}
          </div>

          <div className="field">
            <label className="label" htmlFor="fw">Flight firmware</label>
            <select id="fw" disabled={disabled} value={ol.fc?.firmware || 'ardupilot'}
                    onChange={(e) => send('output', { cmd: 'set', path: 'output_layer.fc.firmware', value: e.target.value })}>
              {FIRMWARES.map((f) => <option key={f.id} value={f.id}>{f.label}</option>)}
            </select>
          </div>

          <div className="field">
            <label className="label" htmlFor="fcu">Flight computer</label>
            <select id="fcu" disabled={disabled} value={ol.fc?.controller || 'pixhawk6c'}
                    onChange={(e) => send('output', { cmd: 'set', path: 'output_layer.fc.controller', value: e.target.value })}>
              {CONTROLLERS.map((c) => <option key={c} value={c}>{c}</option>)}
            </select>
          </div>

          <div className="field">
            <label className="label" htmlFor="loop">
              Loop mode<span className="badge">confirms</span>
            </label>
            <select id="loop" disabled={disabled || busy === 'output'} value={loop}
                    onChange={(e) => {
                      const v = e.target.value;
                      setPending({
                        kind: 'loop', value: v,
                        text: v === 'closed'
                          ? 'Closed loop writes the PREDICTED position to the flight controller. '
                            + 'It stays gated per fix on the inlier floor, the 100 m altitude cap '
                            + 'and the absence of live device errors in any layer.'
                          : v === 'open'
                            ? 'Open loop sends the ACTUAL GPS over the ExternalNav path. The '
                              + 'position cannot mislead the filter, so this tests the plumbing.'
                            : 'Nothing will be sent to the vehicle.',
                        apply: () => send('output', { cmd: 'set_loop_mode', mode: v }),
                      });
                    }}>
              {LOOPS.map((l) => <option key={l.id} value={l.id}>{l.label}</option>)}
            </select>
          </div>

          {/* THE COVARIANCE BACKEND, and the reason it is a labelled choice
              rather than a toggle. `gate_only` emits a fixed sigma and says so
              in every record. `learned` emits metres from a trained model --
              but a model is trained on ONE matcher, and this project's central
              finding is that a system which swaps matchers silently inherits a
              covariance model that no longer works. So every model shows what
              it was trained on, and a mismatch is called out before it is
              applied, not after. */}
          <div className="field">
            <label className="label" htmlFor="covb">Covariance</label>
            <select id="covb" disabled={disabled || busy === 'processing'}
                    value={cov?.current?.backend || pl.covariance?.backend || 'gate_only'}
                    onChange={async (e) => {
                      const b = e.target.value;
                      await send('processing', {
                        cmd: 'set', path: 'processing_layer.covariance.backend', value: b });
                      loadCov();
                    }}>
              <option value="gate_only">Fixed placeholder (gate_only)</option>
              <option value="learned" disabled={!cov?.models?.length}>
                Learned estimator{cov?.models?.length ? '' : ' (no model on disk)'}
              </option>
            </select>
            {(cov?.current?.backend || pl.covariance?.backend) !== 'learned' && (
              <span className="note">
                A constant, labelled a placeholder in every record. Honest, and not calibrated
                against measured error.
              </span>
            )}
          </div>

          {(cov?.current?.backend || pl.covariance?.backend) === 'learned' && (
            <div className="field">
              <label className="label" htmlFor="covm">Covariance model</label>
              <select id="covm" disabled={disabled || busy === 'processing'}
                      value={cov?.current?.model_path || ''}
                      onChange={async (e) => {
                        await send('processing', {
                          cmd: 'set', path: 'processing_layer.covariance.model_path',
                          value: e.target.value });
                        loadCov();
                      }}>
                <option value="">select a model</option>
                {(cov?.models || []).map((m) => (
                  <option key={m.path} value={m.path}>
                    {m.name} — trained on {m.trained_on}{m.matches_current ? '' : ' (MISMATCH)'}
                  </option>
                ))}
              </select>
              {(() => {
                const m = (cov?.models || []).find((x) => x.path === cov?.current?.model_path);
                if (!m) return <span className="note">No model selected; the layer falls back to the placeholder.</span>;
                if (!m.matches_current) {
                  return (
                    <span className="note" style={{ color: 'var(--warn)' }}>
                      This model was trained on <b>{m.trained_on}</b> and the matcher is{' '}
                      <b>{cov?.current_method}</b>. Eq. 7 collapsing across descriptors is this
                      project&rsquo;s own finding; a learned model is no different. Expect the
                      metres to be wrong.
                    </span>
                  );
                }
                return <span className="note">{m.validation}. n={m.n_train}.</span>;
              })()}
            </div>
          )}

          <div className="field">
            <label className="label" htmlFor="loss">Loss</label>
            <select id="loss" disabled={disabled} value={ol.loss || 'nll'}
                    onChange={(e) => send('output', { cmd: 'set', path: 'output_layer.loss', value: e.target.value })}>
              <option value="nll">NLL (calibration)</option>
              <option value="l2">L2 (squared error)</option>
            </select>
          </div>
        </div>

        {/* THE REFERENCE MAP, uploaded the way an operator does it on the
            ground: point the system at imagery of where it is about to fly.
            Two steps, and the second is the expensive one -- the store holds
            descriptors for the whole map and is built once here rather than
            per frame in the air.

            The georeference is reported back because the failure that matters
            is silent: a plain image with a .tif extension loads without error
            and then matches against nothing. */}
        <div style={{ marginTop: 16, borderTop: '1px solid var(--line)', paddingTop: 13 }}>
          <span className="kicker">Reference map</span>
          <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', alignItems: 'center', marginTop: 7 }}>
            <input type="file" id="mapfile" accept=".tif,.tiff,.png,.jpg,.jpeg"
                   disabled={disabled || busy === 'map'}
                   onChange={async (e) => {
                     const f = e.target.files?.[0];
                     if (!f) return;
                     setBusy('map'); setErr(null); setMapInfo(null);
                     try {
                       const info = await uploadMap(f);
                       setMapInfo(info);
                     } catch (ex) { setErr(String(ex.message || ex)); }
                     finally { setBusy(null); }
                   }} />
            <button className="btn amber"
                    disabled={disabled || !mapInfo?.path || busy === 'data' || !mapInfo?.projected}
                    onClick={() => setPending({
                      kind: 'map', value: mapInfo.path,
                      text: `Use "${mapInfo.path}" as the reference map and rebuild the feature `
                          + 'store. That is one pass of the detector over the whole map, so it '
                          + 'takes as long as the map is large, and fixes stop until it is done.',
                      apply: async () => {
                        await send('data', { cmd: 'set', path: 'data_layer.map.source', value: mapInfo.path });
                        await send('data', { cmd: 'rebuild_map' });
                      },
                    })}>
              Use this map
            </button>
            <span className="meta">current: <code>{dl.map?.source || '--'}</code></span>
          </div>

          {mapInfo && (
            <p className="note" style={{ marginTop: 9, color: mapInfo.projected ? undefined : 'var(--bad)' }}>
              {mapInfo.projected ? (
                <>Uploaded <code>{mapInfo.path}</code> &mdash; {mapInfo.width}&times;{mapInfo.height} px,
                  {' '}{mapInfo.crs}, {Number(mapInfo.gsd_m_px).toFixed(4)} m/px. Ready to use.</>
              ) : (
                <>{mapInfo.warning || 'This file carries no projected CRS.'} The pipeline needs a
                  UTM GeoTIFF; it will not be accepted.</>
              )}
            </p>
          )}
        </div>

        {pending && (
          <div className="confirm">
            <span className="kicker">Confirm</span>
            <p>{pending.text}</p>
            <button className="btn amber"
                    onClick={() => { pending.apply(); setPending(null); }}>Apply</button>
            <button className="btn" onClick={() => setPending(null)}>Cancel</button>
          </div>
        )}

        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', alignItems: 'center', marginTop: 13 }}>
          <button className="btn" disabled={disabled} onClick={() => send('data', { cmd: 'pause' })}>Pause feed</button>
          <button className="btn" disabled={disabled} onClick={() => send('data', { cmd: 'resume' })}>Resume</button>
          <button className="btn" disabled={disabled} onClick={() => send('processing', { cmd: 'reset_prior' })}>
            Reset prior
          </button>
          <button className="btn" disabled={disabled} onClick={() => send('output', { cmd: 'flush' })}>Flush session</button>
          <span style={{ flex: 1 }} />
          <button className="btn danger" disabled={disabled}
                  onClick={() => { ['data', 'processing', 'output'].forEach((l) => send(l, { cmd: 'stop' })); }}>
            Stop all layers
          </button>
        </div>

        {busy === 'data' && (
          <p className="note" style={{ marginTop: 12 }}>
            Rebuilding the reference feature store for this method. Already-built methods
            come back from cache immediately; a new one takes as long as one pass over the
            map. Fixes resume automatically.
          </p>
        )}

        {loop === 'closed' && (
          <p className="note" style={{ marginTop: 12, color: 'var(--warn)' }}>
            Closed loop. The predicted position is being written to the flight controller.
            It is still gated per fix on the inlier floor, the 100 m altitude cap and the
            absence of live device errors in any layer.
          </p>
        )}
      </div>
    </Wrap>
  );
}
