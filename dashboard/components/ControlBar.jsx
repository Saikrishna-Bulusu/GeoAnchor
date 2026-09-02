'use client';
import { useState } from 'react';
import { postControl } from '@/lib/api';

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
];
const LOOPS = [
  { id: 'off', label: 'Off — nothing sent' },
  { id: 'open', label: 'Open — send actual GPS' },
  { id: 'closed', label: 'Closed — send predicted GPS' },
];

export default function ControlBar({ layers, methods, disabled }) {
  const [busy, setBusy] = useState(null);
  const [err, setErr] = useState(null);

  const dl = layers?.data?.status?.config || {};
  const pl = layers?.processing?.status?.config || {};
  const ol = layers?.output?.status?.config || {};

  const send = async (layer, body) => {
    setBusy(layer); setErr(null);
    try { await postControl(layer, body); }
    catch (e) { setErr(String(e.message || e)); }
    finally { setBusy(null); }
  };

  const available = methods || pl.methods_available || {};
  const methodNames = Object.keys(available).length
    ? Object.keys(available)
    : ['orb', 'sift', 'akaze', 'xfeat_mnn', 'xfeat_lg'];

  return (
    <div className="panel">
      <header>
        <h2>Configuration</h2>
        {err && <span className="note" style={{ marginLeft: 'auto', color: 'var(--bad)' }}>{err}</span>}
      </header>
      <div className="body">
        <div className="controls">
          <div className="field">
            <label htmlFor="feed">Feed</label>
            <select id="feed" disabled={disabled || busy === 'data'} value={dl.feed?.kind || 'file'}
                    onChange={(e) => send('data', { cmd: 'reopen_feed', feed: { type: e.target.value } })}>
              {FEEDS.map((f) => <option key={f.id} value={f.id}>{f.label}</option>)}
            </select>
          </div>

          <div className="field">
            <label htmlFor="method">Method</label>
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
              {methodNames.map((m) => (
                <option key={m} value={m} disabled={available[m] === false}>
                  {m}{available[m] === false ? ' (unavailable)' : ''}
                </option>
              ))}
            </select>
          </div>

          <div className="field">
            <label htmlFor="fw">Flight firmware</label>
            <select id="fw" disabled={disabled} value={ol.fc?.firmware || 'ardupilot'}
                    onChange={(e) => send('output', { cmd: 'set', path: 'output_layer.fc.firmware', value: e.target.value })}>
              {FIRMWARES.map((f) => <option key={f.id} value={f.id}>{f.label}</option>)}
            </select>
          </div>

          <div className="field">
            <label htmlFor="fcu">Flight computer</label>
            <select id="fcu" disabled={disabled} value={ol.fc?.controller || 'pixhawk6c'}
                    onChange={(e) => send('output', { cmd: 'set', path: 'output_layer.fc.controller', value: e.target.value })}>
              {CONTROLLERS.map((c) => <option key={c} value={c}>{c}</option>)}
            </select>
          </div>

          <div className="field">
            <label htmlFor="loop">Loop mode</label>
            <select id="loop" disabled={disabled || busy === 'output'} value={ol.loop_mode || 'open'}
                    onChange={(e) => send('output', { cmd: 'set_loop_mode', mode: e.target.value })}>
              {LOOPS.map((l) => <option key={l.id} value={l.id}>{l.label}</option>)}
            </select>
          </div>

          <div className="field">
            <label htmlFor="loss">Loss</label>
            <select id="loss" disabled={disabled} value={ol.loss || 'nll'}
                    onChange={(e) => send('output', { cmd: 'set', path: 'output_layer.loss', value: e.target.value })}>
              <option value="nll">NLL (calibration)</option>
              <option value="l2">L2 (squared error)</option>
            </select>
          </div>
        </div>

        <div className="btn-row" style={{ marginTop: 14 }}>
          <button className="btn" disabled={disabled} onClick={() => send('data', { cmd: 'pause' })}>Pause feed</button>
          <button className="btn" disabled={disabled} onClick={() => send('data', { cmd: 'resume' })}>Resume</button>
          <button className="btn" disabled={disabled} onClick={() => send('processing', { cmd: 'reset_prior' })}>
            Reset prior
          </button>
          <button className="btn" disabled={disabled} onClick={() => send('output', { cmd: 'flush' })}>Flush session</button>
          <span className="spacer" />
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

        {ol.loop_mode === 'closed' && (
          <p className="note" style={{ marginTop: 12, color: 'var(--warn)' }}>
            Closed loop. The predicted position is being written to the flight controller.
            It is still gated per fix on the inlier floor, the 100 m altitude cap and the
            absence of live device errors in any layer.
          </p>
        )}
      </div>
    </div>
  );
}
