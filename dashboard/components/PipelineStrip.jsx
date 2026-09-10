'use client';
import { BANDS, CFG, toRow } from '@/lib/thresholds';
import { band, bandColour } from '@/lib/viz';

/**
 * The User view's high-level substitute for LayerCards + StepProgress: three
 * layers, each with a status dot, one plain-English line and five named
 * milestones coloured from the same thresholds as the graphs.
 *
 *   <PipelineStrip layers={state.layers} record={records.at(-1)} config={cfg} />
 *
 * `record` is a raw records.jsonl row (or null). `applying` is the id of a
 * layer with a control message in flight — that card greys until it lands.
 */
const LAYERS = [
  { id: 'data', name: 'Data layer',
    line: 'Map store cached, feed decoding, MAVLink position and attitude arriving. Frames are rescaled to the reference GSD before they leave.' },
  { id: 'processing', name: 'Processing layer',
    line: 'Frame yaw-rectified, candidate tiles taken from the last accepted fix, matched, homography checked for plausibility, covariance attached.' },
  { id: 'output', name: 'Output layer',
    line: 'Each fix is paired with the actual GPS taken at the same instant, scored, and written to the session file.' },
];

export default function PipelineStrip({ layers, record, config = CFG, applying = null,
                                        paused = false, Panel }) {
  const r = record ? toRow(record, 0) : null;
  const k = (m) => (r ? band(m, r[m], BANDS) : 'none');
  const chips = {
    data: [['Map store', 'good'], ['Feed', paused ? 'warn' : 'good'], ['GPS link', 'good'],
           ['Scale lock', k('alt') === 'bad' ? 'warn' : 'good'], ['Publishing', paused ? 'pending' : 'good']],
    processing: [['Store attached', 'good'], ['Rectify', 'good'], ['Tile select', 'good'],
                 ['Match', k('inl')], ['Fix out', k('ms')]],
    output: [['Paired', 'good'], ['Error', k('e')], ['Loss', 'good'], ['Export', 'good'],
             ['To FC', config.loop_mode === 'closed' ? 'good' : 'pending']],
  };

  // One rollup for the header, so the panel answers "is anything wrong" before
  // you read three cards. A layer with no status has not reported at all,
  // which is different from one reporting a bad number.
  const live = LAYERS.filter((L) => layers?.[L.id]?.status?.ready);
  const worst = Object.values(chips).flat()
    .reduce((w, [, t]) => (t === 'bad' ? 'bad' : t === 'warn' && w !== 'bad' ? 'warn' : w), 'good');
  const rollup = live.length < LAYERS.length
    ? { tone: 'bad', text: `${LAYERS.length - live.length} layer not reporting` }
    : worst === 'good' ? { tone: 'ok', text: 'all three layers nominal' }
      : { tone: worst === 'bad' ? 'bad' : 'warn', text: `attention on this fix` };

  const Wrap = Panel || (({ title, right, children }) => (
    <div className="panel"><header><h2>{title}</h2><span className="spacer" />{right}</header>{children}</div>
  ));

  return (
    <Wrap
      title="Pipeline"
      info={<>Three independent processes on one ZeroMQ bus. Killing one does not stop the
        others &mdash; its card simply stops reporting. Each named milestone below takes its
        colour from the same thresholds the graphs use, so a red chip and a red stretch of
        track are the same fix.</>}
      right={(
        <>
          <span className="meta">this fix</span>
          <span className={`badge ${rollup.tone}`}>{rollup.text}</span>
        </>
      )}
    >
      <div className="layers">
        {LAYERS.map((L) => {
          const busy = applying === L.id;
          const st = layers?.[L.id]?.status;
          const dead = !st?.ready;
          return (
            <div key={L.id} className={`layer ${busy || dead ? 'dead' : ''}`}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
                <span className={`dot ${dead ? 'bad' : 'ok'}`} />
                <span className="name">{L.name}</span>
                <span style={{ flex: 1 }} />
                <span className="rate">
                  {busy ? 'applying…' : st?.rate_hz != null ? `${st.rate_hz.toFixed(2)} Hz` : '--'}
                </span>
              </div>
              <p>{L.line}</p>
              <div className="chips">
                {chips[L.id].map(([label, tone]) => (
                  <span key={label}
                        className={`chip ${tone === 'pending' || tone === 'none' ? ''
                          : tone === 'good' ? 'ok' : tone}`}>{label}</span>
                ))}
              </div>
            </div>
          );
        })}
      </div>
    </Wrap>
  );
}
