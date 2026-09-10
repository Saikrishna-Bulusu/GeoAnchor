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

export default function PipelineStrip({ layers, record, config = CFG, applying = null, paused = false }) {
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

  return (
    <div className="grid cols-3">
      {LAYERS.map((L) => {
        const busy = applying === L.id;
        const st = layers?.[L.id]?.status;
        return (
          <div key={L.id} className="panel" style={{ padding: '16px 18px', opacity: busy ? 0.45 : 1, transition: 'opacity .18s' }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
              <span className={`dot ${busy ? '' : 'live'}`} />
              <b style={{ fontSize: 19, textTransform: 'uppercase', letterSpacing: '.04em' }}>{L.name}</b>
              <span style={{ marginLeft: 'auto', fontFamily: 'var(--mono)', fontSize: 12.5, color: 'var(--ink-dim)' }}>
                {busy ? 'applying…' : st?.rate_hz != null ? `${st.rate_hz.toFixed(2)} Hz` : '--'}
              </span>
            </div>
            <p style={{ margin: '9px 0 13px', fontSize: 14.5, lineHeight: 1.5, color: 'var(--ink-dim)' }}>{L.line}</p>
            <div style={{ display: 'flex', flexWrap: 'wrap', gap: 5 }}>
              {chips[L.id].map(([label, tone]) => (
                <span key={label} style={{
                  padding: '4px 9px', fontSize: 12, fontWeight: 600, textTransform: 'uppercase', letterSpacing: '.09em',
                  border: `1px solid ${tone === 'pending' ? 'var(--line)' : bandColour(tone) + '88'}`,
                  background: tone === 'pending' ? 'transparent' : bandColour(tone) + '22',
                  color: tone === 'pending' ? 'var(--ink-faint)' : 'var(--ink)',
                }}>{label}</span>
              ))}
            </div>
          </div>
        );
      })}
    </div>
  );
}
