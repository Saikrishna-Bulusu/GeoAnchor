'use client';
import { useEffect, useState } from 'react';
import { getJSON } from '@/lib/api';

/**
 * Which steps has each layer actually completed.
 *
 * The tallies on the layer cards answer "how many"; this answers "which",
 * which is the question the step-code scheme exists to answer. A layer stuck
 * at DL-08 with no DL-11 has opened its map and not its camera, and that is
 * visible here in one glance instead of by reading a log.
 *
 * Steps are laid out in registry order because that is the order they fire in.
 * A gap in the middle is the interesting case.
 */
const LAYERS = [
  { id: 'data', prefix: 'DL', name: 'Data' },
  { id: 'processing', prefix: 'PL', name: 'Processing' },
  { id: 'output', prefix: 'OL', name: 'Output' },
];

export default function StepProgress({ layers }) {
  const [registry, setRegistry] = useState(null);

  useEffect(() => {
    getJSON('/api/codes').then(setRegistry).catch(() => {});
  }, []);

  if (!registry) return null;

  return (
    <div className="panel">
      <header>
        <h2>Steps completed</h2>
        <span className="note" style={{ marginLeft: 'auto' }}>
          filled = fired this session &middot; hover for the code
        </span>
      </header>
      <div className="body" style={{ display: 'grid', gap: 12 }}>
        {LAYERS.map((L) => {
          const seen = layers?.[L.id]?.status?.counts?.seen || {};
          const codes = registry.filter((c) => c.code.startsWith(L.prefix));
          const steps = codes.filter((c) => c.kind === 'step');
          const faults = codes.filter((c) => c.kind !== 'step' && seen[c.code]);
          const done = steps.filter((c) => seen[c.code]).length;
          return (
            <div key={L.id}>
              <div className="steprow-head">
                <b>{L.name} layer</b>
                <span>{done} / {steps.length} steps</span>
              </div>
              <div className="steps">
                {steps.map((c) => (
                  <span key={c.code}
                        className={`step ${seen[c.code] ? 'on' : ''}`}
                        title={`${c.code} — ${c.description}${seen[c.code] ? ` (${seen[c.code]}x)` : ' — not yet'}`}>
                    {c.code.split('-')[1]}
                  </span>
                ))}
              </div>
              {faults.length > 0 && (
                <div className="steps faults">
                  {faults.map((c) => (
                    <span key={c.code}
                          className={`step on ${c.kind}`}
                          title={`${c.code} — ${c.description} (${seen[c.code]}x)`}>
                      {c.code}
                    </span>
                  ))}
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
}
