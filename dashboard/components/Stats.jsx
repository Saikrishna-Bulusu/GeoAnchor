'use client';
import { fmt } from '@/lib/api';

/**
 * The summary is computed from whatever rows are on screen, so it works
 * identically live and in replay.
 *
 * No mean and no RMSE anywhere. A single degenerate solve -- and this project
 * has seen fixes wrong by 1e93 m -- destroys both, which is why the whole
 * study reports medians and percentiles instead.
 */
function pct(rows, q) {
  if (!rows.length) return null;
  const s = [...rows].sort((a, b) => a - b);
  const pos = (s.length - 1) * q;
  const lo = Math.floor(pos), hi = Math.ceil(pos);
  return lo === hi ? s[lo] : s[lo] + (s[hi] - s[lo]) * (pos - lo);
}

/** Numeric and finite. A NaN would not merely blank a cell: it does not compare
 *  against anything, so it survives sort() wherever it started and drags every
 *  percentile past it to the wrong row. Dropping it is the only safe reading. */
const finite = (xs) => xs.filter((v) => typeof v === 'number' && Number.isFinite(v));

export function summarise(records) {
  const errs = finite(records.map((r) => r.error_m));
  const lat = finite(records.map((r) => r.latency_ms));
  const losses = finite(records.map((r) => r.loss));
  const accepted = records.filter((r) => r.accepted).length;
  return {
    n: records.length,
    accepted,
    // ACCEPT rate, and deliberately not "error rate". A rejected fix is the
    // gate doing its job -- CLAUDE.md's headline env80 result is a 20% accept
    // rate that it calls honest -- so presenting the complement as an error
    // would paint the system's correct behaviour as an 80% failure.
    acceptRate: records.length ? accepted / records.length : null,
    median: pct(errs, 0.5),
    p90: pct(errs, 0.9),
    p99: pct(errs, 0.99),
    scored: errs.length,
    within10: errs.length ? errs.filter((e) => e <= 10).length / errs.length : null,
    medLoss: pct(losses, 0.5),
    medLatency: pct(lat, 0.5),
    p95Latency: pct(lat, 0.95),
  };
}

export default function Stats({ records, budgetMs = 250 }) {
  const s = summarise(records);
  // The only cell with a right and a wrong side. Latency over budget is a real
  // fault: ArduPilot does not reject a late fix, it stamps it as current and
  // fuses it at the wrong time, so nothing downstream will complain either.
  const latencyTone = s.p95Latency == null ? '' : s.p95Latency > budgetMs ? 'bad' : 'good';

  const cells = [
    { k: 'fixes', v: s.n, u: '' },
    { k: 'accepted', v: s.acceptRate == null ? '--' : (s.acceptRate * 100).toFixed(1), u: '%' },
    { k: 'median error', v: fmt(s.median), u: 'm' },
    { k: 'p90 error', v: fmt(s.p90), u: 'm' },
    { k: 'p99 error', v: fmt(s.p99), u: 'm' },
    { k: 'within 10 m', v: s.within10 == null ? '--' : (s.within10 * 100).toFixed(0), u: '%' },
    { k: 'median loss', v: fmt(s.medLoss, 3), u: '' },
    { k: 'p95 latency', v: fmt(s.p95Latency, 0), u: 'ms', tone: latencyTone },
  ];

  return (
    <div className="panel">
      <header>
        <h2>Session</h2>
        <span className="note" style={{ marginLeft: 'auto' }}>
          {s.scored < s.n
            /* Percentiles are over SCORED fixes, which is a smaller set than
               the fix count beside them. Saying so stops the two being read as
               one population. */
            ? `median and percentiles over ${s.scored} scored of ${s.n} — no mean, no RMSE`
            : 'median and percentiles only — no mean, no RMSE'}
        </span>
      </header>
      <div className="stats">
        {cells.map((c) => (
          <div className={`stat ${c.tone || ''}`} key={c.k}>
            <div className="k">{c.k}</div>
            <div className="v">{c.v}<span className="u">{c.u}</span></div>
          </div>
        ))}
      </div>
    </div>
  );
}
