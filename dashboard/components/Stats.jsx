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

export function summarise(records) {
  const errs = records.map((r) => r.error_m).filter((v) => v !== null && v !== undefined);
  const lat = records.map((r) => r.latency_ms).filter((v) => v !== null && v !== undefined);
  const losses = records.map((r) => r.loss).filter((v) => v !== null && v !== undefined);
  const accepted = records.filter((r) => r.accepted).length;
  return {
    n: records.length,
    accepted,
    errorRate: records.length ? 1 - accepted / records.length : null,
    median: pct(errs, 0.5),
    p90: pct(errs, 0.9),
    p99: pct(errs, 0.99),
    within10: errs.length ? errs.filter((e) => e <= 10).length / errs.length : null,
    medLoss: pct(losses, 0.5),
    medLatency: pct(lat, 0.5),
    p95Latency: pct(lat, 0.95),
  };
}

export default function Stats({ records, budgetMs = 250 }) {
  const s = summarise(records);
  const latencyTone = s.p95Latency == null ? '' : s.p95Latency > budgetMs ? 'bad' : 'good';
  const errTone = s.errorRate == null ? '' : s.errorRate > 0.3 ? 'bad' : s.errorRate > 0.1 ? 'warn' : 'good';

  const cells = [
    { k: 'fixes', v: s.n, u: '' },
    { k: 'error rate', v: s.errorRate == null ? '--' : (s.errorRate * 100).toFixed(1), u: '%', tone: errTone },
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
          median and percentiles only &mdash; no mean, no RMSE
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
