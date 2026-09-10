'use client';
import { memo } from 'react';
import { fmt } from '@/lib/api';

/** The export, visible. Every column here is a field in the JSON file.
 *  Memoised: 300 rows should not be rebuilt for every camera frame. */
function RecordTable({ records }) {
  const rows = (records || []).slice(-300).reverse();
  return (
    <div className="panel">
      <span className="marks" aria-hidden="true" />
      <header>
        <h2>Records</h2>
        <span className="spacer" /><span className="meta">
          newest first &mdash; rejected fixes are kept, not dropped
        </span>
      </header>
      <div className="body flush">
        <div className="tablewrap">
          <table>
            <thead>
              <tr>
                <th>step</th><th>actual lat</th><th>actual lon</th>
                <th>pred lat</th><th>pred lon</th>
                <th>error m</th><th>sigma m</th><th>loss</th>
                <th>inliers</th><th>ms</th><th>FC</th>
              </tr>
            </thead>
            <tbody>
              {rows.length === 0 && (
                <tr><td colSpan={11} style={{ textAlign: 'center', padding: 22 }}>no records yet</td></tr>
              )}
              {rows.map((r) => (
                <tr key={r.time_step} className={r.accepted ? '' : 'rejected'}>
                  <td>{r.time_step}</td>
                  <td>{fmt(r.actual_gps?.lat, 6)}</td>
                  <td>{fmt(r.actual_gps?.lon, 6)}</td>
                  <td>{fmt(r.predicted_gps?.lat, 6)}</td>
                  <td>{fmt(r.predicted_gps?.lon, 6)}</td>
                  <td>{fmt(r.error_m, 2)}</td>
                  <td>{fmt(r.sigma_m, 2)}</td>
                  <td>{fmt(r.loss, 2)}</td>
                  <td>{r.inliers ?? '--'}</td>
                  <td>{fmt(r.latency_ms, 0)}</td>
                  <td>{r.sent_to_fc ? 'sent' : '--'}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
}

export default memo(RecordTable);
