'use client';
import { memo } from 'react';
import {
  CartesianGrid, Legend, Line, LineChart, ReferenceLine,
  ResponsiveContainer, Tooltip, XAxis, YAxis,
} from 'recharts';

const AXIS = { stroke: 'var(--ink-faint)', fontSize: 11, tickLine: false };
const TIP = {
  contentStyle: {
    background: 'var(--panel-2)', border: '1px solid var(--line)',
    borderRadius: 8, fontSize: 12, fontFamily: 'var(--mono)', color: 'var(--ink)',
  },
  labelStyle: { color: 'var(--ink-faint)' },
};

function Chart({ title, subtitle, children }) {
  return (
    <div className="panel">
      <header>
        <h2>{title}</h2>
        {subtitle && <span className="note" style={{ marginLeft: 'auto' }}>{subtitle}</span>}
      </header>
      <div className="body" style={{ height: 224 }}>
        <ResponsiveContainer width="100%" height="100%">{children}</ResponsiveContainer>
      </div>
    </div>
  );
}

const WINDOW = 400;

// Memoised: a camera frame arrives 15-30 times a second and re-renders the
// page, but none of those change a record. Without this, four recharts trees
// are rebuilt per frame on a machine whose CPU the matcher wants.
function Charts({ records, budgetMs = 250 }) {
  const data = (records || []).slice(-WINDOW).map((r) => ({
    step: r.time_step,
    error: r.error_m,
    sigma: r.sigma_m,
    loss: r.loss,
    latency: r.latency_ms,
  }));

  return (
    <div className="grid cols-2">
      <Chart title="Position error" subtitle="predicted vs actual, and the sigma claimed for it">
        <LineChart data={data} margin={{ top: 6, right: 10, left: 0, bottom: 0 }}>
          <CartesianGrid stroke="var(--line)" strokeDasharray="2 4" vertical={false} />
          <XAxis dataKey="step" {...AXIS} />
          <YAxis {...AXIS} unit=" m" width={64} />
          <Tooltip {...TIP} />
          <Legend wrapperStyle={{ fontSize: 11, color: 'var(--ink-faint)' }} />
          <Line type="monotone" dataKey="error" name="error (m)" stroke="var(--bad)"
                dot={false} strokeWidth={1.8} isAnimationActive={false} connectNulls />
          <Line type="monotone" dataKey="sigma" name="stated sigma (m)" stroke="var(--accent)"
                dot={false} strokeWidth={1.4} strokeDasharray="4 3" isAnimationActive={false} connectNulls />
        </LineChart>
      </Chart>

      <Chart title="Calibration loss" subtitle="negative log-likelihood of the error under the stated sigma">
        <LineChart data={data} margin={{ top: 6, right: 10, left: 0, bottom: 0 }}>
          <CartesianGrid stroke="var(--line)" strokeDasharray="2 4" vertical={false} />
          <XAxis dataKey="step" {...AXIS} />
          <YAxis {...AXIS} width={64} />
          <Tooltip {...TIP} />
          <Line type="monotone" dataKey="loss" name="loss" stroke="var(--warn)"
                dot={false} strokeWidth={1.8} isAnimationActive={false} connectNulls />
        </LineChart>
      </Chart>

      <Chart title="Latency" subtitle={`capture to fix; ArduPilot compensates only to ${budgetMs} ms`}>
        <LineChart data={data} margin={{ top: 6, right: 10, left: 0, bottom: 0 }}>
          <CartesianGrid stroke="var(--line)" strokeDasharray="2 4" vertical={false} />
          <XAxis dataKey="step" {...AXIS} />
          <YAxis {...AXIS} unit=" ms" width={62} />
          <Tooltip {...TIP} />
          <ReferenceLine y={budgetMs} stroke="var(--bad)" strokeDasharray="5 4"
                         label={{ value: `${budgetMs} ms budget`, fill: 'var(--bad)', fontSize: 10, position: 'insideTopRight' }} />
          <Line type="monotone" dataKey="latency" name="latency (ms)" stroke="var(--ok)"
                dot={false} strokeWidth={1.8} isAnimationActive={false} connectNulls />
        </LineChart>
      </Chart>

      <Chart
        title="Error distribution"
        subtitle={(records || []).length > WINDOW
          /* The Session panel summarises every record; these charts hold the
             last WINDOW. Say so, or the p99 in the panel and the tail of this
             curve look like they disagree. */
          ? `last ${WINDOW} of ${records.length} — sorted ascending`
          : 'sorted ascending — read the knee, not the tail'}
      >
        <LineChart
          data={data.map((d) => d.error).filter((v) => Number.isFinite(v)).sort((a, b) => a - b)
            .map((v, i, arr) => ({ q: Math.round((100 * (i + 1)) / arr.length), error: v }))}
          margin={{ top: 6, right: 10, left: 0, bottom: 0 }}
        >
          <CartesianGrid stroke="var(--line)" strokeDasharray="2 4" vertical={false} />
          <XAxis dataKey="q" {...AXIS} unit="%" />
          <YAxis {...AXIS} unit=" m" width={64} />
          <Tooltip {...TIP} />
          <Line type="monotone" dataKey="error" name="error (m)" stroke="var(--accent)"
                dot={false} strokeWidth={1.8} isAnimationActive={false} />
        </LineChart>
      </Chart>
    </div>
  );
}

export default memo(Charts);
