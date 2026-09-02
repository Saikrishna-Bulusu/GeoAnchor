'use client';

const LAYERS = [
  { id: 'data',       name: 'Data layer',       prefix: 'DL' },
  { id: 'processing', name: 'Processing layer', prefix: 'PL' },
  { id: 'output',     name: 'Output layer',     prefix: 'OL' },
];

/**
 * One card per layer, each showing its own step-code stream.
 *
 * They are separate cards rather than one merged log on purpose: the layers
 * are independent processes, and the dashboard should make it obvious when one
 * has died while the other two carry on.
 */
export default function LayerCards({ layers, logs }) {
  return (
    <div className="grid cols-3">
      {LAYERS.map((L) => {
        const entry = layers?.[L.id];
        const st = entry?.status;
        const fresh = st ? Date.now() / 1000 - st.t_unix < 4 : false;
        const counts = st?.counts || {};
        const dev = counts.live_device_errors || 0;
        const rows = (logs || []).filter((l) => l.layer === L.id).slice(-60).reverse();

        let tone = 'stale';
        if (fresh && dev === 0) tone = 'live';
        else if (fresh) tone = 'warn';

        return (
          <div className="panel layer" key={L.id}>
            <div className="head">
              <span className={`dot ${tone}`} />
              <span className="name">{L.name}</span>
              <span className="code">{st?.last_code || `${L.prefix}-??`}</span>
            </div>
            <div className="tallies">
              <span className="tally"><b>{counts.steps ?? 0}</b> steps</span>
              <span className="tally err"><b>{counts.errors ?? 0}</b> errors</span>
              <span className="tally dev"><b>{counts.device_errors ?? 0}</b> device</span>
              <span style={{ marginLeft: 'auto' }}>
                {st ? `${st.rate_hz?.toFixed?.(2) ?? '0.00'} Hz` : 'no heartbeat'}
              </span>
            </div>
            <div className="codestream">
              {rows.length === 0 && (
                <div className="empty">
                  {fresh ? 'no codes yet' : 'layer is not reporting'}
                </div>
              )}
              {rows.map((l, i) => (
                <div className="row" key={`${l.t_unix}-${i}`} title={l.description}>
                  <span className={`c ${l.kind || 'step'}`}>{l.code}</span>
                  <span className="m">{l.message}</span>
                </div>
              ))}
            </div>
          </div>
        );
      })}
    </div>
  );
}
