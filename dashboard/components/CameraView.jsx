'use client';
import { useEffect, useRef, useState } from 'react';
import { apiBase, fmt } from '@/lib/api';

/**
 * The frame the matcher actually saw -- already rescaled to the reference GSD
 * and JPEG-encoded by the data layer, not the raw sensor image. That is the
 * useful thing to look at: if it is blurred, black or the wrong scale, the
 * matcher's failure is explained before anyone opens a log.
 *
 * Polled rather than streamed. An MJPEG socket would hold a connection open
 * against the same CPU the pipeline is trying to use, and a browser that
 * cannot keep up quietly builds a backlog.
 */
/** `bare` renders only the image and its overlay, for a caller that supplies
 *  its own panel chrome. The capture-age readout stays either way -- it is
 *  latency already spent before the pipeline runs, and it appears nowhere
 *  else on the dashboard. */
export default function CameraView({ frame, fps = 3, bare = false }) {
  const [src, setSrc] = useState(null);
  const timer = useRef(null);
  const url = useRef(null);

  useEffect(() => {
    let cancelled = false;
    const tick = async () => {
      try {
        const r = await fetch(`${apiBase()}/api/frame.jpg?t=${Date.now()}`, { cache: 'no-store' });
        if (r.ok && !cancelled) {
          const blob = await r.blob();
          if (url.current) URL.revokeObjectURL(url.current);
          url.current = URL.createObjectURL(blob);
          setSrc(url.current);
        }
      } catch { /* the board may be down; the next tick retries */ }
      if (!cancelled) timer.current = setTimeout(tick, 1000 / fps);
    };
    tick();
    return () => {
      cancelled = true;
      clearTimeout(timer.current);
      if (url.current) URL.revokeObjectURL(url.current);
    };
  }, [fps]);

  const inner = (
        <div className="camwrap" style={bare
          ? { position: 'absolute', inset: 0, display: 'grid', placeItems: 'center' } : undefined}>
          {src
            ? <img src={src} alt="live frame"
                   style={bare ? { maxWidth: '100%', maxHeight: '100%' } : undefined} />
            : <span className="note">waiting for the data layer</span>}
          {frame && (
            <div className="camoverlay"
                 style={bare ? { position: 'absolute', bottom: 8, right: 8, padding: '5px 9px',
                                 background: 'rgba(0,0,0,.62)', font: '11px/1.6 var(--mono)',
                                 color: 'var(--ink)', pointerEvents: 'none' } : undefined}>
              alt {fmt(frame.altitude_m, 1)} m AGL<br />
              yaw {fmt(frame.yaw_deg, 1)}&deg;
              {/* Age at the moment the data layer received it, from the
                  camera's own V4L2 buffer timestamp. This is latency that has
                  already been spent before any of the pipeline runs, and it is
                  invisible everywhere else -- ArduPilot will not reject a late
                  fix, it will fuse it at the wrong time. */}
              {frame.capture_age_ms != null && (
                <>
                  <br />
                  <span style={{ color: frame.capture_age_ms > 100 ? 'var(--bad)' : 'inherit' }}>
                    capture age {fmt(frame.capture_age_ms, 0)} ms
                  </span>
                </>
              )}
            </div>
          )}
        </div>
  );

  if (bare) return inner;

  return (
    <div className="panel">
      <span className="marks" aria-hidden="true" />
      <header>
        <h2>Camera</h2>
        <span className="spacer" />
        <span className="meta">
          {frame
            // w/h describe the JPEG, which is the rescaled frame the matcher
            // sees. The sensor size is shown beside it when they differ, so a
            // 512 px image is never captioned with the camera's 1280.
            ? `${frame.w}x${frame.h}${frame.source_w && frame.source_w !== frame.w
                ? ` ← ${frame.source_w}x${frame.source_h}` : ''} · seq ${frame.seq}`
            : 'no frame'}
        </span>
      </header>
      <div className="body" style={{ padding: 0 }}>{inner}</div>
    </div>
  );
}
