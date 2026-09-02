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
export default function CameraView({ frame, fps = 3 }) {
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

  return (
    <div className="panel">
      <header>
        <h2>Camera</h2>
        <span className="note" style={{ marginLeft: 'auto', fontFamily: 'var(--mono)' }}>
          {frame ? `${frame.w}x${frame.h} · seq ${frame.seq}` : 'no frame'}
        </span>
      </header>
      <div className="body flush">
        <div className="camwrap">
          {src ? <img src={src} alt="live frame" /> : <span className="note">waiting for the data layer</span>}
          {frame && (
            <div className="camoverlay">
              alt {fmt(frame.altitude_m, 1)} m AGL<br />
              yaw {fmt(frame.yaw_deg, 1)}&deg;
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
