# GeoAnchor dashboard

Next.js, static export, one bundle for two very different hosts.

## On the board (live)

The API serves the built bundle at `/`, so there is nothing to configure:

```bash
npm install && npm run build      # writes out/
```

Then `bash ../run.sh` and open `http://<board>:8000`. The dashboard talks to
the same origin it was served from, so it works over any LAN address without a
rebuild.

To develop against a board while editing on a laptop:

```bash
NEXT_PUBLIC_GEOANCHOR_API=http://<board>:8000 npm run dev
```

## On Vercel (replay)

A public host cannot reach a Jetson behind your router, so the Vercel build
starts in replay mode and reads an exported `session.json` that you drag onto
the page. Live telemetry stays on the LAN; only the exported file travels.

```
Framework preset   Next.js
Root directory     dashboard
Environment        NEXT_PUBLIC_GEOANCHOR_MODE=replay
```

## Panels

- **Layer cards** — one per layer, each with its own step-code stream and its
  own heartbeat. Separate on purpose: when one layer dies its card stops
  reporting while the other two carry on, and that should be obvious.
- **Session** — median, p90, p99, error rate, latency. No mean and no RMSE
  anywhere; one degenerate solve destroys both.
- **Position** — actual and predicted tracks drawn on the reference map's own
  preview, with a confidence circle from the emitted covariance. No tile
  server: the aircraft is offline by definition.
- **Camera** — the frame the matcher actually saw, after rescaling.
- **Charts** — error against stated sigma, calibration loss, latency against
  the 250 ms budget, and the sorted error distribution.
- **Records** — the export, visible. Every column is a field in the JSON.

## Notes

- Plain CSS, no Tailwind and no TypeScript, so the build needs nothing beyond
  `next`, `react` and `recharts`. That matters when it is built on an ARM board.
- Dark by default with a light theme under `prefers-color-scheme`.
