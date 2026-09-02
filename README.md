# GeoAnchor runtime

Three-layer GNSS-denied visual localization for a UAV, plus its dashboard.
Runs on an NVIDIA Jetson AGX Xavier with **no CUDA**, and unchanged on a
Raspberry Pi 5, an Orin Nano or a laptop.

```
data layer  ──▶  processing layer  ──▶  output layer
map + feed + GPS   predicted GPS        error, loss, export, flight controller
```

Three independent processes on a ZeroMQ bus. Kill one and the other two keep
running. Every step, error and device error emits a code (`DL-07`, `PLE-09`,
`OLDE-02`) that appears live on the dashboard and in the session JSON.

## Quick start

```bash
bash bootstrap.sh                             # once per board
python -m geoanchor.data_layer --build-map    # once per new map
bash run.sh                                   # everything
```

Then open `http://<board>:8000`.

Nothing reaches the flight controller until `output_layer.fc.enabled` is true
in `configs/system.yaml`. Until then the system logs and scores fixes and
writes nothing to the vehicle.

## What each layer does

**Data layer** preprocesses two inputs independently. The map is turned into a
tiled feature store **once per map** and cached by content hash, which is what
brings the matcher inside ArduPilot's 250 ms delay budget. The feed is
undistorted and rescaled to the reference GSD using barometric altitude
(`GSD = altitude / fx_px`), so the matcher never sees the raw scale gap. Actual
GPS, altitude and attitude arrive on one MAVLink link.

**Processing layer** rotates the frame to north using attitude, selects
candidate tiles from the last fix, matches, fits a homography with a
plausibility test that rejects degenerate solves, converts to latitude and
longitude, and attaches a covariance. Rejection is an inlier-count threshold;
covariance is a pluggable estimator.

**Output layer** pairs each fix with the actual GPS taken at the same instant,
computes the position error and a calibration loss, writes the export, and
decides what reaches the vehicle.

## Loop modes

| mode | what is sent |
|---|---|
| `off` | nothing |
| `open` | the **actual** GPS, over the ExternalNav path |
| `closed` | the **predicted** GPS |

`open` is the useful middle step: it exercises the message, the covariance, the
origin and the EKF's acceptance logic with a position that is already known
good, so every failure it finds is a plumbing failure.

Closed loop is gated per fix on the inlier floor, the 100 m altitude cap, and
the absence of live device errors in any layer.

## The export

`runs/<timestamp>_<tag>/session.json` — a header, a summary, and one row per
fix:

```json
{
  "time_step": 3,
  "t_unix": 1788335584.5,
  "actual_gps": { "lat": -33.8676871, "lon": 151.20804995, "rel_alt_m": 75.0 },
  "predicted_gps": { "lat": -33.86768715, "lon": 151.20805024 },
  "error_m": 0.027, "loss": 5.996766,
  "accepted": true, "sigma_m": 8.0, "inliers": 451,
  "latency_ms": 712.74, "method": "xfeat_mnn",
  "loop_mode": "open", "sent_to_fc": false,
  "codes": ["OL-08", "OL-09", "OL-10"]
}
```

`records.jsonl` is written line by line as the run proceeds, so a run killed at
minute forty still has thirty-nine minutes of data.

## Dashboard

Next.js, built as a static bundle, served by the API on the board. Two modes
from one bundle: **live** over a WebSocket to the board, and **replay** of an
exported `session.json` with no board at all. The replay mode is what a Vercel
deployment runs, because a public host cannot reach a Jetson on your network.

```bash
cd dashboard && npm install && npm run build
```

## Real data

```bash
GEOANCHOR_CONFIG=configs/env80.yaml bash run.sh
```

Plays AnyVisLoc env80 frames through all three layers against the scene's own
satellite basemap — real UAV imagery, real reference, published ground truth.
The shipped `configs/system.yaml` cuts its frames out of the reference map and
is a plumbing check only; the dashboard says so on a banner.

## Verification

```bash
bash verify.sh          # 46 checks against the specification, ~3 min
bash sitl.sh            # open-loop validation against ArduPilot SITL
```

Real processes, real bus, a layer killed to prove the other two survive, the
export checked, and real MAVLink round-tripped through a transcription of
ArduPilot's own handler.

## The sweep

```bash
bash sweep.sh
```

Every env80 frame through the same modules, bus removed, printing a table
comparable with the harness. The gate is applied afterwards rather than during
the run, so one pass answers what gate the reference needs. Idempotent.

## Bring-up

`docs/bringup.md`, in order. The three tools:

```bash
python scripts/measure_overhead.py --device 0 --n 200   # OVERHEAD_MS. First.
python scripts/calibrate_camera.py --device 0           # fx_px
python scripts/check_extnav.py --endpoint udpin:0.0.0.0:14550
```

## Methods

All CPU, no CUDA: `orb`, `sift`, `akaze`, `xfeat_mnn`, `xfeat_lg`.
Selectable at runtime from the dashboard; changing it rebuilds the reference
store for that descriptor, and returning to a method already built is a cache
hit.

## Licences

XFeat and LighterGlue are Apache 2.0. The rest of this runtime is the project's
own. No SuperPoint or SuperGlue weights are used here.

---

`CLAUDE.md` carries the full context: the architecture rationale, the measured
numbers, the hard rules, the traps already paid for, and what to do next. Read
it before changing anything.
