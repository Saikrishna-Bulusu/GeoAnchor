# GeoAnchor runtime

Absolute visual localization for a UAV with no GNSS. A down-facing camera frame
is matched against a georeferenced map; the result is a position fix **with a
covariance**, fed to ArduPilot's EKF3 over MAVLink.

```
data layer  ──▶  processing layer  ──▶  output layer
map + feed + GPS   predicted position     error, loss, export, flight controller
```

Three independent OS processes on a ZeroMQ bus. Kill one and the other two keep
running. Every step, error and device error emits a code (`DL-07`, `PLE-09`,
`OLDE-02`) that appears live on the dashboard and in the session JSON.

Runs on a laptop, a Raspberry Pi 4B or 5, an NVIDIA Jetson AGX Xavier or an
Orin Nano — **the same code, CPU-only, no CUDA anywhere.** That constraint is
the design, not a limitation: it is what makes joules-per-fix a comparison of
the *boards* rather than of four different implementations.

---

## Contents

- [Start here: will it run on your board?](#start-here-will-it-run-on-your-board)
- [Install and run](#install-and-run)
- [Matchers — which one to pick](#matchers--which-one-to-pick)
- [Configurations](#configurations)
- [The dashboard](#the-dashboard)
- [Keeping every device in sync](#keeping-every-device-in-sync)
- [SITL — flight stack on your laptop](#sitl--flight-stack-on-your-laptop)
- [HITL — simulator on the laptop, matcher on the board](#hitl--simulator-on-the-laptop-matcher-on-the-board)
- [Verification](#verification)
- [Reading a result honestly](#reading-a-result-honestly)
- [Where things are](#where-things-are)

---

## Start here: will it run on your board?

**Do not reason about this from specifications. Measure it.** Copy the repo to
the board and run:

```bash
bash scripts/check_board.sh
```

It answers three questions in order and stops at the first hard no:

1. **Does the runtime fit in RAM?** The only hard wall. Measured by importing
   the real chain and reading peak RSS, not estimated.
2. **Does a matcher run here at all?** Needs torch and the weights.
3. **Does a matcher that actually works fit the 250 ms budget** once capture
   overhead is paid?

Question 3 has been got wrong in both directions on this project. **A board
that fails it is still useful** — the joules-per-fix curve across compute
classes is the headline contribution, and "this class cannot close the loop" is
a measurement, not a failure. The script says so instead of just failing.

It also refuses to call a board viable on ORB, SIFT or AKAZE. Those fit every
budget and return almost nothing usable against real satellite reference
(details in the matcher table below), so it marks them `fits*`.

### What is known so far

| board | RAM | runs? | closes the 250 ms loop? |
|---|---|---|---|
| Laptop (x86) | 24.9 GB | yes, measured | n/a — not a flight target |
| **Pi 4B** 8 GB | 8.2 GB | **yes, measured** — 0.012 m fix, 335 inliers | no — 4231 ms, and it hit 84.7 °C |
| **Pi 5** 8 GB | 8 GB | **yes, measured** | no — 280 ms matcher + 132 ms capture |
| **AGX Xavier** | 32 GB | **yes, measured** | no — 211 ms + overhead. Bench board, 30 W |
| Orin Nano | 8 GB | not yet run here | unknown — run `check_board.sh` |
| Pi 3B 1 GB | 1 GB | not yet run here | very unlikely; RAM is the question |

**Nothing anyone owns closes the loop in 250 ms today, Pi 5 included.** That is
a real finding about this class of hardware, not a setup problem. See
[`docs/BOARD_GUIDE.md`](docs/BOARD_GUIDE.md) for per-board setup.

---

## Install and run

```bash
git clone https://github.com/Saikrishna-Bulusu/geoanchor-rt.git
cd geoanchor-rt
bash bootstrap.sh                             # once per board
python -m geoanchor.data_layer --build-map    # once per new map
bash run.sh                                   # everything
```

Then open `http://<board-address>:8000`.

`bootstrap.sh` creates the venv, installs pinned dependencies, clones XFeat and
EdgePoint2, fetches the weights, and asserts that torch has **no** CUDA. That
last check is not paranoia — see [Traps](#traps-worth-knowing-about).

**Nothing reaches the flight controller** until `output_layer.fc.enabled` is
true in `configs/system.yaml`. Until then the system logs and scores fixes and
writes nothing to any vehicle.

### Running one layer at a time

The fastest way to debug one is to run it alone:

```bash
python -m geoanchor.data_layer
python -m geoanchor.processing_layer --method edgepoint2_s64
python -m geoanchor.output_layer --loop-mode off
```

Layers started by hand must share `GEOANCHOR_RUN_DIR` or their JSONL files
scatter and the session cannot be reassembled. `run.sh` sets it for you.

Override any config key for a single run:

```bash
GEOANCHOR_SET="processing_layer.method=orb;data_layer.feed.fps=2" bash run.sh
```

---

## Matchers — which one to pick

Eight, all CPU. Selectable live from the dashboard; changing it rebuilds the
reference store for that descriptor, and returning to one already built is a
cache hit.

| identifier | name | what it is |
|---|---|---|
| `orb` | ORB | classical |
| `sift` | SIFT | classical |
| `akaze` | AKAZE | classical |
| `xfeat_mnn` | **XFeat + Nearest-Neighbour** | learned detector, plain descriptor matching |
| `xfeat_lg` | **XFeat + LighterGlue** | learned detector *and* learned matcher |
| `edgepoint2_t32` | **EdgePoint2 Tiny (32-D)** | smaller network, narrow descriptor |
| `edgepoint2_s32` | **EdgePoint2 Small (32-D)** | small network, narrow descriptor |
| `edgepoint2_s64` | **EdgePoint2 Small (64-D)** | small network, XFeat-width descriptor |

The short identifiers are what the code, the feature-store filenames and every
exported `session.json` use, and they are **deliberately not renamed** — a
rename would invalidate every built store on every board and make every past
session unreadable. `mnn` is mutual nearest neighbour, `lg` is LighterGlue,
`t`/`s` are EdgePoint2's tiny and small networks and the number is descriptor
width. The dashboard shows the long names.

### Speed — measured, clocks pinned, one tile / 2048 reference keypoints

Milliseconds, p95 of 15 reps. Reproduce with
`python scripts/bench_matchers.py --tiles 1 --reps 15`.

| matcher | Pi 5 | AGX Xavier |
|---|---|---|
| ORB | 67.9 | 92.5 |
| AKAZE | 81.3 | 139.2 |
| SIFT | 183.7 | 120.9 |
| EdgePoint2 Tiny (32-D) | 238.9 | 188.3 |
| EdgePoint2 Small (32-D) | 250.3 | 209.6 |
| EdgePoint2 Small (64-D) | 280.4 | 210.9 |
| XFeat + Nearest-Neighbour | 309.7 | 266.4 |
| XFeat + LighterGlue | 3638.0 | 2380.8 |

**Never compare a number here to one taken at a different reference geometry.**
Match cost is linear in reference keypoints: the same board on the same frame
goes from 28.6 ms at one tile to 439 ms at 25. Most apparent "board X loses to
board Y" results on this project turned out to be tile count.

**On ARM, XFeat is slower than SIFT.** XFeat's README claims otherwise; that is
an x86 result with AVX2 and MKL behind it. Both our boards agree independently.

### Accuracy — real UAV frames, real satellite reference, each at its own gate

AnyVisLoc env80, cold start every frame. This is what the numbers mean; the
shipped `configs/system.yaml` demo does **not** measure accuracy (see below).

| scene | matcher | gate | accept | median | worst |
|---|---|---|---|---|---|
| 09 | XFeat + NN | 10 | 35.1% | 2.67 m | 6.65 m |
| 09 | EdgePoint2 S64 | 8 | 32.1% | 2.92 m | 7.50 m |
| 10 | XFeat + NN | 10 | 17.2% | 3.52 m | 6.75 m |
| 10 | EdgePoint2 S64 | 8 | 41.1% | 3.83 m | 7.36 m |

### What that adds up to

- **EdgePoint2 Small (64-D) is the current recommendation.** Faster than XFeat
  on both scenes, accepts 2.4x as many frames on Scene_10, and its worst error
  is 8.4 m where XFeat's is 187 m. For a system whose contribution is
  *covariance*, a matcher without a catastrophic tail is worth more than one
  with a slightly better median.
- **The inlier gate is per matcher and it matters a lot.** EdgePoint2 wants 8,
  XFeat wants 10. Applying XFeat's gate to EdgePoint2 throws away a third of
  its fixes. XFeat's p99 collapses from 167.85 m to 6.75 m across gates 7, 8, 9
  — there is a cliff and you must be on the right side of it.
- **The gate is also reference-specific.** XFeat returns 12–79 inliers against
  real satellite reference and 200+ against a tile the frames were cut from. A
  gate tuned on one silently rejects everything on the other. **Measure it on
  your own reference.**
- **XFeat + LighterGlue is the most accurate on paper and unusable in flight.**
  It costs 9x MORE on the frames it fails than on the ones it solves (7003 ms
  vs 767 ms), because its point pruning is confidence-driven. For a
  latency-budgeted loop that is the worst possible cost profile.
- **ORB, SIFT and AKAZE are not viable against real satellite reference.** Zero
  geometrically plausible fixes in 192 frames for ORB and SIFT. The few they
  return are wrong by 62–127 m — confident fixes on the wrong building. ORB
  produced 1200+ matches per frame while getting every one wrong, which is the
  argument for gating on inliers rather than on match count.

---

## Configurations

One YAML, read by all three layers.

| config | feed | what it is for |
|---|---|---|
| `configs/system.yaml` | `demo/flight.mp4` | **plumbing check only** |
| `configs/env80.yaml` | AnyVisLoc env80 | **real accuracy** |
| `configs/camera.yaml` | live USB camera | bench bring-up |

```bash
GEOANCHOR_CONFIG=configs/env80.yaml bash run.sh
```

### Why the default demo reports sub-millimetre error, and why that is worthless

`demo/flight.mp4` is built by cropping frames **out of the reference tile
itself**. The pipeline is matching a picture against a copy of that picture, so
of course it lands at 0.007 m. It proves the wiring is connected and nothing
else. Every session built on it is stamped `synthetic_from_reference` and the
dashboard shows a banner saying so.

If that number ever stops being sub-millimetre, something is broken. That is
its entire job. **Never quote it as accuracy.** Use `configs/env80.yaml`.

The demo feed also moves at **85.6 m/s** — 21.41 m between frames at 4 fps, a
synthetic sweep across the tile, not a flight profile. Do not tune
`prior_radius_m` or `prior_max_age_s` against it.

---

## The dashboard

Next.js, built as a static bundle, served by the API on the board.

```bash
cd dashboard && npm install && npm run build
```

Two modes from one bundle:

- **Live** — a WebSocket to the board. Map with the track and a confidence
  circle from the emitted covariance, four synchronised metric graphs, per-layer
  step codes, and controls that change the matcher and feed at runtime.
- **Replay** — drop in an exported `session.json` with no board at all. This is
  what a public deployment runs, because a public host cannot reach a Jetson on
  your network.

A replayed session **carries its own map**: `session.json` embeds the map packet
and the basemap as a ~570 KB data URI, so a flight from a Pi renders on its own
reference tile while displayed on a laptop that has never built that store.

The API is a pure observer. It subscribes and fans out. Nothing in the pipeline
depends on it, and `bash run.sh --no-api` proves it.

---

## Keeping every device in sync

Every board pushes its session transcripts to a shared repo and pulls everyone
else's back, so the dashboard on your laptop lists every device's flights.

```bash
bash scripts/sync_logs.sh              # push mine, pull theirs
bash scripts/sync_logs.sh --pull-only  # just refresh the fleet view
bash scripts/sync_logs.sh --dry-run    # show what would be copied
```

**This runs automatically at the end of every `run.sh`.** Set
`GEOANCHOR_SYNC_ON_EXIT=0` to skip it. It is best-effort by design: no network,
no logs repo, or a failed push must never change the exit status of a flight.
`runs/` is the source of truth and the shared repo is a mirror.

Three repos, on purpose:

| repo | holds |
|---|---|
| `geoanchor-rt` | this — the code, configs, docs |
| `GeoAnchor-logs` | session transcripts, one directory per device |
| `geoanchor-research` | the harness, the numbered steps, the analysis output |

Transcripts are ~100 KB per run and every board writes them continuously. In
the code repo they would bloat history forever and every device would commit to
the same branch on every run — a merge conflict per sync. In their own repo
each device owns `<device>/`, so two devices can never touch the same file.

Full detail, including what is *not* synced and why: [`docs/LOG_SYNC.md`](docs/LOG_SYNC.md).

---

## SITL — flight stack on your laptop

**Do this before touching any hardware.** SITL runs a complete ArduPilot build
with nothing trimmed, so every failure it finds is a real one and a mistake
costs nothing.

Full walkthrough, from installing ArduPilot to reading `posTestRatio`:
**[`docs/SITL_AND_HITL.md`](docs/SITL_AND_HITL.md)**. The short version:

```bash
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14550
```

then, in this repo:

```bash
bash sitl.sh --set-params      # write the ExternalNav parameters (SITL only)
bash sitl.sh                   # check them, then run an open-loop session
```

### The three loop modes

| mode | what is sent |
|---|---|
| `off` | nothing |
| `open` | the **actual** GPS, over the ExternalNav path |
| `closed` | the **predicted** GPS |

`open` is the useful middle step. It exercises the message, the covariance, the
origin and the EKF's acceptance logic with a position already known good — so
every failure it finds is a plumbing failure rather than a matcher failure.

**Three bugs were found this way, none visible from the sending side:**

- `frame_id = MAV_FRAME_LOCAL_NED` is **discarded in silence.** `LOCAL_FRD` is
  20, not 1. ArduPilot's `handle_odometry()` returns early with no warning, no
  status flag and no counter. The only symptom is an estimator that never sees
  external navigation — which looks exactly like a wiring fault.
- **A single NaN in the covariance poisons the whole thing.** ArduPilot computes
  `sqrtf(cov[0] + cov[6] + cov[11])` and only checks `isnan(cov[0])`. Writing
  NaN into `cov[11]` to mean "altitude unknown" — the natural reading of the
  MAVLink spec — feeds NaN straight into EKF3.
- **pymavlink writes MAVLink 1 on a send-only link.** It negotiates the wire
  version from *inbound* traffic, so a link that only ever writes never upgrades
  and every message id above 255 is simply absent. ODOMETRY is 331.

Never use `GPS_INPUT` — it has no covariance field, and covariance is the
contribution.

---

## HITL — simulator on the laptop, matcher on the board

```
   Laptop                                   edge board
   ┌──────────────────────┐                ┌─────────────────────┐
   │ Gazebo world+camera  │───ethernet────▶│ data layer          │
   │ ArduPilot SITL, EKF3 │◀───────────────│ processing layer    │
   └──────────────────────┘   ODOMETRY     │ output layer        │
                                           └─────────────────────┘
```

The board runs the real pipeline on its real CPU, so the timings are board
timings; the laptop runs the simulator and the flight stack. This measures the
thing that matters — can this board match fast enough — without an airframe.

Setup, static IPs, and the Gazebo world: [`docs/SITL_AND_HITL.md`](docs/SITL_AND_HITL.md).

**Two things that decide whether HITL is honest:**

- **Never texture the Gazebo ground with the image used as the reference map.**
  The pipeline then matches a picture against itself and every number is
  meaningless — the same trap as the demo video, but this time undeclared.
- **HITL cannot tell you `OVERHEAD_MS`.** Capture, ISP and USB transfer are
  properties of the *camera*, and a simulated camera has none of them. Two
  measurements on this project differ by **3x** on that term alone — 45.7 ms on
  the Xavier, 132.1 ms on the Pi 5, same camera model, and the gap is the
  camera, not the board. Measure it on the real rig and add it to the HITL
  matcher time.

Ethernet itself is not a concern and this was checked: a 640×360 JPEG is 0.35 ms
on gigabit and 5–15 ms round trip with the stack, against a matcher costing
200–3000 ms.

---

## Verification

```bash
bash verify.sh          # 46 checks against the specification, ~3 min
bash sitl.sh            # open-loop validation against ArduPilot SITL
```

Real processes, a real bus, a layer killed to prove the other two survive, the
export checked, and real MAVLink round-tripped through a transcription of
ArduPilot's own handler. Not imports and not mocks — layer independence cannot
be tested any other way. Run it after any change to a layer boundary.

```bash
python scripts/preflight.py     # what this board can and cannot do
python geoanchor/codes.py       # validate the step-code registry
bash sweep.sh                   # every env80 frame, bus removed, comparable table
```

---

## Reading a result honestly

These are the project's hard rules and they are not stylistic.

- **Never quote the mean, and never RMSE.** Runs in this project contain fixes
  wrong by up to 2.06e93 m — a homography can converge on a degenerate solution
  that passes an `inliers > 0` check. One such row destroys a mean.
  `metrics.Running.summary()` refuses to emit one. Report **median, p90, p99**
  and the fraction inside 5/10/20 m.
- **Latency is the binding constraint, not rate.** ArduPilot has no minimum-rate
  check on ExternalNav. But delay compensation is capped at 250 ms and
  **overrunning it is silent**: a late fix is not rejected, it is stamped as
  current and fused at the wrong time. At 5 m/s each 100 ms of uncompensated
  latency injects 0.5 m. Frames are stamped at *capture*, never at fix
  completion.
- **Cross-validate grouped by scene, never by row.**
- **A number without its clock is not a result.** `bench_matchers.py` records
  governor, pinned state, temperature and load, and warns loudly when the clocks
  are not pinned. `performance` is **not** the same as pinned — the test is
  `scaling_min_freq == scaling_max_freq`, and it does not survive a reboot.

### Traps worth knowing about

- **torch on ARM resolves to a CUDA build.** PyTorch 2.11.0 dropped the
  `platform_machine == "x86_64"` guard on its CUDA dependencies, so an unpinned
  install on any ARM Linux board pulls over a gigabyte that can never execute.
  `bootstrap.sh` pins and then asserts `torch.version.cuda is None`.
- **`opencv-python-headless>=4.8` resolves to OpenCV 5**, where
  `cv2.AKAZE_create` no longer exists. Pinned `<5`.
- **A feature store holds descriptors from one extractor.** Matching XFeat
  float32 against ORB uint8 is an assertion failure, not a bad result. The store
  id hashes the method in and `attach_map` refuses a mismatch (`PLE-14`).
- **Import torch before rasterio.** Reading a GeoTIFF pulls in GDAL and exhausts
  the process's static TLS surplus; a torch imported afterwards dies with
  `cannot allocate memory in static TLS block`.

---

## Where things are

```
geoanchor/codes.py            the step-code registry. Append-only.
geoanchor/contracts.py        the messages that cross layer boundaries
geoanchor/bus.py              ZeroMQ. Drop-not-block, drain-to-newest.
geoanchor/methods.py          all eight matchers, and their display names
geoanchor/geo.py              pixel <-> geodetic. pyproj, not rasterio.
geoanchor/device.py           board detection and the power probe
geoanchor/data_layer/         mapprep, store, feed, gpsin
geoanchor/processing_layer/   rectify, solve, covariance
geoanchor/output_layer/       metrics, recorder, fcout
geoanchor/api/                FastAPI observer + WebSocket
dashboard/                    Next.js, static export, live and replay
configs/                      the one config all three layers read
stores/                       built feature stores, keyed by content hash
runs/<stamp>_<tag>/           one directory per session
```

| doc | what it covers |
|---|---|
| [`docs/BOARD_GUIDE.md`](docs/BOARD_GUIDE.md) | per-board setup and what is measured on each |
| [`docs/SITL_AND_HITL.md`](docs/SITL_AND_HITL.md) | ArduPilot SITL, then Gazebo HITL over ethernet |
| [`docs/LOG_SYNC.md`](docs/LOG_SYNC.md) | the shared logs repo and the fleet view |
| [`docs/FC_AND_HITL_PLAN.md`](docs/FC_AND_HITL_PLAN.md) | the SpeedyBee F405 V3, and why it cannot close the loop |
| [`docs/bringup.md`](docs/bringup.md) | camera calibration and `OVERHEAD_MS`, in order |
| [`docs/PI5_HANDOFF.md`](docs/PI5_HANDOFF.md) | moving the project to a new board |
| `CLAUDE.md` | the full context: every measured number and every trap |

Three bring-up tools, in this order:

```bash
python scripts/measure_overhead.py --device 0 --n 200   # OVERHEAD_MS. First.
python scripts/calibrate_camera.py --device 0           # fx_px
python scripts/check_extnav.py --endpoint udpin:0.0.0.0:14550
```

---

## Licences

XFeat and LighterGlue are Apache 2.0. EdgePoint2 is MIT. NSW Spatial Services
imagery is CC BY. The rest of this runtime is the project's own. **No SuperPoint
or SuperGlue weights are used here** — the Magic Leap licence assigns
derivatives to Magic Leap.

---

Saikrishna Bulusu, Visiting Scholar, UTS Intelligent Drone Lab, under
A/Prof Nabin Sharma.
