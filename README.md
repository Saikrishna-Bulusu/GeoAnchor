# GeoAnchor runtime

Absolute visual localization for a UAV with no GNSS. A down-facing camera frame
is matched against a georeferenced map; the result is a position fix **with a
covariance**, fed to ArduPilot's EKF3 over MAVLink or to PX4's EKF2 over
uXRCE-DDS.

**It has been flown with GNSS taken away, and it holds.** Fixed wing in Gazebo,
`EKF2_GPS_CTRL` set to 0 for three minutes: with the fix the estimator stays
bounded around 8 m; with the fix removed and everything else identical it
diverges at 120 m/min and is 396 m out when the window closes. That comparison
is [the result](#does-it-actually-replace-gnss), and the control is the half
that makes it one.

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

- [Does it actually replace GNSS?](#does-it-actually-replace-gnss)
- [Start here: will it run on your board?](#start-here-will-it-run-on-your-board)
- [Install and run](#install-and-run)
- [Matchers — which one to pick](#matchers--which-one-to-pick)
- [Configurations](#configurations)
- [The dashboard](#the-dashboard)
- [Keeping every device in sync](#keeping-every-device-in-sync)
- [SITL — flight stack on your laptop](#sitl--flight-stack-on-your-laptop)
- [PX4 — a different path in, for a reason](#px4--a-different-path-in-for-a-reason)
- [The Gazebo fixed-wing simulation](#the-gazebo-fixed-wing-simulation)
- [HITL — simulator on the laptop, matcher on the board](#hitl--simulator-on-the-laptop-matcher-on-the-board)
- [How stale may the reference map be?](#how-stale-may-the-reference-map-be)
- [The covariance estimator, and what it does not do](#the-covariance-estimator-and-what-it-does-not-do)
- [Verification](#verification)
- [Reading a result honestly](#reading-a-result-honestly)
- [Where things are](#where-things-are)

---

## Does it actually replace GNSS?

Measured 18 Sept 2026. Full writeup:
[`results/sim_gnss_denied_2026-09-18.md`](results/sim_gnss_denied_2026-09-18.md).

Everything this project measured before it showed fixes being **fused** —
`fused: True`, `cs_aux_gpos: True`, the covariance arriving intact. But it
showed that alongside a healthy GPS, and *an estimator with good GNSS looks
excellent whether or not the vision fix contributes anything at all.* So:

```bash
.venv/bin/python scripts/sim_gnss_denied.py              # vision aiding on
.venv/bin/python scripts/sim_gnss_denied.py --no-vision  # the control
```

| | baseline, GNSS on | GNSS denied, 180 s | drift growth |
|---|---|---|---|
| **vision aiding ON** | median 1.39 m | median **7.78 m**, p90 25.25 | +1.40 m/min |
| **vision aiding OFF** | median 0.70 m | median **204.14 m**, p90 367.68 | **+119.91 m/min** |

**The qualitative difference is the result, not the 26x.** With the fix the
error is bounded. Without it: 1.5 m at 5 s, 102 m at 60 s, 396 m at 180 s, and
still climbing at the cutoff. Dead reckoning has no mechanism to come back; an
absolute position source does. The control's numbers are not a criticism of
PX4's estimator — a fixed wing with a good IMU and no absolute reference is
*supposed* to do that.

Three things that are easy to get wrong here, and all three change the answer:

- **Truth must come from the simulator, never from MAVLink.** With GNSS denied,
  `GLOBAL_POSITION_INT` **is** the estimate — and the estimate is being driven
  by the very fixes under test. Scoring against it asks the fix how well it
  agrees with itself and returns a beautiful number that means nothing. The
  script reads Gazebo's own pose and compares in local metres.
- **`--no-vision` is not optional.** "The estimate held for three minutes" is
  not evidence until the same aircraft, same flight, same denial has been shown
  to lose it.
- **Run it twice.** The first run reported drift *shrinking* at −1.49 m/min,
  which reads as active convergence. The repeat gave +1.40. Medians and p90s
  agreed closely; the sign of the trend did not. Three minutes is too short to
  carry a trend, so the claim is *bounded*, not *improving*.

**With GNSS available the fix makes the estimate slightly worse** — 1.39 and
2.52 m against the control's 0.70 m. The simulated GPS is better than this
pipeline, so fusing a noisier absolute source into a healthy solution costs a
little accuracy. That is correct filter behaviour, and it is the argument for
treating the fix as a fallback that earns its place when GNSS degrades.

### The excursions are the interesting part

Each vision run contains exactly one excursion window and is otherwise tight:

| run | window | peak | median / p90 / max with it removed |
|---|---|---|---|
| 1 | 4.9 s | 214.97 m | 8.19 / 24.89 / **40.86** m |
| 2 | 13.8 s | 542.33 m | 7.20 / 22.05 / **27.17** m |

The control has none — a smooth monotonic ramp. **And these cannot be gaps in
aiding:** if the fix simply stopped arriving the estimator would dead-reckon at
the control's own 120 m/min, which over 13.8 s is 28 m, not 542. Reaching
542 m requires the estimate to be *pulled* — a wrong fix passing the inlier
gate and being fused with an 8 m sigma that says it is trustworthy.

That was the hypothesis, and **measuring it refuted it**
([`results/excursions_are_starvation_2026-09-18.md`](results/excursions_are_starvation_2026-09-18.md)).
Across 300 s of denial with the fix stream tapped and scored against Gazebo
truth: **953 accepted fixes, median 3.4 m, worst 24.0 m.** No bad fix ever
reached the estimator. What collapses is the number of fixes the pipeline
*emits* — 15 in 60 s where a healthy window emits 172 in 30 — while the
acceptance ratio holds at 87–100%. Ten gaps exceeded AGP's 5 s timeout, and the
largest drove the error at **+117.3 m/min against the control's measured
+119.91 m/min**: the estimator is coasting, not being pulled.

**That strengthens the rejection finding and moves the binding constraint.**
The gate and the plausibility check ahead of it kept every bad fix out across
five minutes with no GNSS. Under denial the question is not how good a fix is
but **how long since the last one**, measured against AGP's 5 s timeout and
ArduPilot's 7 s `posTimeout`. A pipeline emitting a perfect fix every 20 s is
useless here; one emitting an 8 m fix every second is fine.

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
git clone https://github.com/Saikrishna-Bulusu/GeoAnchor.git
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
| `scripts/sim_fixedwing.sh` | Gazebo camera | **the whole system, in flight** |

```bash
GEOANCHOR_CONFIG=configs/env80.yaml bash run.sh
```

### Why the default demo reports sub-millimetre error, and why that is worthless

`demo/flight.mp4` is built by cropping frames **out of the reference tile
itself**. The pipeline is matching a picture against a copy of that picture, so
of course it lands at 0.007 m. It proves the wiring is connected and nothing
else. Every session built on it is stamped `synthetic_from_reference` and the
dashboard shows a banner saying so — **only for this feed.** The banner used to
key off `data_layer.feed.path`, which stays `demo/flight.mp4` whatever feed is
actually running, so it also appeared over Gazebo flights whose ground and
reference are deliberately different captures.

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

### Loading a reference map, the way an operator would

Upload a GeoTIFF from the control bar. The endpoint reports the georeferencing
it found, **renders a preview so you can look at the tile before committing to
it**, and refuses to apply one with no projected CRS — a plain image with a
`.tif` extension loads without complaint and matches against nothing.

Applying it switches the live reference and rebuilds the feature store, which
is one pass of the detector over the whole map. `/api/map/preview` shows the
file you uploaded; `/api/map.png` shows the store the pipeline is currently
matching against.

### What the dashboard reports, and what it used to

Every control reads the **running** configuration: live value first, then the
config snapshot, and only then a literal. That ordering is not cosmetic — six
display faults were found and fixed in one pass, all the same shape, a control
reading a field that only exists in some configurations and falling back to a
hardcoded default:

- **Firmware said ArduPilot through an entire PX4 flight.** It read
  `fc.describe()`, which only exists when the FC *writer* is enabled — and every
  rig here starts with `fc.enabled: false` ("open loop before closed, always").
  The configured value was in the snapshot all along, correct and unread.
- **Loop mode showed `open` while the layer ran `closed`**, because the display
  preferred the boot-time snapshot over the value the layer updates at runtime.
  That is the control that decides what reaches the vehicle.
- **The synthetic-feed banner fired on every non-file run**, because it tested
  `feed.path` — which stays `demo/flight.mp4` no matter what feed is running.
  It was telling the reader to discard numbers that were real.
- **Uploading a map did not apply it.** The file landed on disk and nothing
  switched the reference or rebuilt the store; the only sign was that the
  numbers did not change.

A control that falls back to a hardcoded default is asserting a fact it does not
have. Fixed at the source where possible — the output layer now reports its
configured identity even when the writer is off.

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

## PX4 — a different path in, for a reason

Full writeup:
[`docs/px4_ekf2_extnav_2026-09-17.md`](docs/px4_ekf2_extnav_2026-09-17.md).
End-to-end proof: [`results/px4_agp_e2e_2026-09-17.md`](results/px4_agp_e2e_2026-09-17.md).

```bash
bash scripts/px4_sitl.sh                      # PX4 SITL, params, injection, check
.venv/bin/python scripts/check_extnav.py --firmware px4
```

**PX4's external-vision path has a hard rate floor and ArduPilot's does not.**
`EV_MAX_INTERVAL` is 200000 µs, so a fix arriving slower than 5 Hz never starts
fusion. Measured across four passes: **12 of 12 fused at ≥5 Hz, 4 of 4 failed
at 4 Hz.** This pipeline runs at 0.3–5 Hz depending on matcher and board, so on
a Pi it sits below the floor and the EV path is closed to it.

**Aux Global Position has no rate check at all.** Its starting condition is a
finite lat/lon plus yaw alignment and its only timing rule is a 5 s timeout.
Measured fusing at 2 Hz with GNSS denied. The cost is that AGP is reachable
**only over uXRCE-DDS** — there is no MAVLink message for it:

```
pipeline ──ZeroMQ──▶ scripts/agp_bridge.py ──ROS 2──▶ /fmu/in/aux_global_position
```

Parameters go in from `configs/px4_agp.params`. `EKF2_AGP_CTRL` **defaults to 0
— AGP is off until it is set**, and `EKF2_AGP_NOISE` defaults to 0.9, which
silently floors every sigma tighter than 90 cm and defeats the whole point of a
covariance estimator.

Confirm it fused, rather than that it was delivered:

```
listener estimator_aid_src_aux_global_position
```

`observation_variance` should be the square of the `eph` supplied — 8.0 m in,
`[64.0, 64.0]` out — with `estimator_status_flags` showing `cs_aux_gpos: True`.

**The two firmwares read the same MAVLink differently, and both are silent
about it.** `fcout.py` branches on `output_layer.fc.firmware` and refuses any
value but `ardupilot` or `px4` (`OLDE-01`):

| | ArduPilot | PX4 |
|---|---|---|
| pose frame | `MAV_FRAME_LOCAL_FRD` (20) | `MAV_FRAME_LOCAL_NED` (1) |
| covariance | `σ²/2` per axis — they are summed into `posErr` | `σ²` per axis, read as-is |

Halving for one and not the other is the difference between a correct
covariance and one wrong by √2, in a field nothing validates.

---

## The Gazebo fixed-wing simulation

**[`sim/README.md`](sim/README.md)** is the full walkthrough — requirements,
how to build each piece, every failure mode, and a diagnostic that separates
renderer faults from date-gap faults from flight-path faults.

```bash
bash scripts/sim_fixedwing.sh              # everything
bash scripts/sim_fixedwing.sh --headless   # no GUI
bash scripts/sim_fixedwing.sh --open-loop  # no write to the flight controller
bash scripts/sim_fixedwing.sh --kill       # stop it
```

One command puts a fixed wing with a nadir camera over real satellite imagery,
matches every frame against a **different, older** capture of the same ground,
and feeds the fixes to PX4's EKF2. Every other rig here tests one seam; this is
the only one where a camera on a moving aircraft produces frames that become
fixes that reach an estimator with nothing stubbed in between.

**The ground is not the reference map, and the runner exits if it is.**
Texturing the simulated ground with the image the pipeline matches against
makes it match a picture to itself and report centimetres that mean nothing.
Here the world is a 2026-08-05 capture and the reference is 2024-06-06 — 2.2
years apart — so the run is also a genuine cross-date test.

Results: [`results/sim_fixedwing_2026-09-17.md`](results/sim_fixedwing_2026-09-17.md).
Median 3.35 m, p90 6.32, max 10.30, 99% within 10 m, latency p95 159.8 ms.

**Do not compare that median with the env80 table above.** A rendered view of a
flat textured plane has no relief displacement, no cloud, no seasonal change
and no exposure difference, which is why this run contains no catastrophic
fixes and real satellite runs do.

### The flight path is worth more than any other knob

Same rig, same matcher, same tiles — only where and how the aircraft flew:
median **7.02 m** on a sloppy path against **3.35 m** on a correct one. Two
mechanisms, both measured:

- **Orbit radius is a camera parameter.** A coordinated turn banks at
  `tan(φ) = v²/(g·r)` and the camera is rigidly mounted, so the radius sets its
  tilt off nadir: at 20 m/s, 80 m → 27°, 120 m → 19°, 500 m → 4.7°. An oblique
  view does not relate to a north-up orthorectified map by the near-affine
  homography the solve expects — 387 matches collapsed to **5** inliers.
- **Land cover varies inside one tile.** Against the 2.2-year-older reference:
  **30 inliers over the commercial centre, 96 over the industrial fringe 800 m
  away**, with the control (the ground's own imagery) at ~270 in both. An
  acceptance rate from this rig is a statement about the flight path as much as
  about the method.

---

## HITL — simulator on the laptop, matcher on the board

This is the **ArduPilot** split-machine rig, and it is a different thing from
[the Gazebo fixed-wing simulation](#the-gazebo-fixed-wing-simulation) above:
that one runs everything on one machine against PX4 and exists to test the
whole chain, this one puts the matcher on the board it will actually fly on.

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

## How stale may the reference map be?

**It is a question about the flight area, not about the pipeline.** Measured
17 Sept 2026 over four Australian areas with Esri World Imagery Wayback, held
at one zoom so only the date varies. Full writeup:
[`results/crossdate_cities_2026-09-17.md`](results/crossdate_cities_2026-09-17.md).

| area | result |
|---|---|
| **Griffith, NSW — low-rise town centre** | **works at every measured gap, out to 8.8 years** |
| Brisbane CBD | not monotonic — 4 of 8 gaps work |
| Perth CBD | not monotonic — 3 of 11 gaps work |
| Melbourne CBD | not monotonic — 1 of 12 gaps work |

Every control passed, so none of the failures is a harness fault. The mechanism
is **building height**: what changes between two satellite passes over a CBD is
not the ground but the apparent geometry of tall structures — facade parallax
with view angle, shadow with sun angle — and both scale with height. A one- and
two-storey town centre has very little of either.

Four things this study got wrong first, all worth inheriting:

- **The area labelled `rural_griffith` is not rural.** It is the centre of the
  town of Griffith — 2.17 km of suburban streets, shop awnings and industrial
  sheds. Caught by texturing a Gazebo world from that tile and *looking at it*.
  The measurements stand; the "farmland" explanation built on the name does
  not, and **nothing in this project has measured open country.**
- **The x-axis is publication date, not acquisition date.** Wayback release
  dates are when Esri published; a release can republish older imagery. Three of
  four areas are non-monotonic because of it. `scripts/wayback_source_meta.py`
  reads the real `SRC_DATE`/`SRC_RES`/`SRC_ACC`.
- **Fixed zoom is not fixed resolution.** Web Mercator scale goes as 1/cos(lat),
  and the native source differs per area.
- **Stated georeferencing accuracy is 2–10 m** — the same order as the errors
  being measured. Solve rate and inlier count are the trustworthy columns; the
  metre values are an upper bound.

An earlier "three years" figure is a **Sydney CBD** number and must not be
carried anywhere else.

**Licence:** Esri World Imagery is not CC BY. Those pixels are
measurement-only working data. For a publishable figure use the NSW Historical
Imagery Viewer (CC BY), which also carries a real *capture* date —
[`docs/HISTORICAL_IMAGERY.md`](docs/HISTORICAL_IMAGERY.md).

---

## The covariance estimator, and what it does not do

`GET /api/covariance` lists every trained model with what it was trained on and
whether it matches the running matcher, and the dashboard warns when they
disagree. That warning is the project's central finding made visible: **a system
that swaps matchers — which any embedded deployment must, to fit the compute
budget — silently inherits a covariance model that no longer works.**

**The honest state of the learned estimator is that it does not order error.**
Retrained on the complete runtime sweep, leave-one-scene-out:

| method | n | Spearman | p |
|---|---|---|---|
| `edgepoint2_s64` | 156 | **−0.032** | 0.7 |
| `xfeat_mnn` | 145 | **0.003** | 0.97 |

Both indistinguishable from zero. The published 0.551 was measured on the
**pre-gate** harness population, and much of it is the model learning to
separate catastrophes from good fixes — *the rejection problem*, which an
inlier threshold already solves for free. Scored on what it is actually for —
ranking the error of fixes that already passed — there is nothing there.

Worse, the failure is structured rather than noisy: **Scene_16 is inverted on
both methods**, called the most confident scene while being the worst. That is
a scene-identity ordering that is wrong on the held-out scene, which is exactly
what the group-by-scene rule exists to expose.

One property survives: the highest-sigma bin's p90 is 136.85 m against 17.22 m
for the lowest. The model still finds the tail after losing the middle — the
rejection problem again.

**So describe it as a magnitude model, never as a confidence ranking.** EKF3
needs *a* number, and an estimate that is right on average beats a constant 8 m
and beats NGPS Eq. 7's saturation at 10.00 m. Full working:
[`results/covariance_postgate_2026-09-17.md`](results/covariance_postgate_2026-09-17.md).

Caveat recorded there and worth repeating: "4 scenes" overstates it. Scene_22
contributes 4 rows and never appears as a fold, Scene_21 is excluded, so this is
three effective scenes — thin for a group-wise claim **in either direction,
including this negative one.**

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
- **Cross-validate grouped by scene, never by row.** With few scenes, `rho`,
  `log_rho` and `altitude_m` separate them perfectly and a tree splits on those
  first. The grouping is what exposed the learned estimator inverting Scene_16.
- **How stale the reference map may be is a question about the flight area.**
  A low-rise town survives 8.8 years and a high-rise CBD fails at months. Do not
  carry one area's number to another, and note that **no measurement here covers
  open country.**
- **A simulator result is not an accuracy result.** The Gazebo rig produces no
  catastrophic fixes because a flat textured plane has no relief displacement,
  no cloud, no seasonal change and no exposure difference. Real satellite runs
  do. Never put the two medians in one table.
- **A control is half of a result.** "The estimator held with GNSS denied" meant
  nothing until the same flight with the fix removed was shown to diverge.
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
- **PX4 locks each UDP MAVLink instance to its first peer, permanently.** Once
  the pipeline, `px4_set_params.py` and the launcher have claimed theirs, a tool
  attached later gets `no heartbeat` from a perfectly healthy autopilot —
  `px4-listener` on the uORB console still works, which is how you tell. Every
  probe burns an instance against a cap of six. Start your own and be its first
  client: `px4-mavlink start -x -u 14590 -o 14591 -r 200000 -m onboard`.
- **`udpin` on both ends is a deadlock, silently.** pymavlink cannot transmit on
  a listening socket until a packet reveals a peer, and PX4 will not send until
  something contacts it. No traffic, no error — every frame just logs "no
  altitude". Connect *out* to the autopilot's listening port.
- **`gz sim` runs as `ruby`.** `pkill -x gz` matches nothing, so every previous
  run's Gazebo survives. Three servers stepping one world took the load average
  to 81 and PX4 to `Accel #0 fail: TIMEOUT!`, which surfaces as a *mission
  upload being rejected*. Match on the command line.
- **`PYTHONPATH` to reach Gazebo's APT bindings breaks the dashboard.** It goes
  *ahead* of the venv, so the system `typing_extensions` shadows the venv's,
  pydantic dies on `cannot import name 'Sentinel'`, and fastapi never imports.
  Append to `sys.path` inside the importer instead.
- **A live feed that returns "nothing new yet" as `None` means end-of-file** to a
  data layer written for finite feeds. The run stopped after one published frame
  with a cheerful "end of feed".

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
sim/                          Gazebo world, aircraft model, and its README
stores/                       built feature stores, keyed by content hash
results/                      every measurement, with its caveats
runs/<stamp>_<tag>/           one directory per session
```

Simulation and flight-stack scripts:

```
scripts/sim_fixedwing.sh      the whole system on a fixed wing in Gazebo
scripts/sim_fly.py            mission upload, arm, and hold over the map
scripts/sim_gnss_denied.py    take GNSS away and measure what happens
scripts/make_gz_world.py      GeoTIFF -> Gazebo world + texture
scripts/px4_sitl.sh           PX4 SITL rig
scripts/agp_bridge.py         ZeroMQ fix -> PX4 Aux Global Position over DDS
scripts/crossdate_cities.sh   how stale the reference map may be
scripts/train_covariance.py   the learned estimator, from the runtime's sweep
```

| doc | what it covers |
|---|---|
| [`docs/BOARD_GUIDE.md`](docs/BOARD_GUIDE.md) | per-board setup and what is measured on each |
| [`docs/SITL_AND_HITL.md`](docs/SITL_AND_HITL.md) | ArduPilot SITL, then Gazebo HITL over ethernet |
| [`docs/LOG_SYNC.md`](docs/LOG_SYNC.md) | the shared logs repo and the fleet view |
| [`docs/FC_AND_HITL_PLAN.md`](docs/FC_AND_HITL_PLAN.md) | the SpeedyBee F405 V3, and why it cannot close the loop |
| [`docs/bringup.md`](docs/bringup.md) | camera calibration and `OVERHEAD_MS`, in order |
| [`docs/PI5_HANDOFF.md`](docs/PI5_HANDOFF.md) | moving the project to a new board |
| [`sim/README.md`](sim/README.md) | the Gazebo fixed-wing rig, end to end |
| [`docs/px4_ekf2_extnav_2026-09-17.md`](docs/px4_ekf2_extnav_2026-09-17.md) | PX4's rate floor, and why the path is AGP |
| [`docs/HISTORICAL_IMAGERY.md`](docs/HISTORICAL_IMAGERY.md) | licence-clean imagery with real capture dates |
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
