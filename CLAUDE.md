# GeoAnchor runtime

GNSS-denied absolute visual localization for a UAV at 50-100 m AGL. Match a
down-facing camera frame against a georeferenced map, produce a position fix
with a trustworthy covariance, feed it to ArduPilot EKF3 over MAVLink.

Saikrishna Bulusu, Visiting Scholar, UTS Intelligent Drone Lab, under
A/Prof Nabin Sharma. Sept-Dec 2026.

This folder is the **runtime**: the three-layer system the professor specified,
plus its dashboard. The research harness, the matcher sweeps, the covariance
estimator training and the full project context live in the parent repo at
`~/GeoAnchor/`, whose `CLAUDE.md` is the authority on anything not covered here.

---

## The architecture, and why it is shaped this way

Three layers, three OS processes, one message bus. They talk over ZeroMQ and
share nothing else.

    data layer          map (preprocessed ONCE, cached) + camera feed + actual GPS
      |                 publishes: map, frame, gps
      v
    processing layer    rectify -> match -> homography -> geodetic -> covariance
      |                 publishes: fix  (the Predicted GPS)
      v
    output layer        pair with actual GPS -> error + loss -> export JSON
                        publishes: record; optionally writes to the flight controller

**Independence is enforced, not hoped for.** Each layer binds its own PUB
socket and its own PULL control socket. Kill any one and the others keep
running and say so on the dashboard. `run.sh` reports a dead layer rather than
tearing the system down. A frame that raises inside the processing layer is
`PLE-13` and a skipped frame, never a dead process.

**The API is an observer.** It subscribes and fans out to the browser. Nothing
in the pipeline depends on it.

---

## Step codes

110 of them, in `geoanchor/codes.py`. Three kinds per layer:

    DL-nn    DLE-nn    DLDE-nn      data layer:       step / error / device error
    PL-nn    PLE-nn    PLDE-nn      processing layer
    OL-nn    OLE-nn    OLDE-nn      output layer

The distinction is operational: an `E` code means this fix is bad, a `DE` code
means the board or a peripheral is bad. Closed loop refuses to send while any
`DE` is live anywhere in the system.

**The registry is append-only. Never renumber a code** -- old session files
carry these strings and a renumber silently rewrites history.

`python geoanchor/codes.py` prints the tallies and validates the registry.

---

## Hard rules

Inherited from the parent repo and still binding here.

- **Never quote the mean, and never RMSE.** Satellite runs in this project
  contain fixes wrong by up to 2.06e93 m, because a homography or PnP solve can
  converge on a degenerate solution that passes an `inliers > 0` check. One such
  row destroys a mean. Report median, p90, p99 and the fraction inside
  5/10/20 m. `metrics.Running.summary()` refuses to emit a mean.
- **No CUDA anywhere in the pipeline.** Not a limitation being worked around --
  it is what makes the same code run on the Pi 5, the Xavier, the Orin Nano and
  the 2019 Nano, so joules per fix compares the *boards*. The moment one board
  runs TensorRT and another runs PyTorch, the headline measurement compares
  implementations instead.
- **The reference is never warped.** Stage 02 rectifies the FRAME using vehicle
  attitude and the tile stays north-up. If the tile ever has to be warped per
  frame, the precomputed feature store is void and so is every latency number.
- **Never texture a simulator ground plane with the image used as the reference
  map**, or the pipeline matches a picture against itself. `demo/flight.mp4`
  does exactly this on purpose, for bring-up, and every session built on it is
  stamped `synthetic_from_reference` with a banner on the dashboard.
- **Never GPS_INPUT.** It has no covariance field. ODOMETRY or
  VISION_POSITION_ESTIMATE only.
- **Any descriptor fine-tune uses XFeat, never SuperPoint** -- the Magic Leap
  licence assigns derivatives to Magic Leap.
- **One runnable script, never a code block mixing `#` comments and commands.**

---

## The two findings this runtime is built around

**1. The reference feature store is what makes the budget.** Measured on a
Pi 5, 2 Sept 2026: XFeat sparse cost 1017 ms per fix, four times over budget,
and 856 ms of that was running the detector over the reference tile. The map is
static, so those descriptors are computed once on the ground. The same matcher
then came in at 202 ms and fits. `data_layer/store.py` is that mechanism, and
the professor's "preprocess each map once and save it" is the same instruction
arrived at from the other direction.

**2. Rectification is load-bearing, not a refinement.** Reproduced here on the
NSW Sydney tile, first frame, cold start:

    rectify=False    6 inliers   no fix (rejected PLE-08, degenerate)
    rectify=True   227 inliers   0.02 m

Confirmed again on real AnyVisLoc frames against a real satellite basemap,
which is the hard case rather than the easy one:

    frame       rectify=True        rectify=False
    L09_0001    12 inliers, 1.33 m   6 inliers, no fix
    L09_0002    79 inliers, 1.47 m   8 inliers, 31.66 m
    L09_0003    39 inliers, 2.61 m   6 inliers, no fix

XFeat is not rotation invariant. Without attitude the matcher fails on a task
where the query is literally cut out of the reference, and on real data it
produces either nothing or a 30 m error.

**3. The inlier gate is reference-specific, and that is a finding.** Measured
here on AnyVisLoc Scene_09 against the satellite basemap, XFeat returns **12-79
inliers**. On the Sydney tile, where frames are cut from the reference itself,
the same matcher returns **200+**. A gate tuned on one reference type silently
rejects everything on another -- structurally the same failure as NGPS Eq. 7
across descriptors. `configs/system.yaml` uses 25, `configs/env80.yaml` uses 12,
and neither is transferable. Measure it on your own reference before trusting
either.

**4. Fit per candidate tile, not over the whole map.** Matching against every
reference keypoint on the map is cheap to compute and badly conditioned to fit:
the correct correspondences are a handful among thousands spread over hundreds
of metres, and RANSAC returns a degenerate homography that passes an
`inliers > 0` check. Partitioning only the RANSAC input by tile took the
accept rate on Scene_09 from **10% to 20%** at no extra matching cost.
`processing_layer.tile_ransac`, on by default.

**Rejection and covariance are different problems** (parent repo, 1323 fixes,
leave-one-scene-out): an inlier-count threshold wins rejection at AUC 0.724 and
beats everything learned, while only a learned estimator produces metres that
track actual error. NGPS Eq. 7 saturates at 10.00 m and cannot express a bad
fix at all. So: **threshold to reject** (`inlier_gate` in solve.py), **learned
estimator to set the covariance** on what survives
(`processing_layer/covariance.py`, backend `learned`). The shipped default is
`gate_only`, a fixed placeholder that labels itself as one in every record.

---

## Real numbers: AnyVisLoc env80

`configs/env80.yaml` plays the env80 frames through all three layers against
the scene's own satellite basemap. Unlike the Sydney demo these are real UAV
frames over a real reference, so the errors mean something.

Scene_09, xfeat_mnn, satellite reference, gate 12, cold start every frame:
**accept rate 20%, median error 2.56 m, 75% of scored fixes inside 5 m**, and
one fix wrong by 175 m that passed both the gate and the plausibility test.
That outlier is the whole rejection problem in one row, and it is why the
project reports p90 and p99.

AnyVisLoc ground truth is scene-local and SfM-refined, **not geodetic**. Rather
than inventing coordinates, the adapter gives the map a transverse-Mercator CRS
anchored at a declared origin: every distance is exact to the millimetre, the
geodetic path downstream needs no second code path, and `local_frame` is stamped
on the map packet, the session header and the dashboard. Do not quote the
latitudes and longitudes as positions; do quote the errors.

Two conventions, both verified against the benchmark's own code rather than
assumed. **Pitch is zero at nadir** and the statistics table's view angle is
`90 - |pitch|`. **There is no altitude key** -- altitude is `xyz[2]` and
position is `xyz[:2]` in the scene frame, mapped by
`col = (x - origin_x) / res_x` with no y-flip (`avl_utils.py:842`). The adapter
shifts the stored affine by half a pixel so that the centre-based convention in
`geo.pixel_to_crs` reproduces that corner-based formula exactly.

## The sweep

    bash sweep.sh

Runs every env80 frame through the same modules the live system uses, with the
bus taken out, and prints a table comparable with the harness results already
in the parent repo's `results/`. Idempotent -- a finished combination is reused,
`FORCE=1` redoes it. Tens of minutes for 326 frames across five matchers.

Two things it does deliberately. **The gate is not applied during the run**:
every plausible solve is recorded with its inlier count and its error, and the
accept rate is computed afterwards across gates 0-60, so one run answers what
gate a reference needs instead of assuming one. And **cold start by default**,
because that is how the harness evaluates; `--prior sequential` measures the
easier case and labels itself.

`--no-rectify` and `--no-tile-ransac` are the two A/Bs behind the findings
above.

Early result, 10 frames of Scene_09 against the satellite basemap: **ORB
produced 1200+ matches per frame and 6-19 inliers, every one geometrically
implausible.** Match count is not a quality signal, which is the argument for
gating on inliers rather than on matches.

## Verification

    bash verify.sh          46 checks against the specification, ~3 minutes
    bash sitl.sh            open-loop validation against ArduPilot SITL

`verify.sh` runs the real processes over the real bus, kills a layer to prove
the other two survive, checks the export, and round-trips real MAVLink through
a transcription of ArduPilot's own handler. Not imports and not mocks:
independence cannot be tested any other way. Run it after any change to a layer
boundary.

`scripts/test_fc_encoding.py` is the MAVLink half on its own. It opens a
socket, sends what the output layer would send in flight, and decodes it with
the logic copied from `GCS_Common.cpp`. Every bug it catches was invisible from
the sending side.

## NGPS: what was read, and what it does NOT do

Read on 2 Sept 2026: `snktshrma/ngps_flight` (the paper repo) and
`snktshrma/ap_nongps` (the GSoC simulation repo), plus the author's two setup
documents.

`ap_vo2/src/ap_vo2/map_match_node.py` is the closest thing to this runtime, and
every structural decision here matches it independently:

| | ap_vo2 | geoanchor-rt |
|---|---|---|
| reference | UTM GeoTIFF, GDAL `GetGeoTransform()` | UTM GeoTIFF, same affine formula |
| reference features | AKAZE over the tile once at startup, in RAM | tiled store cached to disk by content hash |
| frame | centre-crop ROI, 512 px | rescaled to the reference GSD from altitude |
| search | `search_window_px: 1500` round the last fix, 0 = whole tile | `prior_radius_m` -> tiles, cold start = whole map |
| geometry | `findHomography` USAC_MAGSAC | same |
| gate | `min_inliers: 12` | `inlier_gate`; env80 also landed on 12 |
| covariance | `position_covariance_xy: 2.0`, one constant | pluggable, placeholder labelled as one |
| local frame | `utm - (E0, N0)` | ellipsoidal offset from a declared origin |

Two differences that matter.

### Three bugs in the write path, none of which raise anything useful

All three were found by decoding our own output the way ArduPilot decodes it.
None of them is visible from the sender.

1. **`frame_id = MAV_FRAME_LOCAL_NED` is discarded in silence.** `LOCAL_FRD` is
   **20**, not 1. `handle_odometry()` returns early with no warning, so the
   symptom is an estimator that never sees external navigation.
2. **A NaN in the translational covariance poisons posErr.** ArduPilot computes
   `sqrtf(cov[0] + cov[6] + cov[11])` and only checks `isnan(cov[0])`.
3. **pymavlink writes MAVLink 1 on a link that only ever writes.** It negotiates
   the wire version from INBOUND traffic and starts at 1.0, so a send-only link
   never upgrades and every message id above 255 is absent from the object.
   ODOMETRY is 331. The failure is `'MAVLink' object has no attribute
   'odometry_send'`, thrown the first time a fix is ready to send. `fcout.py`
   binds a v2 protocol object explicitly rather than relying on the MAVLINK20
   environment variable, which is order-dependent across imports.

**NGPS never sends MAVLink.** It publishes `nav_msgs/Odometry` on a ROS 2 topic
and lets MAVROS or the uXRCE-DDS bridge carry it to the autopilot. So there was
no reference implementation for the write path in `output_layer/fcout.py`, and
it was wrong twice -- see below. This runtime writes MAVLink directly and runs
no ROS, which is one fewer stack on the board and one fewer thing to install on
JetPack 5.

**Their covariance is a ROS convention, not a MAVLink one.** ap_vo2 sets the
unknown diagonal entries to `9999.0`, which is right for `robot_localization`.
Copying that shape into a MAVLink message would be wrong, because ArduPilot
reads those six numbers completely differently.

### The ArduPilot parameter block, from the author's own setup doc

    SERIALx_BAUD    = 921600     UART with hardware flow control (RTS/CTS)
    VISO_TYPE       = 1
    VISO_DELAY_MS   = 50         his measured latency; use YOUR measured median
    EK3_SRC1_POSXY  = 6          ExternalNav
    EK3_SRC1_YAW    = 1          compass. This pipeline sends an identity
                                 quaternion, so do NOT set 6
    EK3_SRC1_VELXY  = 0
    EK3_SRC1_VELZ   = 0

`scripts/check_extnav.py` checks all of these plus `AHRS_EKF_TYPE`,
`VISO_POS_X/Y/Z`, `VISO_QUAL_MIN` and `EK3_SRC1_POSZ`. The config baud was 57600
and is now 921600 to match.

## QGroundControl

`output_layer.qgc.enabled` puts the predicted position on the QGC map as a
second vehicle: a heartbeat and GLOBAL_POSITION_INT under a different system id
(the autopilot is normally 1). Display only, on its own socket, independent of
`loop_mode`, and a failure is `OLDE-07` and nothing else. During a real flight
that is the view already open, showing where the pipeline thinks the aircraft is
beside where the autopilot thinks it is.

The built-in map stays the primary view: it draws on the reference tile the
matcher actually used, with a confidence circle from the emitted covariance, and
it needs no network on an aircraft that is offline by definition. QGC can do
neither of those.

## Bring-up

`docs/bringup.md` is the ordered guide, and `scripts/` carries the three tools:

    scripts/measure_overhead.py   OVERHEAD_MS. Do this first.
    scripts/calibrate_camera.py   fx_px, which the whole rescale depends on.
    scripts/check_extnav.py       the live ExternalNav path, SITL first.

## Board notes: AGX Xavier

- **JetPack 5.1.x is the ceiling.** L4T 35.x, Ubuntu 20.04, Python 3.8,
  CUDA 11.4. JetPack 6 is Orin-only. Do not follow Orin instructions.
- Python 3.8 means **torch caps at 2.4.x** for cp38 aarch64 wheels.
  `bootstrap.sh` pins accordingly. For a newer stack, install python3.10 from
  deadsnakes and re-run with `PYTHON=python3.10`.
- **Fix the clocks before any timing run**, once per boot, and record the mode:

      sudo nvpmodel -m 0 && sudo jetson_clocks && sudo nvpmodel -q

- The Xavier is a **bench board, not a flight step** -- 30 W and the weight
  rule it out of the airframe. It is the easiest board to set up, which is why
  the full pipeline is developed here first and then ported down. Take its
  numbers as the upper bound of the compute curve.
- Power comes from INA3221 rails via sysfs; `geoanchor/device.py` searches the
  known L4T 32 and L4T 35 layouts and reports which one answered. A missing
  reading returns `None`, never `0.0` -- a zero silently becomes a J/fix of
  zero, and that has already cost one run.

### Pi 5 baseline to compare against

Canonical N=200 run, `performance` governor, no throttling. Warm means the
reference tile was precomputed, which is the deployed case:

    matcher              cold p95    warm p95   end-to-end   J/fix
    orb                     152.3        56.6         96.6    0.63
    sift                    420.6        98.9        138.9    2.32
    akaze                   545.9       107.5        147.5    2.91
    xfeat_cpu              1017.4       161.6        201.6    4.51
    xfeat_lighterglue      2786.4      2108.6       2148.6   16.05

XFEAT_LG does not fit even warm; its cost is LighterGlue itself, and the only
lever is `top_k` (attention is quadratic in keypoints). xfeat_cpu fits with
48 ms of margin against an `OVERHEAD_MS` of 40 ms **that is still a guess**.

**Measure OVERHEAD_MS on the real rig before anything else.** Camera capture,
ISP, copy and the MAVLink hop above 88 ms and xfeat_cpu misses; below, it fits.
Everything else about the deployment question is downstream of that number.

---

## Latency, and why it is the binding constraint

There is **no minimum-rate check** on ArduPilot's ExternalNav path. Below 1 Hz
sets `dead_reckoning`, below 1/7 Hz trips `posTimeout`, above 50 Hz the fix is
dropped. So rate is easy.

Latency is not. Delay compensation is capped at 250 ms (`VISO_DELAY_MS` range
is 0-250, clamped again at `AP_NavEKF3_core.cpp:83`) and **overrunning it is
silent**: `writeExtNavData()` does `MAX(timeStamp_ms, imuDataDelayed.time_ms)`,
so a late fix is not rejected, it is stamped as current and fused at the wrong
time. At 5 m/s each 100 ms of uncompensated latency injects 0.5 m.

Hence: frames are stamped at CAPTURE, never at fix completion; the processing
layer always matches the newest frame and drops the backlog (`PLE-10`); and
`PLE-09` records every overrun rather than hiding it.

EKF3 clamps horizontal position variance to [0.01, 100] m, so the covariance
field has far more range than Eq. 7 can use.

---

## Where things are

    geoanchor/codes.py          the step-code registry. Append-only.
    geoanchor/contracts.py      the messages that cross layer boundaries
    geoanchor/bus.py            ZeroMQ. Drop-not-block, and drain-to-newest.
    geoanchor/methods.py        orb sift akaze xfeat_mnn xfeat_lg. All CPU.
    geoanchor/geo.py            pixel <-> geodetic. pyproj, not rasterio.
    geoanchor/device.py         board detection and the power probe
    geoanchor/data_layer/       mapprep, store, feed, gpsin
    geoanchor/processing_layer/ rectify, solve, covariance
    geoanchor/output_layer/     metrics, recorder, fcout
    geoanchor/api/              FastAPI observer + WebSocket
    dashboard/                  Next.js, static export, live and replay modes
    configs/system.yaml         the ONE config all three layers read
    scripts/preflight.py        what this board can and cannot do
    scripts/make_demo_video.py  synthetic flight over the reference tile
    stores/                     built feature stores, keyed by content hash
    runs/<stamp>_<tag>/         one directory per session

`python scripts/preflight.py` before trusting anything.

---

## Running it

    bash bootstrap.sh                                  once per board
    python -m geoanchor.data_layer --build-map         once per new map
    bash run.sh                                        everything

Each layer also runs alone, which is the fastest way to debug one:

    python -m geoanchor.data_layer
    python -m geoanchor.processing_layer --method orb
    python -m geoanchor.output_layer --loop-mode off

`GEOANCHOR_SET="a.b.c=value;d.e=value"` overrides config keys for one run.
`GEOANCHOR_RUN_DIR` puts several layers in the same session directory --
`run.sh` sets it, and layers started by hand must share it or the JSONL files
scatter and the session cannot be reassembled.

---

## Traps already paid for. Do not re-discover these.

- **torch on ARM resolves to a CUDA build.** PyTorch 2.11.0 dropped the
  `platform_machine == "x86_64"` guard on its CUDA deps, so an unpinned install
  on any ARM Linux board pulls over a gigabyte that can never execute. On a
  Jetson it is worse: PyPI's CUDA wheels target discrete and SBSA GPUs, not the
  integrated iGPU, so they install cleanly and fail at the first kernel launch.
  `bootstrap.sh` pins and then asserts `torch.version.cuda is None`.
- **`opencv-python-headless>=4.8` resolves to OpenCV 5**, which moved the
  2D-features constructors: `cv2.AKAZE_create` disappears and the AKAZE
  baseline dies mid-run. Pinned `<5`.
- **FastAPI + `from __future__ import annotations` + a function-local import
  of `WebSocket`** makes every WebSocket upgrade return a bare HTTP 403 with no
  traceback anywhere. Annotations become strings, `get_type_hints()` cannot
  resolve `WebSocket` from the module globals, dependency solving fails, and
  FastAPI's handler closes the socket instead of raising. The fastapi imports in
  `api/server.py` are at module scope for this reason.
- **A replay sidecar carries the ORIGINAL flight's clock.** Passing it through
  as the GPS timestamp makes every fix fail to pair by however long ago the
  flight was: `OLE-02` on every row and an export with no errors in it.
  `ReplayGps` stamps with the session clock.
- **A feature store holds descriptors from ONE extractor.** Matching XFeat
  float32 against ORB uint8 is an OpenCV assertion, not a bad result. The store
  id hashes the method in, `attach_map` refuses a mismatch (`PLE-14`), and
  changing the method from the dashboard rebuilds the store.
- **`match_lighterglue` does `image_size[None, ...]` internally**, so it must
  be handed shape `(2,)`. Handing it `(1, 2)` surfaces far away as
  `mat1 and mat2 shapes cannot be multiplied` inside kornia's attention. Note
  that the parent repo's harness patch calls the matcher directly rather than
  through `match_lighterglue`, so `(1, 2)` is correct *there*. Different call
  paths, different shapes.
- **haversine is out by 0.2% at Sydney's latitude** -- 21 cm over 100 m --
  because it assumes a sphere of mean radius. That is a systematic scale bias
  sitting underneath a 0.5 m ground-truth noise floor, and it would quietly
  shift every error in every table. `geo.distance_m` uses `pyproj.Geod` and is
  exact; `haversine_m` is kept only for cheap comparisons.
- **ODOMETRY with `frame_id = MAV_FRAME_LOCAL_NED` is discarded in silence.**
  `handle_odometry()` (`GCS_Common.cpp:4097`) returns early unless `frame_id`
  is `MAV_FRAME_LOCAL_FRD` (**20**, not 1) and `child_frame_id` is
  `MAV_FRAME_BODY_FRD` (12). No warning, no status flag, no counter. The only
  symptom is an estimator that never sees external navigation, which looks
  exactly like a wiring fault.
- **A NaN anywhere in the translational covariance makes posErr NaN.**
  ArduPilot collapses the 21-element array with
  `posErr = sqrtf(cov[0] + cov[6] + cov[11])` and only checks `isnan(cov[0])`.
  Writing NaN into `cov[11]` to mean "altitude unknown" -- the natural reading
  of the MAVLink spec -- feeds NaN straight to EKF3. Every summed entry must be
  finite. And because posErr is a 3D magnitude, a radial sigma has to be split
  as `cov[0] = cov[6] = sigma^2 / 2` with `cov[11] = 0`, or the filter receives
  `sigma * sqrt(2)`. `verify.sh` asserts posErr equals the emitted sigma.
- **`vcgencmd pmic_read_adc` uses two keywords**: `_A` rails answer
  `current(n)=`, `_V` rails answer `volt(n)=`. Matching only `current(` leaves
  the volts empty and turns J/fix into NaN for a whole run.

---

## Open, in order

1. **Measure `OVERHEAD_MS`** on the real rig -- capture to first byte of the
   frame the matcher sees. Everything else is downstream of it.
2. **Calibrate the camera** and put the real `fx_px` in the config. Without it
   frames cannot be scaled to the reference GSD and the matcher eats the full
   scale gap. Do not take fx from a datasheet.
3. **Run the top_k sweep** for `xfeat_lg`: latency on the board, accuracy on
   env80 on the Legion. A Pareto front on real hardware is contribution-shaped.
4. **Export a covariance estimator** to `processing_layer/covariance.py`'s
   `learned` backend. Validate leave-one-scene-out; never by row.
5. **Raise the env80 accept rate.** 20% cold-start on Scene_09 is honest but
   low. The prior almost never establishes because too few frames succeed in a
   row. Worth trying, in order: `xfeat_lg` (79 inliers where mnn got 26 on
   L09_0002), the aerial reference as a control, and a retrieval step ahead of
   matching so the cold start is not a whole-map search.
6. Verify how ArduPilot derives `posErr` from the 21-element covariance, with
   the parent repo's `docs/step22_ardupilot_extnav_check.py`.
