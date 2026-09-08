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

## env80 sweep result, 3 Sep 2026 (laptop x86, CPU only)

`results/env80_sweep/summary.json` — 326 frames (Scene_09 n=134, Scene_10 n=192),
satellite reference, five matchers, gates 0-60, `prior: none`, rectify on,
tile RANSAC on. Read `docs/` and the numbers themselves before re-deriving any
of this.

- **Use `xfeat_mnn` at inlier gate 10-12.** Only matcher clearing the
  plausibility check at a useful rate on both scenes (46.3% / 18.8%), and about
  18 ms per reference tile.
- **Gate 10 is the knee, and it is worth a lot.** Scene_09 xfeat_mnn: gate 0
  accepts 62 fixes with a 187.4 m worst case; gate 10 accepts 46 with a 5.96 m
  worst case and 100% within 10 m. Above gate 15 the gate costs fixes and buys
  no accuracy. Gates below 10 are dead controls — gate 0 and gate 5 are
  identical because the plausibility check already removes everything under
  five inliers.
- **ORB, SIFT and AKAZE are not viable against real satellite reference.**
  Zero plausible fixes in 192 Scene_10 frames for ORB and SIFT. The few they
  return on Scene_09 are wrong by 62 m / 127 m / 81 m at the median — confident
  fixes on the wrong building. Do not read their match counts as a quality
  signal. (AKAZE on Scene_10 is the one oddity: 6 fixes of 192, all inside 5 m.)
- **Latency is linear in reference tile count.** Scene_09's map is 9 tiles,
  Scene_10's is 1, and per-tile cost is near-constant across both: orb ~5 ms,
  akaze ~8 ms, sift ~13 ms, xfeat_mnn ~18 ms, xfeat_lg ~290-500 ms. So
  xfeat_lg's 4539 ms median on Scene_09 is not a Scene_09 pathology, it is
  nine tiles of LighterGlue.
- **The GPS prior is load-bearing, not an optimisation.** This sweep ran
  exhaustively. A 5 km x 5 km NSW Spatial reference at 0.5 m GSD is roughly 120
  store tiles, which is ~2.2 s per frame with xfeat_mnn and over a minute with
  xfeat_lg. `prior_radius: 120.0` already exists in `configs/env80.yaml` and is
  unused — switch it on and measure before concluding anything about whether
  the Xavier is fast enough.
- **Unsettled: the two scenes disagree on which XFeat wins.** Scene_09 favours
  xfeat_mnn over xfeat_lg 4.4:1 on plausible rate; Scene_10 reverses it 1.9:1.
  Until that is explained, "xfeat_mnn wins" is a claim about Scene_09. Scene_04,
  Scene_08, Scene_20 and Scene_21 are already downloaded in ~/Downloads and are
  the obvious extension.
- Percentiles only in anything written up: median, p90, p99, max. No mean, no
  RMSE — a distribution with a 187 m tail has no meaningful average.

## Board notes: AGX Xavier

- **JetPack 5.1.7 (L4T 35.6.5) is the ceiling and the last release for this
  board.** Ubuntu 20.04, Python 3.8, CUDA 11.4. JetPack 6 is Orin-only, so do
  not follow Orin instructions for wheels, TensorRT or flashing.
- **Flashing host: the Legion runs Ubuntu 24.04, which NVIDIA does not support
  for JetPack 5.** SDK Manager refuses to install natively; use NVIDIA's
  SDK Manager Docker image. Its docs also say the container "does not currently
  support flashing to external storages on all Jetson devices", so flash the
  eMMC with it and move the rootfs to NVMe separately.
- **Take the Ubuntu 20.04 Docker variant, not 22.04 or 24.04.** NVIDIA offers
  all three, and only one works. Their host-OS compatibility matrix on
  developer.nvidia.com/sdk-manager ticks JetPack 5.x for Ubuntu 18.04 and 20.04
  only; 22.04 and 24.04 are blank for that row. A 22.04 container installs and
  runs perfectly well and then simply does not list JetPack 5 for the Xavier,
  which reads as a hardware or recovery-mode fault and is not. Verified on the
  live page 3 Sep 2026; the file is
  `sdkmanager-2.4.1.13536-ubuntu_20.04_docker.tar.gz`.
- **JetPack 5.1.7 may sit behind `sdkmanager --archived-versions`.** SDK Manager
  2.4.1 hides older SDK releases by default. If the Xavier appears but no
  JetPack 5 release is listed, add that flag before concluding anything is
  wrong.
- **Boot firmware lives on the eMMC and cannot be moved.** "Booting from the
  SSD" on this board means bootloader, kernel and extlinux.conf on eMMC, root
  filesystem on NVMe. Full procedure in `docs/xavier_setup.md`.
- **Pin the kernel after moving the rootfs** (`apt-mark hold nvidia-l4t-kernel
  nvidia-l4t-kernel-dtbs nvidia-l4t-kernel-headers`). The bootloader reads the
  kernel from eMMC while apt installs updates onto NVMe, so an upgrade leaves
  the kernel and `/lib/modules` out of step and the board boots with no working
  modules.
- **rasterio here is `rasterio<1.3`, and the pin is not optional.** Ubuntu
  20.04 ships GDAL 3.0.4; rasterio 1.3+ demands GDAL >= 3.1 and refuses at the
  "getting requirements to build wheel" step with `ERROR: GDAL >= 3.1 is
  required`. The working sequence is `sudo apt install libgdal-dev` then
  `pip install "rasterio<1.3"`, which builds 1.2.10 from source in a few
  minutes. Needed only to ingest a GeoTIFF into a store -- which happens
  whenever `data_layer.map.method` changes, since the store id carries the
  method. `data/sydney/ref_tile.raw.png` is NOT a substitute for the tif: it is
  4033x4033 against the tif's 4112x4093, a different raster.
- **Import torch before rasterio, or torch will not import at all.** Reading a
  GeoTIFF pulls in GDAL and its dependency tree, which exhausts the process's
  static TLS surplus; a torch imported afterwards dies with `libgomp-....so:
  cannot allocate memory in static TLS block`. On this board that surfaced as
  `DLE-01 method 'edgepoint2_s64' unavailable: torch is not installed` during a
  map build, on a machine where torch imports perfectly well on its own --
  every import ordering tested by hand worked, because the failure needs the
  full GDAL load that only `read_georeference` triggers. `mapprep.build_store`
  now constructs and validates the method before reading the georeference, and
  the order is load-bearing; do not tidy it back.
- Python 3.8 means **torch caps at 2.4.x** for cp38 aarch64 wheels.
  `bootstrap.sh` pins accordingly. For a newer stack, install python3.10 from
  deadsnakes and re-run with `PYTHON=python3.10`.
- **Fix the clocks before any timing run**, once per boot, and record the mode:

      sudo nvpmodel -m 0 && sudo jetson_clocks && sudo nvpmodel -q

  How forgetting presents: the board boots `MODE_15W_DESKTOP`, which onlines
  **four of the eight cores** (`/sys/devices/system/cpu/online` reads `0-3`)
  while leaving per-core clocks at 2.19 GHz. Nothing errors. `PL-01` and the
  dashboard's board card both report "4 cores" -- correct, and easy to read as
  the board's spec rather than as a mode. Measured end-to-end on the replay,
  MAXN roughly halves the solve (1.0-1.9 s -> 0.56-1.04 s) and roughly doubles
  the fixes that land. Check `nproc` before believing any number off this board.

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

### Do not compare a Xavier number to this table without checking the geometry

`scripts/bench_matchers.py` prints detect and match separately at a stated
reference geometry, so any two boards compare on the same row. Run it on each
board and diff; do not compare a number to the Pi table without checking the
tile count behind it.

Xavier at MAXN, 8 threads, 646x484 frame, xfeat_mnn, same frame throughout:

    reference set                       detect     match      total
    1 tile,   2048 ref kp               258.6      28.6       287.2
    9 tiles, 13305 ref kp               233.8     112.0       345.9
    25 tiles, 51200 ref kp              223.3     439.0       662.3

**Re-baselined against the Pi table on matched geometry** -- one tile, 2048
reference keypoints, p95 of 15 reps, which is the column the Pi table quotes
(`python scripts/bench_matchers.py --tiles 1 --reps 15`):

    matcher              Pi 5 p95    Xavier p95    Xavier is
    orb                      56.6          96.6      1.71x slower
    sift                     98.9         197.5      2.00x slower
    akaze                   107.5         157.1      1.46x slower
    xfeat_cpu / _mnn        161.6         291.9      1.81x slower
    xfeat_lighterglue      2108.6        2655.9      1.26x slower

So the Xavier is 1.3-2.0x slower than a Pi 5 across every matcher, not 10x.
The ordering of the matchers is unchanged, so conclusions drawn from the Pi
table about which matcher to use still hold on this board.

Two things follow, and both were got wrong once already:

- **The Pi table above was measured against a one-tile reference.** The replay
  runs against `ref_tile__xfeat_mnn__*`, which is 25 tiles and 51200 keypoints.
  Match cost is linear in reference keypoints, so the same board on the same
  frame goes from 28.6 ms to 439 ms purely on search width. An apparent 10x
  "the Xavier loses to a Pi" is mostly this, and is not a board result at all.
  Compare like geometry or do not compare.
- **On like geometry the Xavier is about 1.8x slower than the Pi 5**
  (287 ms against 161.6 ms), and that gap is real. It is a per-core gap, not a
  throughput one: this workload barely threads.

      threads      detect     match(25 tiles)
      1             301.6        637.3
      8             240.3        407.1

  8x the cores buys 1.26x on detection and 1.57x on matching. Carmel is a 2018
  core and loses to the Pi 5's A76 per clock, so the Xavier's one advantage on
  CPU -- eight cores instead of four -- is mostly unavailable here.

### On ARM, XFeat is slower than SIFT -- on both boards

XFeat's own README claims it is "faster than SIFT on CPU", and the paper's CPU
result is qualified as "tested on laptop with an i5 CPU". That is an x86 result
with AVX2 and MKL behind it. On ARM the ranking inverts, and our own two boards
already agreed on this before anyone looked it up:

    matcher     Pi 5 p95    Xavier p95
    sift            98.9        197.5
    xfeat_mnn      161.6        291.9      1.6x / 1.5x SLOWER than sift

torch on this board reports `MKL not found` and falls back to OpenMP + oneDNN,
while OpenCV's SIFT has hand-tuned NEON. The authors publish no embedded CPU
timings at all -- only GPU figures (150+ FPS single-batch VGA, 1400 FPS batched
on a 4090) and the i5 laptop claim -- so there is no published number for this
class of hardware to be measured against. Ours is the baseline. XFeat is still
worth keeping for robustness to viewpoint and illumination, which is what it
actually buys over SIFT, but not for speed on a CPU-only ARM target.

### EdgePoint2: faster, far fewer catastrophic failures, and the gate hides it

`geoanchor/methods.py::EdgePoint2Method` (`edgepoint2_t32/_s32/_s64`, MIT,
clone at `<repo>/edgepoint2`). Measured on env80, Xavier at MAXN, cold start.

**The reason to use it is NOT the compact descriptor.** 32-D against XFeat's
64-D changes matching by 2% (464.6 -> 454.5 ms against a 51200-keypoint
reference), because the cost is materialising and reducing the ~2459 x 51200
similarity matrix -- about 500 MB of intermediate -- not the descriptor product.
Halving the width halves only the input read. Worse, on the hard scene the 32-D
variants collapse: Scene_09 accept at gate 25 is 5.2% for S64 against 0.8%
(T32) and 0.0% (S32). Narrow descriptors cost accuracy and buy no speed here.
**Use S64.** The speed comes from the network: detect 248 -> 149-170 ms.

**Set `score`, not just `top_k`.** Upstream defaults to `score=-5`, and on
env80 query frames that gate -- not `top_k` -- decides the keypoint count:
2278 where XFeat gives 4096. Since accept rate rises monotonically with
keypoint count, that alone made EdgePoint2 look worse. `DEFAULT_SCORE = -12`
saturates `top_k` and puts both methods on the same budget; Scene_10 accept
then goes 5.7% -> 9.9%, exactly XFeat's.

**The gate is per METHOD, and the old grid jumped over where it matters.**
Pooled over both scenes, 326 frames, cold start:

    xfeat_mnn        gate     7      8      9     10     12     15
                   accept  29.1%  27.6%  27.0%  24.5%  21.8%  18.1%
                   p99    167.85  18.46   6.75   6.64   6.08   6.08
                   max    187.42  40.61  18.46   6.75   6.75   6.75

    edgepoint2_s64   gate     6      8     10     12     15     20
                   accept  39.0%  37.4%  31.0%  27.3%  21.2%  13.5%
                   p99      7.50   7.38   7.36   5.70   5.26   5.15
                   max      8.36   7.50   7.50   6.62   6.62   5.15

XFeat's p99 collapses over three gates -- 167.85 to 6.75 m across 7, 8, 9 --
and 10 is where the maximum stops being catastrophic. **EdgePoint2 never has
that cliff: its worst error is 8.36 m even at gate 6**, so it wants a much
lower gate, and 8 gives 37.4% accept where xfeat_mnn manages 24.5% at its own
best gate. Applying one number to both throws away a third of EdgePoint2's
fixes. `configs/env80.yaml` is 10 (xfeat_mnn); use 8 if you switch the method.

`GATES` in `env80_sweep.py` now includes 6, 8, 12 and 14 -- the old 5/10/15
grid straddled the entire collapse.

**top_k is a real tradeoff for EdgePoint2, unlike for xfeat_mnn.** env80,
clocks pinned, gate 8, `results/topk_sweep_ep2/`:

    sc     k   detect   match   total     p95   plaus   acc@8   med_m
    09   512    433.1    16.3   467.0   670.4    5.2%    0.8%   3.061
    09  1024    418.1    48.7   485.1   704.3   13.4%    2.2%   2.230
    09  2048    403.1    46.4   478.1   684.1   30.6%   23.1%   2.545
    09  4096    447.9   444.2   933.3  1086.7   43.3%   37.3%   2.948
    10   512    113.8     3.6   126.8   186.6   23.4%   18.2%   3.866
    10  1024    122.8     9.1   141.8   186.8   25.0%   24.5%   3.967
    10  2048    103.0    24.7   137.1   201.1   40.1%   40.1%   3.829
    10  4096    127.5    61.0   198.6   260.1   45.3%   44.3%   3.591

For xfeat_mnn, k=4096 dominated every other point on both axes. Here **k=2048
is the knee**: on Scene_09 it buys 23.1% accept at 478 ms where 4096 buys 37.3%
at 933 ms -- 1.6x the accept rate for 2x the latency -- and on Scene_10 it
reaches 40.1% at a p95 of 201 ms, inside the 250 ms budget, against xfeat_mnn's
17.2% at 168 ms. `configs/*.yaml` still ship `max_keypoints: 4096`, which is
right for xfeat_mnn and probably wrong for edgepoint2_s64.

The Scene_09 latency cliff between 2048 and 4096 is `WIDE_REF` firing, not the
detector: 9 tiles x 2048 = 18432 reference keypoints uses the one-matmul path
at 46 ms, and 9 x 4096 = 36864 crosses 20000 into the transpose-product path at
444 ms. The two findings are the same finding seen twice -- reference-keypoint
count, not frame keypoints, is what the matcher cost tracks.

**Its failures are far less wild.** Ungated p90 on Scene_09 is 7.58 m against
XFeat's 36.18 m. For a covariance estimator, and for anything that has to
survive a bad fix, that matters more than the accept rate does.

**The matcher's fast form depends on the reference size, and the crossover is
a cliff.** Mutual nearest neighbour needs a max along each axis. `max(dim=0)`
walks a (Q, N) row-major tensor across its stride; past a point that falls off
a cliff, and paying for a second matmul to reduce the transpose along the
contiguous axis wins instead. AGX Xavier, pinned clocks, Q=4096:

    ref kpts    2048   4800  11589  16000  22528  51200
    one-matmul  39.9   40.5   87.0  116.3  401.9  883.7
    two-matmul  27.8   64.6  117.9  203.6  233.4  610.3

Note 116 -> 402 ms for one-matmul between 16000 and 22528, against a 1.4x size
increase. `EdgePoint2Method.WIDE_REF = 20000` picks the form; both return
IDENTICAL index sets, so a wrong threshold costs milliseconds, never
correctness. **It is a cache effect and therefore board-specific -- re-measure
on a Pi 5 or TX2 rather than assuming this number.** Chunking to bound the
working set was tried and is worse than both at every size (1043 ms at 51200).

Two ways this was got wrong first: always using one form made EdgePoint2 look
SLOWER than XFeat on the live pipeline (949 against 837 ms) while winning every
isolated benchmark; then always using the other regressed env80 Scene_09's
match by 71% (72 -> 123 ms), because 11589 keypoints sits below the cliff.

**Head to head, each at its own gate.** All of the following is one session,
clocks pinned, same frames, with xfeat_mnn re-measured alongside rather than
quoted from an older run:

    sc  method          gate  accept  med_m  p90_m  max_m  lat med   p95
    09  xfeat_mnn         10   35.1%  2.666  4.507  6.645    674.5  906.2
    09  edgepoint2_s64     8   32.1%  2.921  4.944  7.497    535.7  729.5
    10  xfeat_mnn         10   17.2%  3.518  4.687  6.748    168.5  235.5
    10  edgepoint2_s64     8   41.1%  3.829  5.050  7.356    129.5  185.4

EdgePoint2 is faster on both scenes (1.26x, 1.30x) and accepts 2.4x as many
frames on Scene_10, at errors within a few hundred millimetres of XFeat's.
Scene_09 is the one place XFeat still leads on accept rate, 35.1% to 32.1%.
Its plausible-solve ceiling tells the same story from the other side: XFeat
47.0% / 18.8% across the two scenes, EdgePoint2 37.3% / 42.2%.

---

### The demo flight moves at 85.6 m/s -- do not tune latency against it

`demo/flight.mp4` steps 21.41 m between frames at 4 fps: 1464 m of track over
60 frames, about 308 km/h. It is a synthetic sweep across the reference tile,
not a flight profile. This matters because `prior_radius_m` is sized against
ground speed -- the config comment reasons "100 m at 20 m/s is 5 s" -- and on
this feed the vehicle crosses 66 m during a single 0.77 s solve. Any search
radius or `prior_max_age_s` validated on this feed is being validated against
dynamics no small UAV has. Tune those two against a real profile, or against
env80, and treat the synthetic feed as a wiring test only, which is what the
dashboard banner already says it is.

**The GPU is the unused lever, and it is the reason to have this board.** The
512-core Volta sits at 1.377 GHz doing nothing: `methods.py::XFeatMethod._load`
hardcodes `self._x.dev = torch.device("cpu")`, there is no device knob in any
config, and `bootstrap.sh` installs from the PyTorch CPU index by design. The
CUDA-build warning in `_load` is correct on x86 (a CUDA wheel there means pip
resolved the wrong one) but wrong on a Jetson, where NVIDIA ships a real
aarch64 CUDA wheel for JetPack 5. XFeat is a small CNN -- the detection half,
which is now the floor at ~230 ms and is the part that will not thread, is
exactly what a GPU fixes. Nothing below 250 ms is reachable on this board on
CPU: even a one-tile search is 287 ms.

---

## xfeat_lg costs the most exactly when it fails

LighterGlue ships `width_confidence: 0.95` -- point pruning on, early stopping
off. Pruning is confidence-driven, so a confident match sheds points and runs
fast while a failing one keeps them all and runs long. On env80 at k=2048,
326 frames:

    outcome            n     match_ms median   matches median
    plausible solve    74         767              124
    failed solve      252        7003                4

**9.1x slower when the answer is useless**, and 28x over the 250 ms budget
while producing nothing. For a latency-budgeted loop that is the worst possible
cost profile: the hard frames, which are the ones worth spending time on, are
also the ones that stall the layer and drop every frame arriving behind them.
It is a stronger reason to keep xfeat_lg out of the flight path than the
average cost is.

This also explains an unstable row in `bench_matchers.py`: whether a benchmark
frame happens to match decides which regime you measure. One reading came in at
3321 ms where two others gave 2237 and 2254 on the same store. Run-to-run
jitter on identical input is a further 8.4% stdev (1.38x min to max) -- at two
seconds, this is simply a noisy thing to measure. Measurement ORDER was tested
and is not a factor (1941 / 1777 / 1896 ms first, after seven other methods,
and after gc).

---

## PLE-09: why 250 ms is out of reach here, and what would move it

Xavier, MAXN, clocks pinned, edgepoint2_s64 at k=2048, sydney reference:

    tiles  ref kp    match   detect+match
        1    2048     20.7          223.9   fits
        4    8192     88.8          292.0   1.2x over
        9   18432    171.3          374.5   1.5x over
       25   51200    246.2          449.4   1.8x over

**Detection alone is 203 ms -- 81% of the budget before a single descriptor is
compared.** That is the whole story, and it means the search-narrowing levers
cannot fix it: even a one-tile search leaves 46 ms for everything else, and the
prior realistically selects 9-13 tiles. Tightening `prior_radius_m` is worth
doing but it is bounded by a term it does not touch.

The only lever that reaches detection is pixels, and it is roughly linear:

    scale     frame     Mpx   detect   match(9t)   total
     1.00   646x484   0.313    181.7       161.1   342.8
     0.85   549x411   0.226    120.0       162.5   282.4
     0.55   355x266   0.094     85.8       152.3   238.0

0.55 scale fits, at 238 ms. **The accuracy cost of that is unpriced** -- a
smaller frame covers less ground and changes the GSD match to the reference, so
env80 has to be re-run before anyone believes it.

Three things follow. The classical matchers already fit on both boards (orb
91.9 ms total at one tile here, 56.6 ms p95 on the Pi 5) and the question for
them is accuracy, not speed. This board is the wrong end of the compute curve
and its own notes say so -- a Pi 5 is 1.4-2.0x faster per core on this workload,
which puts edgepoint2's detection near 110 ms rather than 203 ms. And the
number that actually decides deployment is still `OVERHEAD_MS`, which needs a
camera on the rig and has never been measured.

### The architecture is not the choke point. It costs 12 ms of the 250.

Asked directly on 8 Sept 2026, and worth having a number for rather than an
opinion. `stage_ms` had been measured on every fix since the beginning and
published on `T_FIX`, and **nothing read it** -- it died at the bus, so no
session file could answer "where did the 250 ms go". It is now persisted into
`RecordPacket`, which is what makes the table below reproducible from any run.

Sydney replay, Xavier at MAXN pinned, `edgepoint2_s64`, k=2048, 10 tiles
median, steady state (first three frames dropped -- frame 1 costs 3.2 s of
lazy model load):

    stage              median      p95
    decode                3.1      3.4      cv2.imdecode of a 53 KiB JPEG
    rectify               1.8      3.1
    detect_frame        230.8    284.6      <-- 56% of everything
    load_reference        1.0     22.2      warm; 22 ms p95 is a cold tile
    match               143.6    172.9      <-- 35%
    ransac               15.6     22.2
    -----------------------------------
    COMPUTE sum         411.7    461.4
    latency_ms          579.4    690.0
    NON-COMPUTE         164.6    253.4

That 164.6 ms gap looks like architecture overhead. It is not. Two
measurements separate them:

**Transport, measured directly** -- a 53 KiB JPEG through the same `ipc://`
socket the data layer uses, publish timestamp to subscriber receipt, n=195:

    median 1.22 ms   p95 1.57 ms   max 2.13 ms

and `json.dumps` of a full fix header is 0.028 ms. ZeroMQ plus serialisation
is **under 1.3 ms**, or half a percent of the budget.

**The rest is queueing, and it is self-inflicted by the feed rate.** Compute is
412 ms, so the board sustains about 2.4 fps; the config asks for 4. Frames
arrive every 250 ms into a loop that consumes one every 412 ms, so the freshest
frame available at drain time is already 0-250 ms old. Re-running the identical
pipeline at `fps: 2` -- a 500 ms feed period, slower than compute, so nothing
ever queues -- collapses it:

    feed rate            compute    latency    NON-COMPUTE
    4 fps (250 ms)         411.7      579.4      164.6
    2 fps (500 ms)         370.9      383.1       12.3   p95 17.7

**12.3 ms.** That is the whole cost of the three-process split: JPEG encode,
the ipc hop, the covariance call and logging, end to end. The architecture
spends 5% of the budget; the detector spends 92%.

Three things follow:

- **Asking for more frames than the board can match makes latency worse, not
  throughput better.** 4 fps costs 196 ms of pure staleness against 2 fps and
  returns no extra fixes, because the conflating drain throws the surplus away
  anyway. Set `feed.fps` to what the board sustains. This is free.
- The bus design is already right and should not be touched. PUB drops rather
  than blocks, `SNDHWM` is 8, and `drain(keep_latest_of=[T_FRAME])` collapses a
  backlog to the newest frame. Backpressure is bounded; latency does not run
  away. Splitting into three processes to get fault isolation cost 12 ms and
  the isolation is real.
- **Do not optimise the plumbing.** Every millisecond of the transport, the
  serialisation and the process boundaries together is 1/17th of one call to
  `detect_frame`. There is no version of this where rewriting the bus matters.

Reproduce with: `bash run.sh --no-api`, then sum `stage_ms` per record from
`runs/<id>/records.jsonl` (excluding `tiles_fitted`, which is a count) and
subtract from `latency_ms`.

### The feed rate curve is not monotonic, and 2 fps is a Xavier number

The obvious conclusion from the two rows above -- "set fps to what the board
sustains" -- is right, but the mechanism is not what it looks like and the
curve has two sides. Swept on the Xavier, sydney replay, `edgepoint2_s64`,
`loop: true`, 100 s per arm, first five fixes dropped:

    fps   n    fixes/s   compute   latency   lat p95   staleness   p95     err_m med
      2   185     2.01     379.0     390.6     454.2        12.1    16.7      0.0057
      4   240     2.60     370.0     506.5     633.4       136.0   248.3      0.0057
      8   229     2.49     388.2     462.7     547.1        66.5   120.1      0.0059
     16   204     2.23     435.1     482.0     611.1        45.3   129.7      0.0057
     30   171     1.87     517.9     712.5    1024.7       190.2   480.1      0.0059

Staleness **falls** from 4 fps to 16 fps, which contradicts a naive queueing
story and confirms the conflating one: `drain(keep_latest_of=[T_FRAME])` keeps
only the newest queued frame, so a faster publisher means the newest frame is
younger when the loop finally picks it up. 136 ms at 4 fps is just the mean age
of a frame published every 250 ms; at 16 fps that age is 45 ms.

It reverses at 30 fps because the data layer is then reading and JPEG-encoding
30 frames a second that nobody will match, and it does that on the same eight
cores. Compute itself rises 379 -> 518 ms, and p95 error degrades (0.14 m
against a flat 0.006 m everywhere else). **The producer competing with the
consumer is the ceiling, not the queue.**

So there are two defensible settings and they optimise different things:

- **2 fps: minimum latency** (390.6 ms), at the cost of 23% of the fix rate.
  Chosen as the default, because latency is this project's binding constraint
  and rate is not -- ArduPilot's ExternalNav has no minimum-rate check above
  1 Hz, and 2.01 fixes/s has margin.
- **8 fps: the balanced point.** Half the staleness of 4 fps with 96% of its
  throughput. Switch to this if fix rate ever becomes the constraint.

4 fps -- the video's native rate, and the previous default -- is the worst of
the three: it publishes slowly enough to be stale but fast enough to queue.

**Do not inherit `fps: 2` onto another board.** It is correct where a fix costs
~410 ms. A board that matches in 250 ms wants 4. Re-run this sweep rather than
copying the number; `configs/system.yaml` carries the table for that reason.
The portable fix -- have the data layer pace itself to the processing layer's
observed rate instead of a hardcoded constant -- is not written, and is the
obvious thing to do if a third board turns up.

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

## Portability across boards

**Moving the project to a new board: read `docs/PI5_HANDOFF.md`.** It is the
migration checklist -- what git does not carry, the five traps in the order
they bite, the Pi's equivalent of `jetson_clocks`, and the Xavier numbers to
diff against. `bash scripts/pack_for_board.sh` builds the tarball of exactly
the gitignored payload (166 MB: `stores/`, `data/sydney/`, `demo/`,
`results/`); `--with-env80` adds the 2.4 GB AnyVisLoc scenes. Copying
`stores/` is what lets the second board skip rasterio/GDAL entirely.

Nothing in `geoanchor/` or `configs/` contains an absolute path, and it must
stay that way. `config.REPO_ROOT` is derived from `__file__`, and
`Config.resolve()` expands `~` and makes any relative path repo-relative, so
the same checkout runs from `/data/geoanchor-rt` on one board and
`/home/<someone>/GeoAnchor/geoanchor-rt` on another with no edit. A new
`/home/<user>/...` or `/data/...` string in code or YAML is a bug, not a
default -- put it in the config as a relative path, or behind an env var.

What is per-board, and therefore gitignored and rebuilt rather than copied:
`data` (a symlink to wherever the datasets landed on that machine), `stores/`
(keyed by content hash, so a rebuild is safe), `runs/`, `.venv/`,
`dashboard/out/`, `xfeat/`, `demo/`. `bootstrap.sh` is the once-per-board step
that recreates the ones that can be recreated.

**Absolute paths in a session are provenance, not a dependency.** `OL-16`
prints `str(self.rec.session)` and `session.json`'s header carries `run_dir`,
both absolute on the machine that produced them -- so a session exported from a
laptop shows that laptop's `/home/<user>/...` in the Output layer card even when
the dashboard displaying it is on a different host entirely. That is correct and
worth keeping: it says which machine produced the numbers, which matters as soon
as more than one board is in play. Do not "fix" it to a relative path, and never
read it back as a path -- replay loads the file it was handed, never the
directory the header names.

Corollary for the dashboard: a dashboard on host A served by an API on host A
shows host A's session, whatever code is checked out on host B. When a number on
screen disagrees with the tree you are editing, confirm which API the page is
talking to before believing either -- `/api/state` carries `board.arch` and
`board.cores`, which is the fastest way to tell two hosts apart.

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
