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

### Board notes: Pi 4

- **`import torch` SIGILLs on 2.10.0+.** Cortex-A72 (this board's core, and the
  same core as the Zero 2 W and CM4) is ARMv8.0-A and has no `asimddp`
  (`FEAT_DotProd`) -- `grep Features /proc/cpuinfo` will not show it, where a
  Pi 5's A76 does. From PyTorch 2.10.0 the aarch64 wheel's oneDNN/ACL backend
  emits `SDOT`/`UDOT` unconditionally rather than dispatching on the runtime
  CPU, so the interpreter dies on the bare import with no Python traceback --
  it is a hardware trap, not a catchable exception, so `except ImportError`
  in `preflight.py` cannot see it either; the symptom there is the whole
  process exiting mid-check. Bisected 10 Sept 2026: 2.6.0-2.9.0 import clean,
  2.10.0 does not. `bootstrap.sh` step 5 now pins `torch<2.10,>=2.6` on any
  aarch64 board without `asimddp`; kornia (LighterGlue) inherits the fix since
  it only imports torch, nothing binary of its own.
- No power probe on a plain Pi 4 (`vcgencmd` reports temperature but not
  current/voltage the way the INA3221-equipped boards do) -- see the
  preflight note under "power probe" in `geoanchor/device.py`. Use an inline
  meter and log it by hand; joules per fix is the headline number this
  project reports.
- `vcgencmd get_throttled` is the check to run after any timing or energy
  run, same as `jetson_clocks` on the Xavier and the governor/frequency pin on
  the Pi 5 -- a nonzero bit 19 (0x80000) means the soft temperature limit has
  already fired since boot, and a number taken after that is not comparable
  to one taken on a cool board. Active cooling before trusting anything here.

### Pi 4B measured, 11 Sept 2026 -- retaken on a quiet board

Conditions, because a number without them is not a result: clocks pinned
(`scaling_min_freq == scaling_max_freq == 1800000`, governor `performance`),
`vcgencmd get_throttled` **0x0 for the whole session**, peak 53 C, one method
per process with the board cooled below 58 C before each, 15 reps, 1 tile /
2048 reference keypoints, frame 646x484. Raw:
`results/bench_matchers_pi4.json`.

    method              detect     match     total       p95   two quiet runs
    orb                   55.9      96.1     152.0     155.5       151-152
    akaze                221.3      60.8     282.1     338.8       256-282
    edgepoint2_t32       358.3     109.0     467.3     503.1       467-469
    edgepoint2_s32       422.2     102.1     524.3     676.4       516-524
    sift                 219.6     347.3     566.9     595.2       526-567
    edgepoint2_s64       439.5     127.8     567.3     605.2       567-567
    xfeat_mnn            526.4     137.7     664.1     723.0       657-664
    xfeat_lg             536.2    5690.3    6226.6    6541.2     6227-6257

Three orderings hold, the same direction as the Pi 5 and the Xavier. **XFeat is
slower than SIFT on ARM** -- 664 against 567. **EdgePoint2 beats xfeat_mnn** --
567 against 664. And `edgepoint2_t32` at 467 is now the fastest thing here that
is not ORB, SIFT included.

**The first version of this table was wrong by up to 45%, and the cause was one
daemon.** It was measured while `avahi-daemon` burned ~58% of a core parsing a
multicast flood -- 162 of 249 packets/s inbound on the wifi were multicast, and
the daemon was doing real syscall work on every one of them, not spinning. It
came back at full CPU after both a restart and a reboot because the cause was
the network, not the process. Disabling it dropped idle loadavg from ~2.3 to
~1.2 and moved every row:

    method            contended     quiet    delta
    orb                   164.7     152.0     -7.7%
    akaze                 250.1     282.1    +12.8%
    sift                  690.5     566.9    -17.9%
    edgepoint2_s32        722.1     524.3    -27.4%
    edgepoint2_s64        892.7     567.3    -36.5%
    edgepoint2_t32        844.3     467.3    -44.7%
    xfeat_mnn            1109.3     664.1    -40.1%
    xfeat_lg             9048.2    6226.6    -31.2%

`prior_contended_total_ms` in the JSON keeps the old number so the size of that
error stays visible. **Nothing about the contended table was thermal** --
`get_throttled` read 0x0 through both sessions. A pinned clock and a cool board
were necessary and not sufficient; the third check is that nothing else is
running, and this file's own `loadavg` warning fired on every contended run and
was, correctly, believed.

**The spread inverted, which is the part worth remembering.** Contended, the
torch methods moved up to 40% run-to-run and the OpenCV ones about 5%. Quiet,
the torch methods repeat to under 2% -- `edgepoint2_s64` lands on 566.9 and
567.3 -- while `akaze` (10.4%) and `sift` (7.8%) are now the loosest rows.
Sustained multi-threaded work is what contention damages, and once contention
is gone it is also what averages out best; a 150-280 ms single-threaded run is
the one left exposed to scheduler jitter. Read a wide spread on a short OpenCV
row as noise, and a wide spread on a long torch row as a busy board.

Two traps this session paid for again, both already written down above and both
still worth the reminder:

- **The clock pin does not survive a reboot.** The board rebooted mid-session
  and came back `ondemand`, 600000-1800000. Re-pin before every timing run.
- **An attempt was thrown away at 81.3 C** with `throttled=0x80000` and a
  loadavg of 4.00 on 4 cores before rep 1. Bit 19 is sticky until reboot, so
  once it fires nothing measured afterwards on that boot is comparable.

One harness trap, new: **`--tiles 1` is not a unique key.** The env80 sweep
leaves its own 1-tile stores in `stores/`, so a benchmark loop filtering only on
tile count silently measured `satellite_map__*` alongside `ref_tile__*` and,
because the outputs were keyed by method name, the second run of each method
overwrote the first. Filter on the store name, and key outputs by store.

### The Pi 4B end-to-end run: 1833 ms, and the matcher is all of it

`configs/system.yaml`, edgepoint2_s64, 25 tiles / 51200 reference keypoints,
60 frames, three layers over the real bus, `fc.enabled` false throughout.
58 fixes, 58 accepted, median error 0.007 m -- which measures plumbing, not
localization, because this config cuts its frames out of the reference map.

    median latency   1833.0 ms          p95   2555.9 ms

    stage_ms, median          match            923.5
                              detect_frame     734.9
                              ransac            20.9
                              tiles_fitted      11.0
                              rectify            7.9
                              decode             4.2
                              load_reference     3.1
                              sum             1705.6

**The architecture costs 47 ms of 1833.** Detect and match are 1658 of the
1706 summed stage milliseconds. That is the same conclusion the Xavier reached
at 12 ms of 250, on a board seven times slower: the bus, the rectifier, the
tile selection and the solve are not what is missing the budget, and no amount
of work on them moves this number. Against ArduPilot's 250 ms this board is
7.3x over, so a Pi 4B does not fly this pipeline at edgepoint2_s64 / 25 tiles
-- it runs it, scores it, and exports it, which is what a bench board is for.

### env80 on the Pi 4B: the accuracy transfers, the latency does not

All twelve combinations, `results/env80_sweep_pi4/`. Scene_09 and Scene_10,
satellite reference, cold start, gate applied afterwards, same modules as the
live system with the bus removed.

    Scene_09        n  plaus   medLat | acc@12  med_m  p90_m  p99_m   max_m
    orb           134   0.03    748.8 |   0.01  12.81  18.28  19.51   19.64
    akaze         134   0.02    941.6 |   0.00      -      -      -       -
    sift          134   0.06   4866.8 |   0.00      -      -      -       -
    xfeat_mnn     134   0.47   3036.8 |   0.29   2.67   4.49   5.93    5.96
    edgepoint2_s64 134  0.30   3141.4 |   0.14   2.51   2.97   3.39    3.43
    xfeat_lg      134   0.10  60819.4 |   0.08   3.21   4.93   7.37    7.64

    Scene_10        n  plaus   medLat | acc@12  med_m  p90_m  p99_m   max_m
    orb           192   0.00     59.2 |   0.00      -      -      -       -
    akaze         192   0.03     69.8 |   0.01   4.56   4.88   4.95    4.96
    sift          192   0.00    189.7 |   0.00      -      -      -       -
    xfeat_mnn     192   0.19    306.3 |   0.17   3.48   4.73   6.54    6.75
    edgepoint2_s64 192  0.42    260.2 |   0.34   3.79   4.68   6.03    6.62
    xfeat_lg      192   0.36   2686.1 |   0.36   3.74   4.60   5.77    5.80

**The plausible rate matches the laptop on nine of ten shared rows exactly**,
and the tenth is one frame (orb on Scene_09, 0.03 against 0.04). Errors are
geometry, so this was the expected outcome -- but it is the check worth having,
because it says the board is running the same pipeline rather than a subtly
different one, and it is the reason the accuracy columns above can be read as
results rather than as this board's results.

Latency is 8.6x to 39.4x the laptop, and the ratio is not a constant: sift is
the worst at 39.4x, xfeat_lg the mildest at 9.5-13.4x. Do not scale a laptop
number by one factor to predict this board.

Two things this adds that the older sweep could not say:

- **EdgePoint2 is the only matcher that works on both scenes.** At gate 12 it
  accepts 0.14 and 0.34 where xfeat_mnn does 0.29 and 0.17 -- xfeat_mnn is
  strong on Scene_09 and weak on Scene_10, and EdgePoint2 is the one that does
  not collapse on either. Its tail on Scene_09 is also the tightest of any
  method, p99 3.39 and a worst case of 3.43 m, against xfeat_mnn's 5.93/5.96
  and xfeat_lg's 7.37/7.64.
- **xfeat_lg is not usable here and Scene_09 is why.** 60.8 s per frame median,
  a p95 of 65.8 s, for a gate-12 accept rate of 0.08. On Scene_10 the same
  matcher costs 2.7 s and accepts 0.36. That is the quadratic-in-keypoints cost
  landing on the scene with more matches, which is the same shape as the
  "xfeat_lg costs the most exactly when it fails" finding above, now with a
  second scene to contrast against.

The gate is still an open question rather than a settled one: these are the
gate-12 columns, and the CSVs carry gates 0-60 per frame for whoever wants to
argue the trade between EdgePoint2's accept rate and its tail.

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

### The Pi 5 wins every matcher except EdgePoint2, and that one reverses

Filled in on the Pi 5, 8 Sept 2026 (`results/bench_matchers_pi5.json`),
`performance` governor, idle board, `vcgencmd get_throttled` 0x0 at 51.6 C.
Same geometry as the table above: 1 tile, 2048 reference keypoints, 646x484
frame, p95 of 15 reps.

    method            Pi detect   Pi match   Pi total   Xavier total
    orb                    27.6       29.6       57.2           96.6
    sift                   85.0       68.2      153.2          197.5
    akaze                  57.3       18.0       75.3          157.1
    xfeat_mnn             176.1       30.6      206.7          291.9
    edgepoint2_s64        183.5       89.1      272.5          223.9   <-- reversed

The prediction in the handoff -- "if the 1.4-2.0x per-core gap holds,
edgepoint2's detection lands near 110 ms" -- was wrong, and wrong in an
instructive way. **Detection barely moved** (183.5 against ~203, only 1.1x)
and **matching went the wrong way by 4.3x** (89.1 against 20.7). The board that
loses on every other matcher wins this one.

The split is the explanation, not the total. Detection is a CNN forward pass
that does not thread -- the Xavier's own numbers say 8 threads buys 1.26x on
detect and 1.57x on match -- so on detection the two boards are close and the
A76's per-clock edge is mostly spent. Matching is a 2048x2048 matmul over 64-D
descriptors, which threads well, and there the Xavier's eight cores finally pay
for themselves against the Pi's four.

**On the same board, edgepoint2's match costs 3x xfeat_mnn's** (89.1 against
30.6). Both emit exactly 2048 frame keypoints with 64-D descriptors on this
frame -- checked, not assumed -- so the problem size is identical and the only
difference is which form of the mutual-NN reduction they take. XFeat's matcher
always pays for a second GEMM (`xfeat/modules/xfeat.py:328`);
`EdgePoint2Method.match` takes `cossim.max(dim=0)` below `WIDE_REF`. That is
the entire delta.

### WIDE_REF is the wrong SHAPE of model, not just the wrong number

Swept on the Xavier, 8 Sept 2026 (`scripts/wide_ref_sweep.py`,
`results/wide_ref_xavier*.json`), synthetic L2-normalised descriptors, Q=2048,
dim 64, median of 7:

    N (ref kp)   one-matmul   two-matmul   winner
          1024        17.17        16.55   two
          2048        23.76        20.30   two
          4096        49.95        27.88   two
          8192        87.99        56.80   two
         12000        50.12        68.33   one
         16000        71.50        87.37   one
         20000        74.14       107.46   one
         25600       209.17       140.34   two
         32768       357.36       168.51   two
         51200       427.20       272.03   two

**one-matmul wins only inside a window**, roughly N=12000-20000, and loses on
both sides of it. A single threshold cannot express that, so no value of
`WIDE_REF` is correct. The window sits in the same place at Q=1024 and Q=4096,
and both forms return identical matches (checked at N=4096).

The component breakdown says the window is not a cache curve -- it is torch
picking different reduction kernels by shape:

    N        gemm   max(dim=1)   max(dim=0)   2nd gemm
    8192    23.77         1.96        66.04      24.64
    12000   31.87         4.19        15.03      30.85
    16000   39.28         3.20        24.05      38.71
    20000   47.83         3.83        23.67      50.63
    25600   69.83         8.56       162.95      66.86

GEMM is smooth and linear. `max(dim=1)` is cheap and smooth. **`max(dim=0)` is
the whole story and it swings 15 -> 163 ms, faster on 94 MB than on 64 MB.**
Reproduced ascending and descending, so it is not measurement order. Chunking
the reduction (1k and 4k columns) is worse at every size, and `argmax` instead
of `max` is not reliably better -- both were tried.

So the practical choice is between an erratic path and a predictable one:

- **one-matmul** is 1.45x better inside the window and up to 1.5x worse
  outside it, with a 2x cliff between N=20000 and N=25600.
- **two-matmul** is monotonic in N at every Q measured. Never catastrophic.

The pipeline sits exactly on the cliff. At 10 tiles (20480 refs, the observed
median) one-matmul costs ~76 ms against two-matmul's ~112; at 13 tiles (26624)
it costs ~214 against ~148. **The prior's tile count decides which side of a 2x
discontinuity each frame lands on**, which is the worst possible property for a
latency budget, and it is invisible in a median.

### Applied: `WIDE_REF` is gone, the matcher always takes the second GEMM

The Pi 5 curve came back (`results/wide_ref_pi5.json`, torch 2.10.0+cpu, 4
threads) and settled it. Two-matmul wins 8 of 10 sizes there, and the margins
are not close:

    N        Pi one    Pi two   ratio  |  Xav one   Xav two   ratio
    1024      23.38      7.80    3.00  |    17.17     16.55    1.04
    2048      52.27     15.33    3.41  |    23.76     20.30    1.17
    8192     264.29     84.18    3.14  |    87.99     56.80    1.55
    16000    310.03    149.49    2.07  |    71.50     87.37    0.82
    25600    965.50    231.94    4.16  |   209.17    140.34    1.49
    32768   2019.59    443.59    4.55  |   357.36    168.51    2.12
    51200   1947.04    484.30    4.02  |   427.20    272.03    1.57

Same non-monotonic signature, four times more violent. The two sizes where
one-matmul edges ahead on the Pi (12000, 20000) sit between neighbours at 264
and 310 ms; they are the same kernel-dispatch artifact, not a regime to build
on.

**Then the real store contradicted the synthetic sweep, and the real store
wins.** Re-measured on `ref_tile__edgepoint2_s64`, actual descriptors, actual
frame, Xavier at MAXN -- matches identical at every size:

    tiles  ref kp   max(dim=0)   2nd GEMM    delta
        1    2048         21.1       19.5     -1.6
        4    8192         85.9       51.5    -34.4
        9   18432        166.4      123.2    -43.3
       10   20480        226.7      113.5   -113.2
       13   26624        224.7      153.3    -71.4
       25   51200        448.3      332.1   -116.1

**Re-taken pinned** (8 Sept, `sudo jetson_clocks`, min == max == 2265600,
8 cores, MAXN, load 0.76, 55.0 -> 59.5 C over the run, 9 reps) -- the numbers
above were unpinned and these supersede them:

    tiles  ref kp   max(dim=0)   2nd GEMM    delta   speedup
        1    2048         22.5       17.9     -4.6     1.26x
        4    8192         86.4       46.0    -40.4     1.88x
        9   18432        159.8      106.4    -53.4     1.50x
       10   20480        213.9      119.7    -94.2     1.79x
       13   26624        252.4      134.1   -118.3     1.88x
       25   51200        414.3      276.3   -138.0     1.50x

Same conclusion, cleaner: 1.26-1.88x at every geometry, matches identical
everywhere. Two-matmul is faster at **every** geometry, including the 9-10 tile
band where the synthetic sweep predicted it would lose by 36 ms. Synthetic descriptors
have the right shape and dtype but not the real store's memory layout, and at
these sizes that is what the reduction kernel is reacting to. **Trust the store
measurement over the synthetic one** -- the sweep script is still the right
tool for spotting the discontinuity, but not for choosing the operating point.

So `WIDE_REF` is deleted rather than retuned, and `EdgePoint2Method.match`
always pays for the second GEMM, as XFeat's own matcher does.

End to end on the Xavier replay the change is small -- match 145.8 -> 142.4 ms
median, latency 409.4 -> 403.3, accuracy and error unchanged -- because the
pacer keeps the prior warm and the tile count low. That is the honest headline:
**on this board, in this replay, it is a wash.** What it buys is the removal of
a 2x discontinuity that the prior's tile count was choosing at random, and on
the Pi 5 it is worth 2-4x on the match half at every geometry that matters.

### The Pi's confirming run, and why its absolute numbers are not usable

`results/bench_matchers_pi5_postfix.json`, taken after the fix. The match half
did what the curve predicted: `edgepoint2_s64` at 2048 went 89.1 -> 38.9 ms,
and every edgepoint2 row moved the same way (0.34-0.67x).

**But the run is not comparable to the pre-fix one**, and the tell is in the
methods the change cannot touch, because `xfeat` never calls
`EdgePoint2Method.match`:

    method       refkp   detect x   match x
    orb           2048       1.02       1.05
    akaze         2048       1.06       1.11
    sift          2048       1.18       1.13
    xfeat_mnn     2048       1.65       1.69
    xfeat_lg      2048       1.56       1.31

OpenCV barely moved; every torch path is 1.3-1.7x slower. That is not a code
change, it is the board in a different state -- short single-threaded work
rides out a thermal or contention problem that sustained multi-threaded work
does not. The pre-fix run recorded `performance` governor, idle, 0x0 throttled,
51.6 C; the post-fix file recorded no conditions at all, because the script did
not capture any.

Two things follow.

**The fix is bigger than it measured.** Normalising by the torch-path drift
(1.39x, median of the xfeat detect ratios) puts the real improvement at 3.18x,
against the synthetic curve's predicted 3.41x at N=2048. Those agree.

**Within-run comparisons still hold, cross-run ones do not.** On the same run,
`edgepoint2_s64` total is now 252.7 ms against `xfeat_mnn`'s 341.5, where
before the fix it was 272.5 against 206.7. The reversal against xfeat is
genuinely gone. Comparing 252.7 to the Xavier's 223.9 is NOT valid from this
run and needs a pinned re-run.

### `scripts/bench_matchers.py` now records the clock, because this cost a run

It prints governor, pinned state (`scaling_min_freq == scaling_max_freq`),
temperature, load and throttle flags before the table, **warns loudly when the
clocks are not pinned**, and writes `conditions_before` and `conditions_after`
into the JSON so a board that heated up mid-run is visible afterwards.

It caught the Xavier on its first run: `schedutil`, NOT PINNED, 3.5 hours
uptime. `nvpmodel -m 0` had survived the reboot and `jetson_clocks` had not --
exactly the trap already documented under "Board notes", found again because
nothing was checking. Every Xavier number in this session's WIDE_REF work was
taken unpinned. The DIRECTION survives it (both matcher forms were timed
back-to-back in one process, all six deltas have the same sign, and the Pi
agrees independently) but the magnitudes need a pinned re-run before they are
quoted anywhere.

**A number without its clock is not a result.** That is why this is in the
script now rather than in a commit message.

### The first Xavier run with documented conditions, 8 Sept

`results/bench_matchers_xavier_pinned.json`. `jetson_clocks` pinned
(min == max == 2265600), MAXN, 8 cores, torch 2.4.1 / 8 threads, load 1.96 on
8 cores, 50.0 -> 58.5 C, `--tiles 1 --reps 15`. Every earlier Xavier number in
this file was taken either unpinned or under unrecorded load, so **this is the
reference run from here on.**

    matcher           Pi det  Pi mat  Pi tot |  Xav det  Xav mat  Xav tot | winner
    orb                 33.5    34.4    67.9 |     63.6     28.9     92.5 | Pi 1.36x
    sift               103.5    80.2   183.7 |     79.3     41.6    120.9 | Xav 1.52x
    akaze               64.3    17.0    81.3 |    127.7     11.5    139.2 | Pi 1.71x
    xfeat_mnn          253.9    55.7   309.7 |    237.5     28.9    266.4 | Xav 1.16x
    edgepoint2_t32     199.1    39.8   238.9 |    168.8     19.5    188.3 | Xav 1.27x
    edgepoint2_s32     215.7    34.5   250.3 |    188.6     21.0    209.6 | Xav 1.19x
    edgepoint2_s64     237.6    42.8   280.4 |    184.3     26.6    210.9 | Xav 1.33x
    xfeat_lg           312.2  3325.8  3638.0 |    237.8   2143.0   2380.8 | Xav 1.53x

That reads as a reversal of this file's standing claim that "the Xavier is
1.3-2.0x slower than a Pi 5 across every matcher" -- the Xavier wins six of
eight here. **Do not record that as a finding yet, because the two runs are not
comparable and the difference is in the direction that would produce exactly
this result.**

    board    pinned   loadavg / cores   torch threads
    Pi 5     yes      2.90 / 4 = 0.73   4
    Xavier   yes      1.96 / 8 = 0.245  8

**The Pi was three-quarters busy and the Xavier a quarter.** A 3x difference in
contention, on the axis that this file has already twice caught misreading as a
code change, and it hits multi-threaded torch work hardest -- which is exactly
where the Xavier's six wins are. The two rows the Pi still wins are OpenCV
detectors, the least threaded work in the table.

So what is established is narrower than it looks: **the Xavier now has one run
whose conditions are known.** The cross-board comparison needs an idle Pi run
before any of it goes in the baseline table, and until then the Pi 5 baseline
under "Pi 5 baseline to compare against" stays provisional.

Worth being explicit about a limit here too: **a Claude Code session runs on
this board**, and is most of the 1.96. A truly idle Xavier measurement is not
available from inside the session doing the measuring, which is a floor on how
clean any number taken this way can be.

### The Pi's re-run: `performance` is not pinned, and a pinned clock is not enough

The check found the same trap on the other board, in a different disguise. The
Pi 5's `performance` governor left `scaling_min_freq` at 1500000 against a
2400000 max, so `cur_freq` could step down between reps with nothing at the
governor level to say so. **The governor string is not the test;
`scaling_min_freq == scaling_max_freq` is.** Like `jetson_clocks`, it does not
survive a reboot. That means the canonical Pi table under "Pi 5 baseline to
compare against" was taken under conditions nobody verified, and its
cross-board comparisons should be treated as provisional until it is re-taken.

**But the pinned re-run came back SLOWER, and that is the more useful finding:**

    method       refkp   unpinned   pinned   ratio
    orb           2048       57.2     67.9    1.19
    akaze         2048       75.3     81.3    1.08
    sift          2048      153.2    183.7    1.20
    xfeat_mnn     2048      206.7    309.7    1.50
    xfeat_lg      2048     2551.3   3638.0    1.43

Pinning at maximum frequency cannot make a board slower, so the clock is not
what changed. The conditions block says what did: **`loadavg` 2.90 before the
run and 6.01 after, on four cores.** The board was three-quarters busy with
something else before the first rep, and torch's four threads were contending
for cores that were already taken. Clock was pinned, temperature 49.6 -> 57.3 C,
`throttled` 0x0 -- both existing checks passed cleanly.

The signature to recognise: **OpenCV methods drift 1.08-1.20x while every torch
method drifts 1.43-1.50x.** Short, barely-threaded work rides out contention
that sustained multi-threaded work does not. It looks exactly like a code
regression in the torch path and it is not one.

`bench_matchers.py` now warns when `loadavg` exceeds a quarter of the core
count before the run starts. Three checks: pinned, cool, idle. The Pi's
post-fix numbers still need one more re-take on an idle board before any of
them are compared to the Xavier -- the within-run conclusion (edgepoint2_s64
at 280.4 ms now beats xfeat_mnn's 309.7, so the reversal is gone) holds
regardless, because both saw the same contention.

### The Pi 5's own pinning trap: `performance` governor is not pinned

Found the same day, on this board, by the same check. `scaling_governor` was
already `performance` -- the state `bootstrap.sh`-adjacent instructions leave
it in -- and `bench_matchers.py` still printed `NOT PINNED`, because the
governor and the frequency range are two different knobs. `performance` only
means the governor requests the top of whatever range `scaling_min_freq` /
`scaling_max_freq` allow; this board's range was still `1500000-2400000`, so
`cur_freq` sat at 2400000 under load but was free to step down the instant
load dropped, and a benchmark that idles between reps (model load, store
open) can retime mid-run without ever showing up as a governor change.

Fix is the range, not the governor:

    echo 2400000 | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq

`scaling_min_freq == scaling_max_freq` is what the script's `pinned` check
actually tests, and it is the correct test -- checking the governor string
alone would have kept passing throughout this trap. Confirmed pinned, re-ran,
same result held (`edgepoint2_s64` 280.4 ms against `xfeat_mnn`'s 309.7 ms at
2048 ref keypoints): the reversal fix does not depend on this bug, but the
absolute numbers in `results/bench_matchers_pi5_postfix.json` do, and the file
now carries `conditions_before`/`conditions_after` showing `pinned: true`.

**This does not survive a reboot**, same as `jetson_clocks` on the Xavier --
`scaling_min_freq` resets to the hardware default range on boot, so re-pin
before trusting any timing run on this board, not just once per session.

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

### `fps: auto` -- so no board needs the number at all

Written 8 Sept 2026, once the Pi 5 had confirmed the same non-monotonic shape
and there were two hand-tuned constants to keep in sync. `geoanchor/data_layer/
pacer.py`; `fps: auto` is now the default in `configs/system.yaml`.

The rule is one line: **publish one frame every (median `stage_ms` x
`fps_margin`)**, so the consumer waits on an empty queue rather than the frame
waiting in it. Default margin 1.15, clamped to `fps_bounds` [0.5, 10].

Two decisions in it are load-bearing:

- **It paces off `sum(stage_ms)`, not the fix interval.** The interval is the
  obvious signal and it is poisoned: it is partly set by how fast we publish,
  so pacing off it feeds its own output back in -- publish slower, fixes arrive
  slower, conclude the board got slower, publish slower still, converge on the
  floor. `stage_ms` is the consumer's intrinsic cost and does not move when the
  publish rate does (379-388 ms across the 2, 4 and 8 fps arms), which is the
  property a control input needs. Persisting `stage_ms` turned out to matter
  for more than post-hoc analysis.
- **It starts at the LOWER bound and adapts up.** The first fix on a cold
  process pays the model load -- 3.2 s against a 0.41 s steady state -- so the
  first three are discarded, and starting at the top would spend the entire
  warmup in exactly the regime the pacer exists to avoid.

Median over a 15-fix window, 10% hysteresis, so one 7 s `xfeat_lg` failure or a
cold tile load does not re-rate the feed. The data layer takes a read-only
`Subscriber` on the processing layer's socket with a zero timeout: a processing
layer that dies costs the feed nothing, it just stops being re-rated. Layer
independence is unchanged.

Measured against the two pinned rates it replaces, same board, same 100 s:

    config          fixes/s   compute   latency   staleness   acc%    err_m
    pinned fps 2       2.01     379.0     390.6        12.1   82.2   0.0057
    pinned fps 4       2.60     370.0     506.5       136.0   77.9   0.0057
    auto               2.12     387.4     409.4        10.9   83.9   0.0057

It converges 0.5 -> 1.91 -> 2.40 fps within about fifteen fixes and holds.
Against the hand-tuned 2 it gives up 19 ms of median latency and takes back 5%
of the fix rate and the lowest staleness of the three; against 4 it is 97 ms
better. The point is not that it wins either column outright -- it is that the
number is now derived on whatever board is running, and the Xavier's 2 and the
Pi's 3.2 stop being two constants somebody has to remember to re-measure.

`fps: auto` is rejected on a `uvc` or `rtsp` feed (`DLE-14`) because there the
number is what the CAMERA is asked to produce, and pacing it down discards
frames at the sensor rather than at the queue -- the opposite of what a
conflating consumer wants. Subsampling on read is the right answer there and is
not written. Pin a number to benchmark; `scripts/feed_fps_sweep.py` reproduces
the table above.

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

## OVERHEAD_MS, measured at last: 45.7 ms

8 Sept 2026. Logitech C270 on the Xavier's USB, `sudo jetson_clocks` pinned,
1280x720 MJPG, n=200 through the real `UvcFeed` and `Preprocessor`:

    capture_age_ms    med  37.20   p95  41.20    V4L2 buffer stamp -> read() returns
    preprocess_ms     med   4.20   p95   5.05    rescale to GSD + JPEG encode
    bus                     1.22         1.57    measured separately, 53 KiB payload
    decode                  3.10         3.40    cv2.imdecode in the processing layer
    ------------------------------------------
    OVERHEAD_MS       med  45.7    p95  51.3

**The 40 ms assumption was close, and slightly optimistic.** The number that
mattered was 88 ms -- above it `xfeat_cpu` misses on the Pi 5, below it fits.
51.3 ms p95 is comfortably below, so **that conclusion survives contact with a
real camera**, which was not guaranteed.

Capture dominates it: 37 of the 45 ms, and nothing in this repo can shorten
that. Preprocess, transport and decode together are 8.5 ms.

Three limits on the number, all of which make it a floor rather than a ceiling:

- **`capture_age_ms` starts at the V4L2 buffer timestamp, not at exposure.**
  On a UVC webcam that stamp is closer to "the host finished receiving the
  frame" than to "the sensor started integrating". True sensor-to-matcher is
  higher by the exposure and the USB transfer, which this cannot see.
- **The MAVLink hop is not in it.** The flight-controller link was disabled for
  this run (`OL-15`), so this is capture-to-matcher, which is the half the
  matcher's budget is measured against. Add the hop separately for the fusion
  timestamp.
- **A C270 is not the flight camera.** This is the right order of magnitude for
  a USB camera on this board, not the number for whatever ships.

### What else the camera run showed

The full three-layer stack ran on live video for 90 s: 1264 frames published,
193 fixes attempted, **0 accepted -- which is correct**, and `configs/
camera.yaml` says so in its header. The camera is looking at a room and the
reference is a tile of Sydney.

- **MJPG is not a preference, restated with this camera.** 1280x720 YUYV
  negotiates 7.5 fps and delivers 3.7; MJPG negotiates 30 and delivers 11.9.
  640x480 MJPG delivers 14.9. The config already says MJPG; this is the second
  camera to confirm it.
- **`fx_px` is the blocker, exactly as item 2 predicted.** With no intrinsics
  the preprocessor cannot scale to the reference GSD (`DLE-13`, `DL-13 scale
  None`), so the processing layer sees a 1280x720 frame at the wrong scale and
  rejects every one on `PLE-08` (`scale 1.849 is more than 0.35 from 1.0`).
  Nothing downstream of calibration can be tested on this rig until that is
  done.
- `PLE-09` reports 1240 ms over budget, which is not news: with no accepted fix
  there is never a prior, so every frame searches 25/25 tiles with `xfeat_mnn`
  against 51200 keypoints. That is the cold-search cost, not the deployed one.
- The C270's MJPG stream emits `Corrupt JPEG data: N extraneous bytes before
  marker` on most frames. Cosmetic -- every frame decodes and the shapes are
  right -- but it is noisy in logs and is the camera, not the code.

### The Pi 5's OVERHEAD_MS is 3x the Xavier's, and it is the camera, not the board

Measured 8 Sept 2026, same model camera as the Xavier run above -- a Logitech
C270, `/dev/video0`, 1280x720 MJPG -- clocks pinned (`performance`,
`scaling_min_freq == scaling_max_freq`), n=200:

    stage           median      p95     Xavier median
    capture         123.58   127.61          37.20
    preprocess        7.92    12.45           4.20
    encode             0.01    0.22            n/a (bus below covers it)
    bus                0.24    0.77            1.22
    ------------ --------- --------
    OVERHEAD_MS    132.08   138.75           45.7

**~3x the Xavier's number, and every millisecond of the gap is in `capture`.**
Preprocess, encode and bus are the same order of magnitude or better -- this
is the Pi, after all, and it wins every other CPU-bound stage in this project.

`capture` in `measure_overhead.py` is not a fixed device latency: it is the
time `UvcFeed.read()` blocks inside its poll loop, and that loop returns only
when the background grabber thread has a new frame, so it is bounded below by
1/(achieved fps), not by anything the Pi's CPU does. Confirmed directly,
independent of this codebase, with `v4l2-ctl -d /dev/video0
--set-fmt-video=width=1280,height=720,pixelformat=MJPG --stream-mmap
--stream-count=60 --stream-to=/dev/null`: the raw driver climbed 1.65 -> 7.26
-> 12.74 fps as auto-exposure settled under this room's lighting, which lands
on the same ~8 fps average (123.58 ms is 1000/8.1) the measured run saw. The
USB link itself is not the limit -- `lsusb -t` shows the C270 on a dedicated
480M port, not sharing a hub with something slow.

**So this is the same finding CLAUDE.md already has for a different camera
run** ("MJPG negotiates 30 and delivers 11.9") **restated with numbers**:
a C270 under indoor lighting does not deliver anywhere near its negotiated
rate, and `OVERHEAD_MS` inherits that directly because the pipeline can only
run as fast as frames arrive. Better lighting, fixed exposure/gain, or a
different sensor would move this number a lot; nothing in `geoanchor/` would
need to change to capture the improvement.

**This changes what fits in the 250 ms budget, on this rig, today.** At
132 ms median overhead, only about 118 ms is left for detect + match + solve
-- not the 204 ms the Xavier's number implied. Against the pinned matcher
totals in this file's tables, only `orb` (66-68 ms) and `akaze` (81-88 ms)
currently fit; `edgepoint2_s64` and `xfeat_mnn` (both ~250-310 ms total) do
not, on this camera, in this room, right now. This is a statement about this
capture rig, not about the Pi 5's compute -- the matcher tables above were
taken with a synthetic feed at whatever fps the demo video specifies, and
never depended on this camera at all.

**A C270 is not the flight camera, restated:** treat 132 ms as the right
order of magnitude for a webcam on a desk, not as this project's OVERHEAD_MS
going forward. Re-measure with the actual flight sensor before this number is
used to accept or reject a matcher.

---

## Calibrating without a printer: put the board on a screen

`scripts/make_chessboard.py` writes `docs/chessboard.png`; display it
full-screen on a laptop, monitor or phone. `scripts/calibrate_camera.py`
captures from the camera, auto-banks distinct views, calibrates and prints the
intrinsics, with `--apply configs/camera.yaml` to write them in.

A screen is a *better* target than a printed page, not a fallback: it is
genuinely flat, where a taped print bows, and its geometry is exact.

**No ruler is needed either, and that is the non-obvious part.**
`cv2.calibrateCamera` returns fx in PIXELS, and fx is invariant to the assumed
physical square size -- scale every object point by k and the solved
translation scales by k while fx does not move. Verified numerically before
relying on it: synthesised 20 views from a known K, then calibrated assuming
square sizes of 0.025, 1.0 and 137.0, and all three returned
`fx=1150.000 fy=1148.000 cx=632.00 cy=361.00` against a truth of exactly that.
So the script assumes 1.0 and never asks. Only the extrinsics would need a real
measurement, and nothing here uses them.

Three things that will ruin it:

- **Calibrating at the wrong resolution.** fx scales with image width, so a
  640x480 calibration is wrong by 2x for a 1280x720 pipeline. The script
  defaults to the data layer's 1280x720 MJPG, records the size it actually
  negotiated, and `--apply` refuses to write into a config whose capture size
  disagrees.
- **Too few oblique views.** fx and Z trade off against each other in a frontal
  view and only tilt separates them. The script rejects a view whose corners
  have not moved far enough from one already banked, and warns if fx and fy end
  up more than 5% apart, which is the usual symptom.
- **Moire.** Keep the camera far enough back that squares are comfortably more
  than ~20 px across, or the screen's pixel grid beats against the sensor's and
  walks the detected corners around.

The generated board was checked to detect flat and under three oblique warps
(54/54 corners each), so a detection failure is the display or the geometry,
not the pattern.

### The first run failed, and the way it failed is the thing to know

8 Sept, 20 views, C270 at 1280x720: `fx = 47308.3` on a 1280 px frame. That is
an implied **1.55 degree** horizontal field of view for a webcam that has about
sixty, with `fy` 23% away from `fx` and radial distortion terms reaching 3e8.

**A planar target cannot separate focal length from distance in a frontal
view.** Twice as far with twice the focal length produces an identical image.
Only perspective breaks the tie -- the near edge of the board subtending more
pixels than the far edge. Bank twenty views by sliding the camera around
parallel to the screen and every one is the same degenerate observation, so the
solver runs fx off toward infinity and hides the residual in the distortion
coefficients. It does not complain while doing it.

Three defects in the first version of the script, all now fixed:

- **The novelty test measured the wrong thing.** "Mean corner displacement > 40
  px" is satisfied by pure translation, which carries no information about fx.
  It now measures signed foreshortening -- `log(top edge / bottom edge)` and
  `log(left / right)`, which needs no intrinsics -- and **refuses to finish
  until it has at least `views/8` frames tilted in each of the four
  directions**, printing which ones are still missing while you move.
- **The per-view error was understated 7.35x.** `cv2.norm(..., NORM_L2)` is
  already the square root of the sum of squares, so dividing by N rather than
  sqrt(N) turned 0.89 px views into 0.12 px ones and hid the bad fit.
- **Nothing checked the result before writing it.** `--apply` put fx = 47308
  straight into `configs/camera.yaml`, over a comment explaining why that field
  was deliberately empty. There is now a sanity gate on fx/width, fx-vs-fy and
  rms, and a failing calibration writes nothing anywhere.

**The sanity gate alone is not sufficient, and that is worth knowing.**
Simulated 20 near-frontal views from a known `fx = 1150`: the solve returns
**3584.70, an error of 212%**, and passes every sanity check -- fx/width is
2.80, rms is 0.17. Only the tilt-coverage requirement catches it (0 views in
all four directions). Simulated 20 tilted views return fx = 1150.00 exactly.
So the capture-side requirement is the real defence and the output-side gate is
just the backstop.

`results/camera_intrinsics_FAILED_degenerate.json` is kept as the worked
example.

### It passed on the fourth attempt: fx = 1421.48

C270 at 1280x720 MJPG, chessboard on a screen, 704 detections, 20 views banked:

    rms reprojection error   0.2568 px    (per-view 0.148 / 0.206 / 0.544)
    fx_px  1421.48    fy_px  1421.03      0.03% apart
    cx_px   630.01    cy_px   345.72      against a centre of 640.0, 360.0
    dist   [0.00875, 0.80241, 0.00051, 0.00097, -2.79954]
    tilt coverage: up, down, left, right all satisfied

`fx/width` is 1.11, a 47.8 degree horizontal field of view, which is right for
a C270 in 16:9. GSD at 75 m AGL is 5.28 cm/px against the reference tile's
12.38, so frames downscale about 2.34x.

**The scale path still cannot run on the bench, and that is correct.** GSD =
altitude / fx_px, and `camera.yaml` has `gps: source: none`, so with no
altitude the preprocessor falls back to a fixed long edge (`DLE-13`) and every
frame is rejected on `PLE-08`. Verified that the intrinsics themselves are
live: the preprocessor reports `fx_px = 1421.48` and, given 75 m, returns
scale 0.426 with no warning. Altitude is now the open input, not calibration.

### The second failure was the opposite problem, and needed the opposite fix

With tilt coverage enforced, the next run came back **well conditioned and
still rejected**: `fx = 1451.75`, `fy = 1474.27` -- 1.6% apart, `fx/width`
1.13, an implied 47.8 degree field of view, which is right for a C270 in 16:9.
Coverage was met in all four directions. The only failing check was
`rms = 2.795`.

The per-view errors say why, and it is not a conditioning problem at all:

    8.931  7.633  2.624  2.237  1.978  1.753  1.648  1.568  1.296  1.183  1.076
    0.964  0.885  0.841  0.786  0.757  0.715  0.664  0.627  0.515  0.409  0.368

Twenty views under 2.7 px and two at 8.9 and 7.6. **rms is over POINTS, so a
couple of ruined views dominate it** -- the 8.9 px view alone outweighs the
twelve best combined. The two usual causes are motion blur (the board is found
in a blurred frame perfectly well, and `cornerSubPix` then localises smeared
corners confidently and wrongly) and the chessboard's 180-degree ordering
ambiguity, which pairs corners with the wrong object points and is invisible
in any single view.

Both are fixed the same way, and the script now does three things about it:

- **Drops the worst view and refits, repeatedly,** until rms meets the target
  or the set falls to `max(8, views/2)`. It stops there rather than dropping
  until the target is met at any cost, because a calibration fitted to six
  hand-picked views is a calibration fitted to its own residuals. Validated on
  a synthetic set of 22 views with two deliberately spoiled: rms 3.646 ->
  0.000 and fx 1144.09 -> 1150.00 against a truth of exactly 1150, dropping
  precisely the two bad ones.
- **Refuses blurred frames at capture** (variance of the Laplacian) and
  **requires the board to be held still** -- two consecutive detections within
  2 px. The stationarity test is the more direct of the two, since auto-capture
  while the camera is still moving is what produces the blur.
- **Saves the corners into the JSON**, so a failed run can be refitted offline
  instead of re-shot. Both earlier failures had to be re-shot only because this
  was not saved.

---

## Open, in order

1. **Get an altitude source onto the live-camera rig.** With intrinsics
   measured, this is the only remaining input the scale path lacks: GSD =
   altitude / fx_px, and `configs/camera.yaml` runs `gps: source: none`, so
   the preprocessor still falls back to a fixed long edge (`DLE-13`) and every
   frame is rejected on `PLE-08`. On a bench that is correct behaviour, not a
   fault. A static test altitude, or a rangefinder, or the FC's own AGL.
2. ~~**Measure `OVERHEAD_MS`**~~ -- done, 45.7 ms median / 51.3 p95. See above.
3. ~~**Calibrate the camera**~~ -- done 8 Sept, C270 at 1280x720:
   `fx_px 1421.48`, `fy_px 1421.03`, `cx 630.01`, `cy 345.72`, rms 0.2568 px
   over 20 views. In `configs/camera.yaml` and `results/camera_intrinsics.json`.
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
