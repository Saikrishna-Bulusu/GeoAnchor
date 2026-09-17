# GeoAnchor: methods and results as of 17 Sept 2026

GNSS-denied absolute visual localization for a UAV at 50–100 m AGL. A
down-facing camera frame is matched against a georeferenced map, producing a
position fix with a calibrated covariance that is fed to the flight
controller's EKF over MAVLink.

This assembles the numbers that exist. Where a question is open it says so
rather than filling the cell. **No mean and no RMSE appear anywhere**: a
satellite run in this project can contain a fix wrong by 2.06e93 m, because a
homography solve can converge on a degenerate solution that still passes an
`inliers > 0` check, and one such row destroys both statistics. Median, p90,
p99 and the fraction inside 5/10/20 m throughout.

---

## 1. The contribution

**Rejection and covariance are different problems, and different methods win
each.** Measured on 1323 fixes, four scenes, leave-one-scene-out:

| | rejection AUC | covariance in metres |
|---|---|---|
| inlier-count threshold | **0.724 — wins** | cannot produce one |
| NGPS Eq. 7 + Eq. 8 | 0.467 | 2.4× understated, saturates at 10 m |
| learned estimator | 0.641 | **tracks: 12.0→12.0, 21→21, 38→45** |

1. For **rejection**, a raw inlier threshold beats everything learned. An
   honest negative result: it is free and needs no training.
2. For **covariance**, only a learned estimator works. A threshold outputs a
   bit. Eq. 7 outputs metres that are wrong by 2.4× and **saturate at 10.00 m**
   (its clip floor; 31% of rows pin there) while actual errors reach 168 m.
3. **Eq. 7 is descriptor-specific.** It beats a threshold on the pairing it was
   tuned for (SP_LG 0.661 vs 0.622) and collapses on XFeat (0.506). Its
   reprojection term carries correct-sign signal on SuperPoint and **inverts on
   XFeat**.

The sentence: *a system that swaps matchers — which any embedded deployment
must, to fit the compute budget — silently inherits a covariance model that no
longer works.* So: **threshold to reject, learned estimator to set the
covariance on what survives.**

---

## 2. Matcher accuracy on real data

AnyVisLoc env80, 326 frames (Scene_09 n=134, Scene_10 n=192), satellite
reference, cold start, gates evaluated post hoc across 0–60.
`results/env80_sweep/summary.json`.

| matcher | plausible S09 | plausible S10 | gate-10 accept S09 | median m S09 | latency med S09 |
|---|---|---|---|---|---|
| `xfeat_mnn` | **46.3%** | 18.8% | 34.3% | 2.63 | 161 ms |
| `xfeat_lg` | 10.4% | **35.9%** | 8.2% | 3.21 | 4539 ms |
| `sift` | 6.0% | 0.0% | 0.0% | — | 124 ms |
| `orb` | 3.7% | 0.0% | 2.2% | 19.64 | 40 ms |
| `akaze` | 2.2% | 3.1% | 0.0% | — | 64 ms |

**The classical baselines are not viable against real satellite reference, and
that is the cleanest argument in the project for a learned detector.**
`results/classical_baselines_env80.txt`: across 326 frames ORB, SIFT and AKAZE
return **zero usable fixes** — a fix being usable meaning accepted at some gate
*and* inside 20 m. ORB is 7.2× cheaper per fix than `xfeat_mnn` at 0.63 J and
its joules-per-*usable*-fix is infinite. SIFT's one qualifying gate has a median
error of 126.6 m: confident fixes on the wrong building. Match count is not a
quality signal.

**The two scenes disagree on which XFeat wins** (S09 favours `mnn` 4.4:1 on
plausible rate, S10 reverses it 1.9:1) and that is unexplained. "`xfeat_mnn`
wins" is a claim about Scene_09.

### EdgePoint2 changes the tail, and wants a different gate

| | gate | accept | p99 | max |
|---|---|---|---|---|
| `xfeat_mnn` | 7 → 10 | 29.1% → 24.5% | **167.85 → 6.64 m** | 187.42 → 6.75 |
| `edgepoint2_s64` | 6 → 10 | 39.0% → 31.0% | 7.50 → 7.36 m | **8.36 → 7.50** |

XFeat's p99 collapses over three gates; **EdgePoint2 never has that cliff — its
worst error is 8.36 m even at gate 6.** It therefore wants a much lower gate,
and applying XFeat's gate to it throws away a third of its fixes. For a
covariance study, a matcher whose failures are bounded matters more than its
accept rate.

---

## 3. The top_k Pareto front

`results/topk_front.json`, assembled by `scripts/topk_front.py` from sweeps at
k = 512/1024/2048/4096. Each point is evaluated at **that method's own best
gate** — the lowest gate holding p99 inside 10 m — because scoring every method
at one gate compares gate choice rather than matcher. Axes are p95 latency and
accept rate; median error barely moves across k (2.2–3.1 m) while accept rate
moves 5×.

**`xfeat_lg` is dominated at every k on both scenes.** It never reaches the
front. On Scene_09 it costs 1780–9095 ms for 1.5–4.5% accept, against
`edgepoint2_s64` k=2048 at 23.1% for 684 ms. That is stronger than "it does not
fit the budget": there is no budget at which it is the right choice.

**`edgepoint2_s64` owns the entire Scene_10 front**, and k=2048 is the only
configuration that both fits 250 ms (201 ms p95) and accepts usefully (40.1%).

Latency here is the Legion's; the *ordering* transfers across boards, the values
do not. Not measured: `xfeat_lg` at k=4096.

### Why `xfeat_lg` costs the most exactly when it fails

LighterGlue ships `width_confidence: 0.95` — point pruning on. Pruning is
confidence-driven, so a confident match sheds points and runs fast while a
failing one keeps them all. On env80 at k=2048:

| outcome | n | match ms median | matches median |
|---|---|---|---|
| plausible solve | 74 | 767 | 124 |
| failed solve | 252 | **7003** | 4 |

**9.1× slower when the answer is useless**, and 28× over budget while producing
nothing. For a latency-budgeted loop that is the worst possible cost profile.

---

## 4. Boards

Pi 5 canonical N=200, `performance` governor, no throttling. Warm means the
reference tile was precomputed, which is the deployed case:

| matcher | cold p95 | warm p95 | end-to-end | J/fix |
|---|---|---|---|---|
| `orb` | 152.3 | 56.6 | 96.6 | 0.63 |
| `sift` | 420.6 | 98.9 | 138.9 | 2.32 |
| `akaze` | 545.9 | 107.5 | 147.5 | 2.91 |
| `xfeat_cpu` | 1017.4 | 161.6 | **201.6** | 4.51 |
| `xfeat_lighterglue` | 2786.4 | 2108.6 | 2148.6 | 16.05 |

**The reference feature store is what makes the budget.** 856 ms of
`xfeat_cpu`'s 1017 ms was running the detector over the reference tile. The map
is static, so those descriptors are computed once on the ground. *If the tile
ever has to be warped per frame, this whole table dies with it.*

**On ARM, XFeat is slower than SIFT** — 161.6 vs 98.9 ms on the Pi, 291.9 vs
197.5 on the Xavier. XFeat's "faster than SIFT on CPU" claim is an x86 result
with AVX2 and MKL behind it; torch on these boards reports `MKL not found`
while OpenCV's SIFT has hand-tuned NEON. No published embedded-CPU timing for
XFeat exists to compare against; ours is the baseline.

**A number without its clock is not a result.** `scripts/bench_matchers.py`
records governor, pinned state (`scaling_min_freq == scaling_max_freq` — the
governor string is *not* the test), temperature and load, and warns when load
exceeds a quarter of the core count. It caught the Xavier unpinned after a
reboot and the Pi three-quarters busy, each of which looked exactly like a code
regression in the torch path and was not: OpenCV methods drift 1.08–1.20× under
contention while torch methods drift 1.43–1.50×.

---

## 5. Flight stack

### ArduPilot: verified, and rate is not the problem

EKF3 fuses our ExternalNav fixes — GPS disabled at runtime, 20 m injected, the
estimator walked to 19.81 m within 5 s and held.
`results/sitl_ardupilot_2026-09-17.md`.

**There is no minimum-rate check on the ExternalNav path.** The floor is implied
by timeouts: below 1 Hz sets `dead_reckoning`, below 1/7 Hz trips `posTimeout`.
The ceiling is explicit at 50 Hz.

**Latency is the binding constraint.** Delay compensation caps at 250 ms and
overrunning it is *silent*: `writeExtNavData()` does
`MAX(timeStamp_ms, imuDataDelayed.time_ms)`, so a late fix is stamped as current
and fused at the wrong time. At 5 m/s each 100 ms of uncompensated latency
injects 0.5 m. Hence: stamp at **capture**, never at fix completion.

### PX4: it fuses, but it has a rate floor ArduPilot does not

`docs/px4_ekf2_extnav_2026-09-17.md`. EKF2 fuses our fixes — but only at 5 Hz
and above. `EV_MAX_INTERVAL = 200e3` µs (`common.h:71`) gates *starting* the aid
source, and below it the failure is silent: the aid source publishes every frame
with `innovation_rejected: false` and a test ratio of 0.00003, and only
`fused: false` says anything is wrong.

That matters because the measured pipeline sits below it in most
configurations — the Xavier replay is 2.4 Hz, `xfeat_lighterglue` is 0.47 Hz.
**A fix stream EKF3 treats as merely slow, EKF2 treats as nonexistent.**

Three encoding differences, all silent in both directions, all verified against
source and then against a live estimator:

| | ArduPilot | PX4 |
|---|---|---|
| pose covariance | sums `cov[0]+cov[6]+cov[11]` into one scalar | reads `cov[0]`, `cov[6]` as **per-axis** |
| frame | **only** `LOCAL_FRD` (20) | wants `LOCAL_NED`; rotates an FRD sample by an estimated EV→EKF rotation |
| INT32 parameter | plain `REAL32` | the integer's **bit pattern** in the float field |

Using ArduPilot's covariance split on PX4 gives it σ/√2 — **29% too tight**,
the direction that makes the filter trust a bad fix more than the estimator
said to. Using `LOCAL_FRD` on PX4 turned a 20 m injection into an observation
of −0.04 m. `scripts/test_fc_encoding.py` decodes our own output both ways and
asserts all of it.

**PX4 gives the covariance estimator more range**: EKF3 clamps horizontal
variance to [0.01, 100] m; PX4 only floors it and has no upper clamp.

**MAVLink, not uXRCE-DDS**, decided rather than defaulted: `fcout.py` already
speaks both messages and every byte is now verified against both firmwares,
whereas DDS would put a ROS stack on the board for no capability this project
needs.

---

## 6. Reference maps

### Rectification is load-bearing, not a refinement

XFeat is not rotation invariant. On real AnyVisLoc frames against a real
satellite basemap:

| frame | rectify=True | rectify=False |
|---|---|---|
| L09_0001 | 12 inliers, 1.33 m | 6 inliers, **no fix** |
| L09_0002 | 79 inliers, 1.47 m | 8 inliers, 31.66 m |
| L09_0003 | 39 inliers, 2.61 m | 6 inliers, **no fix** |

Stage 02 rectifies the **frame** using vehicle attitude; the tile stays
north-up. Warping the tile instead would void the precomputed feature store and
every latency number in section 4.

### How stale the map may be is a question about the flight area

`results/crossdate_cities_2026-09-17.md`. Four areas, all controls passing:

| area | result |
|---|---|
| **rural farmland (Griffith)** | **works at every gap out to 8.8 years**, 118–443 inliers |
| Brisbane / Perth / Melbourne CBD | not monotonic; 1–4 of 8–12 gaps work, 5–38 inliers |

**The cliff is a property of land cover, not of the matcher.** A city ought to
be the easy case — crops change every season, buildings do not — and it is the
hard one, because what changes between two satellite passes over a CBD is the
apparent geometry of tall structures, not the ground.

Two limits on the earlier Sydney curve, found here:

- **The x-axis is publication date, not acquisition date.** Wayback labels a
  capture with the date Esri *published* a basemap version; a release can
  republish older imagery. That is why three of four areas are non-monotonic.
  `scripts/wayback_source_meta.py` reads the real `SRC_DATE`, `SRC_RES`,
  `SRC_DESC` and `SRC_ACC` from Esri's per-release metadata services.
- **Fixed zoom is not fixed resolution.** Web Mercator scale goes as 1/cos(lat)
  (0.2644 m/px at Brisbane against 0.2356 at Melbourne), and the native source
  differs per area — a 0.31 m WorldView-3 source and a 0.1 m aerial source carry
  very different real detail at the same nominal pixel size.

Within-area results are unaffected by both, which is why the land-cover
conclusion stands.

### The inlier gate is reference-specific

XFeat returns **12–79 inliers** against a real satellite basemap and **200+**
on the Sydney tile, where frames are cut from the reference itself. A gate tuned
on one reference type silently rejects everything on another — structurally the
same failure as Eq. 7 across descriptors.

---

## 7. Architecture

Three layers, three OS processes, one ZeroMQ bus. **The architecture costs
12.3 ms of the 250 ms budget** and the detector spends 92%:

| stage | median | p95 |
|---|---|---|
| decode | 3.1 | 3.4 |
| rectify | 1.8 | 3.1 |
| **detect_frame** | **230.8** | 284.6 |
| load_reference | 1.0 | 22.2 |
| **match** | **143.6** | 172.9 |
| ransac | 15.6 | 22.2 |
| transport (measured separately) | **1.22** | 1.57 |

**Do not optimise the plumbing.** Every millisecond of transport,
serialisation and process boundaries together is 1/17th of one call to
`detect_frame`. The fault isolation the three-process split buys is real and
costs 12 ms.

**Asking for more frames than the board can match makes latency worse, not
throughput better** — and the curve is not monotonic, so `fps: auto` derives
the rate from median `stage_ms` on whatever board is running rather than from
two hand-tuned constants.

---

## 8. What is not yet answered

- **The covariance estimator is not exported.** `processing_layer/covariance.py`
  ships `gate_only`, a fixed placeholder that labels itself as one in every
  record. The `learned` backend is the contribution and the pipeline does not
  yet carry it.
- **`OVERHEAD_MS` is measured but not with a flight camera.** 45.7 ms median on
  the Xavier and 132 ms on the Pi 5 — and the whole gap is `capture`, because a
  C270 under indoor lighting does not deliver its negotiated frame rate. That is
  a statement about a webcam on a desk, not about either board.
- **No altitude source on the live-camera rig**, so the scale path cannot run
  and every frame is rejected on `PLE-08`. Correct behaviour on a bench; the
  last missing input.
- **Scene_09 and Scene_10 disagree on which XFeat wins**, unexplained.
- **`xfeat_lg` at k=4096** is the one hole in the Pareto front.
- **Nothing has flown.** Every number here is SITL, replay or bench.

---

## Hard rules that produced these numbers

- Never quote the mean, never RMSE.
- `PDE`, `R_i`, `pdm_at_k`, `recall_at_k`, `retrieval_gt_rank`, `pred_error`,
  `location_error_list`, `truePos.*` are **ground truth and may never be
  predictors**.
- Cross-validate grouped by **scene**, never by row.
- Exclude `Scene_21` from every estimator number and report it separately — it
  is a matcher failure, not a localisation result.
- Never texture a simulator ground plane with the image used as the reference
  map. The Sydney replay does this deliberately, for bring-up, and its 0.006 m
  is a wiring proof that must never be quoted as accuracy.
- A zero-fix result is indistinguishable from a broken harness without a
  **control**.
- Any descriptor fine-tune uses XFeat, never SuperPoint (Magic Leap licence).
