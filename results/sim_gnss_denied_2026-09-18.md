# GNSS denied: the fix holds the estimator, and without it the estimator leaves

18 Sept 2026. The first measurement in this project of the thing the project is
actually about. Everything before it showed fixes being *fused* — `fused: True`,
`cs_aux_gpos: True`, the covariance arriving intact — but it showed that
**alongside a healthy GPS**, and an estimator with good GNSS looks excellent
whether or not the vision fix contributes anything at all.

Reproduce, against a live `scripts/sim_fixedwing.sh`:

```bash
.venv/bin/python scripts/sim_gnss_denied.py --endpoint udpout:127.0.0.1:14570
.venv/bin/python scripts/sim_gnss_denied.py --endpoint udpout:127.0.0.1:14572 --no-vision
```

## The result

PX4 v1.16.2 SITL, fixed wing, 500 m orbit at ~80 m AGL over Griffith,
`edgepoint2_s64` against a reference capture 2.2 years older than the ground.
`EKF2_GPS_CTRL` set to 0 for 180 s. Error is against **Gazebo's own pose**, not
against MAVLink — see below, this is the whole design of the test.

| | baseline, GNSS on | GNSS denied, 180 s | drift growth |
|---|---|---|---|
| **vision aiding ON** | median 1.39 m | median **7.78 m**, p90 25.25 | +1.40 m/min |
| **vision aiding OFF** (control) | median 0.70 m | median **204.14 m**, p90 367.68 | **+119.91 m/min** |

**The qualitative difference is the result, not the 26x.** With the fix the
error stays bounded in single-digit metres for three minutes. Without it the
error grows without bound — 1.5 m at 5 s, 102 m at 60 s, 396 m at 180 s, and
still climbing when the window closed. That is what an absolute position source
does and what dead reckoning cannot: the drift has no mechanism to come back.

The control's numbers are not a criticism of PX4's estimator. A fixed wing with
a good IMU and no absolute reference is *supposed* to do that.

### It was run twice, and the repeat corrected an over-claim

The first vision run gave median 8.35 m, p90 26.95, and a drift growth of
**−1.49 m/min** — shrinking. Written up alone that reads as the estimator
actively converging. The repeat gave median 7.78 m, p90 25.25, growth
**+1.40 m/min**. The medians and p90s agree closely; the *sign of the trend
does not*. So the honest claim is **bounded and roughly flat over three
minutes**, not improving, and the first run's negative slope was noise in a
window too short to carry a trend.

## The excursions are the interesting part, and they are not dead reckoning

Each vision run contains **exactly one excursion window** and is otherwise very
tight:

| run | excursion window | max inside it | median / p90 / max with it removed |
|---|---|---|---|
| 1 | 33.5–38.4 s (4.9 s) | 214.97 m | 8.19 / 24.89 / **40.86** m |
| 2 | 145.5–159.3 s (13.8 s) | 542.33 m | 7.20 / 22.05 / **27.17** m |

The control has **none** — its error is a smooth monotonic ramp.

Outside its one excursion the vision-aided estimator never exceeds 41 m in
three minutes, so the p90 of 25 m is carrying the excursion's shoulders rather
than describing normal behaviour.

**REFUTED THE SAME DAY.** The paragraph that stood here argued the excursions
could not be gaps in aiding — that 542 m in 13.8 s is impossible by dead
reckoning at the control's 120 m/min (which gives 28 m), so the estimate must
have been *pulled* by a wrong fix passing the inlier gate. It called that the
strongest argument yet for the rejection half of the finding, and flagged it
unproven.

It was measured, and it is wrong. See
[`excursions_are_starvation_2026-09-18.md`](excursions_are_starvation_2026-09-18.md).
Across 300 s of denial with the fix stream tapped and scored against Gazebo
truth: **953 accepted fixes, median error 3.4 m, worst 24.0 m.** No bad fix
ever reached the estimator. What collapses is the number of fixes the pipeline
*emits* — 15 in 60 s where a healthy window emits 172 in 30 — while the
acceptance ratio stays at 87–100%. Ten gaps exceeded AGP's 5 s timeout and the
largest drove the error at **+117.3 m/min against the control's measured
+119.91 m/min**. The estimator is coasting, at exactly the rate coasting
produces.

The bad reasoning is worth keeping: the premise was right and the conclusion
did not follow, because it assumed the only alternatives were "coasting" and
"poisoned". A gap can simply be far longer than the excursion window measured
at a 5x-median threshold — the 53.6 s gap here produced a 43.8 s window.

This **strengthens** the rejection finding rather than undermining it: the gate
and the plausibility check ahead of it kept every bad fix out across five
minutes with no GNSS. And it moves the binding constraint from correctness to
**availability** — under denial the question is not how good a fix is but how
long since the last one.

## With GNSS available, the fix makes the estimate slightly worse

Baseline medians, GNSS on: **1.39 and 2.52 m** across the two vision runs
against the control's **0.70 m**. The control's baseline was taken with the
bridge already stopped, so that is a clean comparison rather than an artefact.

The simulated GPS is better than this pipeline, so fusing a noisier absolute
source into an already-healthy solution costs a little accuracy. That is the
expected and correct behaviour of a filter, and it is the argument for treating
the fix as a **fallback aid source that earns its place when GNSS degrades**,
rather than as something to run permanently alongside a good one. It also means
any demonstration that leaves GNSS on cannot show the fix helping — only that
it is not hurting much.

## Truth must come from Gazebo, and this is the trap the test is built around

With GNSS denied, `GLOBAL_POSITION_INT` and `vehicle_global_position` **are the
estimate** — and the estimate is being driven by the very fixes under test.
Scoring against them asks the fix how well it agrees with itself and returns a
beautiful number that means nothing. `scripts/sim_gnss_denied.py` subscribes to
the simulator's `/world/<world>/dynamic_pose/info` instead.

The comparison is done in **local metres, never lat/lon**, because both sides
already speak it and a geodetic conversion is one more place to be quietly
wrong:

    Gazebo pose is ENU about the world origin        north = y, east = x
    PX4 LOCAL_POSITION_NED is about the EKF origin   north = x, east = y

and `sim_fixedwing.sh` sets `PX4_HOME` from the world's own
`spherical_coordinates`, so the two origins are the same point. The baseline
phase exists to prove that: 0.70–2.52 m of standing disagreement with GNSS on,
against errors of hundreds of metres in the measurement, so the frames agree
well enough for the result not to rest on them.

**`--no-vision` is not optional.** "The estimate held for 180 s" is not
evidence the vision fix did anything until the same aircraft, same flight, same
denial has been shown to lose it.

## PX4 locks each MAVLink instance to its first peer, permanently

This cost most of the time spent on the experiment and is worth writing down.
Every UDP MAVLink instance PX4 starts binds to whichever peer speaks first and
**never releases it**, so after a run has been up for a while:

- the pipeline owns the onboard instance,
- `px4_set_params.py` owns the spare one and has already exited,
- `sim_fly.py` owned the GCS one and has already exited,

and a tool attached afterwards gets `no heartbeat` from all of them while the
autopilot is perfectly healthy — `px4-listener` on the uORB console still works
fine, which is how you can tell the difference. Worse, **each probe burns an
instance**, and PX4 caps at six:

    ERROR [mavlink] Maximum MAVLink instance count of 6 reached.

The way through is to start a fresh instance and let the real tool be its first
client, never a test connection:

```bash
px4-mavlink start -x -u 14570 -o 14571 -r 200000 -m onboard
```

and `px4-mavlink stop -u <port>` recycles a burnt one.

## What this does not show

- **Not a flight-worthiness result.** 180 s of denial in a simulator with a
  flat ground plane, no wind, no rolling shutter and no motion blur.
- **Not an accuracy result for the method** — see
  `results/sim_fixedwing_2026-09-17.md` on why this rig produces no
  catastrophic fixes and real satellite runs do.
- **The aircraft was loitering**, not transiting. A straight leg over unseen
  ground is a harder and more realistic test of whether the fix keeps arriving.
- **180 s is short.** The control had not levelled off; the vision run's bound
  is established over three minutes, not a sortie.
