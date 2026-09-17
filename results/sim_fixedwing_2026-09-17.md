# The whole system, closed loop, on a fixed wing in Gazebo

17 Sept 2026, Legion Pro 5. First run in this project where a camera on a
moving aircraft produces frames that become fixes that a flight controller's
estimator actually fuses, with nothing stubbed anywhere in between.

Reproduce: `bash scripts/sim_fixedwing.sh`. Full setup in `sim/README.md`.

## Rig

| | |
|---|---|
| aircraft | PX4 v1.16.2 SITL, `rc_cessna` fixed wing, nadir `mono_cam` |
| simulator | Gazebo Harmonic 8.15.0, real-time factor 1.0 |
| flight | 400 m box then unlimited loiter, 72-94 m AGL |
| ground texture | Esri Wayback **2026-08-05** over Griffith NSW, 0.4926 m/px |
| reference map | Esri Wayback **2024-06-06**, same ground, **2.2 years older** |
| matcher | `edgepoint2_s64`, 2048 keypoints, inlier gate 8 |
| covariance | `gate_only`, fixed sigma 8.0 m |
| into PX4 | Aux Global Position over uXRCE-DDS |

**The ground and the reference are different captures.** That is enforced in
the runner, and it makes this a genuine cross-date test as well as a
system test.

## Result

1766 frames published, **481 accepted fixes (27%)**.

    error   median      6.60 m
            p90         7.86 m
            p99        11.97 m
            max        14.82 m
            within  5 m  16%
            within 10 m  98%
            within 20 m 100%

    inliers median 75, range 13-190
    latency median 63.2 ms, p95 97.6 ms   (budget 250 ms)

**No catastrophes.** The worst fix of 481 is 14.82 m. Every satellite run on
real AnyVisLoc data in this project contains fixes wrong by hundreds of metres
and some by more than a kilometre; this one has none. That is a statement about
the simulator, not about the method — a rendered view of a flat textured plane
has no relief displacement, no cloud, no seasonal change and no exposure
difference — and it is the reason the median here must never be compared with
the env80 table in `CLAUDE.md`.

**Latency clears the budget with 150 ms to spare**, on a laptop with a
precomputed reference feature store. The 250 ms figure that matters is still
the one measured on the target board.

## The closed loop actually closes

    estimator_aid_src_aux_global_position
        observation_variance  [64.00000, 64.00000]     <- 8.0 m sigma, squared
        innovation            [11.57368, 8.04162]
        test_ratio            [0.23251, 0.11225]       <- gate is 3 sigma
        innovation_rejected   False
        fused                 True

    estimator_status_flags
        cs_aux_gpos           True     (sustained over 6 samples, 4 s apart)

`observation_variance` being exactly the square of the sigma the pipeline
emitted is the proof the covariance survived the whole path rather than being
replaced by a default somewhere in it.

**The acceptance rate is the thing to watch, not the error.** 27% of frames at
~10 fps published is about 2.7 Hz of accepted fixes, which clears AGP's 5 s
timeout comfortably. Earlier in the same run, while the aircraft was still
transiting, acceptance was 6.3% — about 0.63 Hz — and `cs_aux_gpos` dropped to
False between fixes. The aid source is only continuous while the gate is
passing often enough, so a rejection streak walks the estimator toward timeout
exactly as `CLAUDE.md` warns for ArduPilot's `posTestRatio`.

## What this run does not show

- **Not a cross-date result to quote.** The pipeline matched a rendered view of
  one capture against another capture. The tile-to-tile probe in
  `results/crossdate_cities_2026-09-17.md` is the cross-date measurement; this
  is a system test that happens to use two dates.
- **Not an accuracy result for the method.** See "no catastrophes" above.
- **Not a board result.** Laptop timings.
- **GNSS was never denied.** The fix went in as an independent absolute source
  alongside a healthy GPS. Denying it is `scripts/px4_sitl.sh`.
- **The learned covariance estimator was not used.** `gate_only` at a fixed
  8.0 m. Both backends are selectable from the dashboard; on the evidence in
  `results/covariance_postgate_2026-09-17.md` the learned one would have
  supplied magnitudes rather than a useful ranking.

## Six faults this rig exposed, all real and all fixed

Each looked like something other than what it was, which is why they are worth
recording.

1. **ROS 2's vendored `gz` shim shadows the real binary.** PX4 polls for the
   world with a bare `gz topic -l`, gets "I cannot find any available 'gz'
   command", and waits forever against a world that is up and publishing.
2. **A generated world that declares a plugin list gets only what it declares.**
   Omitting `gz-sim-magnetometer-system` surfaces as PX4's
   `Preflight Fail: Found 0 compass`, which reads like a model fault.
3. **`PYTHONPATH` to reach Gazebo's APT bindings breaks the dashboard.** It
   goes ahead of the venv, the system `typing_extensions` shadows the venv's,
   pydantic dies on `cannot import name 'Sentinel'` and fastapi never imports.
   Appending to `sys.path` inside the feed fixes it and asks nothing of callers.
4. **A live feed returning `None` for "nothing new yet" means end-of-file to a
   data layer written for finite feeds.** The run stopped after one published
   frame with a cheerful "end of feed".
5. **`udpin` on both ends is a deadlock.** pymavlink cannot transmit on a
   listening socket until a packet reveals a peer; PX4 does not know where to
   send until something contacts it. No traffic, no error. And PX4 locks to one
   peer per instance, so the parameter script on the pipeline's port stole its
   stream and then exited.
6. **`fx_px` from the config belonged to a different camera.** 1200 against
   mono_cam's real 540, so every frame was rescaled by 0.14 instead of 0.31,
   leaving ~176 px of image and under 30 keypoints. The runner now reads the
   intrinsics off the `camera_info` topic instead of trusting the config.

And one that was not a bug: **five inliers against the tile the ground is
literally textured with** meant the diagnostic crop was centred on the tile
while the aircraft was 794 m away. Centred correctly it gave 268 inliers
against that tile and 96 across the 2.2-year gap. The aircraft was 794 m away
because `autocontinue` was set on the unlimited loiter, so PX4 walked through
it into the landing approach.
