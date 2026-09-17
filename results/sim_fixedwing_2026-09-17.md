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
| flight | 400 m box, then a 500 m-radius orbit over the map centre, 72-94 m AGL |
| ground texture | Esri Wayback **2026-08-05** over Griffith NSW, 0.4926 m/px |
| reference map | Esri Wayback **2024-06-06**, same ground, **2.2 years older** |
| matcher | `edgepoint2_s64`, 2048 keypoints, inlier gate 8 |
| covariance | `gate_only`, fixed sigma 8.0 m |
| into PX4 | Aux Global Position over uXRCE-DDS |

**The ground and the reference are different captures.** That is enforced in
the runner, and it makes this a genuine cross-date test as well as a
system test.

## Result

Properly configured — 500 m orbit, no landing item, no failsafe firing —
1012 frames published, **139 accepted fixes (14%)**:

    error   median      3.35 m
            p90         6.32 m
            p99         9.89 m
            max        10.30 m
            within  5 m  73%
            within 10 m  99%
            within 20 m 100%

    inliers median 86, range 9-128
    latency median 97.6 ms, p95 159.8 ms   (budget 250 ms)

**No catastrophes.** The worst fix of 139 is 10.30 m. Every satellite run on
real AnyVisLoc data in this project contains fixes wrong by hundreds of metres
and some by more than a kilometre; this one has none. That is a statement about
the simulator, not about the method — a rendered view of a flat textured plane
has no relief displacement, no cloud, no seasonal change and no exposure
difference — and it is the reason the median here must never be compared with
the env80 table in `CLAUDE.md`.

**Latency clears the budget**, on a laptop with a precomputed reference feature
store and several hours of these runs already behind it. The 250 ms figure that
matters is still the one measured on the target board.

### The flight path is worth more than any other single knob

An earlier configuration of the *same rig, same matcher, same tiles* gave
2986 frames, 638 accepted (21%), median **7.02** m, max 14.82 m. It differed
only in where and how the aircraft flew: a wide accidental orbit around a
landing point 750 m south, over the industrial fringe. Fixing the flight path
halved the median and took the worst fix from 14.82 m to 10.30 m, while
*lowering* the acceptance rate — because the corrected orbit also crosses the
commercial centre, which is the hard ground. See "where it flies" below.

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

**The acceptance rate is the thing to watch, not the error.** 21% of frames at
~10 fps published is about 2 Hz of accepted fixes. That clears AGP's 5 s
timeout on average — but only on average: `cs_aux_gpos` was sampled True six
times running at one point in the loiter and False at another, because
acceptance is bursty and a rejection streak longer than 5 s drops the aid
source. Early in the same run, while the aircraft was still transiting,
acceptance was 6.3% — about 0.63 Hz — and it dropped out regularly.

**So the useful output of this rig is not the median error, it is the
continuity of the aid source.** A rejection streak walks the estimator toward
timeout while data is still arriving on time, exactly as `CLAUDE.md` warns for
ArduPilot's `posTestRatio`. Measuring the distribution of gaps between accepted
fixes, against AGP's 5 s and ArduPilot's 7 s, is the experiment this rig now
makes possible and the obvious next one to run.

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
against that tile and 96 across the 2.2-year gap.

## Where it flies changes the answer more than anything else does

Measured after the run above, one live frame against 800 px crops of both tiles
at two positions **inside the same 2.17 km tile**, plain ORB:

| position | vs the ground's own imagery | vs the 2.2-year-older reference |
|---|---|---|
| commercial centre | 270 | **30** |
| industrial fringe, 800 m south | 268 | **96** |

The control column is flat, so the camera, projection, rectification and GSD
scaling are equally correct in both places. The cross-date column is not: the
commercial core is **3.2x harder** across 2.2 years than the industrial fringe
800 m away. Cars, awnings, street trees and a market change; big roofs and
yards do not.

That is the land-cover effect from
`results/crossdate_cities_2026-09-17.md` reappearing **within a single tile**,
at a scale far below the one that study compares. It also means an acceptance
rate from this rig is a statement about the flight path as much as about the
method — parking the aircraft on the town centre, which an unlucky loiter did,
takes acceptance to zero while every upstream stage is working perfectly.

**So the orbit radius is a camera parameter and a sampling parameter at once.**
A coordinated turn banks at `tan(phi) = v^2 / (g * r)` and the camera is
rigidly mounted, so the radius sets how far off nadir it looks: at 20 m/s, 80 m
gives 27 degrees, 120 m gives 19, 500 m gives 4.7. A steeply oblique view does
not relate to a north-up orthorectified map by the near-affine homography the
solve expects — 387 matches collapsed to 5 inliers with the solved scale 3-9x
off. And a tight orbit samples one patch of ground. `NAV_LOITER_RAD` is set to
500 m for both reasons; `DO_REPOSITION`'s radius argument is advisory and
`AUTO.LOITER` takes its radius from the parameter.

## The loiter, and a fix that did not work

The aircraft was 794 m from the origin because it ran past the unlimited loiter
into the landing approach that exists only to satisfy the feasibility checker.
Setting `autocontinue` to 0 on that item is the documented way to make a
mission stop, and **on this PX4 it did not**: sampled five times over a minute,
`seq_current` stayed at 8 — the landing item — with the aircraft orbiting
730-790 m south at 93 m.

It was caught only because a background check printed a position that
contradicted a claim already written down. **Nothing in the fix statistics
showed it**: 750 m is still well inside a 2168 m tile, so the camera was over
mapped ground the whole time and the numbers above were produced under it. A
flight-path bug that keeps the aircraft over the map is invisible to every
measurement this rig makes.

The working version does not rely on mission-item semantics at all:
`MAV_CMD_DO_REPOSITION` to the map centre followed by `AUTO.LOITER`, which
holds a commanded point regardless of what the mission thinks it is doing.
Sent as `COMMAND_INT` — the `COMMAND_LONG` form carries lat/lon in float32
fields, which quantises a position at this latitude to about a third of a
metre.
