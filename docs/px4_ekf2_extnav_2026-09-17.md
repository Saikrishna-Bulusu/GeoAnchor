# PX4 takes our fixes, and it wants the opposite of everything ArduPilot wants

Measured 17 Sept 2026 on the Legion against **PX4 v1.16.2** in SITL, using
simulation-in-hardware (`PX4_SIMULATOR=sihsim`, airframe 10040) so the run needs
no Gazebo. The estimator, the MAVLink handler and the parameter set are the real
ones; the flight dynamics are not, and do not matter to any question here.

Reproduce the whole thing with one command:

```bash
bash scripts/px4_sitl.sh
```

It boots PX4, applies `configs/px4_extnav.params`, reboots so the
`reboot_required` parameters take, runs `check_extnav.py`, then runs
`watch_extnav.py` with GNSS disabled and 20 m injected.

The ArduPilot counterpart is `results/sitl_ardupilot_2026-09-17.md`. Read both:
the differences between them are the finding.

---

## The headline

**EKF2 fuses our ExternalNav fixes, but only above 5 Hz — and ArduPilot has no
rate floor at all.**

That is a reversal of this project's standing conclusion. `docs/
ardupilot_extnav_limits_2026-09-01.md` established that EKF3 has *no*
minimum-rate check on the ExternalNav path, that the floor is implied only by
timeouts (1 Hz for `dead_reckoning`, 1/7 Hz for `posTimeout`), and therefore
that **rate is easy and latency is the binding constraint**. On PX4 rate is a
hard gate, and it sits above what most of this pipeline's measured
configurations achieve.

Swept with a **fresh PX4 boot per rate**, 20 m injected north, 25 s per arm.
The 4–20 Hz band was then repeated twice more, because one sample per rate
cannot tell a floor from a flake — and the first pass contained exactly such a
flake:

| inject rate | interval | pass 1 | pass 2 | pass 3 | pass 4 | fused |
|---|---|---|---|---|---|---|
| 2 Hz | 500 ms | 0.00 m | — | — | — | no |
| 3 Hz | 333 ms | −0.01 m | — | — | — | no |
| 4 Hz | 250 ms | 0.00 m | 0.00 m | −0.01 m | 0.00 m | **no, 4/4** |
| **5 Hz** | **200 ms** | 19.74 | 19.96 | 19.50 | 20.28 | **yes, 4/4** |
| **6 Hz** | **166 ms** | 20.47 | 19.92 | 20.27 | 19.97 | **yes, 4/4** |
| **10 Hz** | **100 ms** | *−0.01* | 20.28 | 19.92 | 19.53 | **yes, 3/4** |
| **20 Hz** | **50 ms** | 19.90 | 19.92 | 20.26 | 19.92 | **yes, 4/4** |

Four passes of 4 Hz all fail. Twelve of twelve arms at or above 5 Hz fuse, the
single exception being pass 1's 10 Hz.

**Pass 1's 10 Hz failure did not reproduce and was noise.** It is left in the
table rather than deleted, because a single anomalous cell is exactly what
would have been written up as "the floor is 5 Hz and it also breaks above
10 Hz" had the band not been repeated. Everything at or above 5 Hz fuses;
everything below it does not.

5 Hz is 200 ms, the constant itself, and it works because real send jitter puts
most intervals just under. **Do not design to 5 Hz** — it is the boundary, not
a margin.

The boundary is not empirical guesswork — it is a named constant:

```c
// src/modules/ekf2/EKF/common.h:71
static constexpr uint64_t EV_MAX_INTERVAL = 200e3;   // microseconds
```

and it gates *starting* the aid source, not just using a sample:

```c
// src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp:57
const bool starting_conditions_passing = quality_sufficient
        && ((ev_sample.time_us - _ev_sample_prev.time_us) < EV_MAX_INTERVAL)
        && ...
```

**The failure is completely silent.** At 4 Hz the aid source is still computed
and published every frame, and every indicator on it looks healthy:

```
estimator_aid_src_ev_pos
    observation_variance: [4.13338, 4.13339]
    innovation:  [0.05828, -0.00480]
    test_ratio:  [0.00003, 0.00000]
    innovation_rejected: False
    fused: False          <-- the only field that says anything is wrong
    time_last_fuse: 0
```

`innovation_rejected: False` with `fused: False` is the signature. Nothing is
logged, no status flag is raised, and `cs_ev_pos` simply stays false.

### Why this matters more than it looks

Measured end-to-end rates for this pipeline, from `CLAUDE.md`:

| configuration | latency | rate | clears PX4's 5 Hz? |
|---|---|---|---|
| Pi 5, `xfeat_cpu`, warm store | 202 ms | 5.0 Hz | marginal |
| Xavier, `edgepoint2_s64`, replay | 409 ms | 2.4 Hz | **no** |
| Pi 5, `xfeat_lighterglue`, warm | 2149 ms | 0.47 Hz | **no** |

So on the boards this project actually measures, **PX4 would silently ignore
the pipeline in most configurations that ArduPilot accepts without complaint.**
A fix stream that EKF3 treats as merely slow, EKF2 treats as nonexistent.

Two ways out, neither free, both worth stating rather than discovering later:

- **Publish faster than you solve.** Nothing requires the ExternalNav rate to
  equal the fix rate; re-sending the most recent fix at 6 Hz with its original
  capture timestamp keeps the aid source alive. That is a real change in what
  the filter is being told, though — it turns one measurement into several, and
  the innovation gate will see the same observation repeatedly.
- **Treat PX4 as requiring ≥ 6 Hz and choose the matcher accordingly.** The
  `top_k` front (`results/topk_front.json`) already says `edgepoint2_s64`
  k=2048 is the only point inside 250 ms with a useful accept rate.

This has not been decided here. It needs `fps: auto`'s pacer and the
ExternalNav publish rate to be separated first, which they currently are not.

---

## Three encoding differences, all silent in both directions

Every one was verified against PX4 source and then against a live EKF2.
`scripts/test_fc_encoding.py` now decodes our own output both ways and asserts
all of it (23 checks).

### 1. The covariance layout

ArduPilot collapses the 21-float pose covariance into one scalar:

```c
// GCS_Common.cpp
posErr = sqrtf(cov[0] + cov[6] + cov[11]);
```

so a radial sigma is split as `sigma^2 / 2` per axis to make `posErr` come out
at `sigma`. PX4 never sums:

```c
// EKF2.cpp:2265
ev_data.position_var(0) = fmaxf(evp_noise_var, ev_odom_pos_var(0));
ev_data.position_var(1) = fmaxf(evp_noise_var, ev_odom_pos_var(1));
```

It takes `cov[0]` and `cov[6]` as the X and Y variances directly. The ArduPilot
split therefore hands PX4 `sigma / sqrt(2)` — a covariance **29% too tight**,
which is the dangerous direction: it tells the filter to trust a bad fix *more*
than the estimator said to. `fcout.py` now selects the layout from `firmware`,
and the invariant held across both is that **the per-axis sigma the filter
applies equals the `sigma_m` handed to `send()`**.

Confirmed inside PX4's own uORB, sigma 2.0 m in:

```
vehicle_visual_odometry.position_variance: [4.00000, 4.00000, 0.00000]
```

### 2. The frame

ArduPilot accepts **only** `MAV_FRAME_LOCAL_FRD` (20) and drops `LOCAL_NED`
without a word. PX4 accepts both at the MAVLink layer — and then EKF2 treats
them completely differently:

```c
// ev_pos_control.cpp:67
case PositionFrame::LOCAL_FRAME_NED:
    if (yaw_align) { pos = ev_sample.pos - pos_offset_earth; }   // as given
case PositionFrame::LOCAL_FRAME_FRD:
    if (!ev_yaw) {
        const Dcmf R_ev_to_ekf = Dcmf(_ev_q_error_filt.getState());
        pos = R_ev_to_ekf * ev_sample.pos - pos_offset_earth;    // ROTATED
        pos_cov(i, i) = math::max(pos_cov(i, i), orientation_var_max);
    }
```

An FRD sample from a source that does not also claim yaw is rotated by an
*estimated* EV-to-EKF rotation. This pipeline sends an identity quaternion and
never claims yaw, so that rotation tracks the difference between "no attitude"
and the vehicle's real attitude, and it turns an absolute georeferenced position
into nonsense. Measured, 20 m north injected:

```
vehicle_visual_odometry     position:    [19.97460, 0.0, 0.0]     correct
estimator_aid_src_ev_pos    observation: [-0.036795, -0.139136]   after rotation
```

The same branch also raises our covariance to the orientation variance,
discarding the calibrated sigma this project exists to produce. So the frame is
firmware-dependent too: **ArduPilot LOCAL_FRD, PX4 LOCAL_NED.**

### 3. An INT32 parameter goes on the wire as its bit pattern

```c
// mavlink_parameters.cpp:135
param_set(param, &(set.param_value));
```

the *address* of the float field, handed to a function that reads four bytes as
`int32`. A `PARAM_SET` carrying float `1.0` stores **1065353216**, and PX4
acknowledges it. ArduPilot sends and accepts everything as `REAL32` and does
not behave this way.

This cost a run by **cancelling out**: the set stored 1065353216, the read-back
was decoded as a raw float and came back as `1.0`, and the check printed `ok` —
while `EKF2_EV_CTRL`'s bit 0, the bit that switches external vision on, was
*clear*, because 1065353216 is even. Both directions now honour `param_type`.

---

## The parameters

`configs/px4_extnav.params`, read from PX4 v1.16.2's own metadata rather than a
wiki. The ones that matter most:

| parameter | value | why |
|---|---|---|
| `EKF2_EV_CTRL` | 1 | bitmask, bit 0 = horizontal position only. **Defaults to 0** — external vision is off until set |
| `EKF2_EV_NOISE_MD` | 0 | use the variance in *our* message. 1 discards it entirely |
| `EKF2_EVP_NOISE` | 0.05 | lower bound on our sigma. **There is no upper clamp**, unlike ArduPilot's 100 m |
| `EKF2_HGT_REF` | 0 | baro. **Defaults to 1 (GPS)**, wrong for a GNSS-denied vehicle |
| `EKF2_EV_DELAY` | 50 | ms. Range 0–300 here, where `VISO_DELAY_MS` stops at 250 |

Two are `reboot_required` and are accepted, stored and ignored until the next
boot, which is why `px4_sitl.sh` boots twice.

**PX4 gives the covariance estimator more range than ArduPilot does.** EKF3
clamps horizontal position variance to [0.01, 100] m; `ev_pos_control.cpp:145`
only floors it, at `max(cov, EKF2_EVP_NOISE^2, 0.01^2)`. Anything above the
floor is taken as given, however large. That sharpens the finding in `CLAUDE.md`
that NGPS Eq. 7's saturation at 10.00 m wastes filter input: on PX4 it wastes
even more of it.

---

## Four rig-level traps, each of which cost a run here

- **PX4's UDP instances lock their remote to the FIRST peer and never
  re-target.** A second client on the same port — even after the first has
  closed — gets nothing back and times out against a healthy vehicle. PX4 does
  stream continuously to its configured remote (14540) regardless, so listening
  and writing are now separate endpoints (`--write-endpoint`).
- **The write link wears the vehicle's own system id** (1, 197) — correct for a
  companion computer, and it means PX4 will not route telemetry back to it.
  `watch_extnav.py` observes on a second socket as 255.
- **PX4 ignores `REQUEST_DATA_STREAM`**; it wants `SET_MESSAGE_INTERVAL`.
  Asking the ArduPilot way is silent and surfaced as "the EKF has no origin
  yet" against a vehicle with a perfectly good position.
- **`watch_extnav.py` disabled GNSS and left it disabled.** PX4 autosaves a
  changed parameter, so `EKF2_GPS_CTRL = 0` persisted into `parameters.bson`
  and the *next* boot came up permanently GNSS-denied — never reaching "Ready
  for takeoff", never taking an origin, and failing this very test with "SITL
  never got GPS lock". It now reads the original value and restores it in a
  `finally` block, and confirms the restore.

---

## MAVLink, not uXRCE-DDS — decided, not defaulted

`fcout.py` already speaks `ODOMETRY` and `VISION_POSITION_ESTIMATE`, and after
this work every byte of it is verified against both firmwares by
`scripts/test_fc_encoding.py`. DDS is the modern PX4 path and is what NGPS uses
through ROS 2, but it is a second write path nothing in this repo speaks, and
it would put a ROS stack on the board — which is precisely the thing this
runtime avoids (`CLAUDE.md`: "one fewer stack on the board and one fewer thing
to install on JetPack 5").

Revisit only if something needs DDS for its own sake. Nothing here does.

---

## What this does NOT cover

SITL shares PX4's estimator, MAVLink handler and parameter set, but not the
serial link, the timing or the camera — the same limit as the ArduPilot run.
And the rate finding above was measured by *injecting* at a fixed rate, not by
running the real pipeline at its natural rate; the two differ in that a real
fix stream also has jitter, which at 5 Hz sits exactly on the boundary.
