# SITL, then HITL

How to get the whole thing running against a flight stack, starting on your
laptop with no hardware at all, and ending with a simulator on the laptop
driving the real pipeline on a real board over ethernet.

**Do it in this order.** Every step is cheap to fail and each one removes a
class of bug from the next.

| stage | what runs where | what it proves |
|---|---|---|
| 1. SITL, open loop | everything on the laptop | the MAVLink write path is correct |
| 2. SITL, closed loop | everything on the laptop | the filter accepts our fixes |
| 3. HITL | simulator on laptop, pipeline on board | the board is fast enough |

---

## Verified end to end, 10 Sept 2026

Everything in stage 1 below was run against a real ArduCopter SITL build on the
Legion, not written from the source. What happened:

```
ArduPilot master, shallow clone, ./waf configure --board sitl && ./waf copter
  -> build/sitl/bin/arducopter, 5.8 MB, ~12 min
12/12 ExternalNav parameters present        <- the contrast that condemns the F405 V3
check_extnav.py                             4/4 pass
sitl_openloop.py --seconds 60               92 records, 2 fixes sent, 0 device errors
EKF_STATUS_REPORT posTestRatio              0.0052        <- accepted, gate is 1.0
```

**The fixes were accepted.** `posTestRatio` of 0.0052 is three orders under the
5-sigma rejection gate, and no `OLDE-*` was raised, so the whole write path —
frame, covariance, origin, timestamp — is correct against a real EKF3.

### The thing that surprised me, and it is the important one

After the 60-second session ended, `EKF_STATUS_REPORT` read:

```
pos_horiz_abs   no
pos_horiz_rel   no
pos_vert_abs    yes
attitude        yes
```

**The vehicle had no horizontal position at all.** That is correct behaviour and
it is the sharpest illustration of what `EK3_SRC1_POSXY = 6` means: the EKF is
now depending on *this pipeline* for horizontal position, and when the pipeline
stops feeding it, position is gone within the 7-second `posTimeout`. GPS is no
longer the fallback, because you told it not to be.

So the parameter set is not a configuration detail, it is a commitment. Do not
set `EK3_SRC1_POSXY = 6` on a vehicle you intend to fly until the pipeline is
producing fixes continuously and you have watched it survive a dropout.

### Two bugs in our own tooling, found by running it

Both made the script report a *wrong verdict*, not an error, which is why they
had survived being written down as correct.

- **`check_extnav.py` reported `VISO_*` as "absent on this firmware"** against a
  SITL build that has all of them. It sent `param_request_list`, which streams
  all ~1200 parameters, and the VISO entries did not arrive inside the timeout.
  That is the worst possible failure for this script: "absent on this firmware"
  is exactly the verdict that condemns a board, and it is how you tell an F405
  with visual odometry compiled out from an F7 that has it. It now reads each
  parameter **by name**, three times — a parameter that exists answers in
  milliseconds, one that does not is never answered — so absent now means absent.

- **It waited for `GLOBAL_POSITION_INT` without requesting the stream.**
  ArduPilot sends only what a GCS has asked for, so step 3 reported "the EKF has
  no origin yet" against a vehicle with a 10-satellite RTK-fixed lock and a
  perfectly good position. `FC_AND_HITL_PLAN.md` has said "ArduPilot only
  streams what a GCS has requested" since it was written; the script did not do
  it. It requests the stream now.

**`EKF_STATUS_REPORT` is not in `MAV_DATA_STREAM_EXTENDED_STATUS`** on current
ArduPilot either. Ask for it explicitly:

```bash
python -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:127.0.0.1:14550'); m.wait_heartbeat()
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 200000, 0,0,0,0,0)
while True:
    e = m.recv_match(type='EKF_STATUS_REPORT', blocking=True)
    print(f'posTestRatio {e.pos_horiz_variance:.4f}  flags 0x{e.flags:04x}')
"
```

### Re-verified against the source, not the notes

The clone made it cheap to re-check the constants this project's findings rest
on. All three hold, and one line number has drifted:

| claim | where | status |
|---|---|---|
| `HAL_VISUALODOM_ENABLED` is `HAL_PROGRAM_SIZE_LIMIT_KB > 1024` | `AP_VisualOdom_config.h:7` | confirmed |
| `speedybeef4v3` is `FLASH_SIZE_KB 1024` | `hwdef.dat:16` | confirmed — so `1024 > 1024` is false |
| wrong `frame_id` returns in silence | `GCS_Common.cpp:4097` | confirmed, bare `return`, no warning |
| `posErr = sqrtf(cov[0]+cov[6]+cov[11])`, only `cov[0]` NaN-checked | `GCS_Common.cpp:4106` | confirmed |
| delay compensation clamped to 250 ms | `AP_NavEKF3_core.cpp:86` | confirmed — **notes say :83, upstream moved** |

---

## Why SITL first, always

SITL builds ArduPilot with **nothing trimmed**. Every parameter exists, every
feature is compiled in, and a mistake costs nothing.

That matters here specifically, because the flight controller you own cannot do
this. The SpeedyBee F405 V3 has 1 MB of flash, and ArduPilot gates visual
odometry on `HAL_PROGRAM_SIZE_LIMIT_KB > 1024` — which `1024` is not — so
`VISO_*` parameters do not exist in that firmware at all. Details and the
proof: [`FC_AND_HITL_PLAN.md`](FC_AND_HITL_PLAN.md).

So: develop against SITL, use the F405 V3 to prove the *telemetry* half on real
hardware, and treat closed loop as a SITL-only result until there is a 2 MB
board.

---

## Stage 1 — SITL, open loop

### Install ArduPilot, once

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Then reload your shell (`. ~/.profile`) so `sim_vehicle.py` is on `PATH`. The
first build takes several minutes; after that it is cached.

### Start the simulator

In its own terminal, and leave it running:

```bash
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14550
```

`--out` is the part that matters — it forks a second MAVLink stream for us, so
the console keeps its own link and the two do not fight over the port.

Wait for `EKF3 IMU0 is using GPS` in the console before continuing. Until the
EKF has an origin there is nothing for an external position to be fused against.

### Set the parameters

```bash
bash sitl.sh --set-params
```

This **writes** to the autopilot, which is why it is opt-in and never the
default. Point it at SITL only.

What it writes, and why each one:

```
AHRS_EKF_TYPE   = 3      EKF3
VISO_TYPE       = 1      visual odometry enabled
VISO_DELAY_MS   = 50     REPLACE with your own measured median latency
VISO_POS_X/Y/Z  = 0      camera offset from the IMU, in body frame
EK3_SRC1_POSXY  = 6      horizontal position comes from ExternalNav
EK3_SRC1_POSZ   = 1      altitude stays on the barometer
EK3_SRC1_VELXY  = 0      we send no velocity
EK3_SRC1_VELZ   = 0
EK3_SRC1_YAW    = 1      compass. This pipeline sends an identity
                         quaternion, so do NOT set this to 6.
```

`VISO_DELAY_MS` is the one you must change. Its range is 0–250 and EKF3 clamps
it again internally. Set it from `scripts/measure_overhead.py` plus your median
`latency_ms`, not from the default.

### Run the open-loop session

```bash
bash sitl.sh
```

It checks every parameter is live, then runs a 90-second session sending the
**actual** GPS back over the ExternalNav path. The position cannot mislead the
filter because the vehicle already has it — so anything that breaks is
plumbing, not matching.

Useful flags:

```bash
bash sitl.sh --seconds 300                 # longer run
bash sitl.sh --skip-session                # parameters only
bash sitl.sh --config configs/env80.yaml   # real frames instead of the demo
python scripts/check_extnav.py --endpoint udpin:0.0.0.0:14550
```

### What to watch, and it is not the feed rate

In the SITL console:

```
status EKF_STATUS_REPORT
```

- **`posTestRatio`** is the number that matters. It is the innovation against
  the 5-sigma gate. Under 1.0 means our fixes are being accepted.
- **A fix failing the gate does not refresh `lastGpsPosPassTime_ms`.** So
  sustained rejection walks the filter toward `posTimeout` on a 7-second clock
  *while data is still arriving perfectly on time*. Watching the feed rate will
  not show you this. Watch `posTestRatio`.
- `dead_reckoning` set means you dropped below 1 Hz.

### The three bugs this stage exists to catch

None of them is visible from the sending side. All three were real.

1. **`frame_id = MAV_FRAME_LOCAL_NED` is discarded in silence.** `LOCAL_FRD` is
   **20**, not 1, and `child_frame_id` must be `MAV_FRAME_BODY_FRD` (12).
   `handle_odometry()` returns early with no warning, no status flag and no
   counter. The symptom is an estimator that never sees external navigation —
   indistinguishable from a wiring fault.
2. **One NaN in the covariance poisons everything.** ArduPilot collapses the
   21-element array with `sqrtf(cov[0] + cov[6] + cov[11])` and only checks
   `isnan(cov[0])`. Writing NaN into `cov[11]` to mean "altitude unknown" — the
   natural reading of the MAVLink spec — feeds NaN into EKF3. And because that
   is a 3D magnitude, a radial sigma must be split as
   `cov[0] = cov[6] = sigma²/2` with `cov[11] = 0`, or the filter silently
   receives `sigma × √2`.
3. **pymavlink writes MAVLink 1 on a send-only link.** It negotiates the wire
   version from *inbound* traffic and starts at 1.0, so a link that only ever
   writes never upgrades and every message id above 255 is absent from the
   object. ODOMETRY is 331. It surfaces as
   `'MAVLink' object has no attribute 'odometry_send'`.

`python scripts/test_fc_encoding.py` is the MAVLink half on its own: it opens a
socket, sends what the output layer would send in flight, and decodes it with
logic transcribed from ArduPilot's `GCS_Common.cpp`.

**Never `GPS_INPUT`.** It has no covariance field, and covariance is the whole
contribution.

---

## Stage 2 — SITL, closed loop

Only after stage 1 is clean.

```bash
GEOANCHOR_SET="output_layer.loop_mode=closed;output_layer.fc.enabled=true" \
  GEOANCHOR_CONFIG=configs/env80.yaml bash run.sh
```

Closed loop is gated per fix on the inlier floor, the 100 m altitude cap, and
the **absence of live device-error codes in any layer**. A `DE` code anywhere
stops it sending.

Keep it in SITL until the covariance estimator is exported and validated
leave-one-scene-out. Sending a position with a covariance you have not
validated is worse than sending nothing — the filter trusts it.

---

## Stage 3 — HITL: simulator on the laptop, pipeline on the board

```
   Laptop                                     edge board
   ┌────────────────────────┐                ┌─────────────────────┐
   │ Gazebo  world + camera │───ethernet────▶│ data layer          │
   │ ArduPilot SITL   EKF3  │◀───────────────│ processing layer    │
   └────────────────────────┘   ODOMETRY     │ output layer        │
                                             │ dashboard API :8000 │
                                             └─────────────────────┘
```

The board runs the real pipeline on its real CPU, so the timings are board
timings. The laptop runs the simulator and the flight stack. This measures the
question that actually decides deployment — *can this board match fast enough* —
without needing an airframe.

### Wire it

Static-IP both ends so nothing depends on DHCP:

```bash
sudo ip addr add 192.168.50.1/24 dev eth0 && sudo ip link set eth0 up
```

```bash
sudo ip addr add 192.168.50.2/24 dev eth0 && sudo ip link set eth0 up
```

Ethernet is not a concern and this was checked: a 640×360 JPEG is 0.35 ms on
gigabit and 5–15 ms round trip with the stack, against a matcher costing
200–3000 ms. Inside a 250 ms budget that is negligible.

Point SITL's output at the board and the board's config at the laptop:

```bash
sim_vehicle.py -v ArduCopter --console --map --out=udp:192.168.50.2:14550
```

```yaml
data_layer:
  gps:
    source: mavlink
    endpoint: udpin:0.0.0.0:14550
output_layer:
  fc:
    enabled: true
    endpoint: udpout:192.168.50.1:14550
```

### Gazebo

ArduPilot's own plugin is the maintained path:

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
```

Build it per its README, then launch a world with a down-facing camera and run
`sim_vehicle.py -v ArduCopter -f gazebo-iris` against it.

### The rule that decides whether any of this is honest

**Never texture the Gazebo ground plane with the image used as the reference
map.** The pipeline then matches a picture against itself and every number it
produces is meaningless.

`demo/flight.mp4` does exactly this on purpose, for bring-up, and every session
built on it is stamped `synthetic_from_reference` with a dashboard banner. A
HITL world has no such banner, so the discipline has to be yours: use imagery
the reference does **not** contain — a different capture date, a different
source, or a deliberately offset tile.

This is what makes the historical-imagery work matter. NSW publishes a 1943
survey of inner Sydney as a separate service from the current mosaic, which
gives a genuinely independent second epoch of the same ground. See
[`HISTORICAL_IMAGERY.md`](HISTORICAL_IMAGERY.md).

### What HITL cannot tell you

`OVERHEAD_MS` — capture, ISP, USB transfer — is a property of the **camera**,
and a simulated camera has none of it. Two measurements on this project differ
by **3x** on that term alone:

```
AGX Xavier    45.7 ms median
Pi 5         132.1 ms median      same camera model
```

and the gap is the camera, not the board. So HITL gives you the matcher half of
the budget honestly and the capture half not at all. Measure `OVERHEAD_MS` on
the real rig with the real sensor and add it to the HITL matcher time to get a
number you can deploy against.

---

## The order, condensed

1. `python scripts/check_extnav.py` against SITL — proves the parameter set.
2. `bash sitl.sh --set-params` then `bash sitl.sh` — proves the write path with
   a position already known good.
3. `bash scripts/check_board.sh` on the board — proves it can run at all.
4. Static-IP ethernet, board runs the pipeline against a laptop feed — proves
   the split and gives real board timings.
5. Gazebo world with **non-reference** imagery — closes the HITL loop honestly.
6. Closed loop, **in SITL only**, until the covariance estimator is validated
   leave-one-scene-out.

Nothing reaches a vehicle until `output_layer.fc.enabled` is true, and closed
loop refuses to send while any device-error code is live anywhere in the system.
