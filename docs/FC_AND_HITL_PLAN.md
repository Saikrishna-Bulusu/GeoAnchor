# SpeedyBee F405 V3, and the road to HITL

Written 10 Sept 2026, in response to: *use my SpeedyBee F405 V3 stack, see if
the Pi is picking up data from the FC, and eventually do HITL with Gazebo,
laptop connected to the edge device over ethernet.*

Decision taken with the plan: **stay on ArduPilot.** Every finding in this
project is ArduPilot-specific — `docs/step22_ardupilot_extnav_check.py`
re-asserts 14 EKF3 constants, `VISO_DELAY_MS` is an ArduPilot parameter, and
the whole latency analysis is `AP_NavEKF3`. PX4 uses EKF2 and
`vehicle_visual_odometry` with different clamps and timeouts, so a port would
invalidate the covariance and latency work rather than extend it.

---

## Read this before wiring anything

### The F405 V3 is a Betaflight board, and that has consequences

It is an STM32F405 with **1 MB of flash**, sold for Betaflight. Two things
follow:

- **PX4 is not an option on it.** PX4 dropped F4 support; there is no
  maintained target. This is not a preference, it is the absence of firmware.
- **ArduPilot on F405 is flash-constrained.** ArduPilot has F405 targets, but
  at 1 MB the build strips features to fit, and what gets stripped varies by
  target and release. **Whether EKF3 + ExternalNav survives the trim on this
  specific board is the first thing to check, and it is not safe to assume.**

Check it before buying into the plan:

```bash
python scripts/check_extnav.py --port /dev/ttyACM0
```

It verifies `VISO_TYPE`, `VISO_DELAY_MS`, `EK3_SRC1_POSXY=6`, `AHRS_EKF_TYPE`
and the rest. **If `VISO_*` parameters do not exist, ExternalNav was not
compiled in** and this board cannot be the closed-loop target — no amount of
configuration adds it back. That is a firmware limit, not a wiring fault.

### So treat the F405 V3 as a bench target, not the flight controller

Its job in this project is to answer one question — *does the Pi↔FC MAVLink
link work?* — using real hardware, cheaply. Everything about covariance,
latency budgets and EKF3 acceptance should be developed against **SITL**, where
the firmware is a full build with nothing trimmed.

If closed-loop on real hardware becomes the goal, budget for an F7/H7 board
(2 MB flash). NGPS ran a Cube Orange Plus.

---

## Step 1 — is the Pi receiving anything from the FC?

This is the question you actually asked, and it has a direct answer.

Wire the FC's spare UART to the Pi's UART, or just use USB, which is simpler
and adequate for a bench test:

```
FC USB  ─────  Pi USB        appears as /dev/ttyACM0
```

For the UART path, cross TX/RX and share ground. **Do not power the Pi from
the FC's 5 V BEC** — a Pi 5 pulls more than the regulator is sized for, and
a brownout mid-flight looks exactly like a software hang.

```
FC TX  ──────  Pi RX   (GPIO15, pin 10)
FC RX  ──────  Pi TX   (GPIO14, pin 8)
FC GND ──────  Pi GND  (pin 6)
```

On the Pi, free the UART first — it is a console by default:

```bash
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1
sudo reboot
```

Then ask directly whether frames are arriving:

```bash
python -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
print('waiting for heartbeat...')
hb = m.wait_heartbeat(timeout=10)
print('heartbeat:', hb)
for _ in range(20):
    msg = m.recv_match(blocking=True, timeout=2)
    if msg: print(msg.get_type(), msg.to_dict())
"
```

**A heartbeat and nothing else is the normal first result.** ArduPilot only
streams what a GCS has requested. `ATTITUDE` and `GLOBAL_POSITION_INT` — the
two the data layer needs — come from `SRx_EXTRA1` and `SRx_POSITION` on the
serial port you are connected to. Set them, or request the rates explicitly.

Set the FC side once, in Mission Planner or via MAVProxy:

    SERIAL2_PROTOCOL = 2          MAVLink2
    SERIAL2_BAUD     = 921600     with flow control; 115200 is fine for a bench
    SR2_EXTRA1       = 10         ATTITUDE at 10 Hz
    SR2_POSITION     = 5          GLOBAL_POSITION_INT at 5 Hz

Then point the runtime at it, in `configs/system.yaml`:

```yaml
data_layer:
  gps:
    source: mavlink
    device: /dev/ttyACM0
    baud: 115200
```

and watch for `DL-11`/`DL-12` on the dashboard. `DLDE-*` means the link is the
problem, not the fix.

### What "is it picking up data" really has to mean

A heartbeat proves the wire. It does not prove the pipeline can run. The data
layer needs **attitude** (for the stage-02 yaw rectification, which is
load-bearing — without it the matcher fails on a task where the query is cut
out of the reference) and **altitude** (for `GSD = altitude / fx_px`).

On a bench with no GPS lock, `GLOBAL_POSITION_INT.relative_alt` is zero or
garbage, so the scale path cannot run and every frame is rejected on `PLE-08`.
**That is correct behaviour, not a fault** — `configs/camera.yaml` documents
it. Set a static test altitude to exercise the rest:

```bash
GEOANCHOR_SET="data_layer.gps.static_altitude_m=75" bash run.sh
```

---

## Step 2 — SITL first, hardware second

Everything about EKF3 acceptance should be proven in SITL, because the
firmware there is complete and a mistake costs nothing.

```bash
bash sitl.sh                        # ArduPilot SITL + the runtime, open loop
python scripts/sitl_openloop.py     # the MAVLink half on its own
```

`open` loop mode sends the **actual** GPS over the ExternalNav path. That is
the useful middle step: it exercises the message, the covariance, the origin
and the EKF's acceptance logic with a position already known good, so every
failure it finds is a plumbing failure rather than a matcher failure.

Three bugs already found this way, none visible from the sending side:

- `frame_id = MAV_FRAME_LOCAL_NED` is **discarded in silence**. `LOCAL_FRD`
  is 20, not 1. `handle_odometry()` returns early with no warning.
- A NaN anywhere in the translational covariance poisons `posErr` — ArduPilot
  computes `sqrtf(cov[0]+cov[6]+cov[11])` and only checks `isnan(cov[0])`.
- pymavlink negotiates the wire version from **inbound** traffic, so a
  send-only link stays MAVLink 1 and every message id above 255 is absent.
  ODOMETRY is 331.

Never `GPS_INPUT` — it has no covariance field, and covariance is the
contribution.

---

## Step 3 — HITL: Gazebo on the laptop, matcher on the board, ethernet between

The target topology:

```
   Legion (laptop)                          edge board
   ┌──────────────────────┐                ┌─────────────────────┐
   │ Gazebo               │                │ data layer          │
   │  world + camera      │───ethernet────▶│ processing layer    │
   │ ArduPilot SITL       │◀───────────────│ output layer        │
   │  EKF3                │   ODOMETRY     │ dashboard API :8000 │
   └──────────────────────┘                └─────────────────────┘
```

The board runs the real pipeline on its real CPU, so the timings are board
timings. The laptop runs the simulator and the flight stack. **This is the
right shape**: it measures the thing that matters (can this board match fast
enough) without needing an airframe.

### Ethernet is not a concern, and it was checked

A 640×360 JPEG is 0.35 ms on gigabit and 5–15 ms round trip with the stack,
against a matcher costing 200–3000 ms. Inside a 250 ms budget that is
negligible. Static-IP both ends so nothing depends on DHCP:

```bash
# laptop
sudo ip addr add 192.168.50.1/24 dev eth0 && sudo ip link set eth0 up
# board
sudo ip addr add 192.168.50.2/24 dev eth0 && sudo ip link set eth0 up
```

### The ordering constraint that decides whether HITL is honest

**Never texture the Gazebo ground plane with the image used as the reference
map.** The pipeline then matches a picture against itself, and every number is
meaningless. `demo/flight.mp4` does this deliberately for bring-up, and every
session built on it is stamped `synthetic_from_reference` with a banner on the
dashboard. A HITL world must use imagery the reference does **not** contain —
a different capture date, a different source, or a deliberately offset tile.

### The thing HITL cannot tell you

`OVERHEAD_MS` — capture, ISP, USB transfer — is a property of the **camera**,
and a simulated camera has none of it. Two measurements in this project differ
by 3× on that term alone (45.7 ms Xavier, 132.1 ms Pi 5, same camera model,
and the gap is the camera not the board).

So HITL gives you the matcher half of the budget honestly and the capture half
not at all. Measure `OVERHEAD_MS` on the real rig with the real sensor, and add
it to the HITL matcher time to get a deployable number.

---

## Suggested order

1. **`check_extnav.py` against the F405 V3.** If `VISO_*` is missing, stop and
   decide on a different FC before investing further. One command, decides the
   shape of everything below.
2. **Heartbeat + ATTITUDE + GLOBAL_POSITION_INT on the Pi.** Proves the link.
3. **SITL open loop on the laptop.** Proves the write path with a known-good
   position.
4. **Static-IP ethernet, board runs the pipeline against a laptop feed.**
   Proves the split and gives real board timings.
5. **Gazebo world with non-reference imagery**, closing the HITL loop.
6. **Closed loop, in SITL only,** until the covariance estimator is exported
   and validated leave-one-scene-out.

Nothing reaches a vehicle until `output_layer.fc.enabled` is true, and closed
loop refuses to send while any device-error code is live anywhere in the
system.
