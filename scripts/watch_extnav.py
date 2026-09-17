#!/usr/bin/env python3
"""Does the estimator actually CONSUME our fixes, or merely accept the bytes?

    python scripts/watch_extnav.py --endpoint tcp:127.0.0.1:5760
    python scripts/watch_extnav.py --endpoint udpin:0.0.0.0:14540 --firmware px4

check_extnav.py proves the message is well formed and the parameters are right,
then tells you to go and watch Mission Planner. This closes that loop without a
human in it, and it does so by taking GPS away:

  1. boot with GPS so the EKF gets an origin
  2. take GNSS away -- the vehicle is now GNSS-denied, which is the case this
     whole project exists for
  3. send ODOMETRY carrying a position deliberately OFFSET from the vehicle's own
  4. watch whether the EKF's reported position walks toward the offset

If it walks, the external fix is being fused. If it holds station, it is being
ignored and everything upstream of here is decoration. A constant offset that
never closes is the origin mismatch fcout.py warns about.

**IT PUTS GNSS BACK.** This test deliberately disables the vehicle's primary
navigation source, and leaving it disabled is wrong everywhere and dangerous on
hardware. It is not hypothetical: PX4 SITL autosaves a changed parameter a few
seconds later, so an early version of this script left EKF2_GPS_CTRL = 0 in
parameters.bson and the NEXT boot came up permanently GNSS-denied -- never
reaching "Ready for takeoff", never taking an origin, and failing this very
test with "SITL never got GPS lock". The original value is read before the
change and restored in a finally block.

Step 2 is the one thing that differs between firmwares:

    ArduPilot   GPS1_TYPE     0   no GPS driver at all
    PX4         EKF2_GPS_CTRL 0   GNSS aiding bitmask cleared -- the driver keeps
                                  running, EKF2 simply stops fusing it

Both are set at RUNTIME and never in the defaults file, because the EKF needs a
GNSS origin before it can accept a local position at all. Booting without one
means the estimator never takes an origin and this test cannot start -- which
is a trap already paid for once on the ArduPilot side.

The firmware is detected from the HEARTBEAT; --firmware only overrides it. It
also selects the pose covariance layout, which the two read differently and
neither complains about -- see the fcout.py docstring.

TWO ENDPOINTS ON PX4, AND IT IS NOT OPTIONAL. PX4's UDP MAVLink instances lock
their remote address to the FIRST peer that talks to them and never re-target:
a second client on the same port -- even after the first has closed -- gets
nothing back and times out against a perfectly healthy vehicle. What PX4 does
do is stream continuously to its configured remote port (14540 for the Onboard
instance) whether anyone is listening or not. So --endpoint is where we LISTEN
and --write-endpoint is where we SEND, and on PX4 they are different sockets:

    --endpoint udpin:0.0.0.0:14540  --write-endpoint udpout:127.0.0.1:14580

On ArduPilot --write-endpoint defaults to the peer of --endpoint, which is the
behaviour this had before and needs no flag.

TWO IDENTITIES, AND THEY ARE NOT INTERCHANGEABLE. The write link identifies as
(system 1, component 197) -- the vehicle's own system id with the visual-odometry
component id, which is the MAVLink convention for a companion computer and what
fcout.py flies with. PX4 will not route telemetry BACK to a peer claiming its own
system id, so that socket receives nothing at all: no heartbeat, no
GLOBAL_POSITION_INT, no sign that anything is wrong. Observation therefore runs
on a second connection identifying as 255, the GCS id, which is the same split
check_extnav.py already uses. Sending and watching are different roles and they
need different identities.
"""
from __future__ import annotations

import argparse, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from geoanchor.output_layer.fcout import FlightControllerLink  # noqa: E402

EARTH_M_PER_DEG = 111_320.0
AUTOPILOT_ARDUPILOTMEGA = 3
AUTOPILOT_PX4 = 12
MAV_PARAM_TYPE_INT32 = 6


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="tcp:127.0.0.1:5760")
    ap.add_argument("--firmware", choices=["ardupilot", "px4", "auto"], default="auto")
    ap.add_argument("--write-endpoint", default=None,
                    help="where to SEND, when it differs from where we listen. "
                         "PX4 needs this; see the module docstring.")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--offset-m", type=float, default=20.0, help="north offset to inject")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=4.0)
    ap.add_argument("--sigma", type=float, default=2.0)
    ap.add_argument("--fix-rate", type=float, default=None,
                    help="how often a NEW fix is computed, when that is slower "
                         "than --rate. The most recent fix is then RE-SENT to "
                         "pad the stream up to --rate. This models the only "
                         "route around PX4's 5 Hz floor that does not fork the "
                         "firmware; see docs/px4_ekf2_extnav_2026-09-17.md.")
    ap.add_argument("--repeat-stamp", choices=["capture", "now"], default="capture",
                    help="timestamp on a RE-SENT fix. 'capture' keeps the "
                         "original, which is honest and is what delay "
                         "compensation needs; 'now' restamps it, which is what "
                         "a naive pad would do.")
    a = ap.parse_args()

    # The firmware has to be known BEFORE the link is built, because it picks
    # the covariance layout. So: a bare connection for the heartbeat, then the
    # real link.
    from pymavlink import mavutil
    probe = mavutil.mavlink_connection(a.endpoint, baud=a.baud, source_system=255)
    # A udpout link is SEND-ONLY until something goes out of it: the peer has
    # no address to answer to. PX4 in particular streams nothing to a port it
    # has never heard from, so waiting for a heartbeat on a fresh udpout socket
    # times out against a perfectly healthy vehicle -- which this script then
    # reports as "no heartbeat / check wiring". One GCS heartbeat is also just
    # what a GCS is supposed to send.
    # ANNOUNCE REPEATEDLY, not once. A udp peer is only known to the autopilot
    # while it keeps talking: PX4 streams to the last address it heard from, so
    # a client that announces itself a single time and then blocks for 30 s can
    # lose that slot to anything else on the port -- including a previous stage
    # of this same test whose socket has since closed. One lost datagram had
    # the same effect. Re-sending once a second costs nothing and makes the
    # wait mean what it says.
    m = None
    for _ in range(30):
        probe.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        m = probe.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if m is not None:
            probe.target_system = m.get_srcSystem()
            probe.target_component = m.get_srcComponent()
            break
    if m is None:
        print("no heartbeat"); return 2
    detected = {AUTOPILOT_ARDUPILOTMEGA: "ardupilot", AUTOPILOT_PX4: "px4"}.get(m.autopilot)
    firmware = a.firmware if a.firmware != "auto" else detected
    if firmware is None:
        probe.close()
        print(f"MAV_AUTOPILOT {m.autopilot} is neither ArduPilotMega (3) nor PX4 (12); "
              "pass --firmware")
        return 2

    # The probe stays open and becomes the OBSERVER. See the docstring: the
    # write link cannot receive, because it wears the vehicle's own system id.
    obs = probe
    write_ep = a.write_endpoint or a.endpoint
    link = FlightControllerLink(write_ep, baud=a.baud, message="ODOMETRY",
                                firmware=firmware)
    print(f"system {obs.target_system} | firmware {firmware}")
    print(f"  watch {a.endpoint} as (255, 0)")
    print(f"  write {write_ep} as ({link.conn.mav.srcSystem}, "
          f"{link.conn.mav.srcComponent})")

    pos = None
    if firmware == "px4":
        # PX4 ignores REQUEST_DATA_STREAM -- it is an ArduPilot-era message --
        # and uses SET_MESSAGE_INTERVAL instead. Asking the ArduPilot way on
        # PX4 is silent and leaves this loop waiting out its full 60 s.
        for msgid, hz in ((33, 4.0),):          # GLOBAL_POSITION_INT
            obs.mav.command_long_send(
                obs.target_system, obs.target_component,
                511,                             # MAV_CMD_SET_MESSAGE_INTERVAL
                0, msgid, int(1e6 / hz), 0, 0, 0, 0, 0)
    else:
        obs.mav.request_data_stream_send(obs.target_system, obs.target_component,
                                         6, 4, 1)  # POSITION
        obs.mav.request_data_stream_send(obs.target_system, obs.target_component,
                                         2, 2, 1)  # EXTENDED_STATUS, carries EKF_STATUS_REPORT
    t0 = time.time()
    while time.time() - t0 < 60:
        g = obs.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if g and abs(g.lat) > 1e-7:
            pos = (g.lat / 1e7, g.lon / 1e7); break
    if pos is None:
        print("no origin -- SITL never got GPS lock, or GLOBAL_POSITION_INT is not streaming")
        return 2
    lat0, lon0 = pos
    print(f"origin {lat0:.7f}, {lon0:.7f}")

    denial = b"EKF2_GPS_CTRL" if firmware == "px4" else b"GPS1_TYPE"

    # Read the CURRENT value first, so it can be put back. A hardcoded
    # "restore to 1" would be a guess: EKF2_GPS_CTRL is a bitmask that defaults
    # to 7, and ArduPilot's GPS1_TYPE depends on what is actually fitted.
    obs.mav.param_request_read_send(obs.target_system, obs.target_component, denial, -1)
    original, deadline = None, time.time() + 5.0
    while time.time() < deadline:
        m = obs.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m and m.param_id == denial.decode():
            original, original_type = m.param_value, m.param_type
            break
    if original is None:
        print(f"could not read {denial.decode()} -- refusing to disable GNSS without "
              "knowing what to restore")
        return 2
    # Zero, and only zero, is safe to send here without the bit-pattern dance
    # that scripts/px4_set_params.py does: PX4 reinterprets the float field's
    # BYTES as an int32 for an int-typed parameter, and 0.0f and int 0 share a
    # bit pattern. Any other value set this way would land as nonsense.
    obs.mav.param_set_send(obs.target_system, obs.target_component,
                           denial, 0, MAV_PARAM_TYPE_INT32)
    time.sleep(2)
    print(f"{denial.decode()} set to 0 -- vehicle is now GNSS-denied")

    print(f"injecting a position {a.offset_m:.0f} m north, sigma {a.sigma} m, "
          f"{a.rate} Hz"
          + (f" (new fix at {a.fix_rate} Hz, padded)" if a.fix_rate else "")
          + f" for {a.seconds:.0f}s\n")
    # EKF_STATUS_REPORT is an ArduPilot message. PX4 does not send it, so on
    # PX4 the column is omitted rather than printed as a permanent zero -- a
    # zero here already read as evidence once when it was only an absent stream.
    show_flags = firmware == "ardupilot"
    print(f"{'t':>5}  {'drift N (m)':>12}" + (f"  {'ekf flags':>10}" if show_flags else ""))

    def restore_gnss():
        obs.mav.param_set_send(obs.target_system, obs.target_component,
                               denial, original, original_type)
        # Confirm rather than assume. A PARAM_SET that does not land leaves the
        # vehicle without GNSS, which is the one outcome worth being noisy about.
        end = time.time() + 5.0
        while time.time() < end:
            m = obs.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if m and m.param_id == denial.decode():
                print(f"{denial.decode()} restored")
                return True
        print(f"WARNING: {denial.decode()} was NOT confirmed restored. "
              "The vehicle may still be GNSS-denied -- check before flying, and "
              "on SITL delete the working directory's parameters.bson.")
        return False

    try:
        return _inject(a, link, obs, lat0, lon0, firmware, show_flags)
    finally:
        restore_gnss()


def _inject(a, link, obs, lat0, lon0, firmware, show_flags) -> int:
    tgt_lat = lat0 + a.offset_m / EARTH_M_PER_DEG
    link.ensure_origin(lat0, lon0)
    period, t0, nxt, last = 1.0 / a.rate, time.time(), 0.0, None
    flags, nxt_row = 0, 0.0
    # Padding: a NEW fix appears every fix_period, and every send in between
    # re-sends the most recent one. fix_period == period means no padding,
    # which is the original behaviour.
    fix_period = 1.0 / a.fix_rate if a.fix_rate else period
    nxt_fix, cur_stamp, n_new, n_repeat = 0.0, None, 0, 0
    while time.time() - t0 < a.seconds:
        now = time.time()
        if now >= nxt:
            if now >= nxt_fix or cur_stamp is None:
                cur_stamp, nxt_fix = now, now + fix_period
                n_new += 1
            else:
                n_repeat += 1
            stamp = cur_stamp if a.repeat_stamp == "capture" else now
            link.send(tgt_lat, lon0, a.sigma, t_capture_unix=stamp)
            nxt = now + period
        g = obs.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if g and abs(g.lat) > 1e-7:
            last = (g.lat / 1e7 - lat0) * EARTH_M_PER_DEG
        e = obs.recv_match(type="EKF_STATUS_REPORT", blocking=False)
        if e:
            flags = e.flags
        if last is not None and now - t0 >= nxt_row:
            print(f"{now - t0:5.1f}  {last:12.2f}"
                  + (f"  {flags:#010x}" if show_flags else ""))
            nxt_row += 5.0
        time.sleep(0.02)

    print(f"\nsent {link.sent} messages"
          + (f" ({n_new} new fixes, {n_repeat} re-sends, stamp={a.repeat_stamp})"
             if a.fix_rate else "")
          + f", final drift {last if last is not None else float('nan'):.2f} m "
            f"of {a.offset_m:.0f} m injected")
    print("walked toward the injection = fused.  held at 0 = ignored.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
