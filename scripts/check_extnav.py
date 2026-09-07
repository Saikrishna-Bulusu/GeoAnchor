#!/usr/bin/env python3
"""Live check of the ExternalNav path, against SITL first and hardware later.

Run this BEFORE closed loop, and before believing any covariance reaches EKF3.
It does four things in order and stops at the first that fails:

  1. connects and identifies the autopilot
  2. reads the parameters ArduPilot needs for external navigation
  3. sends ODOMETRY carrying a known position and a known covariance
  4. watches whether the estimator's position follows it

    python scripts/check_extnav.py --endpoint udpin:0.0.0.0:14550
    python scripts/check_extnav.py --endpoint /dev/ttyTHS0 --baud 921600

The test position is the vehicle's OWN reported position, deliberately. That is
the open-loop mode of the output layer: it exercises the message, the origin,
the covariance and the acceptance logic with a position that cannot mislead the
filter, so anything that fails is a plumbing failure rather than a bad fix.

The parameter list here is what the ArduPilot docs specify. The behaviour of
EKF3 underneath is checked against source by the parent repo's
docs/step22_ardupilot_extnav_check.py -- run that too; they answer different
questions.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from geoanchor.output_layer.fcout import FlightControllerLink  # noqa: E402

# The values NGPS flies with, from snktshrma's own "Non-GPS Navigation
# Documentation" (the ngps_flight setup guide), plus the two this pipeline
# needs that his list does not mention.
#
#     SERIALx_BAUD = 921600     UART with hardware flow control (RTS/CTS)
#     VISO_TYPE = 1
#     VISO_DELAY_MS = 50
#     EK3_SRC1_POSXY = 6
#     EK3_SRC1_YAW = 1
#     EK3_SRC1_VELXY = 0
#     EK3_SRC1_VELZ = 0
WANT = {
    "AHRS_EKF_TYPE":  ("3 = EKF3", lambda v: int(v) == 3),
    "VISO_TYPE":      ("non-zero enables AP_VisualOdom. NGPS uses 1", lambda v: v > 0),
    "VISO_POS_X":     ("camera offset forward of the IMU, metres", None),
    "VISO_POS_Y":     ("camera offset right of the IMU, metres", None),
    "VISO_POS_Z":     ("camera offset below the IMU, metres", None),
    "VISO_DELAY_MS":  ("measured pipeline latency. Range 0-250, EKF3 clamps it "
                       "again. NGPS uses 50; use YOUR measured median",
                       lambda v: 0 <= v <= 250),
    "VISO_QUAL_MIN":  ("minimum ODOMETRY quality accepted. 0 accepts anything; "
                       "raise it once inlier counts on your reference are known", None),
    "EK3_SRC1_POSXY": ("6 = ExternalNav", lambda v: int(v) == 6),
    "EK3_SRC1_VELXY": ("0 = none. This pipeline sends no velocity", lambda v: int(v) in (0, 6)),
    "EK3_SRC1_VELZ":  ("0 = none", lambda v: int(v) == 0),
    "EK3_SRC1_POSZ":  ("1 = baro. Altitude stays on the flight controller; this "
                       "pipeline never claims it", lambda v: int(v) != 6),
    "EK3_SRC1_YAW":   ("1 = compass. NGPS keeps the compass for yaw, and this "
                       "pipeline sends an identity quaternion, so do not set 6",
                       lambda v: int(v) != 6),
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="udpin:0.0.0.0:14550")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--message", default="ODOMETRY",
                    choices=["ODOMETRY", "VISION_POSITION_ESTIMATE"])
    ap.add_argument("--sigma", type=float, default=5.0, help="test covariance, metres")
    ap.add_argument("--rate", type=float, default=4.0)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError:
        print("pymavlink is not installed -- run bootstrap.sh")
        return 2

    print(f"1/4  connecting to {args.endpoint}")
    conn = mavutil.mavlink_connection(args.endpoint, baud=args.baud, source_system=255)
    hb = conn.wait_heartbeat(timeout=args.timeout)
    if hb is None:
        print(f"     no heartbeat within {args.timeout}s.")
        print("     SITL:      sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550")
        print("     hardware:  check wiring, baud, and that nothing else holds the port")
        return 1
    print(f"     system {conn.target_system} component {conn.target_component} | "
          f"autopilot {hb.autopilot} type {hb.type}")

    print("\n2/4  parameters")
    conn.mav.param_request_list_send(conn.target_system, conn.target_component)
    got, deadline = {}, time.time() + args.timeout
    while time.time() < deadline and len(got) < len(WANT):
        m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if m and m.param_id in WANT:
            got[m.param_id] = float(m.param_value)
    problems, unknown = 0, 0
    for name, (why, ok) in WANT.items():
        if name not in got:
            # NOT harmless. param_request_list streams every parameter on the
            # vehicle, and over a serial link the ten we care about may simply
            # not have arrived inside the timeout. "Did not arrive" and "does
            # not exist" are indistinguishable from here, so this counts rather
            # than shrugging: reporting success for a parameter never seen is
            # how a misconfigured autopilot passes its own pre-flight check.
            print(f"     ??   {name:16s} not reported -- absent on this firmware, or the "
                  "parameter stream did not finish in time")
            unknown += 1
            continue
        v = got[name]
        good = ok(v) if ok else True
        problems += 0 if good else 1
        print(f"     {'ok  ' if good else 'BAD '} {name:16s} {v:<10g} {why}")
    if problems:
        print(f"\n     {problems} parameter(s) wrong. Set them and reboot the autopilot "
              "before going further -- an ExternalNav fix is ignored silently otherwise.")
    if unknown:
        print(f"     {unknown} parameter(s) never reported. Re-run with a longer --timeout "
              "before treating this as a pass.")

    print("\n3/4  waiting for a position to echo back")
    pos = None
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        m = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if m and m.lat != 0:
            pos = (m.lat / 1e7, m.lon / 1e7)
            break
    if pos is None:
        print("     no GLOBAL_POSITION_INT with a valid fix. The EKF has no origin yet;")
        print("     arm in SITL or wait for GPS lock, then re-run.")
        return 1
    print(f"     {pos[0]:.7f}, {pos[1]:.7f}")

    print(f"\n4/4  sending {args.message} at {args.rate} Hz for {args.seconds:.0f}s "
          f"with sigma {args.sigma} m")
    link = FlightControllerLink(args.endpoint if args.endpoint.startswith("udpout")
                                else f"udpout:{_peer(args.endpoint)}",
                                baud=args.baud, message=args.message, origin=pos)
    sent = 0
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        link.send(pos[0], pos[1], sigma_m=args.sigma,
                  t_capture_unix=time.time(), quality=80)
        sent += 1
        time.sleep(1.0 / args.rate)
    link.close()
    print(f"     sent {sent} messages")

    print("\nWhat to check now, in Mission Planner or MAVProxy:")
    print("  * the message arrives:      watch the inbound stream for your source system")
    print("  * EKF3 uses it:             EKF_STATUS_REPORT, and posTestRatio staying below 1")
    print("  * innovations are sane:     a sustained ratio above 1 means the fix is being")
    print("                              rejected by the 5-sigma gate, and sustained rejection")
    print("                              walks the filter to posTimeout on the 7 s clock while")
    print("                              data is still arriving on time")
    print("  * the origin agrees:        an ExternalNav position is LOCAL. If the EKF origin")
    print("                              and the origin used here differ, the offset is silent")
    print("                              and constant, and it is the most common way this fails")

    # The exit code has to carry the verdict. This returned 0 unconditionally,
    # so a wrong EK3_SRC1_POSXY printed "BAD" and still passed -- and anything
    # gating on this script, a person included, read that as approval.
    if problems or unknown:
        print(f"\nFAILED: {problems} wrong, {unknown} unverified.")
        return 1
    print("\nAll checked parameters are correct.")
    return 0


def _peer(endpoint: str) -> str:
    """udpin:0.0.0.0:14550 -> a udpout target on the same port, localhost."""
    parts = endpoint.split(":")
    port = parts[-1] if parts[-1].isdigit() else "14550"
    return f"127.0.0.1:{port}"


if __name__ == "__main__":
    raise SystemExit(main())
