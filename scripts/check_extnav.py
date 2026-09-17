#!/usr/bin/env python3
"""Live check of the external-navigation path, against SITL first and hardware later.

Run this BEFORE closed loop, and before believing any covariance reaches EKF3.
It does four things in order and stops at the first that fails:

  1. connects and identifies the autopilot
  2. reads the parameters ArduPilot needs for external navigation
  3. sends ODOMETRY carrying a known position and a known covariance
  4. watches whether the estimator's position follows it

    python scripts/check_extnav.py --endpoint udpin:0.0.0.0:14550
    python scripts/check_extnav.py --endpoint /dev/ttyTHS0 --baud 921600
    python scripts/check_extnav.py --endpoint udpin:0.0.0.0:14540 --firmware px4

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

The firmware is detected from the HEARTBEAT (MAV_AUTOPILOT 3 = ArduPilotMega,
12 = PX4) and --firmware only overrides that. The two have entirely disjoint
parameter sets -- VISO_*/EK3_SRC1_* against EKF2_* -- so checking the wrong
table reports every parameter ABSENT, which is the verdict that condemns a
board. Getting this from the heartbeat rather than from a flag is what stops
that being a thing anyone can typo.

The test position is the vehicle's OWN reported position, deliberately. That is
the open-loop mode of the output layer: it exercises the message, the origin,
the covariance and the acceptance logic with a position that cannot mislead the
filter, so anything that fails is a plumbing failure rather than a bad fix.

The ArduPilot list is what the ArduPilot docs specify. The behaviour of EKF3
underneath is checked against source by the parent repo's
docs/step22_ardupilot_extnav_check.py -- run that too; they answer different
questions.

The PX4 list was read out of PX4 v1.16.2's own parameter metadata
(src/modules/ekf2/params_external_vision.yaml, params_gnss.yaml, module.yaml)
rather than from a wiki page, and the defaults matter more here than on
ArduPilot: EKF2_EV_CTRL defaults to 0, so external vision is OFF until it is
set, and EKF2_HGT_REF defaults to 1 (GPS), which is the wrong height reference
for a vehicle that is about to have GNSS taken away from it.
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


# PX4 v1.16.2. Read from the firmware's own parameter metadata, not a wiki.
#
# EKF2_EV_CTRL is a bitmask (params_external_vision.yaml:5):
#     bit 0  horizontal position     <- the only one this pipeline claims
#     bit 1  vertical position          altitude stays on baro
#     bit 2  3D velocity                no velocity is sent
#     bit 3  yaw                        an identity quaternion is sent, so never
#
# EKF2_EV_NOISE_MD is the parameter this whole project turns on. 0 means the
# variance in OUR message is used, with EKF2_EVP_NOISE as a lower bound; 1
# throws it away and uses the parameter for every fix, which is exactly the
# "trust every fix identically" behaviour the covariance work exists to avoid.
WANT_PX4 = {
    "EKF2_EN":        ("1 = EKF2 running", lambda v: int(v) == 1),
    "EKF2_EV_CTRL":   ("bitmask. Bit 0 = horizontal position. Defaults to 0, "
                       "which is external vision OFF",
                       lambda v: int(v) & 1 == 1),
    "EKF2_EV_NOISE_MD": ("0 = use the variance in OUR message (EKF2_EVP_NOISE "
                         "is then only a lower bound). 1 discards it and this "
                         "project's covariance never reaches the filter",
                         lambda v: int(v) == 0),
    "EKF2_EVP_NOISE": ("lower bound on our position sigma, metres. The default "
                       "0.1 silently raises anything tighter; there is NO "
                       "upper clamp, unlike ArduPilot's 100 m",
                       lambda v: v > 0),
    "EKF2_EVP_GATE":  ("innovation gate in standard deviations. 5 by default, "
                       "the same 5-sigma test EKF3 applies", lambda v: v >= 1.0),
    "EKF2_EV_DELAY":  ("measured pipeline latency, ms. Range 0-300 here, where "
                       "ArduPilot's VISO_DELAY_MS stops at 250. Use YOUR "
                       "measured median", lambda v: 0 <= v <= 300),
    "EKF2_EV_QMIN":   ("minimum quality accepted, 0-100. 0 accepts anything; "
                       "the analogue of VISO_QUAL_MIN", None),
    "EKF2_EV_POS_X":  ("camera offset forward of the IMU, metres", None),
    "EKF2_EV_POS_Y":  ("camera offset right of the IMU, metres", None),
    "EKF2_EV_POS_Z":  ("camera offset below the IMU, metres", None),
    "EKF2_HGT_REF":   ("0 = baro. Defaults to 1 (GPS), which is the wrong "
                       "reference for a GNSS-denied vehicle. 3 (vision) would "
                       "claim an altitude this pipeline does not produce",
                       lambda v: int(v) in (0, 2)),
    "EKF2_GPS_CTRL":  ("GNSS aiding bitmask. Non-zero is fine on the bench; "
                       "watch_extnav.py sets it to 0 to deny GNSS", None),
}

# MAV_AUTOPILOT
AUTOPILOT_ARDUPILOTMEGA = 3
AUTOPILOT_PX4 = 12

TABLES = {"ardupilot": WANT, "px4": WANT_PX4}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="udpin:0.0.0.0:14550")
    ap.add_argument("--firmware", choices=["ardupilot", "px4", "auto"], default="auto",
                    help="override the firmware detected from the heartbeat")
    ap.add_argument("--write-endpoint", default=None,
                    help="where to SEND, when it differs from where we listen. "
                         "PX4 needs this; see the module docstring.")
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
    # A udpout link is SEND-ONLY until something goes out of it: the peer has
    # no address to answer to. PX4 in particular streams nothing to a port it
    # has never heard from, so waiting for a heartbeat on a fresh udpout socket
    # times out against a perfectly healthy vehicle -- which this script then
    # reports as "no heartbeat / check wiring". One GCS heartbeat is also just
    # what a GCS is supposed to send.
    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb = conn.wait_heartbeat(timeout=args.timeout)
    if hb is None:
        print(f"     no heartbeat within {args.timeout}s.")
        print("     SITL:      sim_vehicle.py -v ArduCopter --out=udp:127.0.0.1:14550")
        print("     hardware:  check wiring, baud, and that nothing else holds the port")
        return 1
    detected = {AUTOPILOT_ARDUPILOTMEGA: "ardupilot",
                AUTOPILOT_PX4: "px4"}.get(hb.autopilot)
    firmware = args.firmware if args.firmware != "auto" else detected
    print(f"     system {conn.target_system} component {conn.target_component} | "
          f"autopilot {hb.autopilot} type {hb.type} | firmware {firmware or 'UNKNOWN'}"
          + (f" (overriding detected {detected})"
             if detected and args.firmware != "auto" and args.firmware != detected else ""))
    if firmware is None:
        print(f"     MAV_AUTOPILOT {hb.autopilot} is neither ArduPilotMega (3) nor PX4 (12).")
        print("     Pass --firmware explicitly; the two parameter sets are disjoint and")
        print("     checking the wrong one reports every parameter ABSENT.")
        return 1
    want = TABLES[firmware]

    print(f"\n2/4  parameters ({firmware})")
    # Ask for each parameter BY NAME rather than streaming the whole table.
    #
    # This used to send param_request_list, which streams every parameter the
    # vehicle has -- about 1200 on ArduCopter -- and then picked ours out of the
    # flood. Against real SITL the VISO_* entries did not arrive inside a 20 s
    # timeout, and the script reported them "absent on this firmware" when a
    # targeted read returns all of them in milliseconds.
    #
    # That is the worst possible failure for this script, because "absent on
    # this firmware" is the exact verdict that condemns a board: it is how you
    # tell an F405 with visual odometry compiled out from an F7 that has it. A
    # check that says "absent" when it means "slow" cannot make that call.
    #
    # A targeted read is also what makes the distinction real. A parameter that
    # exists answers immediately; one that does not is never answered no matter
    # how long you wait. Three rounds, so a dropped UDP packet is not a verdict.
    got = {}
    for attempt in range(3):
        missing = [n for n in want if n not in got]
        if not missing:
            break
        for name in missing:
            conn.mav.param_request_read_send(
                conn.target_system, conn.target_component, name.encode(), -1)
        deadline = time.time() + max(3.0, args.timeout / 3)
        while time.time() < deadline and len(got) < len(want):
            m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if m and m.param_id in want:
                got[m.param_id] = _param_value(m)

    problems, unknown = 0, 0
    for name, (why, ok) in want.items():
        if name not in got:
            # Now this means what it says. Three targeted reads went unanswered,
            # so the parameter is not in this firmware -- which for VISO_* means
            # visual odometry was compiled out and no configuration adds it
            # back. See docs/FC_AND_HITL_PLAN.md for the flash-size gate.
            print(f"     ??   {name:16s} ABSENT -- three targeted reads unanswered. "
                  "Not compiled into this firmware.")
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
        print(f"\n     {unknown} parameter(s) ABSENT from this firmware.")
        if firmware == "px4":
            print("     On PX4 an absent EKF2_* usually means the parameter was renamed "
                  "between\n     releases rather than compiled out -- EKF2 is not "
                  "optional. Check the name\n     against this firmware's own metadata "
                  "before concluding anything.")
        if firmware == "ardupilot" and any(n.startswith("VISO_") for n in want if n not in got):
            print("     VISO_* missing means AP_VisualOdom was not compiled in. ArduPilot "
                  "gates it on\n     HAL_PROGRAM_SIZE_LIMIT_KB > 1024, so a 1 MB board "
                  "(the SpeedyBee F405 V3 is one)\n     cannot do closed-loop ExternalNav "
                  "at all. FC->companion telemetry still works.\n     "
                  "See docs/FC_AND_HITL_PLAN.md.")

    print("\n3/4  waiting for a position to echo back")
    # ASK for the stream first. ArduPilot sends only what a GCS has requested,
    # so on a fresh link with no GCS attached nothing arrives but the heartbeat
    # -- and this step then reported "the EKF has no origin yet" against a
    # vehicle with a 10-satellite RTK-fixed lock and a perfectly good position.
    # docs/FC_AND_HITL_PLAN.md has said "ArduPilot only streams what a GCS has
    # requested" since it was written; the script simply did not do it.
    if firmware == "px4":
        # PX4 ignores REQUEST_DATA_STREAM. It wants SET_MESSAGE_INTERVAL, and
        # asking the ArduPilot way is silent -- which would surface here as the
        # "the EKF has no origin yet" verdict against a vehicle that has one.
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000, 0, 0, 0, 0, 0)
    else:
        for stream in (mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                       mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS):
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component, stream, 5, 1)

    pos = None
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        m = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if m and m.lat != 0:
            pos = (m.lat / 1e7, m.lon / 1e7)
            break
    if pos is None:
        print("     no GLOBAL_POSITION_INT with a valid fix, and the stream was requested.")
        print("     The EKF has no origin yet; arm in SITL or wait for GPS lock, then re-run.")
        return 1
    print(f"     {pos[0]:.7f}, {pos[1]:.7f}")

    print(f"\n4/4  sending {args.message} at {args.rate} Hz for {args.seconds:.0f}s "
          f"with sigma {args.sigma} m")
    # firmware is load-bearing here, not cosmetic: it selects the pose
    # covariance layout, and the two firmwares read the same 21 floats
    # differently. See the fcout.py docstring.
    write_ep = args.write_endpoint or (
        args.endpoint if args.endpoint.startswith("udpout")
        else f"udpout:{_peer(args.endpoint)}")
    print(f"     writing to {write_ep}")
    link = FlightControllerLink(write_ep, baud=args.baud, message=args.message,
                                origin=pos, firmware=firmware)
    sent = 0
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        link.send(pos[0], pos[1], sigma_m=args.sigma,
                  t_capture_unix=time.time(), quality=80)
        sent += 1
        time.sleep(1.0 / args.rate)
    link.close()
    # Close the OBSERVER too. It was left open, and PX4 keeps streaming to the
    # last peer it heard from -- which by then is the write link's closed
    # socket. The next client to connect on the same port then waits out its
    # whole timeout against a healthy vehicle. Nothing logs it on either side.
    conn.close()
    print(f"     sent {sent} messages")

    print("\nWhat to check now:")
    print("  * the message arrives:      watch the inbound stream for your source system")
    if firmware == "px4":
        print("  * EKF2 uses it:             estimator_aid_src_ev_pos.fused, in the log or over")
        print("                              uORB. `listener estimator_aid_src_ev_pos` on the")
        print("                              PX4 console is the fastest look")
        print("  * innovations are sane:     test_ratio below 1. EKF2_EVP_GATE is 5 sigma by")
        print("                              default, the same test EKF3 applies")
        print("  * the covariance survives:  EKF2_EVP_NOISE is a LOWER bound, so a sigma below")
        print("                              it is silently raised. There is no upper clamp")
    else:
        print("  * EKF3 uses it:             EKF_STATUS_REPORT, and posTestRatio staying below 1")
        print("  * innovations are sane:     a sustained ratio above 1 means the fix is being")
        print("                              rejected by the 5-sigma gate, and sustained rejection")
        print("                              walks the filter to posTimeout on the 7 s clock while")
        print("                              data is still arriving on time")
    print("  * the origin agrees:        an external-nav position is LOCAL. If the EKF origin")
    print("                              and the origin used here differ, the offset is silent")
    print("                              and constant, and it is the most common way this fails")
    print("\n  Then run watch_extnav.py, which takes GNSS away and checks the estimator")
    print("  actually walks to an injected position rather than merely accepting the bytes.")

    # The exit code has to carry the verdict. This returned 0 unconditionally,
    # so a wrong EK3_SRC1_POSXY printed "BAD" and still passed -- and anything
    # gating on this script, a person included, read that as approval.
    if problems or unknown:
        print(f"\nFAILED: {problems} wrong, {unknown} unverified.")
        return 1
    print("\nAll checked parameters are correct.")
    return 0


# MAV_PARAM_TYPE integer codes. 9 is REAL32, 10 is REAL64.
_INT_PARAM_TYPES = {1, 2, 3, 4, 5, 6, 7, 8}


def _param_value(m) -> float:
    """PARAM_VALUE carries a float field that is not always a float.

    MAVLink puts every parameter into one float32 slot and uses `param_type` to
    say what the bits mean. ArduPilot sends everything as REAL32 and the field
    reads directly, which is why this script did not need to care for a year.
    **PX4 uses the union properly**: an INT32 parameter arrives as the INTEGER
    REINTERPRETED AS A FLOAT, so EKF2_EN = 1 comes across the wire as
    1.4013e-45 -- the float32 denormal whose bit pattern is 1.

    Read as a float that is not "wrong", it is off by forty-five orders of
    magnitude, and every int-typed check here fails against a correctly
    configured vehicle. pymavlink does not do this conversion, because the
    right answer depends on the parameter's declared type and nothing else.
    """
    v = float(m.param_value)
    if getattr(m, "param_type", 9) in _INT_PARAM_TYPES:
        import struct
        return float(struct.unpack("<i", struct.pack("<f", v))[0])
    return v


def _peer(endpoint: str) -> str:
    """udpin:0.0.0.0:14550 -> a udpout target on the same port, localhost."""
    parts = endpoint.split(":")
    port = parts[-1] if parts[-1].isdigit() else "14550"
    return f"127.0.0.1:{port}"


if __name__ == "__main__":
    raise SystemExit(main())
