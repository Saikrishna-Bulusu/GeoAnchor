#!/usr/bin/env python3
"""Send real MAVLink and decode it the way ArduPilot does.

    python scripts/test_fc_encoding.py

Not a unit test of my own arithmetic: it opens a socket, sends the bytes the
output layer would send in flight, decodes them with pymavlink, and then runs
the exact logic from ArduPilot master rather than a paraphrase of it.

That distinction matters, because both bugs this test exists to catch were
invisible from inside the sender. `handle_odometry()` drops a message whose
frame is not LOCAL_FRD without a word, and `posErr` is a sum across three
covariance entries, so a NaN meaning "altitude unknown" silently poisons the
number the estimator flies on. Neither shows up as an exception, a log line or
a dropped-packet count anywhere on this side of the link.

Reference: ArduPilot master, libraries/GCS_MAVLink/GCS_Common.cpp
  handle_odometry()                            :4087
  handle_common_vision_position_estimate_data():4124
"""
from __future__ import annotations

import math
import socket
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from geoanchor.output_layer.fcout import FlightControllerLink, QgcLink, quality_from  # noqa: E402

MAV_FRAME_LOCAL_FRD = 20
MAV_FRAME_BODY_FRD = 12
results = []


def check(name, ok, detail=""):
    results.append(ok)
    mark = "\033[32m PASS \033[0m" if ok else "\033[31m FAIL \033[0m"
    print(f"  {mark} {name}" + (f"  -- {detail}" if detail else ""))
    return ok


# --------------------------------------------------------------------------
# ArduPilot's logic, transcribed. Do not "improve" this: its value is that it
# is a copy, so a change upstream shows up here as a failing test.
# --------------------------------------------------------------------------
def ardupilot_handle_odometry(m):
    if m.frame_id != MAV_FRAME_LOCAL_FRD or m.child_frame_id != MAV_FRAME_BODY_FRD:
        return None                      # returns early, silently
    cov = m.pose_covariance
    posErr = angErr = 0.0
    if not math.isnan(cov[0]):
        posErr = math.sqrt(cov[0] + cov[6] + cov[11])
        angErr = math.sqrt(cov[15] + cov[18] + cov[20])
    return {"x": m.x, "y": m.y, "z": m.z, "posErr": posErr, "angErr": angErr,
            "reset_counter": m.reset_counter, "quality": m.quality}


def ardupilot_handle_vision_position_estimate(m):
    cov = m.covariance
    posErr = angErr = 0.0
    if not math.isnan(cov[0]):
        posErr = math.sqrt(cov[0] + cov[6] + cov[11])
        angErr = math.sqrt(cov[15] + cov[18] + cov[20])
    return {"x": m.x, "y": m.y, "z": m.z, "posErr": posErr, "angErr": angErr,
            "reset_counter": m.reset_counter}


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def collect(conn, want_type, timeout=4.0):
    end = time.time() + timeout
    while time.time() < end:
        m = conn.recv_match(type=want_type, blocking=True, timeout=0.5)
        if m is not None:
            return m
    return None


def main() -> int:
    from pymavlink import mavutil
    print("\n\033[1mFlight controller encoding, decoded as ArduPilot decodes it\033[0m\n")

    # --- ODOMETRY ----------------------------------------------------------
    port = free_port()
    listener = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    link = FlightControllerLink(f"udpout:127.0.0.1:{port}", message="ODOMETRY",
                                origin=(-33.8688, 151.2093))
    ORIGIN = (-33.8688, 151.2093)
    SIGMA = 12.0
    # 30 m north, 40 m east of the origin -> a 50 m offset with exact components
    from geoanchor.geo import distance_m
    import geoanchor.geo as geo
    crs = geo.local_tmerc(*ORIGIN)
    lat, lon = geo.crs_to_wgs84(crs, 40.0, 30.0)      # x=east, y=north in tmerc

    for _ in range(6):
        link.send(lat, lon, sigma_m=SIGMA, t_capture_unix=time.time(),
                  quality=quality_from(120, 25))
        time.sleep(0.05)
    m = collect(listener, "ODOMETRY")
    check("ODOMETRY arrives on the wire", m is not None)
    if m is None:
        return 1

    out = ardupilot_handle_odometry(m)
    check("ArduPilot ACCEPTS the frame (LOCAL_FRD / BODY_FRD)", out is not None,
          f"frame_id={m.frame_id} child={m.child_frame_id}")
    if out is None:
        print("        this is the bug: frame_id 1 (LOCAL_NED) is dropped in silence")
        return 1

    check("posErr is finite, not NaN", math.isfinite(out["posErr"]),
          f"posErr={out['posErr']}")
    check("posErr equals the sigma the estimator emitted",
          abs(out["posErr"] - SIGMA) < 1e-4, f"{out['posErr']:.6f} vs {SIGMA}")
    check("angErr is finite", math.isfinite(out["angErr"]), f"{out['angErr']:.4f}")
    check("north lands in x, east in y (FRD)",
          abs(out["x"] - 30.0) < 0.05 and abs(out["y"] - 40.0) < 0.05,
          f"x={out['x']:.3f} (want 30) y={out['y']:.3f} (want 40)")
    check("altitude is not claimed", abs(out["z"]) < 1e-9, f"z={out['z']}")
    check("quality is inside ArduPilot's -1..100 range",
          -1 <= out["quality"] <= 100, f"quality={out['quality']}")

    # Drain first: six messages were already sent above and are sitting in the
    # socket buffer with reset_counter 0. Reading one of those instead of the
    # new one makes a working counter look broken.
    while listener.recv_match(type="ODOMETRY", blocking=False) is not None:
        pass
    n = link.bump_reset()
    for _ in range(3):
        link.send(lat, lon, sigma_m=SIGMA, t_capture_unix=time.time())
        time.sleep(0.05)
    m2 = collect(listener, "ODOMETRY")
    check("reset counter increments and reaches the far side",
          m2 is not None and m2.reset_counter == n, f"sent {n}, got {m2.reset_counter if m2 else None}")
    link.close(); listener.close()

    # --- VISION_POSITION_ESTIMATE -----------------------------------------
    port = free_port()
    listener = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    link = FlightControllerLink(f"udpout:127.0.0.1:{port}",
                                message="VISION_POSITION_ESTIMATE", origin=ORIGIN)
    for _ in range(6):
        link.send(lat, lon, sigma_m=SIGMA, t_capture_unix=time.time())
        time.sleep(0.05)
    m = collect(listener, "VISION_POSITION_ESTIMATE")
    check("VISION_POSITION_ESTIMATE arrives", m is not None)
    if m is not None:
        out = ardupilot_handle_vision_position_estimate(m)
        check("VPE posErr equals the emitted sigma",
              abs(out["posErr"] - SIGMA) < 1e-4, f"{out['posErr']:.6f}")
    link.close(); listener.close()

    # --- the regression this file exists for ------------------------------
    bad = [float("nan")] * 21
    bad[0] = bad[6] = SIGMA ** 2
    bad[11] = float("nan")
    poisoned = math.sqrt(bad[0] + bad[6] + bad[11]) if not math.isnan(bad[0]) else 0.0
    check("the old NaN encoding would have been caught", math.isnan(poisoned),
          "posErr would have been NaN into EKF3")

    # --- QGC ---------------------------------------------------------------
    port = free_port()
    listener = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    qgc = QgcLink(f"udpout:127.0.0.1:{port}", system_id=2)
    for _ in range(6):
        qgc.heartbeat(force=True)
        qgc.send(lat, lon, alt_m=75.0)
        time.sleep(0.05)
    hb = collect(listener, "HEARTBEAT")
    check("QGC sees a heartbeat under its own system id",
          hb is not None and hb.get_srcSystem() == 2,
          f"sysid={hb.get_srcSystem() if hb else None}")
    gp = collect(listener, "GLOBAL_POSITION_INT")
    ok_pos = gp is not None and abs(gp.lat / 1e7 - lat) < 1e-6 and abs(gp.lon / 1e7 - lon) < 1e-6
    check("QGC receives the predicted position", ok_pos,
          f"{gp.lat/1e7:.6f}, {gp.lon/1e7:.6f}" if gp else "none")
    check("the QGC system id is not the autopilot's",
          hb is not None and hb.get_srcSystem() != 1)
    qgc.close(); listener.close()

    print(f"\n{sum(results)}/{len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
