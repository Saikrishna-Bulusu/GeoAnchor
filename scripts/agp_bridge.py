#!/usr/bin/env python3
"""Publish this pipeline's fixes to PX4's Aux Global Position over uXRCE-DDS.

    source /opt/ros/jazzy/setup.bash
    source ~/GeoAnchor/ros2_ws/install/setup.bash
    MicroXRCEAgent udp4 -p 8888 &
    python scripts/agp_bridge.py

WHY THIS EXISTS, AND WHY IT IS A SEPARATE PROCESS
-------------------------------------------------
PX4's external-vision aid source will not START fusing unless samples arrive
less than 200 ms apart (`EV_MAX_INTERVAL`, EKF/common.h:71) -- measured, four
passes, 4 Hz never fuses and 5 Hz always does. The Xavier replay runs at
2.4 Hz and `xfeat_lighterglue` at 0.47 Hz, so on PX4 most measured
configurations of this pipeline are silently ignored. ArduPilot has no such
check.

**Aux Global Position has no rate check at all.** Its entire starting
condition is finite lat/lon plus yaw alignment, and the only timing rule is a
FIVE SECOND timeout. Verified in SITL: AGP at 2 Hz with GNSS denied holds the
estimate, `fused: true`, `cs_aux_gpos: true`.

It also fits this pipeline far better than ODOMETRY. `aux_global_position` is a
`VehicleGlobalPosition`, so it takes **lat/lon in degrees** -- what the geodetic
stage already produces -- and **`eph`, one scalar standard deviation in metres**
-- what the covariance estimator already produces. No 21-float array to pack,
so none of the per-axis-versus-summed ambiguity that makes `fcout.py` pick a
layout per firmware; no local frame, so no `LOCAL_FRD`/`LOCAL_NED` rotation and
no origin to keep in step.

The catch is that `/fmu/in/aux_global_position` is reachable only over
uXRCE-DDS -- there is no MAVLink message for it -- which means a DDS
participant, which in practice means ROS 2 and `px4_msgs`.

**So this is a bridge, not a layer.** It subscribes to the processing layer's
PUB socket exactly as `geoanchor/api` does, and publishes onward. Nothing in
`geoanchor/` imports it, no layer depends on it, and a board flying ArduPilot
never runs it -- which is what keeps "no ROS in the pipeline" true while still
reaching the one PX4 path whose rate requirements this project can meet.

PARAMETERS ON THE VEHICLE (see configs/px4_agp.params):

    EKF2_AGP_CTRL   1     bit 0 = horizontal position. Defaults to 0 = OFF.
    EKF2_AGP_NOISE  0.1   LOWER BOUND on our eph. Default 0.9 would silently
                          floor every sigma tighter than 90 cm.
    EKF2_AGP_GATE   3.0   innovation gate, standard deviations
    EKF2_AGP_DELAY  ms    YOUR measured capture-to-publish median
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

# THIS PROCESS NEEDS TWO PYTHON WORLDS AT ONCE, which nothing else here does.
# rclpy and px4_msgs are ROS packages, built into a workspace and not
# pip-installable, so this must run under ROS's interpreter. zmq and the
# geoanchor package live in the repo venv. Neither environment has both.
#
# APPEND the venv, never prepend: putting it first shadows ROS's own
# dependencies with the venv's copies and produces import failures far from
# here. Appending means ROS wins every shared name and we pick up only what ROS
# does not provide. Both interpreters are 3.12 on Ubuntu 24.04, so the ABI
# matches -- check that before assuming this still holds on another release.
for _sp in sorted(REPO.glob(".venv/lib/python3.*/site-packages")):
    if str(_sp) not in sys.path:
        sys.path.append(str(_sp))

from geoanchor import config as cfgmod          # noqa: E402
from geoanchor import contracts as K            # noqa: E402
from geoanchor.bus import Subscriber            # noqa: E402

TOPIC = "/fmu/in/aux_global_position"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/system.yaml")
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--min-sigma-m", type=float, default=0.1,
                    help="floor applied to our own sigma before publishing. "
                         "EKF2_AGP_NOISE floors it again on the vehicle.")
    ap.add_argument("--send-rejected", action="store_true",
                    help="publish fixes the gate REJECTED as well. Off by "
                         "default and it should stay off in flight: the gate "
                         "is this project's rejection result and forwarding "
                         "what it rejected discards it.")
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would be published; needs no ROS")
    a = ap.parse_args()

    cfg = cfgmod.load(a.config)
    endpoint = cfg.get("bus.processing_pub", "ipc:///tmp/geoanchor/processing.sock")

    pub = None
    if not a.dry_run:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
            from px4_msgs.msg import VehicleGlobalPosition
        except ImportError as exc:
            print(f"ROS 2 / px4_msgs not importable: {exc}")
            print("  source /opt/ros/jazzy/setup.bash")
            print("  source ~/GeoAnchor/ros2_ws/install/setup.bash")
            print("Or pass --dry-run to check the mapping without ROS.")
            return 2
        rclpy.init()
        node = Node("geoanchor_agp_bridge")
        # BEST_EFFORT and depth 1, matching what PX4's uxrce_dds_client offers
        # on its inbound topics. A RELIABLE reader against a BEST_EFFORT writer
        # simply never matches, and the symptom is silence rather than an error.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        pub = node.create_publisher(VehicleGlobalPosition, a.topic, qos)
        msg_cls = VehicleGlobalPosition
        print(f"publishing {a.topic}")

    # Subscriber takes a LIST of endpoints, and recv() returns
    # (topic, header, payload) with the fix in the JSON header -- same shape
    # geoanchor/api/server.py consumes.
    sub = Subscriber([endpoint], [K.T_FIX])
    print(f"subscribed to {endpoint} [{K.T_FIX}]")
    print(f"{'seq':>6} {'lat':>12} {'lon':>12} {'eph m':>8} {'age ms':>8}  state")

    sent = skipped = 0
    try:
        while True:
            got = sub.recv(timeout_ms=500)
            if not got:
                continue
            _topic, f, _payload = got
            lat, lon, sigma = f.get("lat"), f.get("lon"), f.get("sigma_m")
            accepted = bool(f.get("accepted"))
            if not accepted and not a.send_rejected:
                skipped += 1
                continue
            # Refuse a non-finite anything. PX4's starting condition is
            # PX4_ISFINITE(lat) && PX4_ISFINITE(lon), so a NaN is dropped
            # silently on the far side -- and a NaN eph would be taken as
            # the observation variance.
            if not all(isinstance(v, (int, float)) and math.isfinite(v)
                       for v in (lat, lon, sigma)):
                skipped += 1
                continue
            eph = max(float(sigma), a.min_sigma_m)
            t_cap = float(f.get("t_capture_unix") or time.time())
            age_ms = (time.time() - t_cap) * 1e3

            if a.dry_run:
                print(f"{f.get('seq', -1):6d} {lat:12.7f} {lon:12.7f} "
                      f"{eph:8.2f} {age_ms:8.1f}  dry-run")
                sent += 1
                continue

            m = msg_cls()
            # timestamp_sample is CAPTURE time, which is what EKF2_AGP_DELAY
            # compensates from. Stamping at publish hides the pipeline's own
            # latency from the filter, which is the same trap ArduPilot's
            # VISO_DELAY_MS has.
            m.timestamp = int(time.time() * 1e6)
            m.timestamp_sample = int(t_cap * 1e6)
            m.lat = float(lat)
            m.lon = float(lon)
            m.lat_lon_valid = True
            # Altitude is NOT claimed: this pipeline does not produce one,
            # and EKF2_AGP_CTRL bit 1 is left clear so nothing reads it.
            m.alt = 0.0
            m.alt_valid = False
            m.eph = float(eph)
            m.epv = float("nan")
            pub.publish(m)
            sent += 1
            print(f"{f.get('seq', -1):6d} {lat:12.7f} {lon:12.7f} "
                  f"{eph:8.2f} {age_ms:8.1f}  sent")
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n{sent} published, {skipped} skipped "
              f"({'rejected fixes included' if a.send_rejected else 'rejected fixes dropped'})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
