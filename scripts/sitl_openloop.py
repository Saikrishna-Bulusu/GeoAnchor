#!/usr/bin/env python3
"""Open-loop validation against ArduPilot SITL.

    bash sitl.sh                 check only, then run an open-loop session
    bash sitl.sh --set-params    write the ExternalNav parameters first

Open loop here means the professor's middle mode: the ACTUAL GPS is echoed back
to the vehicle over the ExternalNav path. The position cannot mislead the
filter because the vehicle already has it, so every failure this finds is a
plumbing failure -- a wrong frame, a poisoned covariance, a mismatched origin,
a parameter that was never set. Only once this is clean is a predicted position
worth sending.

Start SITL first, in another terminal:

    sim_vehicle.py -v ArduCopter --console --out=udp:127.0.0.1:14550

--set-params writes to the autopilot. That is a configuration change, so it is
opt-in and never the default. Do not point it at an aircraft you have not
finished configuring by hand.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

# From snktshrma's Non-GPS Navigation setup document, plus the two this
# pipeline needs that his list does not name.
PARAMS = {
    "AHRS_EKF_TYPE": 3,
    "VISO_TYPE": 1,
    "VISO_DELAY_MS": 50,      # replace with YOUR measured median latency
    "VISO_POS_X": 0.0,
    "VISO_POS_Y": 0.0,
    "VISO_POS_Z": 0.0,
    "EK3_SRC1_POSXY": 6,      # ExternalNav
    "EK3_SRC1_VELXY": 0,
    "EK3_SRC1_VELZ": 0,
    "EK3_SRC1_POSZ": 1,       # baro: altitude stays on the flight controller
    "EK3_SRC1_YAW": 1,        # compass: this pipeline sends an identity quaternion
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="udpin:0.0.0.0:14550")
    ap.add_argument("--set-params", action="store_true",
                    help="WRITES to the autopilot. SITL only unless you are sure.")
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--config", default="configs/system.yaml")
    ap.add_argument("--skip-session", action="store_true")
    args = ap.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError:
        print("pymavlink is not installed -- run bootstrap.sh")
        return 2

    print(f"connecting to {args.endpoint}")
    conn = mavutil.mavlink_connection(args.endpoint, source_system=255)
    if conn.wait_heartbeat(timeout=25) is None:
        print("  no heartbeat. Start SITL:")
        print("    sim_vehicle.py -v ArduCopter --console --out=udp:127.0.0.1:14550")
        return 1
    print(f"  system {conn.target_system}, component {conn.target_component}")

    if args.set_params:
        print("\nwriting parameters")
        for name, val in PARAMS.items():
            conn.mav.param_set_send(conn.target_system, conn.target_component,
                                    name.encode(), float(val),
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.06)
        time.sleep(1.5)
        print("  written. Some ExternalNav parameters only take effect after a reboot.")

    print("\nchecking parameters")
    rc = subprocess.run([sys.executable, str(REPO / "scripts" / "check_extnav.py"),
                         "--endpoint", args.endpoint, "--seconds", "6"],
                        cwd=REPO).returncode
    if rc != 0:
        print("\ncheck_extnav reported problems. Fix them before the session; an "
              "ExternalNav fix is ignored in silence otherwise.")
        if not args.set_params:
            print("Re-run with --set-params to write them (SITL only).")
        return rc

    if args.skip_session:
        return 0

    print(f"\nrunning an open-loop session for {args.seconds:.0f}s")
    print("  the ACTUAL GPS is echoed to the vehicle; the predicted fix is logged only")
    env = {
        "GEOANCHOR_SET": ";".join([
            "output_layer.fc.enabled=true",
            "output_layer.fc.loop_mode=open",
            f"output_layer.fc.endpoint=udpout:127.0.0.1:{args.endpoint.rsplit(':', 1)[-1]}",
            "data_layer.gps.source=mavlink",
            f"data_layer.gps.endpoint={args.endpoint}",
            "data_layer.feed.loop=true",
            "data_layer.feed.fps=2",
        ]),
    }
    import os
    run_dir = REPO / "runs" / f"sitl_{int(time.time())}"
    env["GEOANCHOR_RUN_DIR"] = str(run_dir)
    proc = subprocess.Popen("bash run.sh --tag sitl", shell=True, cwd=REPO,
                            env={**os.environ, **env}, preexec_fn=os.setsid)
    try:
        deadline = time.time() + args.seconds
        seen = 0
        while time.time() < deadline:
            m = conn.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=2)
            if m is not None:
                seen += 1
                if seen % 10 == 1:
                    print(f"  EKF pos_horiz_variance {m.pos_horiz_variance:.4f}  "
                          f"flags 0x{m.flags:04x}", flush=True)
    finally:
        import signal
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=30)
        except Exception:
            pass

    print(f"\nsession: {run_dir}")
    print("\nWhat to look at now:")
    print("  * runs/*/output.jsonl for OL-14, which is the fix actually leaving the layer")
    print("  * OLDE-02 means the vehicle stopped answering; OLDE-03 means the send failed")
    print("  * in the SITL console, EKF3 should not be reporting a position timeout")
    print("  * posTestRatio matters more than the feed rate: sustained rejection walks")
    print("    the filter to posTimeout on the 7 s clock while data still arrives on time")
    print("  * if nothing at all reaches EKF3, check the ORIGIN before anything else --")
    print("    an ExternalNav position is local, and a mismatched origin is silent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
