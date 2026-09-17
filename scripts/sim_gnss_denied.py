#!/usr/bin/env python3
"""Take GNSS away from the flying simulated aircraft and measure what happens.

    .venv/bin/python scripts/sim_gnss_denied.py                # vision aiding on
    .venv/bin/python scripts/sim_gnss_denied.py --no-vision    # the control

It starts its own MAVLink instance, because by the time a rig has been flying
a while every existing one is locked to a peer that has exited. See
own_instance().

Run it against a live `scripts/sim_fixedwing.sh`, once the aircraft is in the
envelope.

WHY THIS IS A DIFFERENT CLAIM FROM THE ONE ALREADY PROVEN. The rig shows fixes
being FUSED -- `fused: True`, `cs_aux_gpos: True`, the observation variance
arriving intact -- but it shows that alongside a healthy GPS. An estimator with
good GNSS will look excellent whether or not the vision fix contributes
anything at all. The project's claim is that the fix can REPLACE GNSS, and
nothing measured so far tests it.

**TRUTH MUST COME FROM GAZEBO, NOT FROM MAVLINK.** This is the trap the whole
script is built around. With GNSS denied, `GLOBAL_POSITION_INT` and
`vehicle_global_position` are the ESTIMATE -- and the estimate is being driven
by the very fixes under test. Scoring against them asks the fix how well it
agrees with itself and returns a beautiful number that means nothing. The
simulator knows where the aircraft actually is; ask it.

The comparison is done in LOCAL metres, not lat/lon, because both sides already
speak it and a geodetic conversion is an extra place to be quietly wrong:

    Gazebo pose is ENU about the world origin      north = y, east = x
    PX4 LOCAL_POSITION_NED is about the EKF origin north = x, east = y

and sim_fixedwing.sh sets PX4_HOME from the world's own `spherical_coordinates`
so the two origins are the same point.

**--no-vision IS NOT OPTIONAL.** A fixed wing with a good IMU dead-reckons
tolerably for a while, so "the estimate held for 60 s" is not evidence the
vision fix did anything. Run both and report the pair.

IT PUTS GNSS BACK, in a finally block, and confirms it. PX4 autosaves
parameters, so an EKF2_GPS_CTRL left at 0 makes the NEXT boot permanently
GNSS-denied -- it never reaches "Ready for takeoff" and the cause is a
parameter written by a script that exited hours earlier.
"""
from __future__ import annotations

import argparse
import math
import re
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

MAV_PARAM_TYPE_INT32 = 6
DENIAL = b"EKF2_GPS_CTRL"


class Truth:
    """The aircraft's real pose, straight out of the simulator."""

    def __init__(self, world: str, model: str):
        # Same append-don't-prepend rule as GzFeed: PYTHONPATH would put the
        # system dist-packages ahead of the venv and break pydantic.
        apt = "/usr/lib/python3/dist-packages"
        if apt not in sys.path:
            sys.path.append(apt)
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V

        self.model = model
        self._lock = threading.Lock()
        self._xy = None
        self._node = Node()
        topic = f"/world/{world}/dynamic_pose/info"
        if not self._node.subscribe(Pose_V, topic, self._on_pose):
            raise SystemExit(f"could not subscribe to {topic}")
        self.topic = topic

    def _on_pose(self, msg):
        for p in msg.pose:
            if p.name == self.model:
                with self._lock:
                    self._xy = (p.position.x, p.position.y, p.position.z)
                return

    def north_east(self):
        with self._lock:
            if self._xy is None:
                return None
            x, y, _ = self._xy
        return y, x          # ENU -> north, east


def own_instance(px4_dir: str, port: int) -> bool:
    """Give ourselves a MAVLink channel nothing else has touched.

    PX4 binds each UDP MAVLink instance to whichever peer speaks first and
    NEVER releases it. By the time a rig has been flying a while the pipeline
    owns the onboard instance and px4_set_params.py and sim_fly.py own the
    others and have both exited -- so a tool attached afterwards gets
    "no heartbeat" from every one of them while the autopilot is perfectly
    healthy. (`px4-listener` on the uORB console still works, which is how you
    tell that apart from a dead PX4.)

    Worse, every probe burns an instance and PX4 caps at six:
    "ERROR [mavlink] Maximum MAVLink instance count of 6 reached." So this
    stops the port first, then starts it, and the caller must be the FIRST
    thing to speak on it -- do not test the connection before using it."""
    px4 = Path(px4_dir) / "build/px4_sitl_default/bin/px4-mavlink"
    if not px4.exists():
        return False
    cwd = REPO / ".px4_sim"
    subprocess.run([str(px4), "stop", "-u", str(port)],
                   cwd=cwd, capture_output=True)
    r = subprocess.run([str(px4), "start", "-x", "-u", str(port),
                        "-o", str(port + 1), "-r", "200000", "-m", "onboard"],
                       cwd=cwd, capture_output=True, text=True)
    if r.returncode != 0:
        msg = (r.stderr or r.stdout).strip()
        print(f"could not start a MAVLink instance on {port}: {msg}")
        if "Maximum MAVLink instance count" in msg:
            # Actionable, because "maximum instance count" says nothing about
            # which ones are disposable. Anything whose peer has exited is.
            st = subprocess.run([str(px4), "status"], cwd=cwd,
                                capture_output=True, text=True).stdout
            ports = sorted({w.split(":")[-1] for w in st.split()
                            if w.startswith("port") or w.isdigit()})
            print("  PX4 caps at 6 and every instance is taken. Recycle one "
                  "whose client has exited:")
            print(f"    {px4} stop -u <port>")
            if ports:
                print(f"  in use: {', '.join(ports)}")
            print("  `px4-mavlink status` lists them with their partner "
                  "addresses; an instance whose partner no longer exists is "
                  "dead weight and safe to stop.")
        return False
    time.sleep(2)
    return True


class FixTap:
    """Every fix the processing layer emits, scored against GAZEBO truth.

    This is what turns "the estimator excursed" into "and here is what was fed
    to it while it did". The pipeline's own `error_m` cannot answer it: under
    GNSS denial that is computed against MAVLink, which is the estimate the fix
    is itself driving, so a fix that drags the estimate 500 m looks like a fix
    that agrees with the estimate perfectly.

    Converts each fix's lat/lon to the same local ENU frame the truth is in.
    Equirectangular about the world origin -- at a few hundred metres the
    difference from a proper projection is millimetres, and the alternative is
    a second geodetic path to get quietly wrong."""

    def __init__(self, endpoint, lat0, lon0):
        sys.path.insert(0, str(REPO))
        from geoanchor import contracts as K
        from geoanchor.bus import Subscriber
        self.lat0, self.lon0 = lat0, lon0
        self.m_per_deg_lat = 111_320.0
        self.m_per_deg_lon = 111_320.0 * math.cos(math.radians(lat0))
        self.sub = Subscriber([endpoint], [K.T_FIX])
        self.rows = []
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._stop:
            try:
                got = self.sub.recv(timeout_ms=200)
            except Exception:
                continue
            if not got:
                continue
            _topic, header, _payload = got
            self.rows.append((time.time(), header))

    def stop(self):
        self._stop = True

    def to_local(self, lat, lon):
        return ((lat - self.lat0) * self.m_per_deg_lat,
                (lon - self.lon0) * self.m_per_deg_lon)


def world_origin(world: str):
    """The world's own spherical_coordinates -- the same origin PX4 is given."""
    sdf = REPO / "sim" / "gz" / "worlds" / f"{world}.sdf"
    txt = sdf.read_text()
    lat = float(re.search(r"<latitude_deg>([-\d.]+)", txt).group(1))
    lon = float(re.search(r"<longitude_deg>([-\d.]+)", txt).group(1))
    return lat, lon


def connect(endpoint):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(endpoint)
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    if m.wait_heartbeat(timeout=20) is None:
        raise SystemExit(f"no heartbeat on {endpoint}. Is the sim running?")
    m.target_component = 1
    for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0),):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, 1e6 / hz, 0, 0, 0, 0, 0)
    return m


def bridge_pids():
    out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True).stdout
    return [int(l.split()[0]) for l in out.splitlines()
            if "scripts/agp_bridge.py" in l and "grep" not in l]


def sample(m, truth, seconds, label, rows):
    """Collect (t, error_m, est, truth) for `seconds`, printing as it goes."""
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < seconds:
        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            continue
        te = truth.north_east()
        if te is None:
            continue
        tn, tee = te
        err = math.hypot(msg.x - tn, msg.y - tee)
        t = time.time() - t0
        rows.append((label, t, err, msg.x, msg.y, tn, tee, time.time()))
        if t - last_print >= 5.0:
            last_print = t
            print(f"  {label:9s} t={t:5.1f}s  drift {err:7.2f} m", flush=True)


def summarise(rows, label):
    e = sorted(r[2] for r in rows if r[0] == label)
    if not e:
        return None
    def q(p):
        return e[min(len(e) - 1, int(round(p * (len(e) - 1))))]
    return {"n": len(e), "median": q(.5), "p90": q(.9), "max": e[-1]}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14590,
                    help="UDP port to start our OWN MAVLink instance on. See "
                         "own_instance() for why borrowing one does not work.")
    ap.add_argument("--endpoint", default=None,
                    help="skip starting an instance and use this endpoint "
                         "instead. You are on your own for the peer lock.")
    ap.add_argument("--px4", default="/home/sai/thesis_2.0/PX4-Autopilot")
    ap.add_argument("--world", default="geoanchor_rural")
    ap.add_argument("--model", default="geoanchor_cessna_0")
    ap.add_argument("--baseline", type=float, default=30.0,
                    help="seconds with GNSS still on, to prove the tap agrees")
    ap.add_argument("--seconds", type=float, default=180.0,
                    help="seconds to hold GNSS denied")
    ap.add_argument("--fix-endpoint", default="ipc:///tmp/geoanchor/processing.sock",
                    help="where the processing layer publishes fixes")
    ap.add_argument("--no-vision", action="store_true",
                    help="THE CONTROL. Stop the AGP bridge first, so the "
                         "estimator has nothing but inertial dead reckoning.")
    a = ap.parse_args()

    from pymavlink import mavutil

    truth = Truth(a.world, a.model)
    print(f"truth from {truth.topic}")
    deadline = time.time() + 20
    while truth.north_east() is None and time.time() < deadline:
        time.sleep(0.2)
    if truth.north_east() is None:
        return fail(f"no pose for '{a.model}' on {truth.topic}. "
                    "`gz topic -e -t <topic>` lists the model names it carries.")

    endpoint = a.endpoint
    if endpoint is None:
        if not own_instance(a.px4, a.port):
            return fail("could not provision a MAVLink instance. Pass "
                        "--endpoint to use an existing one.")
        endpoint = f"udpout:127.0.0.1:{a.port}"
        print(f"started our own MAVLink instance on {a.port}")
    m = connect(endpoint)
    print(f"px4 on {endpoint}, system {m.target_system}")

    if a.no_vision:
        pids = bridge_pids()
        for p in pids:
            subprocess.run(["kill", "-9", str(p)])
        print(f"CONTROL RUN: stopped {len(pids)} agp_bridge process(es); "
              "the estimator now has no vision aiding")
        time.sleep(2)

    lat0, lon0 = world_origin(a.world)
    tap = None
    try:
        tap = FixTap(a.fix_endpoint, lat0, lon0)
        print(f"fix stream from {a.fix_endpoint}, origin {lat0:.7f} {lon0:.7f}")
    except Exception as exc:
        print(f"no fix stream ({exc}) -- estimator drift only, no correlation")

    rows = []
    print(f"\nbaseline, GNSS ON, {a.baseline:.0f}s "
          "-- drift here is the tap disagreeing, not the estimator failing")
    sample(m, truth, a.baseline, "baseline", rows)
    base = summarise(rows, "baseline")
    if base and base["median"] > 5.0:
        print(f"\n  WARNING: {base['median']:.1f} m median with GNSS still on. "
              "The two origins disagree, so every number below is offset by a "
              "constant. Check PX4_HOME against the world's spherical_coordinates.")

    # ---- read what to restore, then deny -----------------------------------
    m.mav.param_request_read_send(m.target_system, m.target_component, DENIAL, -1)
    original = original_type = None
    end = time.time() + 5.0
    while time.time() < end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id == DENIAL.decode():
            original, original_type = msg.param_value, msg.param_type
            break
    if original is None:
        return fail("could not read EKF2_GPS_CTRL -- refusing to deny GNSS "
                    "without knowing what to put back")

    def restore():
        m.mav.param_set_send(m.target_system, m.target_component,
                             DENIAL, original, original_type)
        end = time.time() + 5.0
        while time.time() < end:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and msg.param_id == DENIAL.decode():
                print("EKF2_GPS_CTRL restored")
                return True
        print("WARNING: EKF2_GPS_CTRL was NOT confirmed restored. PX4 autosaves, "
              "so the next boot may come up permanently GNSS-denied and never "
              "reach 'Ready for takeoff'. Delete .px4_sim/parameters.bson if so.")
        return False

    try:
        # 0.0f and int 0 share a bit pattern, so this one value is safe to send
        # as a float for an int parameter. No other value would be.
        m.mav.param_set_send(m.target_system, m.target_component,
                             DENIAL, 0, MAV_PARAM_TYPE_INT32)
        time.sleep(2)
        print(f"\nGNSS DENIED (EKF2_GPS_CTRL 0), holding {a.seconds:.0f}s "
              f"-- vision aiding {'OFF (control)' if a.no_vision else 'ON'}")
        sample(m, truth, a.seconds, "denied", rows)
    finally:
        restore()

    # ---- report -------------------------------------------------------------
    print()
    for label in ("baseline", "denied"):
        s = summarise(rows, label)
        if s:
            print(f"{label:9s} n={s['n']:5d}  median {s['median']:7.2f} m  "
                  f"p90 {s['p90']:7.2f}  max {s['max']:7.2f}")

    den = [r for r in rows if r[0] == "denied"]
    if len(den) > 20:
        # Drift RATE is the number that matters: a constant offset is a frame
        # disagreement, a growing one is the estimator losing the position.
        first = [r[2] for r in den[:len(den)//5]]
        last = [r[2] for r in den[-len(den)//5:]]
        span = den[-1][1] - den[0][1]
        growth = statistics.median(last) - statistics.median(first)
        print(f"\ndrift growth over {span:.0f}s: "
              f"{statistics.median(first):.2f} -> {statistics.median(last):.2f} m "
              f"({growth/max(span,1)*60:+.2f} m/min)")

    # THE DURATION IS IN THE NAME. Without it a 25-second smoke test silently
    # overwrites the 180-second run it was testing the machinery for, and the
    # loss is invisible -- the file is still there and still parses.
    out = REPO / "results" / (f"sim_gnss_denied_{'control' if a.no_vision else 'vision'}"
                              f"_{int(a.seconds)}s.csv")
    out.parent.mkdir(exist_ok=True)
    with out.open("w") as fh:
        # t_unix, because without an absolute clock these samples cannot be
        # put beside the fix stream, and that correlation is the whole point.
        fh.write("phase,t_s,t_unix,error_m,est_n,est_e,truth_n,truth_e\n")
        for r in rows:
            fh.write(f"{r[0]},{r[1]:.3f},{r[7]:.3f},{r[2]:.4f},{r[3]:.3f},"
                     f"{r[4]:.3f},{r[5]:.3f},{r[6]:.3f}\n")
    print(f"\nwrote {out.relative_to(REPO)}")

    if tap is not None:
        tap.stop()
        fout = out.with_name(out.name.replace("sim_gnss_denied_", "sim_gnss_fixes_"))
        n_acc = 0
        with fout.open("w") as fh:
            fh.write("t_unix,accepted,inliers,matches,sigma_m,reproj_err_px,"
                     "fix_n,fix_e,truth_n,truth_e,fix_error_m\n")
            for t_rx, h in tap.rows:
                if h.get("lat") is None:
                    continue
                fn, fe = tap.to_local(h["lat"], h["lon"])
                # Truth at the nearest estimator sample, which shares this clock.
                near = min(rows, key=lambda r: abs(r[7] - t_rx)) if rows else None
                tn, te = (near[5], near[6]) if near else (float("nan"),) * 2
                err = math.hypot(fn - tn, fe - te)
                n_acc += 1 if h.get("accepted") else 0
                fh.write(f"{t_rx:.3f},{int(bool(h.get('accepted')))},"
                         f"{h.get('inliers', 0)},{h.get('matches', 0)},"
                         f"{h.get('sigma_m') if h.get('sigma_m') is not None else ''},"
                         f"{h.get('reproj_err_px') if h.get('reproj_err_px') is not None else ''},"
                         f"{fn:.3f},{fe:.3f},{tn:.3f},{te:.3f},{err:.3f}\n")
        print(f"wrote {fout.relative_to(REPO)}  "
              f"({len(tap.rows)} fixes seen, {n_acc} accepted)")
    return 0


def fail(msg):
    print(msg, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
