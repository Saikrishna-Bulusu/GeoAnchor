#!/usr/bin/env python3
"""Put the simulated fixed wing in the air, on a box over the reference tile.

    .venv/bin/python scripts/sim_fly.py                  # 80 m AGL, 400 m box
    .venv/bin/python scripts/sim_fly.py --alt 60 --box 300

WHY A MISSION AND NOT `commander takeoff`. A fixed wing cannot hover, so a bare
takeoff leaves it circling the home point at whatever radius the airframe picks
and the camera sees the same few hundred metres of ground forever. The matcher
would then be scored on one tile's worth of texture. A box gives the pipeline
new ground on every leg, which is the only way the cross-date result in
sim/README.md means anything.

ALTITUDE IS THE EXPERIMENT. 50-100 m AGL is the envelope this whole project is
scoped to (CLAUDE.md), because rho -- ground sample ratio between frame and
reference -- falls out of it. Flying outside it does not just degrade the fix,
it tests a different system. --alt is clamped to that band on purpose.
"""
import argparse, math, sys, threading, time
from pymavlink import mavutil

AUTO_MISSION = (4 << 24) | (4 << 16)   # PX4 custom_mode: sub<<24 | main<<16
# ORBIT RADIUS IS A CAMERA PARAMETER, not a comfort one.
#
# A coordinated turn banks at tan(phi) = v^2 / (g * r), and the camera is
# rigidly mounted, so the bank IS the camera's tilt off nadir. At 20 m/s a
# 120 m orbit banks 19 degrees and the whole pipeline degrades: 387 matches
# collapsed to 5 inliers with the solved scale 3-9x off, because a steeply
# oblique view does not relate to a north-up orthorectified map by the
# near-affine homography the solve expects. 500 m banks 4.7 degrees.
#
# This was found by "fixing" a wandering aircraft into a tight orbit and
# watching acceptance go from 21% to zero.
#
# NOTE this value is advisory in DO_REPOSITION: AUTO.LOITER takes its radius
# from NAV_LOITER_RAD, which configs/px4_sim_fixedwing.params sets to match.
LOITER_RADIUS_M = 500.0


def beat(m):
    """Announce ourselves as a GCS, once a second, forever.

    PX4 with COM_RC_IN_MODE set to no-RC refuses to arm on "Preflight Fail: No
    connection to the ground control station" until something on the link is
    heartbeating at it. pymavlink does not do this on its own: opening a
    connection only listens. Without this thread the arm below is refused ten
    times and the script exits blaming the airframe."""
    while True:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


def wait_heartbeat(m, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if m.recv_match(type="HEARTBEAT", blocking=True, timeout=2):
            return True
    return False


def home_position(m, timeout=90):
    """PX4's own idea of home. Deriving the box from anything else -- the world
    SDF, a hardcoded pair -- risks a silent offset between the mission and the
    ground the aircraft is actually standing on."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=["HOME_POSITION", "GLOBAL_POSITION_INT"],
                           blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "HOME_POSITION":
            return msg.latitude / 1e7, msg.longitude / 1e7
        if msg.lat != 0 or msg.lon != 0:
            return msg.lat / 1e7, msg.lon / 1e7
    return None


def box(lat, lon, side_m):
    """Four corners, metres converted to degrees at this latitude."""
    dlat = (side_m / 2) / 111_320.0
    dlon = (side_m / 2) / (111_320.0 * math.cos(math.radians(lat)))
    return [(lat + dlat, lon - dlon), (lat + dlat, lon + dlon),
            (lat - dlat, lon + dlon), (lat - dlat, lon - dlon)]


def upload(m, items, timeout=30):
    """Serve MISSION_REQUESTs until PX4 sends the ACK.

    The ACK is the ONLY completion signal, and it can arrive in the same
    recv_match loop that is serving requests -- so it has to be captured there.
    Breaking out and then waiting for a second ACK waits forever on a mission
    that was in fact accepted, and reports a working upload as a failure.

    PX4 also re-requests items it did not like the look of, so this does not
    track which have been sent: it answers whatever seq is asked for, however
    many times, until the ACK or the timeout."""
    m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        req = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT",
                                 "MISSION_ACK"], blocking=True, timeout=2)
        if req is None:
            continue
        if req.get_type() == "MISSION_ACK":
            if req.type != 0:
                print(f"  PX4 rejected the mission: MAV_MISSION_RESULT {req.type}")
            return req.type == 0
        if req.seq >= len(items):
            continue
        cmd, lat, lon, alt, prm = items[req.seq]
        # AUTOCONTINUE 0 ON THE UNLIMITED LOITER, or it is not unlimited.
        # With autocontinue set, PX4 walks straight through the loiter into the
        # landing approach that only exists to satisfy the feasibility checker,
        # and the aircraft leaves the mapped area -- 794 m from the tile centre
        # when this was caught, far enough that the matcher's cold tile prior
        # was searching ground the camera could not see. It reads as a matcher
        # failure and is a mission one.
        cont = 0 if cmd == mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM else 1
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, req.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd,
            0, cont, prm[0], prm[1], prm[2], prm[3],
            int(lat * 1e7), int(lon * 1e7), alt, 0)
    print("  no MISSION_ACK within the timeout")
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--alt", type=float, default=80.0, help="AGL metres, 50-100")
    ap.add_argument("--box", type=float, default=400.0, help="box side, metres")
    a = ap.parse_args()

    if not 50.0 <= a.alt <= 100.0:
        sys.exit(f"--alt {a.alt} is outside the 50-100 m envelope this project "
                 f"is scoped to. See the module docstring.")

    print(f"connecting {a.endpoint}")
    m = mavutil.mavlink_connection(a.endpoint)
    if not wait_heartbeat(m):
        sys.exit("no heartbeat. Is PX4 SITL up?")
    # ADDRESS THE AUTOPILOT, not component 0. mavutil takes target_component
    # from whatever heartbeat arrives first, and PX4 emits several -- a
    # broadcast MISSION_COUNT is then answered by more than one component and
    # the mission manager returns MAV_MISSION_ERROR on a mission that is
    # perfectly valid. It is a race, so it succeeds about half the time, which
    # is worse than failing every time.
    m.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    print(f"  system {m.target_system}, component {m.target_component}")
    threading.Thread(target=beat, args=(m,), daemon=True).start()

    hp = home_position(m)
    if hp is None:
        sys.exit("no home position. PX4 has no GPS lock yet.")
    lat, lon = hp
    print(f"  home {lat:.7f}, {lon:.7f}")

    corners = box(lat, lon, a.box)
    NONE = (0.0, 0.0, 0.0, 0.0)
    items = [(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
              corners[0][0], corners[0][1], a.alt, NONE)]
    items += [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, c[0], c[1], a.alt, NONE)
              for c in corners + [corners[0]]]
    # Unlimited loiter, so the run does not quietly terminate into a land
    # sequence half an hour in while someone is reading the dashboard.
    items.append((mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, lat, lon, a.alt,
                  (0.0, 0.0, LOITER_RADIUS_M, 0.0)))
    # NO LANDING ITEM, and that is a deliberate dependency on
    # configs/px4_sim_fixedwing.params setting MIS_TKO_LAND_REQ to 0.
    #
    # The alternative -- satisfying the feasibility checker with a real landing
    # pattern -- works, and then makes the aircraft fly to it. A fixed-wing
    # landing must clear the loiter radius and the 8-degree FW_LND_ANG glide
    # limit, which puts the touchdown point alt/tan(8) ~= 740 m away, and PX4
    # goes there: through an unlimited loiter with autocontinue 0, and through
    # a commanded AUTO.LOITER. The camera then spends the run 740 m from the
    # middle of the reference map. Inside a 2.17 km tile that still produces
    # fixes, so nothing in the numbers reveals it.

    print(f"  uploading {len(items)} items, {a.box:.0f} m box at {a.alt:.0f} m AGL")
    for attempt in range(1, 4):
        # Clear first. A half-finished transfer from an earlier attempt leaves
        # the mission manager mid-transaction and it rejects the next one.
        m.mav.mission_clear_all_send(m.target_system, m.target_component, 0)
        m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
        if upload(m, items):
            print("  mission accepted")
            break
        print(f"  retry {attempt}/3")
        time.sleep(2)
    else:
        sys.exit("mission upload was not accepted")

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            AUTO_MISSION >> 16 & 0xFF, AUTO_MISSION >> 24 & 0xFF,
                            0, 0, 0, 0)
    time.sleep(1)

    # ARM PATIENTLY, and say why it is refusing.
    #
    # NOT by waiting for MAV_STATE_STANDBY: PX4 SITL reports system_status 0
    # (MAV_STATE_UNINIT) in its heartbeat indefinitely on this build, so that
    # wait never returns on a vehicle that is perfectly ready. The honest
    # signal is the arming command's own result, and the reason travels
    # separately as STATUSTEXT -- which is why the first version of this loop
    # printed ten identical "result 1" lines and no cause.
    #
    # A fixed wing in SITL needs 20-60 s for the EKF's height and attitude to
    # settle, so the window is generous.
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            AUTO_MISSION >> 16 & 0xFF, AUTO_MISSION >> 24 & 0xFF,
                            0, 0, 0, 0)
    print("  arming")
    deadline = time.time() + 180
    reason, attempt = "", 0
    while time.time() < deadline:
        attempt += 1
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        t1, acked = time.time() + 3, False
        while time.time() < t1:
            msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"],
                               blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                if "Preflight" in msg.text or "Arming den" in msg.text:
                    reason = msg.text.split(":", 1)[-1].strip()
                continue
            if msg.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                continue
            acked = True
            if msg.result == 0:
                print(f"  armed on attempt {attempt}, launching")
                break
        else:
            if attempt % 5 == 1:
                print(f"    still refusing{': ' + reason if reason else ''}")
            time.sleep(2)
            continue
        if acked:
            break
    else:
        sys.exit(f"never armed in 180 s. Last reason: {reason or 'none reported'}")

    # Report altitude until it reaches the band, so the caller knows the camera
    # is looking at ground the pipeline was scoped for rather than at a runway.
    t0 = last_report = time.time()
    while time.time() - t0 < 180:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is None:
            continue
        agl = msg.relative_alt / 1000.0
        # \r only redraws on a terminal. Piped to a log it appends, and the
        # climb becomes eight hundred lines of altitude.
        if sys.stdout.isatty():
            print(f"\r  {agl:6.1f} m AGL", end="", flush=True)
        elif time.time() - last_report > 5.0:
            last_report = time.time()
            print(f"  {agl:6.1f} m AGL", flush=True)
        if agl >= a.alt * 0.9:
            print(f"\n  in the envelope at {agl:.1f} m.")
            # HOLD OVER THE MAP, and do not trust the mission to do it.
            #
            # MAV_CMD_NAV_LOITER_UNLIM with autocontinue 0 is the documented way
            # to make a mission stop, and on this PX4 it does not: the aircraft
            # ran on to the landing approach and orbited 750 m from the world
            # origin instead. That is still inside a 2.17 km tile, so the
            # pipeline kept working and the bug was invisible in the fix
            # statistics -- which is exactly why it is worth not relying on.
            #
            # DO_REPOSITION plus AUTO.LOITER holds a commanded point regardless
            # of where the mission thinks it is, so the camera stays over the
            # middle of the reference map for as long as the rig runs.
            # COMMAND_INT, not COMMAND_LONG: the long form carries lat/lon in
            # float32 fields, which quantises a position at this latitude to
            # roughly a third of a metre. COMMAND_INT carries them as degrees
            # times 1e7 and is the correct message for anything positional.
            m.mav.command_int_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                -1, 0, LOITER_RADIUS_M, float("nan"),   # radius: advisory
                int(lat * 1e7), int(lon * 1e7), a.alt)
            m.mav.command_long_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4, 3, 0, 0, 0, 0)          # PX4 main AUTO=4, sub LOITER=3
            print(f"  holding a {LOITER_RADIUS_M:.0f} m orbit over the map "
                  f"centre, {lat:.6f} {lon:.6f}")
            return 0
    print("\n  did not reach altitude within 180 s")
    return 1


if __name__ == "__main__":
    sys.exit(main())
