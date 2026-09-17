"""Actual GPS, and the rest of the vehicle state that arrives on the same link.

Named for GPS because that is the third input in the professor's diagram, but
the link carries two more things the pipeline cannot work without:

  altitude   selects the reference scale (see feed.Preprocessor) and enforces
             the 50-100 m envelope
  attitude   drives the yaw rectification in the processing layer, which on
             2 Sept 2026 turned out to be load-bearing rather than a
             refinement: XFeat is not rotation invariant, and without
             rectification it failed on 5% of frames of a synthetic task where
             the query was literally cut out of the reference tile

One link, read once, in one place. The output layer opens its own connection
for the write direction so that a fault in one direction cannot take out the
other.
"""
from __future__ import annotations

import math
import time
from pathlib import Path

from ..contracts import GpsPacket


class GpsError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


class VehicleState:
    """Latest known state. Every field carries its own age."""

    def __init__(self):
        self.gps: GpsPacket = None
        self.t_gps: float = 0.0
        self.rel_alt_m: float = None
        self.t_alt: float = 0.0
        self.roll_deg: float = None
        self.pitch_deg: float = None
        self.yaw_deg: float = None
        self.t_att: float = 0.0
        self.heartbeat: float = 0.0

    def gps_age(self) -> float:
        return time.time() - self.t_gps if self.t_gps else float("inf")

    def att_age(self) -> float:
        return time.time() - self.t_att if self.t_att else float("inf")


class GpsSource:
    kind = "base"

    def poll(self, state: VehicleState) -> int:
        """Update state from whatever has arrived. Returns messages consumed."""
        raise NotImplementedError

    def close(self) -> None:
        pass

    def describe(self) -> dict:
        return {"kind": self.kind}


class MavlinkGps(GpsSource):
    kind = "mavlink"

    #  udpin:0.0.0.0:14550          listen for the FC or a SITL forward
    #  /dev/ttyTHS0                 Xavier UART to a Pixhawk, needs baud
    #  udpout:127.0.0.1:14550       push to a listener
    def __init__(self, endpoint: str, baud: int = 57600, heartbeat_timeout_s: float = 5.0,
                 source_system: int = 255):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise GpsError("DLE-01", "pymavlink is not installed -- run bootstrap.sh") from exc
        self.endpoint = endpoint
        self.heartbeat_timeout_s = heartbeat_timeout_s
        if endpoint.startswith("/dev/") and not Path(endpoint.split(":")[0]).exists():
            raise GpsError("DLDE-04", f"serial port {endpoint} does not exist")
        try:
            self.conn = mavutil.mavlink_connection(
                endpoint, baud=baud, source_system=source_system, autoreconnect=True)
        except Exception as exc:
            raise GpsError("DLDE-04", f"cannot open MAVLink endpoint {endpoint}: {exc}") from exc
        self._link_down = False
        self.malformed = 0
        self._last_beat = 0.0
        self._last_request = 0.0

    def _announce(self) -> None:
        """Heartbeat at 1 Hz so the autopilot knows we are here.

        A PIXHAWK OVER SERIAL STREAMS UNPROMPTED, which is why this was never
        needed on the bench. A PX4 on UDP does not: it waits for a peer to
        appear on the port before it sends anything, so a listener that only
        listens receives nothing, for ever, with no error -- the symptom is
        every frame logging "no altitude or no fx_px" and the matcher working
        on an unrotated frame at a guessed scale. Announcing costs one datagram
        a second and is what any GCS does."""
        now = time.time()
        if now - self._last_beat < 1.0:
            return
        self._last_beat = now
        try:
            from pymavlink import mavutil
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            # A link that cannot be written to is still worth reading from.
            pass

    def _request_streams(self) -> None:
        """Ask for the two messages this layer actually needs.

        DO NOT ASSUME THE AUTOPILOT VOLUNTEERS THEM. What an autopilot streams
        unprompted depends on the link's configured mode and rate: PX4's
        low-rate onboard instance sends little more than HEARTBEAT and
        STATUSTEXT, and nothing guarantees GLOBAL_POSITION_INT or ATTITUDE on
        any particular port. Without altitude the data layer cannot compute
        GSD = altitude / fx_px and falls back to a fixed long edge; without
        attitude the processing layer matches an unrotated frame, which
        CLAUDE.md records as the difference between a working matcher and a
        failing one.

        Re-sent while nothing has arrived, because the request is a datagram
        like any other and the first one can be sent before the autopilot has
        registered this peer."""
        now = time.time()
        if now - self._last_request < 5.0:
            return
        self._last_request = now
        try:
            from pymavlink import mavutil
            for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0),
                               (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20.0),
                               (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5.0)):
                self.conn.mav.command_long_send(
                    self.conn.target_system or 1, self.conn.target_component or 1,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, 1e6 / hz, 0, 0, 0, 0, 0)
        except Exception:
            pass

    def poll(self, state: VehicleState) -> int:
        self._announce()
        # Stop asking once the data is flowing; resume if it stops.
        if state.gps is None or state.gps_age() > 5.0 or state.att_age() > 5.0:
            self._request_streams()
        self.malformed = 0
        n = 0
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            n += 1
            try:
                self._apply(msg, state)
            except (AttributeError, TypeError, ValueError):
                self.malformed += 1
        return n

    @staticmethod
    def plausible(lat: float, lon: float) -> bool:
        """A zero-zero fix is the null island the autopilot sends before it has
        an origin, not a position off the coast of Ghana."""
        return (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0
                and not (abs(lat) < 1e-7 and abs(lon) < 1e-7))

    def _apply(self, msg, state: VehicleState) -> None:
        t = msg.get_type()
        now = time.time()
        if t == "HEARTBEAT":
            state.heartbeat = now
            self._link_down = False
        elif t == "GLOBAL_POSITION_INT":
            if not self.plausible(msg.lat / 1e7, msg.lon / 1e7):
                self.malformed += 1
                return
            state.gps = GpsPacket(
                t_unix=now, lat=msg.lat / 1e7, lon=msg.lon / 1e7,
                alt_amsl_m=msg.alt / 1000.0, rel_alt_m=msg.relative_alt / 1000.0,
                fix_type=state.gps.fix_type if state.gps else 0,
                satellites=state.gps.satellites if state.gps else 0,
                source="mavlink")
            state.t_gps = now
            state.rel_alt_m = msg.relative_alt / 1000.0
            state.t_alt = now
        elif t == "GPS_RAW_INT":
            # Quality lives here, position lives in GLOBAL_POSITION_INT. Carry
            # quality forward rather than overwriting the fused position with
            # the raw one, which is noisier.
            eph = None if msg.eph in (0, 65535) else msg.eph / 100.0
            if state.gps is not None:
                state.gps.fix_type = msg.fix_type
                state.gps.satellites = msg.satellites_visible
                state.gps.eph_m = eph
            else:
                state.gps = GpsPacket(t_unix=now, lat=msg.lat / 1e7, lon=msg.lon / 1e7,
                                      alt_amsl_m=msg.alt / 1000.0, fix_type=msg.fix_type,
                                      satellites=msg.satellites_visible, eph_m=eph,
                                      source="mavlink")
                state.t_gps = now
        elif t == "ALTITUDE":
            # A SECOND SOURCE OF RELATIVE ALTITUDE, because the first is not
            # guaranteed. PX4's onboard stream set carries GPS_RAW_INT,
            # ODOMETRY, LOCAL_POSITION_NED and ATTITUDE but NOT
            # GLOBAL_POSITION_INT, and SET_MESSAGE_INTERVAL does not conjure
            # it. Reading altitude from only one message is therefore a silent
            # dependency on the autopilot's stream configuration: with the
            # wrong one, GSD = altitude / fx_px never runs and every frame is
            # scaled to a fixed long edge instead.
            state.rel_alt_m = float(msg.altitude_relative)
            state.t_alt = now
            if state.gps is not None:
                state.gps.rel_alt_m = state.rel_alt_m
        elif t == "LOCAL_POSITION_NED":
            # Third fallback. NED, so down is positive and height is -z,
            # relative to the EKF origin rather than to the terrain -- correct
            # over the flat simulated ground, and an approximation anywhere the
            # ground is not level with the launch point.
            if state.t_alt == 0.0 or now - state.t_alt > 1.0:
                state.rel_alt_m = -float(msg.z)
                state.t_alt = now
        elif t == "ATTITUDE":
            state.roll_deg = math.degrees(msg.roll)
            state.pitch_deg = math.degrees(msg.pitch)
            state.yaw_deg = math.degrees(msg.yaw) % 360.0
            state.t_att = now
        elif t in ("RANGEFINDER", "DISTANCE_SENSOR"):
            d = getattr(msg, "distance", None)
            if d is not None:
                # DISTANCE_SENSOR is centimetres, RANGEFINDER is metres.
                state.rel_alt_m = d / 100.0 if t == "DISTANCE_SENSOR" else float(d)
                state.t_alt = now

    def link_lost(self, state: VehicleState) -> bool:
        if not state.heartbeat:
            return False
        return (time.time() - state.heartbeat) > self.heartbeat_timeout_s

    def close(self) -> None:
        try:
            self.conn.close()
        except Exception:
            pass

    def describe(self) -> dict:
        return {"kind": "mavlink", "endpoint": self.endpoint}


class ReplayGps(GpsSource):
    """Truth from the feed's sidecar. Advanced by the feed, not by the clock,
    so a paused or slowed replay stays in step."""
    kind = "replay"

    def __init__(self):
        self.pending: dict = None

    def offer(self, meta: dict) -> None:
        if meta:
            self.pending = meta

    def poll(self, state: VehicleState) -> int:
        if not self.pending:
            return 0
        m, self.pending = self.pending, None
        now = time.time()
        if "lat" in m and "lon" in m:
            # Stamp with the SESSION clock, not the sidecar's own timestamp.
            # A recorded log carries the time of the original flight, and the
            # output layer pairs a fix to a GPS sample by comparing timestamps
            # against a window of a few hundred milliseconds. Passing the
            # recording's clock through makes every pair fail by however long
            # ago the flight was, which surfaces as OLE-02 on every single row
            # and an export with no errors in it at all.
            state.gps = GpsPacket(
                t_unix=now, lat=float(m["lat"]), lon=float(m["lon"]),
                alt_amsl_m=m.get("alt_amsl_m"), rel_alt_m=m.get("alt_agl_m"),
                fix_type=m.get("fix_type", 3), satellites=m.get("satellites", 12),
                source="replay")
            state.t_gps = now
        if m.get("alt_agl_m") is not None:
            state.rel_alt_m = float(m["alt_agl_m"])
            state.t_alt = now
        for src, dst in (("roll_deg", "roll_deg"), ("pitch_deg", "pitch_deg"), ("yaw_deg", "yaw_deg")):
            if m.get(src) is not None:
                setattr(state, dst, float(m[src]))
                state.t_att = now
        state.heartbeat = now
        return 1

    def link_lost(self, state: VehicleState) -> bool:
        return False


class NoGps(GpsSource):
    kind = "none"

    def poll(self, state: VehicleState) -> int:
        return 0

    def link_lost(self, state: VehicleState) -> bool:
        return False


def open_gps(cfg: dict) -> GpsSource:
    src = (cfg or {}).get("source", "none")
    if src == "mavlink":
        return MavlinkGps(cfg.get("endpoint", "udpin:0.0.0.0:14550"),
                          baud=cfg.get("baud", 57600),
                          heartbeat_timeout_s=cfg.get("heartbeat_timeout_s", 5.0))
    if src == "replay":
        return ReplayGps()
    if src == "none":
        return NoGps()
    raise GpsError("DLE-01", f"unknown gps source '{src}'")
