"""Writing a position back to the flight controller.

NEVER GPS_INPUT. It has no covariance field, so a pipeline that uses it is
telling the estimator to trust every fix identically -- which discards the one
thing this project is trying to contribute. ODOMETRY is ArduPilot's documented
preference and additionally carries a `quality` field; VISION_POSITION_ESTIMATE
also carries covariance and a reset counter, contrary to what the wiki implies
by documenting them only for ODOMETRY.

Both messages carry float[21], the upper triangle of the 6x6 pose covariance
over (x, y, z, roll, pitch, yaw) in row-major order, so the diagonal sits at
indices 0, 6, 11, 15, 18 and 20. ArduPilot collapses them to two scalars
(GCS_Common.cpp:4105 and :4145):

    if (!isnan(cov[0])) {
        posErr = sqrtf(cov[0] + cov[6] + cov[11]);
        angErr = sqrtf(cov[15] + cov[18] + cov[20]);
    }

Two consequences that are easy to get wrong and impossible to notice:

* **It SUMS the three translational variances.** A NaN in cov[11] to mean
  "altitude unknown" -- the natural reading of the MAVLink spec -- makes the
  whole sum NaN, and that NaN is handed straight to the estimator. The only
  NaN this code path understands is one in cov[0], which means "no covariance
  at all". Every entry that gets summed must therefore be a finite number.
* **posErr is a 3D magnitude, not a horizontal one.** The estimator here
  predicts a RADIAL horizontal error, so the per-axis variance is sigma^2 / 2
  and the vertical term is zero. That is not a claim of perfect altitude: it is
  the only assignment for which posErr comes out equal to the sigma the
  estimator actually produced, and ArduPilot reads cov[11] for nothing else.
  Splitting sigma^2 across both axes instead would hand EKF3 sigma * sqrt(2),
  a 41% overstatement of a number this whole project exists to calibrate.

**PX4 READS THE SAME 21 FLOATS COMPLETELY DIFFERENTLY, and the difference is
silent.** Verified against PX4 v1.16.2 source, not assumed:

    EKF2.cpp:2265   ev_data.position_var(0) = fmaxf(evp_noise_var, pos_var(0))
                    ev_data.position_var(1) = fmaxf(evp_noise_var, pos_var(1))

It takes cov[0] and cov[6] as the X and Y variances SEPARATELY and never sums
them. So the sigma^2 / 2 split that makes ArduPilot's posErr come out at sigma
gives PX4 a per-axis sigma of sigma / sqrt(2) -- a covariance 29% too TIGHT on
each axis, which is the dangerous direction: it tells the filter to trust a bad
fix more than the estimator said to. Nothing on either side raises.

So the split is chosen by `firmware`, and the invariant held across both is the
one that matters: **the per-axis position sigma the estimator ends up using
equals the `sigma_m` handed to send().** ArduPilot reaches that by summing
(AP_NavEKF3_PosVelFusion.cpp:832 uses posErr directly as the per-axis sigma for
both N and E); PX4 reaches it by being given sigma^2 in each axis directly.

Two further PX4 differences worth knowing, both from the same source read:

* **PX4 has no upper clamp on our covariance.** ArduPilot constrains posErr to
  [0.01, 100] m; `ev_pos_control.cpp:145` only floors it, at
  `max(cov, EKF2_EVP_NOISE^2, 0.01^2)`. The default `EKF2_EVP_NOISE` is 0.1 m,
  so a sigma below 10 cm is silently raised and anything above it is taken as
  given, however large. A learned estimator has MORE range to work with here
  than on ArduPilot, not less.
* **THE TWO FIRMWARES WANT OPPOSITE FRAMES, and both fail quietly.**
  ArduPilot accepts only `MAV_FRAME_LOCAL_FRD` (20) and drops `LOCAL_NED`
  without a word. PX4 accepts both at the MAVLink layer
  (mavlink_receiver.cpp:1358, :1372) -- and then EKF2 treats them completely
  differently (ev_pos_control.cpp:67):

      LOCAL_FRAME_NED  + yaw_align   pos = ev_sample.pos        used as given
      LOCAL_FRAME_FRD  + no ev_yaw   pos = R_ev_to_ekf * pos    ROTATED

  An FRD sample from a source that does not also claim yaw is rotated by
  `R_ev_to_ekf`, an estimated EV-to-EKF rotation filtered from the attitude in
  our own message. This pipeline sends an IDENTITY quaternion and never claims
  yaw, so that rotation is tracking the difference between "no attitude" and
  the vehicle's real attitude, and it turns an absolute georeferenced position
  into nonsense. Measured in PX4 SITL: a deliberate 20 m north injection
  arrived in `vehicle_visual_odometry` as `position: [19.97, 0, 0]` -- correct
  -- and reached `estimator_aid_src_ev_pos` as `observation: [-0.04, -0.14]`.
  The same branch also INFLATES our covariance by the orientation variance
  (`pos_cov(i,i) = max(pos_cov(i,i), orientation_var_max)`), which discards the
  calibrated sigma this project exists to produce.

  So the frame is firmware-dependent too, and for the same underlying reason as
  the covariance: ArduPilot wants a body-referenced local frame, PX4's EKF2
  wants a north-referenced one for an absolute fix. Nothing logs the mismatch
  on either side -- ArduPilot returns early, PX4 quietly fuses the wrong number
  or, as observed, reports `fused: false` with `innovation_rejected: false`.

The three loop modes are the professor's, and the middle one is the useful
trick: sending the vehicle its OWN position back over the ExternalNav path
exercises the message, the covariance, the origin and the EKF's acceptance
logic with a position that is already known good. Every failure it finds is a
plumbing failure. Only then is it worth sending a predicted one.

    off      nothing reaches the flight controller
    open     the ACTUAL GPS is sent -- validates the path, cannot mislead EKF3
    closed   the PREDICTED GPS is sent -- the real thing

VERIFY BEFORE FLYING: the origin below must match the EKF origin, and how
ArduPilot derives posErr from these 21 numbers should be re-read from source
rather than taken from here. `docs/step22_ardupilot_extnav_check.py` in the
main repo already does that job for the constants it covers.
"""
from __future__ import annotations

import math
import time

from ..geo import ne_offset_m


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


# Upper-triangle indices of the 6x6 pose covariance. ArduPilot reads exactly
# these six entries and nothing else.
IDX_XX, IDX_YY, IDX_ZZ, IDX_RR, IDX_PP, IDX_YAWYAW = 0, 6, 11, 15, 18, 20
UNKNOWN = float("nan")

# VERIFIED against ArduPilot master, libraries/GCS_MAVLink/GCS_Common.cpp.
# These are not the obvious values and getting them wrong fails silently.
#
# handle_odometry() at :4097 returns early unless
#     frame_id == MAV_FRAME_LOCAL_FRD (20)  and
#     child_frame_id == MAV_FRAME_BODY_FRD (12)
# with no warning, no status flag and no counter. MAV_FRAME_LOCAL_NED is 1 and
# is NOT accepted on this path, so sending it means every message is discarded
# and the only symptom is an estimator that never sees external navigation.
MAV_FRAME_LOCAL_FRD = 20
MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_FRD = 12

# Which local frame each firmware's estimator actually wants for an ABSOLUTE
# georeferenced position. See the docstring: this is not a style choice, it
# decides whether the position is used as sent or silently rotated.
POSE_FRAME = {"ardupilot": MAV_FRAME_LOCAL_FRD, "px4": MAV_FRAME_LOCAL_NED}
MAV_ESTIMATOR_TYPE_VISION = 2


def _force_mavlink2(conn, source_system: int, source_component: int):
    """Bind a MAVLink 2 protocol object to the connection, explicitly.

    pymavlink negotiates the wire version by watching INBOUND traffic and
    starts at 1.0. A link that only ever writes -- which is exactly what this
    is -- never sees a v2 frame, so it stays on 1.0 forever, and every message
    with an id above 255 is simply absent from the object. ODOMETRY is 331, so
    the failure is `'MAVLink' object has no attribute 'odometry_send'` the
    first time a fix is ready to send, on the aircraft, and not before.

    Setting the MAVLINK20 environment variable also works, but only if nothing
    has imported a dialect yet, which makes it order-dependent across modules.
    Binding the object here does not depend on import order.
    """
    from pymavlink.dialects.v20 import ardupilotmega as mav2
    conn.mav = mav2.MAVLink(conn, srcSystem=source_system, srcComponent=source_component)
    conn.mav.robust_parsing = True
    conn.WIRE_PROTOCOL_VERSION = "2.0"
    return conn


class FcError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


class FlightControllerLink:
    def __init__(self, endpoint: str, *, baud: int = 57600, message: str = "ODOMETRY",
                 firmware: str = "ardupilot", origin=None, send_origin: bool = False,
                 angle_sigma_rad: float = 0.5, source_system: int = 1,
                 source_component: int = 197):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise FcError("OLDE-01", "pymavlink is not installed -- run bootstrap.sh") from exc
        self.mavutil = mavutil
        self.endpoint = endpoint
        self.message = message.upper()
        # Normalised and checked, because this string now decides how the
        # covariance is packed. A typo would silently select the ArduPilot
        # split on a PX4 vehicle, which is exactly the 29%-too-tight covariance
        # documented above and produces no error anywhere.
        self.firmware = str(firmware).strip().lower()
        if self.firmware not in ("ardupilot", "px4"):
            raise FcError("OLDE-01",
                          f"unknown firmware {firmware!r} -- must be 'ardupilot' or 'px4'. "
                          "It selects the pose covariance layout, so there is no safe default.")
        self.origin = tuple(origin) if origin else None
        self.send_origin = send_origin
        self.angle_var = float(angle_sigma_rad) ** 2
        self.reset_counter = 0
        self.sent = 0
        self.rejected_nonfinite = 0
        self._origin_sent = False
        self._last_heartbeat = None
        if self.message not in ("ODOMETRY", "VISION_POSITION_ESTIMATE"):
            raise FcError("OLE-05", f"unsupported message '{message}'. "
                                    "Use ODOMETRY or VISION_POSITION_ESTIMATE, never GPS_INPUT.")
        try:
            self.conn = mavutil.mavlink_connection(
                endpoint, baud=baud, source_system=source_system,
                source_component=source_component)
            _force_mavlink2(self.conn, source_system, source_component)
        except Exception as exc:
            raise FcError("OLDE-01", f"cannot open {endpoint}: {exc}") from exc

    def ensure_origin(self, lat: float, lon: float) -> tuple:
        if self.origin is None:
            self.origin = (lat, lon)
            if self.send_origin and not self._origin_sent:
                try:
                    self.conn.mav.set_gps_global_origin_send(
                        self.conn.target_system, int(lat * 1e7), int(lon * 1e7), 0)
                    self._origin_sent = True
                except Exception:
                    pass
        return self.origin

    def heartbeat_age(self) -> float:
        """Seconds since the vehicle last said anything, or None if it never
        has. Draining the inbound queue here is deliberate: the link is
        nominally write-only, but an unread receive buffer eventually blocks
        the socket, and the heartbeat is the only proof of life available."""
        try:
            while True:
                m = self.conn.recv_match(type="HEARTBEAT", blocking=False)
                if m is None:
                    break
                self._last_heartbeat = time.time()
        except Exception:
            pass
        if self._last_heartbeat is None:
            return None
        return time.time() - self._last_heartbeat

    def bump_reset(self) -> int:
        """Call whenever the pipeline reinitialises -- a cleared prior, a
        method change, a reopened map. EKF3 uses this to know that the position
        it is being given is no longer continuous with the previous one."""
        self.reset_counter = (self.reset_counter + 1) % 256
        return self.reset_counter

    def send(self, lat: float, lon: float, sigma_m: float, *, t_capture_unix: float = None,
             quality: int = 0, yaw_deg=None) -> bool:
        # isfinite, not `is not None`. max(nan, 1e-3) returns nan, so the clamp
        # below is not a guard against one -- NaN passes straight through every
        # comparison it meets and lands in cov[0]. ArduPilot checks isnan(cov[0])
        # and takes its NO-COVARIANCE branch, which does not assign posErr at
        # all: EKF3 then fuses this position under whatever posErr it happened
        # to hold last. Nothing raises, on either side. Refusing to send is the
        # only safe answer, because a position without a trustworthy sigma is
        # precisely what this project exists not to emit.
        if not (_finite(lat) and _finite(lon) and _finite(sigma_m)):
            self.rejected_nonfinite += 1
            return False
        olat, olon = self.ensure_origin(lat, lon)
        north, east = ne_offset_m(olat, olon, lat, lon)
        sigma = max(float(sigma_m), 1e-3)
        if not _finite(north) or not _finite(east):
            self.rejected_nonfinite += 1
            return False

        # Zeros, not NaN. See the module docstring: ArduPilot sums the three
        # translational entries, so one NaN among them poisons posErr.
        #
        # The split is firmware-dependent because the two estimators read these
        # same floats differently -- ArduPilot sums cov[0]+cov[6]+cov[11] into
        # one 3D magnitude, PX4 takes cov[0] and cov[6] as the per-axis X and Y
        # variances and never sums. Both branches are chosen so that the sigma
        # the filter ends up applying PER AXIS is the sigma passed in here.
        cov = [0.0] * 21
        if self.firmware == "px4":
            cov[IDX_XX] = cov[IDX_YY] = sigma ** 2       # read per axis, as-is
        else:
            cov[IDX_XX] = cov[IDX_YY] = (sigma ** 2) / 2.0   # summed into posErr
        cov[IDX_ZZ] = 0.0                                 # so posErr == sigma
        # Altitude is still not claimed anywhere it matters: z is sent as 0 and
        # EK3_SRC1_POSZ is left off ExternalNav, so the flight controller keeps
        # estimating height from baro, rangefinder and terrain -- which is what
        # NGPS does too.
        a = (float(self.angle_var)) / 3.0                 # so angErr == angle sigma
        cov[IDX_RR] = cov[IDX_PP] = cov[IDX_YAWYAW] = a

        # Timestamp at CAPTURE. ArduPilot compensates delay from this value and
        # clamps the compensation at 250 ms; stamping at send time instead
        # hides the pipeline's own latency from the filter entirely.
        usec = int((t_capture_unix or time.time()) * 1e6)
        try:
            if self.message == "ODOMETRY":
                self.conn.mav.odometry_send(
                    usec, POSE_FRAME[self.firmware], MAV_FRAME_BODY_FRD,
                    float(north), float(east), 0.0,
                    [1.0, 0.0, 0.0, 0.0],
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    cov, [0.0] * 21,
                    self.reset_counter, MAV_ESTIMATOR_TYPE_VISION,
                    int(max(-1, min(100, quality))))
            else:
                self.conn.mav.vision_position_estimate_send(
                    usec, float(north), float(east), 0.0, 0.0, 0.0, 0.0,
                    cov, self.reset_counter)
        except Exception as exc:
            raise FcError("OLDE-03", f"MAVLink send failed: {exc}") from exc
        self.sent += 1
        return True

    def close(self) -> None:
        try:
            self.conn.close()
        except Exception:
            pass

    def describe(self) -> dict:
        return {"endpoint": self.endpoint, "message": self.message, "firmware": self.firmware,
                "origin": list(self.origin) if self.origin else None,
                "reset_counter": self.reset_counter, "sent": self.sent,
                "heartbeat_age_s": (round(time.time() - self._last_heartbeat, 1)
                                    if self._last_heartbeat else None)}


def quality_from(inliers: int, gate: int, good: int = 200) -> int:
    """Map inlier count onto ODOMETRY's 1-100 quality scale.

    -1 means the pipeline failed, 0 means unknown, 1-100 is worst to best. A
    fix below the rejection gate never reaches here, so the scale starts there.
    """
    if inliers <= 0:
        return -1
    if inliers < gate:
        return 1
    span = max(1, good - gate)
    return int(max(1, min(100, 1 + 99 * (inliers - gate) / span)))


def pos_err_as_ardupilot_sees_it(cov) -> float:
    """Reproduce GCS_Common.cpp:4105 exactly, for the regression test.

    The point of this function is that the number ArduPilot computes and the
    number the estimator produced must be the same number. If they drift apart,
    every calibration claim in the writeup is wrong at the last hop and nothing
    reports it.
    """
    import math
    if math.isnan(cov[IDX_XX]):
        return 0.0
    return math.sqrt(cov[IDX_XX] + cov[IDX_YY] + cov[IDX_ZZ])


def ang_err_as_ardupilot_sees_it(cov) -> float:
    import math
    if math.isnan(cov[IDX_XX]):
        return 0.0
    return math.sqrt(cov[IDX_RR] + cov[IDX_PP] + cov[IDX_YAWYAW])


# --------------------------------------------------------------------------
class QgcLink:
    """Show the predicted position in QGroundControl, as a second vehicle.

    Display only, and deliberately on its own socket. QGC renders any system it
    sees a heartbeat from, so emitting HEARTBEAT plus GLOBAL_POSITION_INT under
    a different system id puts a second marker on the map wherever the pipeline
    thinks the aircraft is, next to where the autopilot thinks it is. During a
    real flight that is the view you already have open.

    Three properties matter and all three are deliberate:

    * It is independent of `loop_mode`. QGC sees the predicted position whether
      or not the flight controller is being given it, which is exactly what you
      want while deciding whether to close the loop.
    * It never writes to the autopilot. A separate connection, a separate
      system id, and only messages QGC consumes for display.
    * A failure here is `OLDE-07` and nothing else. Losing the ground-station
      view must not affect the aircraft.

    The autopilot is normally system 1, so pick something else. QGC will show it
    as an unknown vehicle type, which is correct: it is not a vehicle.
    """

    def __init__(self, endpoint: str, system_id: int = 2, component_id: int = 1,
                 label: str = "GeoAnchor"):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise FcError("OLDE-07", "pymavlink is not installed") from exc
        self.endpoint = endpoint
        self.system_id = int(system_id)
        self.label = label
        self.sent = 0
        self._last_hb = 0.0
        try:
            self.conn = mavutil.mavlink_connection(
                endpoint, source_system=self.system_id, source_component=component_id)
            _force_mavlink2(self.conn, self.system_id, component_id)
        except Exception as exc:
            raise FcError("OLDE-07", f"cannot open {endpoint}: {exc}") from exc
        self._mavutil = mavutil

    def heartbeat(self, force: bool = False) -> None:
        """QGC drops a system it has not heard from for a few seconds."""
        now = time.time()
        if not force and now - self._last_hb < 1.0:
            return
        try:
            m = self._mavutil.mavlink
            self.conn.mav.heartbeat_send(
                m.MAV_TYPE_GENERIC, m.MAV_AUTOPILOT_INVALID, 0, 0, m.MAV_STATE_ACTIVE)
            self._last_hb = now
        except Exception as exc:
            raise FcError("OLDE-07", f"heartbeat failed: {exc}") from exc

    def send(self, lat: float, lon: float, alt_m=None, sigma_m=None,
             t_capture_unix: float = None, accepted: bool = True) -> bool:
        if lat is None or lon is None:
            return False
        self.heartbeat()
        # GLOBAL_POSITION_INT is what QGC draws a vehicle marker from.
        # time_boot_ms, not the capture epoch: QGC treats it as its own clock.
        try:
            self.conn.mav.global_position_int_send(
                int((time.monotonic() * 1000) % (2 ** 31)),
                int(lat * 1e7), int(lon * 1e7),
                int((alt_m or 0.0) * 1000), int((alt_m or 0.0) * 1000),
                0, 0, 0, 65535)
        except Exception as exc:
            raise FcError("OLDE-07", f"position send failed: {exc}") from exc
        self.sent += 1
        return True

    def status(self, text: str, severity: int = 6) -> None:
        """A short line in QGC's message panel. Severity 6 is INFO."""
        try:
            self.conn.mav.statustext_send(severity, text[:50].encode())
        except Exception:
            pass

    def close(self) -> None:
        try:
            self.conn.close()
        except Exception:
            pass

    def describe(self) -> dict:
        return {"endpoint": self.endpoint, "system_id": self.system_id, "sent": self.sent}
