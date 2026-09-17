"""Frame acquisition and frame preprocessing.

The preprocessing that matters is the rescale. A frame at 75 m through an
IMX477 (f = 6 mm, 1.55 um pitch, so fx about 3871 px) has a GSD of
75 / 3871 = 0.019 m/px, while the NSW reference tile is 0.124 m/px. The
matcher would be asked to bridge a factor of 6.4 in scale, which is most of
the way to where sparse detectors stop finding the same corners twice.

Barometric altitude is already on the vehicle, so the fix is free: resize the
frame by gsd_frame / gsd_reference and the matcher sees two images at the same
scale. This is Equation (3) of the review used the way the review says it is
used, and it is why altitude is an input to the data layer and not only a
number the output layer checks against the 100 m cap.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".tif", ".tiff", ".bmp"}


class FeedError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


class Feed:
    kind = "base"

    def read(self) -> tuple:
        """Return (frame_bgr, t_capture_unix, meta) or (None, None, {}) at EOF."""
        raise NotImplementedError

    def close(self) -> None:
        pass

    def describe(self) -> dict:
        return {"kind": self.kind}


class FileFeed(Feed):
    """A video file, or a directory of stills. Paced to the configured rate so
    a replay behaves like a flight instead of finishing in three seconds."""
    kind = "file"

    def __init__(self, path, fps: float = 4.0, loop: bool = False, sidecar=None,
                 realtime: bool = True):
        self.path = Path(path)
        self.fps = max(0.1, float(fps))
        self.loop = bool(loop)
        self.realtime = realtime
        self._next_due = None
        self.frames: list = []
        self.cap = None
        self.index = 0

        if not self.path.exists():
            raise FeedError("DLE-02", f"feed path does not exist: {self.path}")
        if self.path.is_dir():
            self.frames = sorted(p for p in self.path.iterdir()
                                 if p.suffix.lower() in IMAGE_SUFFIXES)
            if not self.frames:
                raise FeedError("DLE-02", f"no images in {self.path}")
        else:
            self.cap = cv2.VideoCapture(str(self.path))
            if not self.cap.isOpened():
                raise FeedError("DLE-08", f"cannot decode {self.path}")

        self.sidecar = _load_sidecar(sidecar)

    def read(self) -> tuple:
        self._pace()
        if self.cap is not None:
            ok, frame = self.cap.read()
            if not ok:
                if self.loop:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self.index = 0
                    ok, frame = self.cap.read()
                if not ok:
                    return None, None, {}
        else:
            if self.index >= len(self.frames):
                if not self.loop:
                    return None, None, {}
                self.index = 0
            frame = cv2.imread(str(self.frames[self.index]))
            if frame is None:
                raise FeedError("DLE-08", f"cannot decode {self.frames[self.index]}")

        meta = dict(self.sidecar[self.index]) if self.index < len(self.sidecar) else {}
        meta["frame_index"] = self.index
        self.index += 1
        return frame, time.time(), meta

    def _pace(self) -> None:
        if not self.realtime:
            return
        now = time.monotonic()
        if self._next_due is None:
            self._next_due = now
        if now < self._next_due:
            time.sleep(self._next_due - now)
        self._next_due = max(now, self._next_due) + 1.0 / self.fps

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()

    def describe(self) -> dict:
        n = len(self.frames) if self.frames else int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        return {"kind": "file", "path": str(self.path), "frames": n,
                "fps": self.fps, "sidecar": len(self.sidecar)}


class UvcFeed(Feed):
    """Plain V4L2. No GStreamer, no Argus, no ISP -- the same capture code
    runs on the Xavier, the Pi 5 and a laptop, so a timing comparison between
    boards is a comparison of the boards.

    Two things here are load-bearing rather than tidy, both measured on the
    Xavier against a Sonix USB2 camera on 4 Sept 2026.

    THE PIXEL FORMAT IS NOT COSMETIC. UVC over USB 2.0 has about 480 Mbit/s to
    play with, and uncompressed YUYV at 1280x720 needs more than that, so the
    camera silently negotiates a slower frame rate instead of refusing: 30 fps
    at 640x480, but 10 fps at 1280x720 and 5 fps at 1920x1080, with some modes
    failing to read at all. The same camera in MJPG holds 30 fps all the way to
    1920x1080. Asking for a resolution without asking for MJPG is how a feed
    ends up at 5 fps with nothing in the logs to say so.

    THE DRIVER QUEUE IS A LATENCY LEAK. V4L2 keeps filling its buffers while
    the consumer is busy matching, and cv2.VideoCapture.read() returns the
    OLDEST queued frame, not the newest. Measured with the frame's own V4L2
    timestamp: a consumer that pauses 1 s is handed a frame 1758 ms old, and
    even a consumer that never pauses is handed one 162 ms old. Stamping that
    frame with time.time() on return -- which is what this class used to do --
    reports it as current. CLAUDE.md is explicit that this is the expensive
    failure: ArduPilot's writeExtNavData does MAX(timeStamp_ms,
    imuDataDelayed.time_ms), so a late fix is not rejected, it is stamped as
    current and fused at the wrong time, and at 5 m/s each 100 ms is 0.5 m.

    So a reader thread drains the queue continuously and keeps only the newest
    frame, exactly the drop-not-block rule bus.py already applies to messages,
    and the frame is stamped with the V4L2 buffer timestamp rather than with
    the time it happened to be collected. That bounds staleness at one frame
    interval plus decode -- measured 41-52 ms regardless of consumer speed --
    and, more to the point, whatever remains is now REPORTED instead of hidden.
    """
    kind = "uvc"

    # The camera is dead, as opposed to merely slow, if nothing arrives in this
    # long. Generous enough that a 5 fps mode does not trip it.
    _DEAD_AFTER_S = 2.0

    def __init__(self, device=0, width: int = 1280, height: int = 720, fps: float = 30.0,
                 fourcc: str = "MJPG"):
        import threading

        self.device = device
        self.requested = (int(width), int(height), float(fps), str(fourcc or ""))
        node = f"/dev/video{device}" if isinstance(device, int) else str(device)
        if isinstance(device, int) and not Path(node).exists():
            raise FeedError("DLDE-01", f"{node} does not exist. `v4l2-ctl --list-devices` shows what is attached.")
        self.cap = cv2.VideoCapture(device if isinstance(device, int) else str(device), cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise FeedError("DLDE-02", f"cannot open {node} -- in use by another process, or no V4L2 support")

        # Order matters: the format has to be set before the frame size, or the
        # driver picks a size valid for the OLD format and then keeps it.
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(fourcc)))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self.cap.set(cv2.CAP_PROP_FPS, float(fps))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # honoured or not, the thread is the real fix

        ok, first = self.cap.read()
        if not ok or first is None:
            self.cap.release()
            raise FeedError("DLDE-03", f"{node} opened but returned no frame. A mode the camera "
                                       f"cannot actually deliver is the usual cause: "
                                       f"{width}x{height} @{fps:g} {fourcc or 'default'}.")

        # Read back what the driver ACTUALLY gave us. Asking is not getting, and
        # a run whose logs claim 1280x720@30 while the camera does 640x480@10 is
        # worse than one that admits it.
        self.actual = {
            "width": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": float(self.cap.get(cv2.CAP_PROP_FPS)),
            "fourcc": _fourcc_str(self.cap.get(cv2.CAP_PROP_FOURCC)),
        }

        self.index = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest = None          # (frame, t_unix, t_mono, age_ms_at_grab)
        self._fail_streak = 0
        self._thread = threading.Thread(target=self._pump, name="uvc-drain", daemon=True)
        self._thread.start()

    def _pump(self) -> None:
        """Consume every frame the camera produces, keep the newest one."""
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self._fail_streak += 1
                if self._stop.wait(0.01):
                    break
                continue
            self._fail_streak = 0
            t_mono, t_unix, age_ms = self._stamp()
            with self._lock:
                self._latest = (frame, t_unix, t_mono, age_ms)

    def _stamp(self) -> tuple:
        """When was this frame actually captured?

        V4L2 timestamps the buffer on CLOCK_MONOTONIC and OpenCV surfaces it as
        CAP_PROP_POS_MSEC, so capture time is knowable rather than guessable.
        Not every driver fills it in, so an implausible value falls back to now
        and says so instead of quietly emitting a nonsense timestamp.
        """
        now_mono, now_unix = time.monotonic(), time.time()
        ts_ms = self.cap.get(cv2.CAP_PROP_POS_MSEC)
        age_s = now_mono - (ts_ms / 1000.0) if ts_ms and ts_ms > 0 else None
        if age_s is None or not (0.0 <= age_s <= 5.0):
            return now_mono, now_unix, None          # None == driver gave us nothing usable
        return now_mono - age_s, now_unix - age_s, age_s * 1000.0

    def read(self) -> tuple:
        deadline = time.monotonic() + self._DEAD_AFTER_S
        while True:
            with self._lock:
                latest, self._latest = self._latest, None
            if latest is not None:
                break
            if not self._thread.is_alive() or time.monotonic() > deadline:
                raise FeedError("DLDE-03", f"camera stopped returning frames "
                                           f"({self._fail_streak} consecutive failed reads)")
            time.sleep(0.002)

        frame, t_unix, t_mono, age_ms = latest
        self.index += 1
        meta = {"frame_index": self.index, "t_capture_mono": t_mono}
        # Surface the queue delay rather than absorbing it: this is the capture
        # half of OVERHEAD_MS, and it belongs in the record, not in a comment.
        if age_ms is not None:
            meta["capture_age_ms"] = round(age_ms, 1)
        return frame, t_unix, meta

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.cap.release()

    def describe(self) -> dict:
        rw, rh, rf, rc = self.requested
        d = {"kind": "uvc", "device": str(self.device)}
        d.update(self.actual)
        # A camera that quietly gave us something other than what the config
        # asked for is a thing to see on the dashboard, not to discover in a
        # latency table three days later.
        if (rw, rh) != (self.actual["width"], self.actual["height"]) or \
           (rc and rc != self.actual["fourcc"]):
            d["negotiated_down_from"] = f"{rw}x{rh} @{rf:g} {rc or 'default'}"
        return d


class RtspFeed(Feed):
    kind = "rtsp"

    def __init__(self, url: str, latency_ms: int = 100):
        self.url = url
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            raise FeedError("DLDE-02", f"cannot open stream {url}")
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except cv2.error:
            pass
        self.index = 0

    def read(self) -> tuple:
        ok, frame = self.cap.read()
        t = time.time()
        if not ok:
            raise FeedError("DLDE-03", f"stream {self.url} dropped")
        self.index += 1
        return frame, t, {"frame_index": self.index}

    def close(self) -> None:
        self.cap.release()

    def describe(self) -> dict:
        return {"kind": "rtsp", "url": self.url}


class Env80Feed(Feed):
    """AnyVisLoc .npz frames, played back with their own ground truth.

    Real UAV imagery over a real satellite reference, so the errors that come
    out of this are comparable with the harness tables. Each frame carries its
    own K, so intrinsics travel with the frame rather than coming from the
    config -- a scene shot with a different camera still gets the right rescale.
    """
    kind = "env80"

    def __init__(self, scene_dir, frames_dir=None, mode: str = "satellite", anchor=None,
                 fps: float = 4.0, loop: bool = False, realtime: bool = True,
                 envelope: bool = True, agl_min: float = 50.0, agl_max: float = 100.0,
                 view_angle_min: float = 80.0, limit: int = 0):
        from . import anyvisloc as A
        self.A = A
        self.scene_dir = Path(scene_dir)
        self.georef = A.load_georeference(self.scene_dir, mode, anchor)
        frames = A.list_frames(Path(frames_dir) if frames_dir else self.scene_dir)
        if envelope:
            kept = A.envelope_filter(frames, self.georef, agl_min, agl_max, view_angle_min)
            self.skipped = len(frames) - len(kept)
            frames = kept
        else:
            self.skipped = 0
        if limit:
            frames = frames[:int(limit)]
        if not frames:
            raise FeedError("DLE-02",
                            f"no frames left in {self.scene_dir.name} after the "
                            f"{agl_min}-{agl_max} m / >={view_angle_min} deg envelope filter")
        self.frames = frames
        self.fps = max(0.1, float(fps))
        self.loop = bool(loop)
        self.realtime = realtime
        self._next_due = None
        self.index = 0

    def read(self) -> tuple:
        self._pace()
        if self.index >= len(self.frames):
            if not self.loop:
                return None, None, {}
            self.index = 0
        path = self.frames[self.index]
        try:
            img, meta = self.A.read_frame(path, self.georef)
        except Exception as exc:
            raise FeedError("DLE-08", f"{path.name}: {exc}") from exc
        meta["frame_index"] = self.index
        self.index += 1
        return img, time.time(), meta

    def _pace(self) -> None:
        if not self.realtime:
            return
        now = time.monotonic()
        if self._next_due is None:
            self._next_due = now
        if now < self._next_due:
            time.sleep(self._next_due - now)
        self._next_due = max(now, self._next_due) + 1.0 / self.fps

    def describe(self) -> dict:
        return {"kind": "env80", "scene": self.scene_dir.name, "frames": len(self.frames),
                "skipped_by_envelope": self.skipped, "mode": self.georef["mode"],
                "fps": self.fps, "local_frame": True}


class GzFeed(Feed):
    """A camera sensor inside Gazebo, over gz-transport.

    The simulator's counterpart to UvcFeed, and it inherits that class's one
    hard-won lesson: **the newest frame, never a queued one.** Gazebo renders
    at its own rate regardless of what the matcher is doing, so a callback that
    appended to a list would build a backlog and hand the pipeline a frame
    seconds old, stamped as current -- which is precisely the failure
    CLAUDE.md warns about, because ArduPilot's writeExtNavData does
    MAX(timeStamp_ms, imuDataDelayed.time_ms) and fuses a late fix at the wrong
    time. So the callback keeps exactly one frame and read() takes it.

    THE TIMESTAMP IS THE SIMULATOR'S, NOT THE WALL CLOCK. gz stamps each image
    with sim time, and PX4 SITL runs in lockstep, so sim time and wall time
    drift apart whenever the renderer or the matcher stalls. Reporting
    time.time() here would hide exactly the staleness this feed exists to
    measure. `capture_age_ms` is therefore the gap between the frame's sim
    stamp and the newest sim stamp seen, not an age against the wall clock.

    NOT IMPORTED UNLESS USED. gz-transport is a simulator dependency and has no
    business on an aircraft; the import is inside __init__ so a Pi that never
    sets `feed.type: gz` never loads it, the same way rasterio and torch are
    kept out of the flight path elsewhere in this layer.
    """
    kind = "gz"

    _DEAD_AFTER_S = 5.0        # generous: Gazebo stalls while it loads a world

    def __init__(self, topic: str, timeout_s: float = 30.0):
        import sys, threading
        # gz-transport ships as APT packages under the SYSTEM dist-packages and
        # a venv does not see them. APPEND, never prepend, and never via
        # PYTHONPATH: that env var goes in FRONT of the venv's site-packages,
        # so the system typing_extensions shadows the venv's and pydantic dies
        # on `cannot import name 'Sentinel'` -- taking fastapi, and with it the
        # dashboard, down with it. Appending here leaves venv packages winning
        # and asks nothing of the caller.
        _apt = "/usr/lib/python3/dist-packages"
        if _apt not in sys.path:
            sys.path.append(_apt)
        try:
            from gz.transport13 import Node
            from gz.msgs10.image_pb2 import Image as GzImage
        except ImportError as exc:
            raise FeedError(
                "DLE-01",
                f"gz-transport Python bindings not importable ({exc}).\n"
                "  They are APT packages and this venv does not see them by "
                "default:\n"
                "      sudo apt install python3-gz-transport13 python3-gz-msgs10\n"
                "  This feed appends that path itself, so the remaining cause "
                "is that the\n  packages are not installed.\n"
                "  The venv's python and the system python are both 3.12 on "
                "Ubuntu 24.04, so\n  the ABI matches and the system path simply "
                "works -- verified. Also note ROS 2's\n  vendored `gz` on PATH "
                "shadows the real binary, which makes `gz sim` report that\n"
                "  Gazebo is not installed when it is.") from exc

        self.topic = str(topic)
        self._GzImage = GzImage
        self._lock = threading.Lock()
        self._latest = None            # (frame_bgr, sim_t, seq)
        self._newest_sim_t = 0.0
        self._count = 0
        self._last_rx = time.monotonic()

        self._node = Node()
        if not self._node.subscribe(GzImage, self.topic, self._on_image):
            raise FeedError("DLDE-02", f"could not subscribe to gz topic '{self.topic}'. "
                                       "`gz topic -l` lists what is actually published.")

        # Wait for the FIRST frame rather than returning a feed that is not
        # yet delivering: Gazebo takes seconds to load a world and render, and
        # a data layer that starts publishing nothing looks like a dead camera.
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest is not None:
                    break
            time.sleep(0.05)
        else:
            raise FeedError("DLDE-03", f"no image on '{self.topic}' within {timeout_s:g}s. "
                                       "Is the world running, and does the model carry a camera?")
        with self._lock:
            f = self._latest[0]
        self.actual = {"width": f.shape[1], "height": f.shape[0]}

    def _on_image(self, msg):
        try:
            h, w = msg.height, msg.width
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            fmt = msg.pixel_format_type
            # 3 == RGB_INT8, 1 == L_INT8 in gz.msgs. Anything else is a format
            # the world was configured for and this code has not been told
            # about; say so rather than reshaping garbage.
            if buf.size == h * w * 3:
                frame = cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
            elif buf.size == h * w:
                frame = cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
            else:
                return
            sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            with self._lock:
                self._latest = (frame, sim_t, self._count)
                self._newest_sim_t = max(self._newest_sim_t, sim_t)
                self._count += 1
                self._last_rx = time.monotonic()
        except Exception:
            # A malformed message must not kill the subscriber thread and take
            # the feed down with it.
            return

    def read(self) -> tuple:
        # BLOCK for the next frame, exactly as UvcFeed does. Returning
        # (None, None, {}) to mean "nothing new yet" is how a FINITE feed says
        # end-of-file, and the data layer reads it that way: one read between
        # renders and the run stops after a single published frame with a
        # cheerful "end of feed". A camera is not finite. It either hands over
        # a frame or it is broken, and _DEAD_AFTER_S decides which.
        deadline = time.monotonic() + self._DEAD_AFTER_S
        while True:
            with self._lock:
                latest, newest, n = self._latest, self._newest_sim_t, self._count
                self._latest = None
            if latest is not None:
                break
            if time.monotonic() > deadline:
                raise FeedError("DLDE-03", f"no image on '{self.topic}' for "
                                           f"{self._DEAD_AFTER_S:g}s -- Gazebo stopped or "
                                           "the world was unloaded")
            time.sleep(0.002)
        frame, sim_t, seq = latest
        age_ms = max(0.0, (newest - sim_t) * 1000.0)
        return frame, time.time(), {
            "source": "gz", "topic": self.topic, "seq": seq, "frames_seen": n,
            "sim_time_s": round(sim_t, 4),
            # Sim time, not wall time -- see the class docstring.
            "capture_age_ms": round(age_ms, 1),
        }

    def describe(self) -> dict:
        d = {"kind": "gz", "topic": self.topic, "frames_seen": self._count}
        d.update(getattr(self, "actual", {}))
        return d


def open_feed(cfg: dict) -> Feed:
    kind = cfg.get("type", "file")
    if kind == "file":
        return FileFeed(cfg["path"], fps=cfg.get("fps", 4.0), loop=cfg.get("loop", False),
                        sidecar=cfg.get("sidecar"), realtime=cfg.get("realtime", True))
    if kind == "uvc":
        return UvcFeed(cfg.get("device", 0), cfg.get("width", 1280),
                       cfg.get("height", 720), cfg.get("fps", 30.0),
                       cfg.get("fourcc", "MJPG"))
    if kind == "rtsp":
        return RtspFeed(cfg["url"])
    if kind == "gz":
        return GzFeed(cfg["topic"], cfg.get("timeout_s", 30.0))
    if kind == "env80":
        return Env80Feed(
            cfg["scene_dir"], cfg.get("frames_dir"), cfg.get("mode", "satellite"),
            cfg.get("anchor"), fps=cfg.get("fps", 4.0), loop=cfg.get("loop", False),
            realtime=cfg.get("realtime", True), envelope=cfg.get("envelope", True),
            agl_min=cfg.get("agl_min_m", 50.0), agl_max=cfg.get("agl_max_m", 100.0),
            view_angle_min=cfg.get("view_angle_min_deg", 80.0), limit=cfg.get("limit", 0))
    raise FeedError("DLE-01", f"unknown feed type '{kind}'")


def _fourcc_str(value) -> str:
    """Decode the packed FOURCC that V4L2 reports back into 'MJPG' etc."""
    try:
        v = int(value)
    except (TypeError, ValueError):
        return ""
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4)).strip("\x00 ")


def _load_sidecar(path) -> list:
    """Per-frame truth for a replay: t, lat, lon, alt_agl_m, yaw_deg."""
    if not path:
        return []
    p = Path(path)
    if not p.exists():
        return []
    rows = []
    for line in p.read_text().splitlines():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


# --------------------------------------------------------------------------
class Preprocessor:
    """Undistort, rescale to the reference GSD, encode.

    Deliberately keeps the frame in colour. XFeat is trained on RGB and takes
    three channels; converting to grey here would silently cost accuracy for
    no speed, because the resize below is what actually controls the cost.
    """

    def __init__(self, intrinsics: dict = None, fallback_long_edge: int = 512,
                 jpeg_quality: int = 80, min_scale: float = 0.05, max_scale: float = 4.0):
        self.intrinsics = intrinsics or {}
        self.fallback_long_edge = int(fallback_long_edge)
        self.jpeg_quality = int(jpeg_quality)
        self.min_scale, self.max_scale = min_scale, max_scale
        self._undistort_maps = None
        self.last_note = ""

    @property
    def fx_px(self):
        fx = self.intrinsics.get("fx_px")
        if fx:
            return float(fx)
        f_mm = self.intrinsics.get("focal_mm")
        pitch_um = self.intrinsics.get("pixel_pitch_um")
        if f_mm and pitch_um:
            return (float(f_mm) * 1e-3) / (float(pitch_um) * 1e-6)
        return None

    def frame_gsd(self, altitude_m, fx_px=None, pitch_deg=None):
        """Equation (3): GSD = h * p / f, which in pixel units is h / fx.

        Tilt lengthens the ray, so the benchmark's own geometry divides the
        height by |cos(pitch)| first. Inside the 80-degree envelope that is at
        most a 1.5% correction, which is small but free and keeps the scale
        consistent with how the ground truth was computed.
        """
        fx = fx_px or self.fx_px
        if not fx or not altitude_m:
            return None
        ground = float(altitude_m)
        if pitch_deg is not None:
            c = abs(math.cos(math.radians(float(pitch_deg))))
            if c > 1e-3:
                ground /= c
        return ground / float(fx)

    def scale_for(self, altitude_m, map_gsd_m_px, fx_px=None, pitch_deg=None):
        g = self.frame_gsd(altitude_m, fx_px, pitch_deg)
        if g is None or not map_gsd_m_px:
            return None
        return g / float(map_gsd_m_px)

    def run(self, frame_bgr: np.ndarray, altitude_m=None, map_gsd_m_px=None,
            fx_px=None, pitch_deg=None) -> tuple:
        t0 = time.perf_counter()
        out = frame_bgr
        self.last_note = ""

        if self.intrinsics.get("dist") is not None and self.fx_px:
            out = self._undistort(out)

        scale = self.scale_for(altitude_m, map_gsd_m_px, fx_px, pitch_deg)
        if scale is None:
            h, w = out.shape[:2]
            k = self.fallback_long_edge / max(h, w)
            self.last_note = ("no altitude or no fx_px, so the frame is scaled to a fixed "
                              "long edge instead of to the reference GSD")
            if k < 1.0:
                out = cv2.resize(out, (max(1, int(w * k)), max(1, int(h * k))),
                                 interpolation=cv2.INTER_AREA)
        else:
            clamped = min(max(scale, self.min_scale), self.max_scale)
            if abs(clamped - scale) > 1e-9:
                self.last_note = (f"scale {scale:.3f} clamped to {clamped:.3f}; check fx_px "
                                  "and the altitude source before trusting this fix")
            h, w = out.shape[:2]
            nw, nh = max(16, int(round(w * clamped))), max(16, int(round(h * clamped)))
            interp = cv2.INTER_AREA if clamped < 1.0 else cv2.INTER_LINEAR
            out = cv2.resize(out, (nw, nh), interpolation=interp)
            scale = clamped

        ok, buf = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise FeedError("DLE-08", "JPEG encode failed")
        return out, buf.tobytes(), {
            "preprocess_ms": (time.perf_counter() - t0) * 1000.0,
            "scale": scale,
            "frame_gsd_m_px": self.frame_gsd(altitude_m, fx_px, pitch_deg),
            "note": self.last_note,
        }

    def _undistort(self, img: np.ndarray) -> np.ndarray:
        h, w = img.shape[:2]
        i = self.intrinsics
        if self._undistort_maps is None or self._undistort_maps[0] != (w, h):
            K = np.array([[self.fx_px, 0, i.get("cx_px", w / 2)],
                          [0, i.get("fy_px", self.fx_px), i.get("cy_px", h / 2)],
                          [0, 0, 1]], dtype=np.float64)
            d = np.array(i["dist"], dtype=np.float64).reshape(-1, 1)
            m1, m2 = cv2.initUndistortRectifyMap(K, d, None, K, (w, h), cv2.CV_16SC2)
            self._undistort_maps = ((w, h), m1, m2)
        _, m1, m2 = self._undistort_maps
        return cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
