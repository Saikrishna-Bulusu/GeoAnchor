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
    boards is a comparison of the boards."""
    kind = "uvc"

    def __init__(self, device=0, width: int = 1280, height: int = 720, fps: float = 30.0):
        self.device = device
        node = f"/dev/video{device}" if isinstance(device, int) else str(device)
        if isinstance(device, int) and not Path(node).exists():
            raise FeedError("DLDE-01", f"{node} does not exist. `v4l2-ctl --list-devices` shows what is attached.")
        self.cap = cv2.VideoCapture(device if isinstance(device, int) else str(device), cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise FeedError("DLDE-02", f"cannot open {node} -- in use by another process, or no V4L2 support")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self.cap.set(cv2.CAP_PROP_FPS, float(fps))
        # A driver buffer queue turns into latency the moment matching runs
        # slower than capture, and that latency is invisible in the timestamp.
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except cv2.error:
            pass
        self.index = 0

    def read(self) -> tuple:
        ok, frame = self.cap.read()
        t = time.time()                       # stamp as close to capture as we can get
        if not ok:
            raise FeedError("DLDE-03", "camera stopped returning frames")
        self.index += 1
        return frame, t, {"frame_index": self.index}

    def close(self) -> None:
        self.cap.release()

    def describe(self) -> dict:
        return {"kind": "uvc", "device": str(self.device),
                "width": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "height": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}


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


def open_feed(cfg: dict) -> Feed:
    kind = cfg.get("type", "file")
    if kind == "file":
        return FileFeed(cfg["path"], fps=cfg.get("fps", 4.0), loop=cfg.get("loop", False),
                        sidecar=cfg.get("sidecar"), realtime=cfg.get("realtime", True))
    if kind == "uvc":
        return UvcFeed(cfg.get("device", 0), cfg.get("width", 1280),
                       cfg.get("height", 720), cfg.get("fps", 30.0))
    if kind == "rtsp":
        return RtspFeed(cfg["url"])
    if kind == "env80":
        return Env80Feed(
            cfg["scene_dir"], cfg.get("frames_dir"), cfg.get("mode", "satellite"),
            cfg.get("anchor"), fps=cfg.get("fps", 4.0), loop=cfg.get("loop", False),
            realtime=cfg.get("realtime", True), envelope=cfg.get("envelope", True),
            agl_min=cfg.get("agl_min_m", 50.0), agl_max=cfg.get("agl_max_m", 100.0),
            view_angle_min=cfg.get("view_angle_min_deg", 80.0), limit=cfg.get("limit", 0))
    raise FeedError("DLE-01", f"unknown feed type '{kind}'")


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
