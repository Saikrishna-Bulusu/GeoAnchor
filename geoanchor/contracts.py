"""The messages that cross layer boundaries.

These dataclasses ARE the inter-layer contract. A layer may be rewritten,
moved to another machine or replaced by a stub, and as long as it puts these
shapes on the bus the other two do not notice. Keep them JSON-serialisable:
frames travel as a JSON header plus a raw JPEG payload, never as an array
inside the header.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field, asdict
from typing import Any

SCHEMA_VERSION = 1

# Bus topics. One string, used by publisher and subscriber alike.
T_MAP = "map"
T_FRAME = "frame"
T_GPS = "gps"
T_FIX = "fix"
T_RECORD = "record"
T_LOG = "log"
T_STATUS = "status"


def now_unix() -> float:
    return time.time()


def now_mono() -> float:
    return time.monotonic()


@dataclass
class MapPacket:
    """Says where the reference map is and what shape it has.

    Republished on a timer so a layer that starts late still receives it --
    ZeroMQ PUB/SUB drops anything sent before a subscriber has connected.
    """
    store_id: str                 # content hash of source + params; the cache key
    store_path: str               # directory holding manifest.json and tiles/
    source_path: str
    epsg: int
    transform: list                # affine, rasterio order: a b c d e f
    width: int
    height: int
    gsd_m_px: float
    bounds_wgs84: list             # west, south, east, north
    tile_px: int
    tile_grid: list                # rows, cols
    n_tiles: int
    method: str                   # method the reference descriptors were built with
    built_at: float
    crs: str = None               # PROJ string; overrides epsg when present
    local_frame: bool = False     # True when the coordinates are not real positions
    schema: int = SCHEMA_VERSION


@dataclass
class FramePacket:
    """One preprocessed camera frame. The JPEG rides alongside as the payload."""
    seq: int
    t_capture_unix: float         # stamped at CAPTURE, never at publish. EKF3
    t_capture_mono: float         # compensates delay from this, so it must be true.
    width: int
    height: int
    feed: str                     # "uvc" | "file" | "rtsp"
    altitude_m: float = None
    roll_deg: float = None
    pitch_deg: float = None
    yaw_deg: float = None
    preprocess_ms: float = 0.0
    schema: int = SCHEMA_VERSION


@dataclass
class GpsPacket:
    """Actual GPS, from the flight controller or a replay log."""
    t_unix: float
    lat: float
    lon: float
    alt_amsl_m: float = None
    rel_alt_m: float = None
    fix_type: int = 0
    satellites: int = 0
    eph_m: float = None
    source: str = "mavlink"
    schema: int = SCHEMA_VERSION


@dataclass
class FixPacket:
    """The predicted GPS, and everything needed to judge it."""
    seq: int
    t_capture_unix: float
    t_fix_unix: float
    latency_ms: float             # capture -> fix. The number EKF3 cares about.
    method: str
    accepted: bool
    lat: float = None
    lon: float = None
    sigma_m: float = None          # 1-sigma horizontal, metres
    covariance_source: str = "gate_only"
    inliers: int = 0
    matches: int = 0
    keypoints: int = 0
    inlier_ratio: float = 0.0
    reproj_err_px: float = None
    reject_code: str = None        # a PLE-* code when accepted is False
    altitude_m: float = None
    stage_ms: dict = field(default_factory=dict)
    schema: int = SCHEMA_VERSION


@dataclass
class RecordPacket:
    """One row of the export. This is the shape the JSON file carries."""
    time_step: int
    t_unix: float
    actual_gps: dict
    predicted_gps: dict
    error_m: float
    loss: float
    accepted: bool
    sigma_m: float = None
    inliers: int = 0
    latency_ms: float = None
    method: str = ""
    loop_mode: str = "open"
    sent_to_fc: bool = False
    codes: list = field(default_factory=list)
    schema: int = SCHEMA_VERSION


@dataclass
class LogPacket:
    t_unix: float
    layer: str
    code: str
    level: str                    # info | warn | error
    message: str
    detail: dict = field(default_factory=dict)
    schema: int = SCHEMA_VERSION


@dataclass
class StatusPacket:
    """Heartbeat. Absence of these is how the dashboard knows a layer died."""
    layer: str
    t_unix: float
    ready: bool
    uptime_s: float
    last_code: str
    counts: dict = field(default_factory=dict)   # step/error/device tallies
    rate_hz: float = 0.0
    config: dict = field(default_factory=dict)   # the APPLIED config
    schema: int = SCHEMA_VERSION


def to_dict(obj) -> dict:
    return asdict(obj)
