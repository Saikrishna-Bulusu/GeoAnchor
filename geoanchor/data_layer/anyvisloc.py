"""AnyVisLoc env80 as a live feed, so real frames go through the real pipeline.

Why this exists: the Sydney demo cuts its frames out of the reference map, so it
proves the three layers are wired correctly and nothing else. These are real UAV
frames over a real satellite reference with published ground truth, and the
error the output layer computes from them is comparable with the harness tables
already in `results/`.

Two conventions from the dataset are easy to get wrong and both are verified
against the benchmark's own code rather than assumed:

* **Pitch is zero at nadir.** `euler_deg` is [roll, pitch, yaw] and the
  statistics table's "view angle" is the complement, `90 - |pitch|`. A filter
  written against the table's convention selects the wrong frames and returns
  nothing.
* **There is no altitude key.** Altitude is `xyz[2]`. The position is `xyz[:2]`
  in a scene-local metric frame, and the reference JSON gives the affine into
  the map with `col = (x - origin_x) / res_x` -- a plain positive scaling with
  no y-flip (`avl_utils.py:842`).

Ground truth here is scene-local and refined by structure-from-motion, not GPS,
so there is no real latitude and longitude to be had. Rather than inventing one,
the map is given a transverse-Mercator CRS anchored at a declared origin: every
distance is exact to the millimetre, the geodetic path downstream needs no
second code path, and `local_frame` is stamped on the map packet, the session
header and the dashboard so nobody mistakes the coordinates for real positions.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ..geo import local_tmerc, pixel_to_wgs84

# Where the declared anchor sits. Arbitrary by construction -- it only has to be
# somewhere the projection is well conditioned. UTS, since that is whose lab
# this is. Override with data_layer.map.anchor if a scene needs its real place.
DEFAULT_ANCHOR = (-33.8836, 151.2005)

LOCAL_FRAME_NOTE = (
    "AnyVisLoc ground truth is scene-local and SfM-refined, not geodetic. These "
    "coordinates are a declared anchor plus a true metric offset: the distances "
    "and therefore every error are exact, the absolute positions are not real."
)


class SceneError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def scene_id_of(scene_dir: Path) -> int:
    name = Path(scene_dir).name
    try:
        return int(name.split("_")[-1])
    except ValueError as exc:
        raise SceneError("DLE-01", f"cannot read a scene id from '{name}'") from exc


def load_georeference(scene_dir: Path, mode: str = "satellite", anchor=None) -> dict:
    """Turn Lxx_reference.json into the georeference the store builder wants."""
    scene_dir = Path(scene_dir)
    sid = scene_id_of(scene_dir)
    ref_path = scene_dir / f"L{sid:02d}_reference.json"
    if not ref_path.exists():
        raise SceneError("DLE-02", f"reference JSON not found: {ref_path}")
    ref = json.loads(ref_path.read_text())

    modes = ref.get("modes", {})
    if mode not in modes:
        raise SceneError("DLE-01", f"mode '{mode}' not in {sorted(modes)}")
    m = modes[mode]

    res = [float(v) for v in m["map_resolution"]]
    origin = [float(v) for v in m.get("map_origin_local", [0.0, 0.0])]
    # map_size is [height, width]. Getting this the wrong way round silently
    # transposes the map and every fix lands somewhere plausible but wrong.
    h, w = (int(v) for v in m["map_size"])

    # Half-pixel shift so that the centre-based convention in geo.pixel_to_crs
    # reproduces the benchmark's corner-based `col = (x - origin) / res`
    # exactly. Without it every position carries a fixed 7 cm bias.
    transform = [res[0], 0.0, origin[0] - 0.5 * res[0],
                 0.0, res[1], origin[1] - 0.5 * res[1]]

    alat, alon = anchor or DEFAULT_ANCHOR
    crs = local_tmerc(alat, alon)
    corners = [pixel_to_wgs84(transform, crs, c, r) for c, r in
               ((0, 0), (w, 0), (0, h), (w, h))]
    lats = [p[0] for p in corners]
    lons = [p[1] for p in corners]

    return {
        "epsg": None, "crs": crs,
        "transform": transform, "width": w, "height": h,
        "gsd_m_px": float(res[0]),
        "bounds_wgs84": [min(lons), min(lats), max(lons), max(lats)],
        "source": str(scene_dir / m["map_path"]),
        "map_path": str(scene_dir / m["map_path"]),
        "map_origin_local": origin, "map_resolution": res,
        "scene": scene_dir.name, "mode": mode,
        "anchor_wgs84": [alat, alon],
        "local_frame": True, "local_frame_note": LOCAL_FRAME_NOTE,
    }


def list_frames(frames_dir: Path) -> list:
    frames = sorted(Path(frames_dir).glob("L??_????.npz"))
    if not frames:
        raise SceneError("DLE-02", f"no L??_????.npz under {frames_dir}")
    return frames


def read_frame(path: Path, georef: dict) -> tuple:
    """Return (image_bgr, meta). meta is the sidecar shape the replay path uses."""
    with np.load(path, allow_pickle=True) as z:
        img = np.asarray(z["image"])
        xyz = np.asarray(z["xyz"], dtype=np.float64).reshape(3)
        euler = np.asarray(z["euler_deg"], dtype=np.float64).reshape(3)
        K = np.asarray(z["K"], dtype=np.float64).reshape(3, 3)
        sample_id = str(z["sample_id"])
        dist = np.asarray(z["dist"], dtype=np.float64).reshape(-1) if "dist" in z.files else None

    bgr = img[:, :, ::-1].copy() if img.ndim == 3 and img.shape[2] == 3 else img
    roll, pitch, yaw = (float(v) for v in euler)
    x, y, alt = (float(v) for v in xyz)
    lat, lon = pixel_to_wgs84(
        georef["transform"], georef["crs"],
        *_local_to_pixel(georef, x, y))

    return bgr, {
        "sample_id": sample_id,
        "lat": lat, "lon": lon,
        "alt_agl_m": alt,
        "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
        "view_angle_deg": 90.0 - abs(pitch),   # the statistics table's convention
        "local_xy": [x, y],
        "fx_px": float(K[0, 0]), "fy_px": float(K[1, 1]),
        "cx_px": float(K[0, 2]), "cy_px": float(K[1, 2]),
        "dist": dist.tolist() if dist is not None else None,
        "fix_type": 3, "satellites": 0,
    }


def _local_to_pixel(georef: dict, x: float, y: float) -> tuple:
    res = georef["map_resolution"]
    origin = georef["map_origin_local"]
    return (x - origin[0]) / res[0], (y - origin[1]) / res[1]


def envelope_filter(frames: list, georef: dict, agl_min=50.0, agl_max=100.0,
                    view_angle_min=80.0) -> list:
    """The env80 subset rule, applied here so any scene folder can be used.

    50-100 m AGL at view angle >= 80 degrees. The pitch distribution is bimodal
    -- separate nadir and oblique passes -- so tightening from 70 to 80 degrees
    costs very few frames, while relaxing to 50 would describe a different
    aircraft than this one.
    """
    keep = []
    for f in frames:
        with np.load(f, allow_pickle=True) as z:
            alt = float(np.asarray(z["xyz"]).reshape(3)[2])
            pitch = float(np.asarray(z["euler_deg"]).reshape(3)[1])
        if agl_min <= alt <= agl_max and (90.0 - abs(pitch)) >= view_angle_min:
            keep.append(f)
    return keep
