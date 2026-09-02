#!/usr/bin/env python3
"""Synthesise a flight over a georeferenced tile, for bring-up only.

    python3 scripts/make_demo_video.py --map data/sydney/ref_tile.tif

Writes demo/flight.mp4 and demo/flight_gt.jsonl (per-frame truth).

READ THIS BEFORE QUOTING ANY NUMBER THAT COMES OUT OF IT
--------------------------------------------------------
The frames are cut from the same image used as the reference map. The pipeline
is therefore matching a picture against itself, which is the single easiest
case that exists and is exactly the failure mode the project's standing rule
warns about for the Gazebo ground texture. Every session built on this feed is
stamped `synthetic_from_reference: true` and the dashboard shows a banner.

What it IS good for: proving the three layers talk to each other, that the
step codes fire in order, that the geodetic solve is wired the right way round,
and that the export is well formed. If the error here is not sub-metre,
something is broken -- that is the whole diagnostic value.

What it is NOT: an accuracy result, a matcher comparison, or anything that
belongs in a table. Real numbers need a frame source that is not the reference:
AnyVisLoc env80, or a flight.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parent.parent


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=str(REPO.parent / "data" / "sydney" / "ref_tile.tif"))
    ap.add_argument("--out", default=str(REPO / "demo"))
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--fx-px", type=float, default=1200.0)
    ap.add_argument("--altitude", type=float, default=75.0)
    ap.add_argument("--fps", type=float, default=4.0)
    ap.add_argument("--degrade", type=float, default=1.0,
                    help="0 = identical pixels, 1 = default blur/exposure/noise, >1 = harder")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    src = Path(args.map)
    if not src.exists():
        print(f"map not found: {src}")
        return 2

    try:
        import rasterio
        from rasterio.warp import transform_bounds
        with rasterio.open(src) as ds:
            t = ds.transform
            transform = [t.a, t.b, t.c, t.d, t.e, t.f]
            epsg = ds.crs.to_epsg()
            n = min(ds.count, 3)
            tile = np.transpose(ds.read(list(range(1, n + 1))), (1, 2, 0))[:, :, ::-1].copy()
    except ImportError:
        print("rasterio is required to read the georeference")
        return 2

    sys_path = str(REPO)
    import sys
    if sys_path not in sys.path:
        sys.path.insert(0, sys_path)
    from geoanchor.geo import pixel_to_wgs84

    H, W = tile.shape[:2]
    gsd = abs(transform[0])
    # Footprint in map pixels at this altitude and focal length.
    fw = args.altitude * args.width / args.fx_px / gsd
    fh = args.altitude * args.height / args.fx_px / gsd
    margin = math.hypot(fw, fh) / 2 + 8
    if margin * 2 >= min(W, H):
        print(f"footprint {fw:.0f}x{fh:.0f} px does not fit inside {W}x{H}; raise --fx-px or lower --altitude")
        return 2

    rng = np.random.default_rng(args.seed)
    out_dir = Path(args.out); out_dir.mkdir(parents=True, exist_ok=True)
    vid = out_dir / "flight.mp4"
    gt = out_dir / "flight_gt.jsonl"

    writer = cv2.VideoWriter(str(vid), cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (args.width, args.height))
    if not writer.isOpened():
        print("cannot open the video writer -- is OpenCV built with FFmpeg?")
        return 2

    path = _lawnmower(W, H, margin, args.frames)
    rows = []
    t0 = 1_800_000_000.0
    for i, (cx, cy, yaw) in enumerate(path):
        win = _window(tile, cx, cy, fw, fh, yaw)
        frame = cv2.resize(win, (args.width, args.height), interpolation=cv2.INTER_CUBIC)
        frame = _degrade(frame, rng, args.degrade)
        writer.write(frame)
        lat, lon = pixel_to_wgs84(transform, epsg, cx, cy)
        rows.append({
            "t": t0 + i / args.fps, "frame_index": i,
            "lat": round(lat, 8), "lon": round(lon, 8),
            "alt_agl_m": args.altitude, "yaw_deg": round(yaw % 360.0, 2),
            "roll_deg": round(float(rng.normal(0, 1.5)), 2),
            "pitch_deg": round(float(rng.normal(0, 1.5)), 2),
            "map_px": [round(cx, 2), round(cy, 2)],
        })
    writer.release()
    gt.write_text("\n".join(json.dumps(r) for r in rows) + "\n")

    print(f"map        {src.name}  {W}x{H} @ {gsd:.4f} m/px  EPSG:{epsg}")
    print(f"camera     {args.width}x{args.height}  fx {args.fx_px:.0f} px  altitude {args.altitude:.0f} m")
    print(f"footprint  {fw:.0f}x{fh:.0f} map px  =  {fw*gsd:.1f}x{fh*gsd:.1f} m on the ground")
    print(f"video      {vid}  ({args.frames} frames @ {args.fps} fps, {vid.stat().st_size/1e6:.1f} MB)")
    print(f"truth      {gt}")
    print()
    print("Frames are cut from the reference map. Plumbing demo only -- not an accuracy result.")
    return 0


def _lawnmower(W: int, H: int, margin: float, n: int) -> list:
    """Three legs with turns, so yaw actually changes and rectification is exercised."""
    x0, x1 = margin, W - margin
    ys = [margin + (H - 2 * margin) * f for f in (0.2, 0.5, 0.8)]
    pts = []
    per = max(2, n // 3)
    for leg, y in enumerate(ys):
        a, b = (x0, x1) if leg % 2 == 0 else (x1, x0)
        for k in range(per):
            f = k / max(1, per - 1)
            x = a + (b - a) * f
            yaw = 90.0 if leg % 2 == 0 else 270.0
            yaw += 6.0 * math.sin(f * math.pi * 2)      # gentle heading wander
            pts.append((x, y, yaw))
    while len(pts) < n:
        pts.append(pts[-1])
    return pts[:n]


def _window(tile: np.ndarray, cx: float, cy: float, w: float, h: float, yaw_deg: float) -> np.ndarray:
    """Crop a rotated window centred on (cx, cy) in map pixels."""
    wi, hi = int(round(w)), int(round(h))
    M = cv2.getRotationMatrix2D((float(cx), float(cy)), yaw_deg, 1.0)
    M[0, 2] += wi / 2.0 - cx
    M[1, 2] += hi / 2.0 - cy
    return cv2.warpAffine(tile, M, (wi, hi), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REFLECT101)


def _degrade(img: np.ndarray, rng, strength: float) -> np.ndarray:
    """Make it not literally the same pixels: optics, exposure and sensor noise.

    Nowhere near a real domain gap -- season, time of day and a different
    sensor are what actually break matching -- but enough that a perfect score
    means the geometry is right rather than that memcmp succeeded.
    """
    if strength <= 0:
        return img
    out = cv2.GaussianBlur(img, (0, 0), 0.6 * strength)
    gain = 1.0 + float(rng.normal(0, 0.05 * strength))
    bias = float(rng.normal(0, 6 * strength))
    out = np.clip(out.astype(np.float32) * gain + bias, 0, 255)
    out += rng.normal(0, 2.5 * strength, out.shape).astype(np.float32)
    return np.clip(out, 0, 255).astype(np.uint8)


if __name__ == "__main__":
    raise SystemExit(main())
