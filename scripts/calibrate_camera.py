#!/usr/bin/env python3
"""Calibrate the camera against a chessboard shown ON A SCREEN. No printer, no ruler.

    python scripts/make_chessboard.py          # then display it full-screen
    python scripts/calibrate_camera.py         # move the camera around; it auto-captures
    python scripts/calibrate_camera.py --apply configs/camera.yaml

WHY NO RULER
------------
`cv2.calibrateCamera` returns fx in PIXELS, and fx is invariant to the assumed
physical square size: scale every object point by k and the solved translation
scales by k while fx does not move. Verified numerically against synthetic
views -- assuming 0.025, 1.0 and 137.0 returns fx identical to three decimal
places. So this assumes 1.0 and never asks how big your squares are. It is the
extrinsics that would need a real measurement, and nothing here uses them.

WHY RESOLUTION IS NOT OPTIONAL
------------------------------
fx scales with image width. A calibration done at 640x480 is wrong by 2x for a
pipeline running 1280x720. This defaults to the same 1280x720 MJPG the data
layer uses and records the size it actually got, and `--apply` refuses to write
intrinsics whose capture size does not match the config's.

WHAT MAKES A CALIBRATION GOOD
-----------------------------
Not the number of views -- the DIVERSITY of them. fx and Z trade off against
each other in a single frontal view, and only oblique views separate them. So
this rejects a frame whose corner geometry is too close to one already banked,
and it reports the spread it achieved. Tilt the camera left, right, up, down
and roll it; work the board into all four corners of the frame as well as the
middle; vary the distance.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError as exc:                       # noqa: E722
    raise SystemExit(
        f"{exc}\n\n"
        "This needs the project virtualenv, which carries OpenCV:\n"
        "    cd %s && source .venv/bin/activate\n"
        "then re-run. (`python3` on its own is the system interpreter and does\n"
        "not have cv2 -- this is the most common way to trip over these scripts.)"
        % Path(__file__).resolve().parent.parent)

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE |
         cv2.CALIB_CB_FAST_CHECK)
TERM = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def object_points(nx: int, ny: int) -> np.ndarray:
    """Unit squares. See the module docstring: the scale cancels out of fx."""
    p = np.zeros((nx * ny, 3), np.float32)
    p[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    return p


def novelty(corners: np.ndarray, banked: list, min_shift: float) -> bool:
    """Is this view different enough from every banked one to be worth keeping?

    Compared on the corner set itself rather than on a solved pose: a pose needs
    intrinsics, which is what we do not have yet.
    """
    c = corners.reshape(-1, 2)
    for b in banked:
        if np.linalg.norm(c - b.reshape(-1, 2), axis=1).mean() < min_shift:
            return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fourcc", default="MJPG")
    ap.add_argument("--pattern", type=int, nargs=2, default=[9, 6],
                    help="INNER corners across and down (a 10x7 board is 9 6)")
    ap.add_argument("--views", type=int, default=20)
    ap.add_argument("--min-shift", type=float, default=40.0,
                    help="mean corner movement, px, before a view counts as new")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--save-frames", default="", help="directory to keep the captures in")
    ap.add_argument("--json", default=str(REPO / "results" / "camera_intrinsics.json"))
    ap.add_argument("--apply", default="", help="config file to write the intrinsics into")
    a = ap.parse_args()

    nx, ny = a.pattern
    cap = cv2.VideoCapture(a.device)
    if not cap.isOpened():
        print(f"cannot open /dev/video{a.device}")
        return 2
    if a.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (got_w, got_h) != (a.width, a.height):
        print(f"NOTE: asked for {a.width}x{a.height}, camera gave {got_w}x{got_h}. "
              f"The intrinsics below belong to {got_w}x{got_h}.")

    print(f"looking for a {nx}x{ny} inner-corner chessboard at {got_w}x{got_h}.")
    print(f"Move the camera between captures -- tilt, roll, and work the board into")
    print(f"the frame corners. Need {a.views} distinct views.\n")

    banked_img: list = []
    banked_obj: list = []
    objp = object_points(nx, ny)
    frames = []
    t0 = time.time()
    last_note = 0.0
    seen = 0
    while len(banked_img) < a.views and time.time() - t0 < a.timeout:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (nx, ny), FLAGS)
        if not found:
            if time.time() - last_note > 5.0:
                print(f"  [{time.time()-t0:5.0f}s] no board in view "
                      f"({len(banked_img)}/{a.views} banked)")
                last_note = time.time()
            continue
        seen += 1
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), TERM)
        if not novelty(corners, banked_img, a.min_shift):
            continue
        banked_img.append(corners)
        banked_obj.append(objp)
        frames.append(frame.copy())
        print(f"  [{time.time()-t0:5.0f}s] captured view {len(banked_img)}/{a.views}")
    cap.release()

    if len(banked_img) < 6:
        print(f"\nonly {len(banked_img)} views ({seen} detections). "
              f"Need at least 6, ideally {max(a.views, 15)}.")
        print("If nothing was detected at all: check the pattern size (--pattern counts")
        print("INNER corners, so a 10x7-square board is '9 6'), the screen brightness,")
        print("and that the whole board including a margin of background is in frame.")
        return 1

    print(f"\ncalibrating on {len(banked_img)} views...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        banked_obj, banked_img, (got_w, got_h), None, None)

    # Per-view reprojection error, because a single bad view drags the mean and
    # is worth being able to see and drop.
    errs = []
    for i in range(len(banked_obj)):
        proj, _ = cv2.projectPoints(banked_obj[i], rvecs[i], tvecs[i], K, dist)
        errs.append(float(cv2.norm(banked_img[i], proj, cv2.NORM_L2) / len(proj)))

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f"\n  rms reprojection error  {rms:.4f} px   (under ~0.5 is good, "
          f"over ~1.0 means redo it)")
    print(f"  per-view error   min {min(errs):.3f}  median "
          f"{sorted(errs)[len(errs)//2]:.3f}  max {max(errs):.3f}")
    print(f"\n  fx_px  {fx:9.2f}      fy_px  {fy:9.2f}")
    print(f"  cx_px  {cx:9.2f}      cy_px  {cy:9.2f}   (centre would be "
          f"{got_w/2:.1f}, {got_h/2:.1f})")
    print(f"  dist   {np.array2string(dist.ravel(), precision=5, suppress_small=True)}")
    if abs(fx - fy) / fx > 0.05:
        print("  WARNING: fx and fy differ by more than 5%. Real sensors are close to")
        print("           square; this usually means too few oblique views.")
    print(f"\n  GSD at 75 m AGL: {75.0/fx*100:.2f} cm/px   "
          f"(the pipeline uses GSD = altitude / fx_px)")

    out = {"device": a.device, "width": got_w, "height": got_h,
           "fourcc": a.fourcc, "pattern": [nx, ny], "views": len(banked_img),
           "rms_px": round(float(rms), 4),
           "per_view_err_px": [round(e, 4) for e in errs],
           "fx_px": round(float(fx), 2), "fy_px": round(float(fy), 2),
           "cx_px": round(float(cx), 2), "cy_px": round(float(cy), 2),
           "dist": [round(float(x), 6) for x in dist.ravel()],
           "note": "square size assumed 1.0; fx in pixels is invariant to it"}
    Path(a.json).parent.mkdir(parents=True, exist_ok=True)
    Path(a.json).write_text(json.dumps(out, indent=2))
    print(f"\nwrote {a.json}")

    if a.save_frames:
        d = Path(a.save_frames); d.mkdir(parents=True, exist_ok=True)
        for i, f in enumerate(frames):
            cv2.imwrite(str(d / f"view_{i:02d}.png"), f)
        print(f"wrote {len(frames)} frames to {d}")

    print("\nPaste into the config's data_layer.feed.intrinsics:")
    print(f"      fx_px: {fx:.1f}\n      fy_px: {fy:.1f}")
    print(f"      cx_px: {cx:.1f}\n      cy_px: {cy:.1f}")
    print(f"      dist: [{', '.join(f'{x:.5f}' for x in dist.ravel())}]")

    if a.apply:
        _apply(Path(a.apply), out)
    return 0


def _apply(cfg: Path, out: dict) -> None:
    import yaml
    doc = yaml.safe_load(cfg.read_text()) or {}
    feed = ((doc.get("data_layer") or {}).get("feed") or {})
    w, h = feed.get("width"), feed.get("height")
    if (w, h) != (out["width"], out["height"]):
        print(f"\nNOT applied: {cfg} captures at {w}x{h} but this calibration is for "
              f"{out['width']}x{out['height']}. fx scales with width -- recalibrate at "
              f"the config's size, or change the config first.")
        return
    # Rewritten textually rather than by round-tripping the YAML, because these
    # config files carry the reasoning in their comments and yaml.dump discards
    # every one of them.
    text = cfg.read_text()
    for key in ("fx_px", "fy_px", "cx_px", "cy_px"):
        import re
        pat = re.compile(rf"^(\s+){key}:\s*\S+.*$", re.M)
        if pat.search(text):
            text = pat.sub(lambda m: f"{m.group(1)}{key}: {out[key]}", text, count=1)
        else:
            print(f"  note: no '{key}:' line found in {cfg}, left alone")
    cfg.write_text(text)
    print(f"\napplied fx/fy/cx/cy to {cfg}")


if __name__ == "__main__":
    raise SystemExit(main())
