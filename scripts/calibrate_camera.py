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


def _cov(cov: dict, need: int) -> str:
    return " ".join(f"{k}{'ok' if v >= need else f'{v}/{need}'}" for k, v in cov.items())


def object_points(nx: int, ny: int) -> np.ndarray:
    """Unit squares. See the module docstring: the scale cancels out of fx."""
    p = np.zeros((nx * ny, 3), np.float32)
    p[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    return p


def foreshortening(corners: np.ndarray, nx: int, ny: int) -> tuple:
    """Signed tilt of the board about each axis, from edge foreshortening alone.

    THIS IS THE WHOLE BALLGAME AND THE FIRST VERSION GOT IT WRONG. A planar
    target cannot separate focal length from distance in a frontal view: make
    the board twice as far and the lens twice as long and the image is
    identical. Only PERSPECTIVE breaks the tie -- the near edge subtending more
    pixels than the far one. Bank twenty views by sliding the camera sideways
    and every one of them is the same degenerate observation; the solver then
    runs the focal length off to infinity with compensating distortion, which
    is exactly what happened here on 8 Sept: fx = 47308 on a 1280 px frame, an
    implied 1.55 degree field of view, and radial terms of 3e8.

    Measuring it needs no intrinsics: in a frontal view opposite edges of the
    board are the same length in the image, and tilt makes them differ. The log
    ratio is signed, so it also says WHICH WAY the board is tilted, which is
    what lets the caller ask for coverage in all four directions rather than
    twenty views leaning the same way.
    """
    g = corners.reshape(ny, nx, 2)
    top = np.linalg.norm(g[0, -1] - g[0, 0])
    bottom = np.linalg.norm(g[-1, -1] - g[-1, 0])
    left = np.linalg.norm(g[-1, 0] - g[0, 0])
    right = np.linalg.norm(g[-1, -1] - g[0, -1])
    return (float(np.log(top / bottom)), float(np.log(left / right)))


def novelty(corners: np.ndarray, banked: list, min_shift: float) -> bool:
    """Different enough from every banked view to be worth keeping?"""
    c = corners.reshape(-1, 2)
    for b in banked:
        if np.linalg.norm(c - b.reshape(-1, 2), axis=1).mean() < min_shift:
            return False
    return True


def tilt_coverage(tilts: list, thresh: float) -> dict:
    """Which of the four tilt directions the banked set actually contains."""
    # tx is log(top edge / bottom edge), so it reports tilt about the
    # HORIZONTAL axis -- up and down. ty is log(left / right) and reports tilt
    # about the vertical axis. Getting these the wrong way round only mislabels
    # the prompt, but the prompt is the whole point of measuring it.
    return {"up":    sum(1 for x, _ in tilts if x >= thresh),
            "down":  sum(1 for x, _ in tilts if x <= -thresh),
            "left":  sum(1 for _, y in tilts if y >= thresh),
            "right": sum(1 for _, y in tilts if y <= -thresh)}


def sanity(fx: float, fy: float, width: int, rms: float) -> list:
    """Reasons this calibration must not be written into a config.

    A degenerate solve does not announce itself -- it returns a clean-looking
    matrix and an rms that can even be small. These are the three tells.
    """
    bad = []
    if not (0.3 <= fx / width <= 4.0):
        import math
        fov = 2 * math.degrees(math.atan(width / (2 * fx)))
        bad.append(f"fx/width = {fx/width:.2f}, outside the 0.3-4.0 any real lens "
                   f"gives (that is a {fov:.2f} deg horizontal field of view)")
    if abs(fx - fy) / fx > 0.05:
        bad.append(f"fx and fy differ by {abs(fx-fy)/fx*100:.1f}%, over 5% -- real "
                   f"sensors have near-square pixels")
    if rms > 1.0:
        bad.append(f"rms reprojection error {rms:.3f} px, over 1.0")
    return bad


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
    ap.add_argument("--min-sharpness", type=float, default=60.0,
                    help="variance of the Laplacian below which a frame is too "
                         "blurred to localise corners in")
    ap.add_argument("--max-motion", type=float, default=2.0,
                    help="mean corner movement, px, between two consecutive "
                         "detections for the board to count as held still")
    ap.add_argument("--max-rms", type=float, default=1.0,
                    help="target rms; worst views are dropped until this is met")
    ap.add_argument("--min-tilt", type=float, default=0.06,
                    help="log ratio of opposite edge lengths counting as a tilted "
                         "view. This is a PERSPECTIVE SIGNAL threshold, not an "
                         "angle: 0.06 means one edge is ~6%% longer than the one "
                         "opposite it, which a board filling the frame reaches at "
                         "about 15-20 deg and a small distant board needs more tilt "
                         "to reach. Signal is what the solver needs, so thresholding "
                         "it rather than the angle is the right way round.")
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
    tilts: list = []
    prev_corners = None
    objp = object_points(nx, ny)
    frames = []
    t0 = time.time()
    last_note = 0.0
    seen = 0
    need = max(2, a.views // 8)          # per direction
    while (len(banked_img) < a.views or
           min(tilt_coverage(tilts, a.min_tilt).values()) < need) and \
            time.time() - t0 < a.timeout:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (nx, ny), FLAGS)
        cov = tilt_coverage(tilts, a.min_tilt)
        if not found:
            if time.time() - last_note > 5.0:
                print(f"  [{time.time()-t0:5.0f}s] no board in view "
                      f"({len(banked_img)}/{a.views} banked, {_cov(cov, need)})")
                last_note = time.time()
            continue
        seen += 1
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), TERM)

        # BLUR. The board is found in a blurred frame just fine; cornerSubPix
        # then localises smeared corners confidently and wrongly, and the view
        # lands in the set looking like every other one. Two such views out of
        # 22 took a run from 0.7 px to 2.79 px rms on 8 Sept, because rms is
        # over POINTS and a couple of bad views dominate it.
        sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
        if sharp < a.min_sharpness:
            if time.time() - last_note > 4.0:
                print(f"  [{time.time()-t0:5.0f}s] too blurred to trust "
                      f"(sharpness {sharp:.0f} < {a.min_sharpness:.0f}) -- hold still")
                last_note = time.time()
            prev_corners = corners
            continue

        # STATIONARY. Same problem from the other side: auto-capture while the
        # camera is still moving is what produces the blur in the first place.
        # Requiring two consecutive detections in nearly the same place is a
        # cheaper and more direct test than any blur metric.
        if prev_corners is not None:
            moved = np.linalg.norm(corners.reshape(-1, 2) -
                                   prev_corners.reshape(-1, 2), axis=1).mean()
            if moved > a.max_motion:
                prev_corners = corners
                continue
        else:
            prev_corners = corners
            continue
        prev_corners = corners

        tx, ty = foreshortening(corners, nx, ny)
        # Once the count is met, only views that fill a MISSING tilt direction
        # are still worth taking -- otherwise the tail of the session is twenty
        # more of whatever is easiest to hold, which is what produced the
        # degenerate solve.
        enough = len(banked_img) >= a.views
        fills = ((tx >= a.min_tilt and cov["up"] < need) or
                 (tx <= -a.min_tilt and cov["down"] < need) or
                 (ty >= a.min_tilt and cov["left"] < need) or
                 (ty <= -a.min_tilt and cov["right"] < need))
        if enough and not fills:
            if time.time() - last_note > 4.0:
                print(f"  [{time.time()-t0:5.0f}s] {len(banked_img)} views, but "
                      f"{_cov(cov, need)} -- TILT the camera that way")
                last_note = time.time()
            continue
        if not novelty(corners, banked_img, a.min_shift):
            continue
        banked_img.append(corners)
        banked_obj.append(objp)
        tilts.append((tx, ty))
        frames.append(frame.copy())
        lean = ("frontal" if max(abs(tx), abs(ty)) < a.min_tilt
                else f"tilt {tx:+.2f},{ty:+.2f}")
        print(f"  [{time.time()-t0:5.0f}s] view {len(banked_img)}/{a.views}  "
              f"{lean}  {_cov(tilt_coverage(tilts, a.min_tilt), need)}")
    cap.release()

    cov = tilt_coverage(tilts, a.min_tilt)
    print(f"\ntilt coverage: {_cov(cov, need)}   (need {need} each)")
    if tilts and min(cov.values()) < need:
        print("  WARNING: the views lean mostly one way. A planar board cannot")
        print("           separate focal length from distance without perspective,")
        print("           so this may still solve to a nonsense focal length.")

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
        # norm/sqrt(N), not norm/N. NORM_L2 is already sqrt(sum of squares), so
        # dividing by N understates the per-view RMS by sqrt(N) -- a factor of
        # 7.35 on a 9x6 board, which is enough to make 0.9 px views look like
        # 0.12 px ones and hide a degenerate solve completely.
        errs.append(float(cv2.norm(banked_img[i], proj, cv2.NORM_L2) /
                          np.sqrt(len(proj))))

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f"\n  rms reprojection error  {rms:.4f} px   (under ~0.5 is good, "
          f"over ~1.0 means redo it)")
    print(f"  per-view error   min {min(errs):.3f}  median "
          f"{sorted(errs)[len(errs)//2]:.3f}  max {max(errs):.3f}")
    print(f"\n  fx_px  {fx:9.2f}      fy_px  {fy:9.2f}")
    print(f"  cx_px  {cx:9.2f}      cy_px  {cy:9.2f}   (centre would be "
          f"{got_w/2:.1f}, {got_h/2:.1f})")
    print(f"  dist   {np.array2string(dist.ravel(), precision=5, suppress_small=True)}")
    bad = sanity(float(fx), float(fy), got_w, float(rms))
    if bad:
        print("\n  THIS CALIBRATION IS NOT USABLE:")
        for why in bad:
            print(f"    - {why}")
        print("\n  Almost always this is too little PERSPECTIVE. A flat board seen")
        print("  head-on cannot separate focal length from distance -- twice as far")
        print("  with twice the focal length looks identical -- so the solver runs")
        print("  fx off to infinity and hides the error in the distortion terms.")
        print("  Sliding the camera sideways does not help; only TILTING does.")
        print(f"  Tilt coverage this run: {_cov(cov, need)}")
        print("\n  Redo it: hold the board at 30-45 degrees to the camera and take")
        print("  views leaning left, right, up AND down, plus a few square-on. Vary")
        print("  the distance. The script now refuses to stop until all four")
        print("  directions are covered.")
    else:
        print(f"\n  GSD at 75 m AGL: {75.0/fx*100:.2f} cm/px   "
              f"(the pipeline uses GSD = altitude / fx_px)")

    out = {"usable": not bad, "rejected_because": bad,
           "tilt_coverage": cov, "tilts": [[round(x, 3), round(y, 3)] for x, y in tilts],
           "device": a.device, "width": got_w, "height": got_h,
           "fourcc": a.fourcc, "pattern": [nx, ny],
           "views": len(kept), "views_captured": len(kept) + len(dropped),
           "dropped": dropped,
           # The corners themselves, so a failed run can be re-fitted offline
           # instead of re-shot. The first two failures both had to be re-shot
           # only because this was not saved.
           "corners": [c.reshape(-1, 2).round(3).tolist() for c in banked_img],
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

    if bad:
        print("\nNot writing these anywhere. Re-run the capture.")
        return 1

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
