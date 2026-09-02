#!/usr/bin/env python3
"""Chessboard calibration. Produces the fx_px the rescale depends on.

Do not take fx from a datasheet. The whole altitude-adaptive rescale is
GSD = altitude / fx_px, so an fx that is 5% wrong makes every frame 5% the
wrong size and hands the matcher a scale gap it did not need to bridge.

    python scripts/calibrate_camera.py --device 0 --rows 6 --cols 9 --square 25
    python scripts/calibrate_camera.py --images 'calib/*.jpg' --rows 6 --cols 9 --square 25

Interactive capture: point the board at a printed chessboard and press SPACE
when the overlay shows a detection. Twenty views from varied angles and
distances is plenty. Prints a config block to paste into system.yaml.
"""
from __future__ import annotations

import argparse
import glob
import sys

import cv2
import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default=None)
    ap.add_argument("--images", default=None, help="glob of already-captured views")
    ap.add_argument("--rows", type=int, default=6, help="INNER corners per column")
    ap.add_argument("--cols", type=int, default=9, help="INNER corners per row")
    ap.add_argument("--square", type=float, default=25.0, help="square size in mm")
    ap.add_argument("--views", type=int, default=20)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_pts, img_pts, size = [], [], None

    def take(gray):
        ok, corners = cv2.findChessboardCorners(
            gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            return False
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
        obj_pts.append(objp.copy()); img_pts.append(corners)
        return True

    if args.images:
        files = sorted(glob.glob(args.images))
        if not files:
            print(f"no images matched {args.images}")
            return 2
        for f in files:
            img = cv2.imread(f)
            if img is None:
                continue
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            size = g.shape[::-1]
            print(f"  {'ok  ' if take(g) else 'miss'} {f}")
    else:
        dev = args.device if args.device is None else (
            int(args.device) if str(args.device).isdigit() else args.device)
        cap = cv2.VideoCapture(dev if dev is not None else 0, cv2.CAP_V4L2)
        if not cap.isOpened():
            print("cannot open the camera")
            return 2
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        print(f"SPACE to keep a view, q to finish. Target {args.views} views, varied angles.")
        while len(obj_pts) < args.views:
            ok, frame = cap.read()
            if not ok:
                break
            g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            size = g.shape[::-1]
            found, corners = cv2.findChessboardCorners(g, pattern, cv2.CALIB_CB_FAST_CHECK)
            vis = frame.copy()
            if found:
                cv2.drawChessboardCorners(vis, pattern, corners, found)
            cv2.putText(vis, f"{len(obj_pts)}/{args.views}  {'DETECTED' if found else 'searching'}",
                        (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 220, 0) if found else (0, 0, 220), 2)
            cv2.imshow("calibrate", vis)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == 32 and found and take(g):
                print(f"  kept view {len(obj_pts)}")
        cap.release(); cv2.destroyAllWindows()

    if len(obj_pts) < 6:
        print(f"only {len(obj_pts)} usable views. Need at least 6, ideally 20.")
        return 1

    rms, Kc, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    fx, fy, cx, cy = Kc[0, 0], Kc[1, 1], Kc[0, 2], Kc[1, 2]

    errs = []
    for i in range(len(obj_pts)):
        proj, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], Kc, dist)
        errs.append(cv2.norm(img_pts[i], proj, cv2.NORM_L2) / len(proj))

    print(f"\n{len(obj_pts)} views at {size[0]}x{size[1]}")
    print(f"RMS reprojection error {rms:.4f} px  (worst view {max(errs):.4f})")
    if rms > 1.0:
        print("  Above 1 px is poor. Re-shoot with more varied angles and a flat, well-lit board.")
    print(f"fx {fx:.2f}   fy {fy:.2f}   cx {cx:.2f}   cy {cy:.2f}")
    print(f"aspect fy/fx {fy/fx:.4f}  (should be within about 1% of 1.0)")
    print(f"\nGSD at 50 m  {50/fx:.4f} m/px")
    print(f"GSD at 75 m  {75/fx:.4f} m/px")
    print(f"GSD at 100 m {100/fx:.4f} m/px")

    print("\nPaste into configs/system.yaml under data_layer.feed:\n")
    print("    intrinsics:")
    print(f"      fx_px: {fx:.3f}")
    print(f"      fy_px: {fy:.3f}")
    print(f"      cx_px: {cx:.3f}")
    print(f"      cy_px: {cy:.3f}")
    print(f"      dist: [{', '.join(f'{v:.6f}' for v in dist.ravel()[:5])}]")
    print(f"\nThese are valid ONLY at {size[0]}x{size[1]}. Change the capture "
          "resolution and fx scales with it -- recalibrate or scale by hand.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
