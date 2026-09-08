#!/usr/bin/env python3
"""Write a chessboard PNG to display FULL-SCREEN on any monitor, phone or laptop.

    python scripts/make_chessboard.py                 # 10x7 squares -> docs/chessboard.png
    python scripts/make_chessboard.py --squares 10 7 --px 1600 1100

A screen is a better calibration target than a printed page: it is genuinely
flat (a taped print is not), its geometry is exact, and it needs no printer.

You do NOT need to measure the squares. `cv2.calibrateCamera` returns fx in
PIXELS, and that is invariant to the assumed physical square size -- scaling
every object point by k scales the solved translation by k and leaves fx
untouched. Verified numerically: assuming 0.025, 1.0 and 137.0 for the same
synthetic views returns fx to the last decimal place. So scripts/
calibrate_camera.py assumes 1.0 and never asks how big the squares are.

Displaying it:
  * Full-screen with NO scaling or zoom (a browser at 100%, or an image viewer
    set to fit, is fine -- what matters is that the pattern is not resampled
    unevenly).
  * Turn the screensaver off and the brightness up.
  * Keep the camera far enough that squares are comfortably more than ~20 px
    across in the captured frame, or the screen's own pixel grid will beat
    against the camera's and produce moire that moves the detected corners.
"""
from __future__ import annotations

import argparse
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--squares", type=int, nargs=2, default=[10, 7],
                    help="squares across and down (inner corners are one fewer each)")
    ap.add_argument("--px", type=int, nargs=2, default=[1600, 1120], help="image size")
    ap.add_argument("--out", default=str(REPO / "docs" / "chessboard.png"))
    a = ap.parse_args()

    nx, ny = a.squares
    W, H = a.px
    # A quiet border matters: findChessboardCorners needs the outer ring of
    # squares to be surrounded by background, and a pattern bled to the screen
    # edge fails to detect at exactly the angles you most want.
    margin = int(min(W, H) * 0.06)
    side = min((W - 2 * margin) // nx, (H - 2 * margin) // ny)
    bw, bh = side * nx, side * ny
    x0, y0 = (W - bw) // 2, (H - bh) // 2

    img = np.full((H, W), 255, np.uint8)
    for r in range(ny):
        for c in range(nx):
            if (r + c) % 2:
                img[y0 + r * side:y0 + (r + 1) * side,
                    x0 + c * side:x0 + (c + 1) * side] = 0

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), img)
    print(f"wrote {out}  ({W}x{H}, {nx}x{ny} squares of {side} px, "
          f"{nx - 1}x{ny - 1} inner corners)")
    print("\nDisplay it full-screen, then run:")
    print(f"  python scripts/calibrate_camera.py --pattern {nx - 1} {ny - 1}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
