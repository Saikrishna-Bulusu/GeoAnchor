"""Yaw rectification. Stage 02, and it is load-bearing.

At N=200 on the Pi 5 on 2 Sept 2026, XFeat + LighterGlue produced a fix on
only 95% of frames of a synthetic task where the query was literally cut out of
the reference tile, with an RMSE of 123 m. The fixture applies a random
in-plane rotation of +-25 degrees and XFeat is not rotation invariant, so the
larger sample simply reached the tail of that distribution. At N=30 it had
looked perfect.

The conclusion is structural rather than numerical: without rectification the
matcher fails on a task that is otherwise trivial. The reference tile is
north-up, the vehicle is not, and attitude is already on the aircraft. So the
frame is rotated to north-up before matching and the reference is never warped,
which is also what keeps the precomputed feature store valid.
"""
from __future__ import annotations

import math

import cv2
import numpy as np


class Rectifier:
    def __init__(self, enabled: bool = True, expand: bool = True,
                 max_tilt_deg: float = 25.0):
        self.enabled = enabled
        self.expand = expand
        self.max_tilt_deg = max_tilt_deg
        self.last_note = ""

    def run(self, frame_bgr: np.ndarray, yaw_deg=None, roll_deg=None, pitch_deg=None) -> tuple:
        """Return (rectified, info). The frame centre stays the frame centre,
        so the solved position needs no correction for the rotation."""
        h, w = frame_bgr.shape[:2]
        info = {"applied": False, "yaw_deg": yaw_deg, "centre": (w / 2.0, h / 2.0),
                "tilt_deg": None, "note": ""}
        self.last_note = ""

        tilt = None
        if roll_deg is not None and pitch_deg is not None:
            tilt = math.degrees(math.acos(max(-1.0, min(1.0,
                math.cos(math.radians(roll_deg)) * math.cos(math.radians(pitch_deg))))))
            info["tilt_deg"] = round(tilt, 2)
            if tilt > self.max_tilt_deg:
                # A homography still fits, but the flat-scene assumption behind
                # it is being stretched, and the review's own envelope work
                # excludes frames beyond about this angle.
                info["note"] = (f"tilt {tilt:.1f} deg exceeds {self.max_tilt_deg} deg; "
                                "the planar assumption is weak here")
                self.last_note = info["note"]

        if not self.enabled or yaw_deg is None:
            if yaw_deg is None and self.enabled:
                info["note"] = "no attitude available, matching an unrotated frame"
                self.last_note = info["note"]
            return frame_bgr, info

        # Rotate by -yaw: a vehicle heading 90 deg sees the world rotated -90.
        angle = -float(yaw_deg)
        cx, cy = w / 2.0, h / 2.0
        M = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
        if self.expand:
            cos, sin = abs(M[0, 0]), abs(M[0, 1])
            nw = int(h * sin + w * cos)
            nh = int(h * cos + w * sin)
            M[0, 2] += nw / 2.0 - cx
            M[1, 2] += nh / 2.0 - cy
        else:
            nw, nh = w, h
        out = cv2.warpAffine(frame_bgr, M, (nw, nh), flags=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
        info.update(applied=True, centre=(nw / 2.0, nh / 2.0),
                    size=(nw, nh), angle_deg=angle)
        return out, info
