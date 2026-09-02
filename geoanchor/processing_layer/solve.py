"""Correspondences to a position on the ground.

Both images are north-up and at the same scale by the time this runs -- the
data layer rescaled the frame to the reference GSD and stage 02 rotated it to
north -- so the residual transform is close to a translation. A homography is
still the right model because the ground is not perfectly flat and the frame is
not perfectly nadir, but knowing it should be near-similarity gives a cheap and
very effective plausibility test.

That test is not optional. The benchmark harness reports fixes wrong by up to
2.06e93 m because its only success check is `inliers > 0`, and a degenerate
homography satisfies that happily. A single such fix destroys a mean, which is
why the project reports medians. Rejecting it here is better than reporting
around it.
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from ..geo import pixel_to_wgs84, wgs84_to_pixel


def manifest_crs(manifest: dict):
    """A GeoTIFF store carries an EPSG code; an AnyVisLoc store carries a PROJ
    string because its ground truth has no geodetic frame at all."""
    return manifest.get("crs") or manifest["epsg"]


class SolveResult:
    def __init__(self):
        self.ok = False
        self.reject_code = None
        self.reject_reason = ""
        self.lat = self.lon = None
        self.map_px = None
        self.inliers = 0
        self.matches = 0
        self.keypoints = 0
        self.inlier_ratio = 0.0
        self.reproj_err_px = None
        self.H = None
        self.scale = None
        self.shear = None
        self.tiles_searched = 0
        self.stage_ms: dict = {}


def select_tiles(store, manifest, prior_lat=None, prior_lon=None, radius_m=150.0,
                 footprint_px=0.0, cold_start_max_tiles=0) -> list:
    """Search-space reduction. The single cheapest speed-up available.

    A previous fix bounds where the vehicle can now be, so only the tiles that
    could contain the current footprint are candidates. With no prior there is
    nothing to bound it with and the whole map is searched, which is slow and
    is reported honestly rather than hidden.
    """
    if prior_lat is None or prior_lon is None:
        keys = store.all_tile_keys()
        if cold_start_max_tiles and len(keys) > cold_start_max_tiles:
            return keys[:cold_start_max_tiles]
        return keys
    col, row = wgs84_to_pixel(manifest["transform"], manifest_crs(manifest), prior_lat, prior_lon)
    radius_px = radius_m / manifest["gsd_m_px"] + footprint_px / 2.0
    keys = store.tiles_covering(col, row, radius_px)
    return keys or store.all_tile_keys()


def _tile_of(manifest: dict, tile_keys, pts: np.ndarray) -> dict:
    """Group reference points by the tile they fall in.

    Tiles overlap, so a point can belong to two. It is assigned to the one
    whose centre it is nearest, which keeps each group compact.
    """
    metas = [t for t in manifest["tiles"] if (t["row"], t["col"]) in set(tile_keys)]
    if not metas:
        return {}
    cx = np.array([t["x0"] + t["width"] / 2.0 for t in metas])
    cy = np.array([t["y0"] + t["height"] / 2.0 for t in metas])
    d = (pts[:, 0:1] - cx[None, :]) ** 2 + (pts[:, 1:2] - cy[None, :]) ** 2
    owner = d.argmin(1)
    groups = {}
    for i, t in enumerate(metas):
        idx = np.nonzero(owner == i)[0]
        if len(idx):
            groups[(t["row"], t["col"])] = idx
    return groups


def solve(method, frame_bgr, store, manifest, tile_keys, *, min_keypoints=30,
          min_matches=12, inlier_gate=25, ransac_reproj_px=3.0,
          frame_centre=None, max_scale_dev=0.35, max_shear=0.25,
          tile_ransac=True) -> SolveResult:
    r = SolveResult()
    r.tiles_searched = len(tile_keys)

    t0 = time.perf_counter()
    f_frame = method.detect(frame_bgr)
    r.stage_ms["detect_frame"] = (time.perf_counter() - t0) * 1000.0
    r.keypoints = len(f_frame)
    if r.keypoints < min_keypoints:
        r.reject_code, r.reject_reason = "PLE-04", f"{r.keypoints} keypoints < {min_keypoints}"
        return r

    t0 = time.perf_counter()
    ref_kp, ref_desc, ref_scores = store.merged(tile_keys)
    r.stage_ms["load_reference"] = (time.perf_counter() - t0) * 1000.0
    if len(ref_kp) < min_keypoints:
        r.reject_code, r.reject_reason = "PLE-05", "reference tiles carry too few keypoints"
        return r

    from ..methods import Features
    f_ref = Features(kpts=ref_kp, desc=ref_desc, scores=ref_scores,
                     image_size=(manifest["width"], manifest["height"]))

    t0 = time.perf_counter()
    ia, ib, conf = method.match(f_frame, f_ref)
    r.stage_ms["match"] = (time.perf_counter() - t0) * 1000.0
    r.matches = len(ia)
    if r.matches < min_matches:
        r.reject_code, r.reject_reason = "PLE-05", f"{r.matches} matches < {min_matches}"
        return r

    src = f_frame.kpts[ia].astype(np.float32)     # frame pixels
    dst = ref_kp[ib].astype(np.float32)           # map pixels

    t0 = time.perf_counter()
    # Fit per candidate tile rather than over the whole map at once.
    #
    # Matching a frame against every reference keypoint on the map is cheap to
    # compute but badly conditioned to fit: the correct correspondences are a
    # handful among thousands of plausible-looking wrong ones spread over
    # hundreds of metres, and RANSAC happily returns a degenerate homography
    # that satisfies `inliers > 0`. Measured on AnyVisLoc Scene_09 against the
    # satellite basemap, a whole-map fit produced 7-23 inliers and rejected 90%
    # of frames on the plausibility test, while the same matches restricted to
    # the correct tile gave 12-79. The matching cost is unchanged -- only the
    # RANSAC input is partitioned -- so this is close to free.
    H = mask = None
    if tile_ransac and len(tile_keys) > 1:
        best = (0, None, None)
        for key, idx in _tile_of(manifest, tile_keys, dst).items():
            if len(idx) < min_matches:
                continue
            Hk, mk = cv2.findHomography(src[idx], dst[idx], cv2.USAC_MAGSAC,
                                        ransac_reproj_px, maxIters=5000, confidence=0.999)
            if Hk is None or mk is None:
                continue
            n = int(mk.sum())
            if n > best[0] and _plausible(Hk, manifest, max_scale_dev, max_shear)[0]:
                # Re-index the tile-local mask back onto the full match list so
                # the reported inlier count and reprojection error stay
                # comparable with a whole-map fit.
                full = np.zeros(len(src), dtype=np.uint8)
                full[idx[mk.ravel().astype(bool)]] = 1
                best = (n, Hk, full.reshape(-1, 1))
        H, mask = best[1], best[2]
        r.stage_ms["tiles_fitted"] = float(len(tile_keys))

    if H is None:
        H, mask = cv2.findHomography(src, dst, cv2.USAC_MAGSAC, ransac_reproj_px,
                                     maxIters=5000, confidence=0.999)
    r.stage_ms["ransac"] = (time.perf_counter() - t0) * 1000.0
    if H is None or mask is None:
        r.reject_code, r.reject_reason = "PLE-06", "no homography fitted"
        return r

    inl = mask.ravel().astype(bool)
    r.inliers = int(inl.sum())
    _det = float(np.linalg.det(H[:2, :2]))
    r.scale = float(np.sqrt(abs(_det))) if np.isfinite(_det) else None
    r.inlier_ratio = r.inliers / max(1, r.matches)
    r.H = H

    proj = cv2.perspectiveTransform(src[inl].reshape(-1, 1, 2), H).reshape(-1, 2)
    r.reproj_err_px = float(np.sqrt(((proj - dst[inl]) ** 2).sum(1)).mean()) if r.inliers else None

    ok, why = _plausible(H, manifest, max_scale_dev, max_shear)
    if not ok:
        r.reject_code, r.reject_reason = "PLE-08", why
        return r

    cx, cy = frame_centre if frame_centre else (frame_bgr.shape[1] / 2.0, frame_bgr.shape[0] / 2.0)
    centre = cv2.perspectiveTransform(np.array([[[cx, cy]]], np.float32), H).reshape(2)
    mx, my = float(centre[0]), float(centre[1])
    margin = 0.25 * max(manifest["width"], manifest["height"])
    if not (-margin <= mx <= manifest["width"] + margin and
            -margin <= my <= manifest["height"] + margin):
        r.reject_code = "PLE-08"
        r.reject_reason = f"solved centre ({mx:.0f}, {my:.0f}) px lies outside the map"
        return r

    r.map_px = (mx, my)
    r.lat, r.lon = pixel_to_wgs84(manifest["transform"], manifest_crs(manifest), mx, my)

    # The gate runs LAST so that a rejected fix still carries its inlier count,
    # reprojection error and position. Those rows are the training data for the
    # covariance estimator; discarding them early would throw away every
    # example of what a bad fix looks like.
    if r.inliers < inlier_gate:
        r.reject_code = "PLE-07"
        r.reject_reason = f"{r.inliers} inliers < gate {inlier_gate}"
        return r

    r.ok = True
    return r


def _plausible(H: np.ndarray, manifest: dict, max_scale_dev: float, max_shear: float) -> tuple:
    """Both images are north-up and at the same GSD, so the linear part of a
    correct homography is close to the identity. Anything else is a degenerate
    solve that RANSAC was happy with."""
    if not np.all(np.isfinite(H)):
        return False, "homography contains non-finite values"
    A = H[:2, :2]
    det = float(np.linalg.det(A))
    if abs(det) < 1e-9:
        return False, "degenerate homography, determinant is zero"
    scale = float(np.sqrt(abs(det)))
    if abs(scale - 1.0) > max_scale_dev:
        return False, (f"scale {scale:.3f} is more than {max_scale_dev:.2f} from 1.0 "
                       "after the frame was already rescaled to the reference GSD")
    # Shear and residual rotation, as the off-diagonal energy of the normalised A.
    An = A / scale
    shear = float(abs(An[0, 1] + An[1, 0]) / 2.0)
    if shear > max_shear:
        return False, f"shear {shear:.3f} exceeds {max_shear:.2f}"
    persp = float(np.hypot(H[2, 0], H[2, 1]))
    if persp > 1e-3:
        return False, f"perspective term {persp:.2e} is too large for a near-nadir frame"
    return True, ""
