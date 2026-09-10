#!/usr/bin/env python3
"""Match frames cut from one reference tile against a DIFFERENT reference tile.

    python scripts/crossdate_probe.py \
        --query data/sydney/ref_tile.tif \
        --reference data/sydney/ref_tile_1943.tif

    python scripts/crossdate_probe.py --query data/sydney/ref_tile.tif   # control

WHAT IT ANSWERS
---------------
"Does this matcher survive the two captures being years apart?"

The demo feed cuts its frames out of the reference tile, so it matches a picture
against a copy of itself and reports 0.007 m. That number proves the wiring and
nothing else. This script cuts frames from one tile and matches them against an
independent capture of the same ground, which is the honest version of the same
test.

Run it with no --reference to get the CONTROL: same tile both sides. The control
is not optional. Without it a zero-fix result is indistinguishable from a broken
harness, and that ambiguity is exactly what this script exists to remove.

Scale is handled the way the pipeline handles it: the query is resampled from
its own GSD to the reference's, which is what `Preprocessor` does with
`GSD = altitude / fx_px`.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))


def probe(query_path, ref_path, method, n, frame_px, max_kp, seed, gate):
    import rasterio
    from geoanchor import methods as M

    q_ds = rasterio.open(query_path)
    r_ds = rasterio.open(ref_path)
    scale = q_ds.res[0] / r_ds.res[0]

    ref = cv2.cvtColor(np.dstack([r_ds.read(b) for b in (1, 2, 3)]),
                       cv2.COLOR_RGB2BGR)
    m = M.build(method, max_keypoints=max_kp)
    f_ref = m.detect(ref)

    rng = np.random.default_rng(seed)
    inliers, errors, n_solved = [], [], 0

    for _ in range(n):
        cx = int(rng.integers(frame_px, q_ds.width - frame_px))
        cy = int(rng.integers(frame_px, q_ds.height - frame_px))
        win = rasterio.windows.Window(cx - frame_px // 2, cy - frame_px // 2,
                                      frame_px, frame_px)
        q = cv2.cvtColor(np.dstack([q_ds.read(b, window=win) for b in (1, 2, 3)]),
                         cv2.COLOR_RGB2BGR)
        if abs(scale - 1.0) > 1e-6:
            q = cv2.resize(q, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)

        f_q = m.detect(q)
        ia, ib, _ = m.match(f_q, f_ref)
        if len(ia) < 12:
            inliers.append(0)
            continue

        H, mask = cv2.findHomography(f_q.kpts[ia], f_ref.kpts[ib],
                                     cv2.USAC_MAGSAC, 3.0)
        k = int(mask.sum()) if mask is not None else 0
        inliers.append(k)
        if H is None or k < gate:
            continue
        n_solved += 1
        c = np.array([[[q.shape[1] / 2, q.shape[0] / 2]]], np.float32)
        p = cv2.perspectiveTransform(c, H)[0][0]
        px, py = r_ds.xy(p[1], p[0])
        tx, ty = q_ds.xy(cy, cx)
        errors.append(float(np.hypot(px - tx, py - ty)))

    # Percentiles only. A degenerate homography can put a fix 1e90 m away and
    # one such row destroys a mean -- this project's first hard rule.
    plausible = sorted(e for e in errors if e < 500.0)
    return {
        "method": method,
        "label": M.label(method),
        "query": str(query_path),
        "reference": str(ref_path),
        "query_gsd_m": round(q_ds.res[0], 4),
        "reference_gsd_m": round(r_ds.res[0], 4),
        "frames": n,
        "inliers_median": int(np.median(inliers)),
        "inliers_max": int(np.max(inliers)),
        "solved": n_solved,
        "plausible": len(plausible),
        "median_error_m": round(plausible[len(plausible) // 2], 3) if plausible else None,
        "p90_error_m": round(plausible[int(len(plausible) * 0.9)], 3) if plausible else None,
        "max_error_m": round(plausible[-1], 3) if plausible else None,
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--query", required=True, help="tile the frames are cut from")
    ap.add_argument("--reference", default=None,
                    help="tile to match against. Omit for the same-tile control.")
    ap.add_argument("--methods", default="edgepoint2_s64,xfeat_mnn,sift")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--frame-px", type=int, default=512)
    ap.add_argument("--max-keypoints", type=int, default=2048)
    ap.add_argument("--gate", type=int, default=8, help="inlier floor")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--json", default=None, help="write results here")
    a = ap.parse_args()

    def resolve(p):
        p = Path(p)
        return p if p.is_absolute() else REPO / p

    query = resolve(a.query)
    ref = resolve(a.reference) if a.reference else query
    control = ref == query

    print(f"query      {query.name}")
    print(f"reference  {ref.name}{'   (CONTROL: same tile both sides)' if control else ''}")
    print(f"{a.frames} frames of {a.frame_px} px, inlier gate {a.gate}, seed {a.seed}")
    print()
    print(f"{'matcher':30s} {'inl med':>8s} {'solved':>7s} {'plaus':>6s} "
          f"{'median':>10s} {'p90':>10s} {'max':>10s}")

    rows = []
    for name in a.methods.split(","):
        r = probe(query, ref, name.strip(), a.frames, a.frame_px,
                  a.max_keypoints, a.seed, a.gate)
        rows.append(r)
        fmt = lambda v: f"{v:10.3f}" if v is not None else f"{'--':>10s}"
        print(f"{r['label']:30s} {r['inliers_median']:8d} "
              f"{r['solved']:7d} {r['plausible']:6d} "
              f"{fmt(r['median_error_m'])} {fmt(r['p90_error_m'])} "
              f"{fmt(r['max_error_m'])}")

    if not control and all(r["plausible"] == 0 for r in rows):
        print()
        print("Every matcher returned nothing. Before concluding the reference is")
        print("too different, run the control -- drop --reference and re-run. If")
        print("the control also returns nothing, the harness is what is broken.")

    if a.json:
        out = resolve(a.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(rows, indent=2) + "\n")
        print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
