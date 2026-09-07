#!/usr/bin/env python3
"""Matcher cost on THIS board, at a stated reference geometry.

The Pi 5 table in CLAUDE.md was measured against a one-tile reference. The
replay runs against 25 tiles. Match cost is linear in reference keypoints, so
those two numbers are not comparable and comparing them once produced a
"the Xavier is 10x slower than a Pi" result that was almost entirely geometry.

This prints detect and match separately, per method, per store, so any two
boards can be compared on the same row. Run it on each board and diff.

    python scripts/bench_matchers.py                  # every store found
    python scripts/bench_matchers.py --tiles 1        # only 1-tile stores
    python scripts/bench_matchers.py --reps 9 --json out.json
"""
import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from geoanchor import methods                       # noqa: E402
from geoanchor.data_layer.store import FeatureStore  # noqa: E402

# The frame the pipeline actually matches: one demo frame, rescaled to the
# reference GSD exactly as the data layer does it (DL-13).
FRAME_INDEX = 2
GSD_SCALE = 0.5048874407248918


def load_frame(path: Path):
    cap = cv2.VideoCapture(str(path))
    for _ in range(FRAME_INDEX):
        cap.read()
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit(f"could not read frame {FRAME_INDEX} from {path}")
    h, w = frame.shape[:2]
    return cv2.resize(frame, (int(w * GSD_SCALE), int(h * GSD_SCALE)))


def timings_ms(fn, reps):
    """Median and p95. The Pi table in CLAUDE.md quotes p95, so p95 is the
    column to compare against -- a median-to-p95 comparison flatters whichever
    board reported the median."""
    ts = []
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1000.0)
    ts.sort()
    p95 = ts[min(len(ts) - 1, int(round(0.95 * (len(ts) - 1))))]
    return statistics.median(ts), p95


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stores", default=str(REPO / "stores"))
    ap.add_argument("--video", default=str(REPO / "demo" / "flight.mp4"))
    ap.add_argument("--reps", type=int, default=7)
    ap.add_argument("--tiles", type=int, default=0, help="only stores with this tile count")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    frame = load_frame(Path(a.video))
    import torch  # after methods, so the thread count below is the real one
    print(f"frame {frame.shape[1]}x{frame.shape[0]}  torch threads {torch.get_num_threads()}  "
          f"reps {a.reps}")
    print(f"{'method':<11} {'tiles':>5} {'ref kp':>7} {'detect':>8} {'match':>8} "
          f"{'total':>8} {'p95':>8}")

    rows = []
    for d in sorted(Path(a.stores).iterdir()):
        if not (d / "manifest.json").exists():
            continue
        man = json.loads((d / "manifest.json").read_text())
        if a.tiles and man["n_tiles"] != a.tiles:
            continue
        name = man.get("method") or d.name.split("__")[1]
        try:
            m = methods.build(name, max_keypoints=4096)
            ok, why = m.available()
            if not ok:
                print(f"{name:<11} {d.name:<40} skipped: {why}")
                continue
            st = FeatureStore(d)
            st.load()
            f = m.detect(frame)                      # warm the weights
            rk, rd, rs = st.merged(st.all_tile_keys())
            # LighterGlue reads image_size, so it has to be the real reference
            # extent -- a (0, 0) placeholder fails its internal assertion.
            ref = methods.Features(kpts=rk, desc=rd, scores=rs,
                                   image_size=(man["width"], man["height"]))
            m.match(f, ref)                          # warm the match path
            det, det95 = timings_ms(lambda: m.detect(frame), a.reps)
            mat, mat95 = timings_ms(lambda: m.match(f, ref), a.reps)
        except Exception as exc:                     # a broken store is not fatal
            print(f"{name:<11} {d.name:<40} failed: {type(exc).__name__}: {exc}")
            continue
        print(f"{name:<11} {man['n_tiles']:>5} {man['n_keypoints']:>7} "
              f"{det:>8.1f} {mat:>8.1f} {det + mat:>8.1f} {det95 + mat95:>8.1f}")
        rows.append({"method": name, "store": d.name, "tiles": man["n_tiles"],
                     "ref_keypoints": man["n_keypoints"], "detect_ms": round(det, 1),
                     "match_ms": round(mat, 1), "total_ms": round(det + mat, 1),
                     "total_p95_ms": round(det95 + mat95, 1)})

    if a.json:
        Path(a.json).write_text(json.dumps(
            {"frame": list(frame.shape[:2]), "reps": a.reps, "rows": rows}, indent=2))
        print(f"\nwrote {a.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
