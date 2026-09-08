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
import os
import platform
import statistics
import sys
import time
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from geoanchor import device                       # noqa: E402
from geoanchor import methods                       # noqa: E402
from geoanchor.data_layer.store import FeatureStore  # noqa: E402


def conditions() -> dict:
    """Everything that makes two runs of this script comparable, or not.

    Added 8 Sept 2026 after a Pi 5 re-run came back with orb and akaze
    unchanged (1.02-1.11x) but every torch method 1.34-1.65x slower. That is
    not a code change -- xfeat never touches the matcher that changed -- it is
    the board being in a different state, and NOTHING in the output said so.
    The earlier run's conditions survived only because they happened to be
    typed into a commit message. A number without its clock is not a result.

    OpenCV methods drifting far less than torch ones is the signature to look
    for: short single-threaded work rides out a thermal or contention problem
    that sustained multi-threaded work does not.
    """
    import torch
    c = {"torch": torch.__version__, "torch_threads": torch.get_num_threads(),
         "machine": platform.machine()}
    try:
        b = device.detect()
        c["board"] = {"kind": b.kind, "model": b.model, "cores": b.cores}
    except Exception:
        pass
    # Pinned means min == max. An unpinned governor is the single most common
    # reason two runs of this script disagree.
    cpu = Path("/sys/devices/system/cpu/cpu0/cpufreq")
    for key, f in (("governor", "scaling_governor"), ("min_khz", "scaling_min_freq"),
                   ("max_khz", "scaling_max_freq"), ("cur_khz", "scaling_cur_freq")):
        try:
            c[key] = (cpu / f).read_text().strip()
        except OSError:
            pass
    if "min_khz" in c and "max_khz" in c:
        c["pinned"] = c["min_khz"] == c["max_khz"]
    try:
        c["loadavg"] = os.getloadavg()[0]
    except OSError:
        pass
    c["temp_c"] = device.read_temp_c()
    c["throttled"] = device.throttled()      # Pi: 0x0 is clean; None elsewhere
    c["power_w"] = device.read_power_w(device.detect())
    return c

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
    before = conditions()
    print(f"frame {frame.shape[1]}x{frame.shape[0]}  torch threads "
          f"{before.get('torch_threads')}  reps {a.reps}")
    print(f"board  {before.get('governor', '?')} governor, "
          f"{'PINNED' if before.get('pinned') else 'NOT PINNED'}, "
          f"{before.get('temp_c')} C, load {before.get('loadavg')}, "
          f"throttled {before.get('throttled')}")
    if not before.get("pinned"):
        print("  WARNING: clocks are not pinned. These numbers are not comparable to\n"
              "           another run. Jetson: sudo nvpmodel -m 0 && sudo jetson_clocks\n"
              "           Pi 5:    echo performance | sudo tee "
              "/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor")
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

    after = conditions()
    drift = ""
    if before.get("temp_c") and after.get("temp_c"):
        d = after["temp_c"] - before["temp_c"]
        drift = f"  (+{d:.1f} C over the run)" if d > 0 else f"  ({d:.1f} C over the run)"
    print(f"\nend    {after.get('temp_c')} C{drift}, throttled {after.get('throttled')}")

    if a.json:
        Path(a.json).write_text(json.dumps(
            {"frame": list(frame.shape[:2]), "reps": a.reps,
             # Both ends, because a board that was cool at the start and
             # throttling by the end produces a table where the last rows are
             # not comparable to the first ones either.
             "conditions_before": before, "conditions_after": after,
             "rows": rows}, indent=2))
        print(f"wrote {a.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
