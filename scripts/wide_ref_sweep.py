#!/usr/bin/env python3
"""Find THIS board's WIDE_REF crossover.

    python scripts/wide_ref_sweep.py                    # default sweep
    python scripts/wide_ref_sweep.py --json out.json
    python scripts/wide_ref_sweep.py --dim 32 --query 4096

WHAT IS BEING DECIDED
---------------------
`EdgePoint2Method.match` needs mutual-nearest-neighbours, which means the argmax
of the (Q, N) cosine matrix along BOTH axes. There are two ways to get the
second one and they have different costs:

  one-matmul   cossim = da @ db.T ; cossim.max(dim=0)
               One GEMM. But max(dim=0) walks a row-major tensor across its
               stride, so it is cache-hostile and falls off a cliff once the
               matrix stops fitting.

  two-matmul   cossim = da @ db.T ; (db @ da.T).max(dim=1)
               Pays for a second GEMM to get a reduction along the contiguous
               axis. This is what XFeat's own matcher does, unconditionally
               (xfeat/modules/xfeat.py:328).

Neither wins everywhere. `WIDE_REF` is the size above which the strided
reduction costs more than a whole extra GEMM -- and it is a property of the
board's cache hierarchy, not of the algorithm, so it does not transfer.

WHY THIS MATTERS RIGHT NOW
--------------------------
Measured 8 Sept 2026 at 1 tile, 2048 reference keypoints, identical geometry:

    board     edgepoint2 match   xfeat_mnn match
    Xavier                20.7              28.6    one-matmul WINS
    Pi 5                  89.1              30.6    one-matmul LOSES, 3x

Both methods emit exactly 2048 frame keypoints with 64-D descriptors on this
frame (checked), so the problem size is identical and the ONLY difference is
which of the two forms above they take. That makes the Pi's edgepoint2 result
-- the one reversal in a table it otherwise wins -- most likely a constant
fitted to the wrong board rather than a fact about its cores.

Synthetic descriptors on purpose: L2-normalised random vectors have the same
shape, dtype and memory layout as real ones, the arithmetic does not care what
the values are, and it means any board can run the full sweep without owning
every store size.
"""
from __future__ import annotations

import argparse
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import torch

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

SIZES = [1024, 2048, 4096, 8192, 12000, 16000, 20000, 25600, 32768, 51200]


def timings_ms(fn, reps: int) -> tuple:
    ts = []
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1000.0)
    ts.sort()
    return statistics.median(ts), ts[min(len(ts) - 1, int(round(0.95 * (len(ts) - 1))))]


def one_matmul(da, db):
    with torch.inference_mode():
        cossim = da @ db.T
        best, m12 = cossim.max(dim=1)
        _, m21 = cossim.max(dim=0)
        return m12, m21, best


def two_matmul(da, db):
    with torch.inference_mode():
        cossim = da @ db.T
        best, m12 = cossim.max(dim=1)
        _, m21 = (db @ da.T).max(dim=1)
        return m12, m21, best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--query", type=int, default=2048, help="frame keypoints (Q)")
    ap.add_argument("--dim", type=int, default=64, help="descriptor width")
    ap.add_argument("--reps", type=int, default=7)
    ap.add_argument("--sizes", type=int, nargs="*", default=SIZES)
    ap.add_argument("--json", type=str, default="")
    a = ap.parse_args()

    torch.manual_seed(0)
    da = torch.nn.functional.normalize(torch.randn(a.query, a.dim), dim=1)

    print(f"board   {platform.machine()}  {platform.processor() or ''}".rstrip())
    print(f"torch   {torch.__version__}  threads={torch.get_num_threads()}")
    print(f"query   Q={a.query}  dim={a.dim}  reps={a.reps}\n")
    print(f"{'N (ref kp)':>11s} {'one-matmul':>11s} {'two-matmul':>11s} {'winner':>11s} {'ratio':>7s}")

    rows = []
    crossover = None
    for n in a.sizes:
        db = torch.nn.functional.normalize(torch.randn(n, a.dim), dim=1)
        one_matmul(da, db); two_matmul(da, db)          # warm
        o_med, o_p95 = timings_ms(lambda: one_matmul(da, db), a.reps)
        t_med, t_p95 = timings_ms(lambda: two_matmul(da, db), a.reps)
        win = "one" if o_med <= t_med else "two"
        if crossover is None and win == "two":
            crossover = n
        rows.append({"n": n, "one_ms": round(o_med, 2), "one_p95_ms": round(o_p95, 2),
                     "two_ms": round(t_med, 2), "two_p95_ms": round(t_p95, 2), "winner": win})
        print(f"{n:11d} {o_med:11.2f} {t_med:11.2f} {win:>11s} {o_med / t_med:7.2f}")
        del db

    # Equality is checked once, at a size both forms handle, because a faster
    # matcher that returns different matches is not a faster matcher.
    db = torch.nn.functional.normalize(torch.randn(4096, a.dim), dim=1)
    o, t = one_matmul(da, db), two_matmul(da, db)
    same = bool(torch.equal(o[0], t[0]) and torch.equal(o[1], t[1]))
    print(f"\nboth forms agree at N=4096: {same}")

    print(f"\nWIDE_REF for this board: {crossover if crossover else 'above the swept range'}")
    print("Set geoanchor/methods.py::EdgePoint2Method.WIDE_REF to that value.")
    print("Current value in this checkout:", _current_wide_ref())

    if a.json:
        out = {"machine": platform.machine(), "torch": torch.__version__,
               "threads": torch.get_num_threads(), "query": a.query, "dim": a.dim,
               "reps": a.reps, "crossover": crossover, "forms_agree": same, "rows": rows}
        Path(a.json).write_text(json.dumps(out, indent=2))
        print("wrote", a.json)
    return 0


def _current_wide_ref():
    try:
        from geoanchor.methods import EdgePoint2Method
        return EdgePoint2Method.WIDE_REF
    except Exception as exc:
        return f"(could not read: {exc})"


if __name__ == "__main__":
    raise SystemExit(main())
