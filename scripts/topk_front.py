#!/usr/bin/env python3
"""Assemble the top_k sweeps into a Pareto front, and say what is dominated.

    python scripts/topk_front.py
    python scripts/topk_front.py --budget 250 --scene Scene_10

The sweeps already exist -- results/topk_sweep/ (xfeat_mnn, xfeat_lg) and
results/topk_sweep_ep2/ (edgepoint2_s64), each at k = 512, 1024, 2048, 4096,
with accuracy AND latency per k. Nothing read them, so the front they describe
had never been written down. This does that and nothing else: it runs no
matcher and measures nothing.

WHAT A POINT ON THE FRONT IS. One (method, top_k) pair, evaluated at THAT
method's own best gate rather than at a shared one. The gate is per method --
xfeat_mnn's p99 collapses from 167.85 m to 6.75 m across gates 7, 8, 9 while
edgepoint2_s64's worst error is 8.36 m even at gate 6 -- so scoring both at a
single gate compares gate choice rather than matcher. "Best" here means the
lowest gate whose p99 is inside `--p99-max`, which keeps the accept rate as
high as the tail allows instead of buying accuracy nobody asked for.

THE TWO AXES are p95 latency and accept rate. Accept rate, not median error:
median error barely moves across k (2.2 to 3.1 m on Scene_09) while accept rate
moves 5x, so a front drawn on median error would be flat and meaningless. The
error tail is handled by the gate constraint instead, which is where it belongs.

A point is DOMINATED if another point is at least as fast AND accepts at least
as often. Those are the ones to stop running.

Latency here is the Legion's, because that is where the sweeps were run. It is
the WRONG board for a deployment decision and the ordering is what transfers,
not the values -- CLAUDE.md has caught a cross-board comparison on mismatched
geometry twice. Re-run env80_sweep.py on the board that matters before reading
the budget verdicts as anything but a shape.
"""
from __future__ import annotations

import argparse
import glob
import json
import re
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def load(roots) -> dict:
    """{(scene, method, k): summary}"""
    out = {}
    for root in roots:
        for f in sorted(glob.glob(str(REPO / root / "k*" / "summary.json"))):
            m = re.search(r"/k(\d+)/", f)
            if not m:
                continue
            k = int(m.group(1))
            doc = json.load(open(f))
            for key, s in doc.get("summaries", {}).items():
                scene, _mode, method = key.split("__")
                out[(scene, method, k)] = s
    return out


def best_gate(s: dict, p99_max: float):
    """The lowest gate whose p99 is inside p99_max, and its stats.

    Lowest, not best-accuracy: above the knee the gate costs fixes and buys no
    accuracy, so the cheapest gate that controls the tail is the right one.
    """
    gates = s.get("gates") or {}
    best = None
    for g in sorted(gates, key=lambda x: int(x)):
        row = gates[g]
        if row.get("accepted", 0) == 0:
            continue
        p99 = row.get("p99_m")
        if p99 is not None and p99 <= p99_max:
            return int(g), row
        best = (int(g), row)
    return (None, None) if best is None else (None, best[1])


def front(points):
    """Pareto-optimal subset of [(latency, accept, label, ...)] -- minimise
    latency, maximise accept."""
    keep = []
    for p in points:
        dominated = any(q is not p and q[0] <= p[0] and q[1] >= p[1]
                        and (q[0] < p[0] or q[1] > p[1]) for q in points)
        if not dominated:
            keep.append(p)
    return sorted(keep, key=lambda r: r[0])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--roots", default="results/topk_sweep,results/topk_sweep_ep2")
    ap.add_argument("--scene", default=None, help="one scene, or all by default")
    ap.add_argument("--p99-max", type=float, default=10.0,
                    help="tail the gate must hold, metres")
    ap.add_argument("--budget", type=float, default=250.0,
                    help="latency budget in ms, for the verdict column")
    ap.add_argument("--out", default=None, help="also write this as JSON")
    a = ap.parse_args()

    data = load(a.roots.split(","))
    if not data:
        print("no sweeps found. Run scripts/topk_pareto.sh first.")
        return 1
    scenes = sorted({k[0] for k in data}) if a.scene is None else [a.scene]

    report = {}
    for scene in scenes:
        rows = []
        for (sc, method, k), s in sorted(data.items()):
            if sc != scene:
                continue
            gate, g = best_gate(s, a.p99_max)
            if g is None:
                continue
            rows.append((s.get("p95_latency_ms", float("nan")),
                         g.get("accept_rate", 0.0), method, k, gate, g,
                         s.get("ref_keypoints_total")))
        if not rows:
            continue
        on = {id(p) for p in front(rows)}

        print(f"\n\033[1m{scene}\033[0m   gate = lowest with p99 <= {a.p99_max:g} m, "
              f"budget {a.budget:g} ms")
        print(f"  {'method':16} {'k':>5} {'gate':>5} {'ref kp':>8} {'lat p95':>9} "
              f"{'accept':>7} {'med_m':>7} {'p90_m':>7} {'p99_m':>7} {'max_m':>7}  front  budget")
        for r in sorted(rows, key=lambda r: (r[2], r[3])):
            lat, acc, method, k, gate, g, refkp = r
            mark = "  *  " if id(r) in on else "     "
            fits = "fits" if lat <= a.budget else f"+{lat - a.budget:.0f}"
            gs = str(gate) if gate is not None else "none"
            print(f"  {method:16} {k:5d} {gs:>5} {str(refkp or '--'):>8} {lat:9.1f} "
                  f"{acc:7.1%} {g.get('median_m', 0):7.2f} {g.get('p90_m', 0):7.2f} "
                  f"{g.get('p99_m', 0):7.2f} {g.get('max_m', 0):7.2f} {mark} {fits:>7}")

        f = front(rows)
        print(f"\n  front, cheapest first:")
        for lat, acc, method, k, gate, g, _m in f:
            print(f"    {method} k={k} gate={gate}: {lat:.0f} ms p95, "
                  f"{acc:.1%} accept, p99 {g.get('p99_m', 0):.2f} m")
        inside = [p for p in f if p[0] <= a.budget]
        if inside:
            b = max(inside, key=lambda r: r[1])
            print(f"\n  Best inside {a.budget:g} ms: {b[2]} k={b[3]} gate={b[4]} "
                  f"-- {b[1]:.1%} accept at {b[0]:.0f} ms p95")
        else:
            c = min(f, key=lambda r: r[0])
            print(f"\n  NOTHING fits {a.budget:g} ms on this board. Cheapest is "
                  f"{c[2]} k={c[3]} at {c[0]:.0f} ms p95.")
        report[scene] = [
            {"method": m, "top_k": k, "gate": gate, "p95_latency_ms": lat,
             "accept_rate": acc, "on_front": id(r) in on, **{
                 kk: g.get(kk) for kk in ("median_m", "p90_m", "p99_m", "max_m")}}
            for r in rows for lat, acc, m, k, gate, g, _ in [r]]

    # What is missing, stated rather than left to be noticed.
    have = {(k[1], k[2]) for k in data}
    methods = sorted({m for m, _ in have})
    ks = sorted({k for _, k in have})
    missing = [(m, k) for m in methods for k in ks if (m, k) not in have]
    if missing:
        print("\n\033[1mNot measured\033[0m (the front is incomplete here):")
        for m, k in missing:
            print(f"  {m} at k={k}")

    print("\nLatency is the Legion's. The ORDERING transfers across boards; the "
          "values do not.\nRe-run env80_sweep.py on the target board before "
          "trusting the budget column.")
    # The sweep that produced these moved BOTH knobs together.
    print("\n\033[1mtop_k here is not top_k alone.\033[0m scripts/topk_pareto.sh passes "
          "--frame-keypoints K\nAND --ref-keypoints K, so every step of k also scaled the "
          "reference total (the\n`ref kp` column). These rows measure frame and reference "
          "keypoints TOGETHER.\nSeparating them needs a sweep that holds one fixed.")
    print("\n`ref kp` is the TOTAL reference keypoints one match() call saw "
          "(tiles x per-tile).\nIt moves xfeat_mnn and xfeat_lg in OPPOSITE "
          "directions by 2-3x, so two rows with\ndifferent ref kp are not the "
          "same experiment -- compare within a scene, or match it.")

    if a.out:
        Path(a.out).write_text(json.dumps(
            {"p99_max_m": a.p99_max, "budget_ms": a.budget,
             "board": "legion (x86_64) -- ordering transfers, values do not",
             "missing": [{"method": m, "top_k": k} for m, k in missing],
             "scenes": report}, indent=2) + "\n")
        print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
