#!/usr/bin/env python3
"""Run every env80 frame through the pipeline and write a comparable table.

    python scripts/env80_sweep.py

Same modules as the running system -- the same feed, preprocessor, rectifier,
solver and covariance backend -- with the bus taken out. That is deliberate.
The live pipeline drops stale frames on purpose so it can hold a latency
budget, which is right in flight and wrong for a results table: a sweep has to
see every frame exactly once, in a fixed order, with no pacing.

Two choices that decide whether the numbers mean anything:

* **The gate is not applied during the run.** Every frame that produces a
  plausible solve is recorded with its inlier count and its error, and the
  accept rate is computed afterwards at a range of gates. One run therefore
  answers "what gate should this reference use", instead of assuming one. It
  also keeps the rejected fixes, which are the training data for the covariance
  estimator -- discarding them early throws away every example of a bad fix.
* **Cold start by default.** Each frame is solved without a prior, which is how
  the benchmark harness evaluates and is the only way these numbers sit next to
  the ones already in `results/`. `--prior sequential` measures the easier,
  more realistic case instead, and says so in the output.

No mean and no RMSE. A single degenerate solve destroys both.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from geoanchor import config as cfgmod                          # noqa: E402
from geoanchor import methods as M                              # noqa: E402
from geoanchor.data_layer import anyvisloc as A                 # noqa: E402
from geoanchor.data_layer.feed import Preprocessor              # noqa: E402
from geoanchor.data_layer.mapprep import build_store_from_image  # noqa: E402
from geoanchor.geo import distance_m                            # noqa: E402
from geoanchor.processing_layer import covariance as cov        # noqa: E402
from geoanchor.processing_layer.rectify import Rectifier        # noqa: E402
from geoanchor.processing_layer.solve import select_tiles, solve  # noqa: E402

# The interesting region is narrow and the old grid jumped straight over it:
# on env80, xfeat_mnn's p99 falls 167.85 -> 18.46 -> 6.75 m across gates
# 7, 8, 9, so 5 -> 10 -> 15 hid where the collapse actually happens.
GATES = [0, 5, 6, 8, 10, 12, 14, 15, 20, 25, 30, 40, 60]
BANDS = [5.0, 10.0, 20.0]
FIELDS = ["scene", "mode", "method", "sample_id", "frame_index", "altitude_m",
          "view_angle_deg", "yaw_deg", "keypoints", "matches", "inliers",
          "inlier_ratio", "reproj_err_px", "scale", "tiles_searched",
          "plausible", "reject_code", "error_m", "sigma_m",
          "latency_ms", "detect_ms", "match_ms", "ransac_ms"]


def pct(v, q):
    if not v:
        return None
    s = sorted(v)
    if len(s) == 1:
        return s[0]
    pos = (len(s) - 1) * q
    lo, hi = math.floor(pos), math.ceil(pos)
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (pos - lo)


def run_one(scene_dir, frames_dir, mode, method_name, cfg, out_csv, args) -> list:
    georef = A.load_georeference(scene_dir, mode, args.anchor)
    store, manifest, hit = build_store_from_image(
        georef["map_path"], georef, cfg.resolve("data_layer.map.store_dir", "stores"),
        method_name, tile_px=args.tile_px, overlap_px=args.overlap_px,
        max_keypoints=args.ref_keypoints, force=args.rebuild)
    print(f"    store {'cached' if hit else 'built '}  {manifest['n_tiles']} tiles, "
          f"{manifest['n_keypoints']} keypoints, {manifest['gsd_m_px']:.4f} m/px")

    frames = A.list_frames(frames_dir)
    frames = A.envelope_filter(frames, georef, args.agl_min, args.agl_max, args.view_angle_min)
    if args.limit:
        frames = frames[: args.limit]
    if not frames:
        print("    no frames inside the envelope")
        return []

    method = M.build(method_name, max_keypoints=args.frame_keypoints)
    ok, why = method.available()
    if not ok:
        print(f"    SKIP {method_name}: {why}")
        return []
    pre = Preprocessor(fallback_long_edge=args.frame_px)
    rect = Rectifier(enabled=not args.no_rectify)
    est = cov.build(cfg.section("processing_layer").get("covariance", {}))

    # Warm the extractor before timing. The first call loads weights and
    # allocates, which on the Pi 5 was hundreds of milliseconds -- charging it
    # to frame 0 poisons the p95 of a short run and looks like a tail.
    method.detect(np.zeros((64, 64, 3), np.uint8))

    rows, prior = [], None
    t0 = time.perf_counter()
    for i, fp in enumerate(frames):
        img, meta = A.read_frame(fp, georef)
        t_start = time.perf_counter()
        scaled, _, pm = pre.run(img, altitude_m=meta["alt_agl_m"],
                                map_gsd_m_px=manifest["gsd_m_px"],
                                fx_px=meta["fx_px"], pitch_deg=meta["pitch_deg"])
        r_img, rinfo = rect.run(scaled, meta["yaw_deg"], meta["roll_deg"], meta["pitch_deg"])
        keys = select_tiles(store, manifest,
                            prior[0] if prior else None, prior[1] if prior else None,
                            radius_m=args.prior_radius, footprint_px=max(r_img.shape[:2]))
        # gate 0: record everything plausible, decide the gate afterwards
        res = solve(method, r_img, store, manifest, keys, inlier_gate=0,
                    min_matches=args.min_matches, ransac_reproj_px=args.ransac_px,
                    frame_centre=rinfo["centre"], tile_ransac=not args.no_tile_ransac)
        latency = (time.perf_counter() - t_start) * 1000.0

        err = None
        if res.lat is not None:
            err = distance_m(meta["lat"], meta["lon"], res.lat, res.lon)
            if args.prior == "sequential" and res.ok:
                prior = (res.lat, res.lon)
        rows.append({
            "scene": scene_dir.name, "mode": mode, "method": method_name,
            "sample_id": meta["sample_id"], "frame_index": i,
            "altitude_m": round(meta["alt_agl_m"], 2),
            "view_angle_deg": round(meta["view_angle_deg"], 2),
            "yaw_deg": round(meta["yaw_deg"], 2),
            "keypoints": res.keypoints, "matches": res.matches, "inliers": res.inliers,
            "inlier_ratio": round(res.inlier_ratio, 4),
            "reproj_err_px": round(res.reproj_err_px, 3) if res.reproj_err_px else None,
            "scale": round(res.scale, 4) if res.scale else None,
            "tiles_searched": res.tiles_searched,
            "plausible": res.lat is not None,
            "reject_code": res.reject_code or "",
            "error_m": round(err, 4) if err is not None else None,
            "sigma_m": round(est.sigma_m(cov.features_from(res, meta["alt_agl_m"])), 3)
            if res.lat is not None else None,
            "latency_ms": round(latency, 1),
            "detect_ms": round(res.stage_ms.get("detect_frame", 0), 1),
            "match_ms": round(res.stage_ms.get("match", 0), 1),
            "ransac_ms": round(res.stage_ms.get("ransac", 0), 1),
        })
        if (i + 1) % 25 == 0 or i + 1 == len(frames):
            done = time.perf_counter() - t0
            print(f"    {i+1:4d}/{len(frames)}  {done:6.1f}s elapsed, "
                  f"{done/(i+1)*(len(frames)-i-1):6.1f}s left", flush=True)

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    return rows


def summarise(rows: list) -> dict:
    n = len(rows)
    solved = [r for r in rows if r["plausible"] and r["error_m"] is not None]
    out = {"n_frames": n, "n_plausible": len(solved),
           "plausible_rate": round(len(solved) / n, 4) if n else None,
           "median_latency_ms": pct([r["latency_ms"] for r in rows], 0.5),
           "p95_latency_ms": pct([r["latency_ms"] for r in rows], 0.95),
           "median_inliers": pct([r["inliers"] for r in solved], 0.5),
           "p10_inliers": pct([r["inliers"] for r in solved], 0.10),
           "gates": {}}
    for g in GATES:
        kept = [r for r in solved if r["inliers"] >= g]
        e = [r["error_m"] for r in kept]
        row = {"accepted": len(kept),
               "accept_rate": round(len(kept) / n, 4) if n else None,
               "median_m": _r(pct(e, 0.5)), "p90_m": _r(pct(e, 0.9)),
               "p99_m": _r(pct(e, 0.99)), "max_m": _r(max(e)) if e else None}
        for b in BANDS:
            row[f"within_{int(b)}m"] = round(sum(1 for x in e if x <= b) / len(e), 4) if e else None
        out["gates"][g] = row
    return out


def _r(v, nd=3):
    return None if v is None else round(float(v), nd)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/env80.yaml")
    ap.add_argument("--data-root", default="data/AnyVisLoc")
    ap.add_argument("--frames-root", default="data/AnyVisLoc_env80")
    ap.add_argument("--scenes", default="Scene_09,Scene_10")
    ap.add_argument("--modes", default="satellite")
    ap.add_argument("--methods", default="xfeat_mnn,xfeat_lg,orb,sift,akaze")
    ap.add_argument("--out", default="results/env80_sweep")
    ap.add_argument("--prior", choices=["none", "sequential"], default="none")
    ap.add_argument("--prior-radius", type=float, default=120.0)
    ap.add_argument("--frame-keypoints", type=int, default=4096)
    ap.add_argument("--ref-keypoints", type=int, default=2048)
    ap.add_argument("--frame-px", type=int, default=512)
    ap.add_argument("--tile-px", type=int, default=1024)
    ap.add_argument("--overlap-px", type=int, default=128)
    ap.add_argument("--min-matches", type=int, default=12)
    ap.add_argument("--ransac-px", type=float, default=3.0)
    ap.add_argument("--agl-min", type=float, default=50.0)
    ap.add_argument("--agl-max", type=float, default=100.0)
    ap.add_argument("--view-angle-min", type=float, default=80.0)
    ap.add_argument("--anchor", default=None)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--no-rectify", action="store_true",
                    help="the A/B that shows stage 02 is load-bearing")
    ap.add_argument("--no-tile-ransac", action="store_true",
                    help="fit over the whole map instead, for the A/B")
    ap.add_argument("--rebuild", action="store_true")
    args = ap.parse_args()

    cfg = cfgmod.load(args.config)
    out_root = REPO / args.out if not Path(args.out).is_absolute() else Path(args.out)
    scenes = [s.strip() for s in args.scenes.split(",") if s.strip()]
    modes = [s.strip() for s in args.modes.split(",") if s.strip()]
    methods = [s.strip() for s in args.methods.split(",") if s.strip()]

    avail = M.survey(max_keypoints=args.frame_keypoints)
    usable = [m for m in methods if avail.get(m, {}).get("available")]
    for m in methods:
        if m not in usable:
            print(f"skipping {m}: {avail.get(m, {}).get('reason', 'unknown')[:80]}")
    if not usable:
        print("no usable methods")
        return 1

    combos = [(s, mo, me) for s in scenes for mo in modes for me in usable]
    print(f"{len(combos)} combinations, prior={args.prior}, "
          f"rectify={'off' if args.no_rectify else 'on'}, "
          f"tile_ransac={'off' if args.no_tile_ransac else 'on'}")
    print(f"Roughly 0.3-1.5 s per frame per method on a laptop CPU, plus about 10 s to build\n"
          f"each store the first time. Expect tens of minutes for the full set. Re-running is\n"
          f"cheap: finished combinations are skipped unless FORCE=1.\n")

    force = os.environ.get("FORCE") == "1"
    summaries = {}
    t_all = time.perf_counter()
    for scene, mode, method in combos:
        tag = f"{scene}__{mode}__{method}"
        out_csv = out_root / f"{tag}.csv"
        print(f"[{tag}]")
        if out_csv.exists() and not force:
            rows = list(csv.DictReader(open(out_csv)))
            for r in rows:
                for k in ("inliers", "matches", "keypoints", "tiles_searched"):
                    r[k] = int(r[k] or 0)
                for k in ("error_m", "latency_ms", "inlier_ratio"):
                    r[k] = float(r[k]) if r[k] not in ("", None) else None
                r["plausible"] = r["plausible"] in ("True", "true", True)
            print(f"    cached ({len(rows)} frames). FORCE=1 to redo.")
        else:
            sd = REPO / args.data_root / scene
            fd = REPO / args.frames_root / scene
            if not sd.is_dir():
                print(f"    no scene at {sd}")
                continue
            rows = run_one(sd, fd if fd.is_dir() else sd, mode, method, cfg, out_csv, args)
        if rows:
            summaries[tag] = summarise(rows)
            print(f"    -> {out_csv.relative_to(REPO)}")

    (out_root / "summary.json").write_text(json.dumps(
        {"args": vars(args), "generated": time.time(), "summaries": summaries}, indent=2))

    print(f"\n{'='*104}")
    print(f"env80 sweep  |  prior={args.prior}  |  rectify={'off' if args.no_rectify else 'on'}"
          f"  |  {time.perf_counter()-t_all:.0f}s")
    print(f"{'='*104}\n")
    print(f"{'combination':38s} {'n':>4s} {'plaus':>6s} {'med inl':>8s} "
          f"{'p95 ms':>7s}   at the best gate:")
    print(f"{'':38s} {'':>4s} {'':>6s} {'':>8s} {'':>7s}   "
          f"{'gate':>5s} {'acc':>6s} {'med m':>8s} {'p90 m':>8s} {'<10 m':>6s}")
    print("-" * 104)
    for tag, s in summaries.items():
        best = _best_gate(s)
        g = s["gates"][best]
        print(f"{tag:38s} {s['n_frames']:4d} {s['plausible_rate']:6.2f} "
              f"{(s['median_inliers'] or 0):8.0f} {(s['p95_latency_ms'] or 0):7.0f}   "
              f"{best:5d} {(g['accept_rate'] or 0):6.2f} "
              f"{_f(g['median_m']):>8s} {_f(g['p90_m']):>8s} {_f(g['within_10m'],2):>6s}")

    print(f"\n{'-'*104}\nGate sweep, per combination. This is the table that shows a gate tuned on")
    print("one reference does not transfer to another.\n")
    for tag, s in summaries.items():
        print(f"  {tag}")
        print(f"    {'gate':>5s} {'accepted':>9s} {'rate':>6s} {'median m':>9s} "
              f"{'p90 m':>8s} {'p99 m':>9s} {'max m':>10s} {'<5m':>6s} {'<10m':>6s}")
        for g in GATES:
            r = s["gates"][g]
            if not r["accepted"]:
                continue
            print(f"    {g:5d} {r['accepted']:9d} {r['accept_rate']:6.2f} "
                  f"{_f(r['median_m']):>9s} {_f(r['p90_m']):>8s} {_f(r['p99_m']):>9s} "
                  f"{_f(r['max_m']):>10s} {_f(r['within_5m'],2):>6s} {_f(r['within_10m'],2):>6s}")
        print()

    print("How to read this")
    print("  * No mean, no RMSE anywhere. One degenerate solve destroys both, and this")
    print("    project has seen fixes wrong by 1e93 m.")
    print("  * 'plaus' is the fraction of frames that produced a geometrically plausible")
    print("    solve at all. It is the ceiling: no gate can accept more than this.")
    print("  * Pick the gate where p99 collapses without accept rate collapsing with it.")
    print("    That crossing point is the rejection result, and it moves with the")
    print("    reference and the descriptor, which is the finding.")
    print("  * Compare median and p90, never the maximum on its own.")
    print(f"\n  per-frame rows: {out_root.relative_to(REPO)}/*.csv")
    print(f"  summary:        {(out_root / 'summary.json').relative_to(REPO)}")
    print("\n  Rejected fixes are in the CSVs with their errors. They are the training")
    print("  data for the covariance estimator -- do not filter them out.")
    return 0


def _best_gate(s: dict) -> int:
    """The gate with the lowest p90 that still accepts at least half of what
    was plausible. Purely a display convenience; the sweep below is the answer."""
    floor = 0.5 * (s["n_plausible"] or 0)
    cands = [(g, r) for g, r in s["gates"].items()
             if r["accepted"] >= floor and r["p90_m"] is not None]
    if not cands:
        return 0
    return min(cands, key=lambda kv: kv[1]["p90_m"])[0]


def _f(v, nd=2):
    return "--" if v is None else f"{v:.{nd}f}"


if __name__ == "__main__":
    raise SystemExit(main())
