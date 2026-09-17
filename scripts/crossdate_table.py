#!/usr/bin/env python3
"""Turn results/crossdate_cities/*.json into one table per area, plus a verdict.

    python scripts/crossdate_table.py
    python scripts/crossdate_table.py --md > results/crossdate_cities.md

The question this exists to answer is narrow and was not answerable from the
Sydney run alone: **is the 3-year cross-date cliff a property of the MATCHER,
or of the Sydney CBD?** One site cannot tell those apart, and the Sydney CBD is
an unusually bad site to generalise from -- dense high-rise, so most of what
changes between captures is facade parallax and shadow rather than ground.

The cliff is located per area as the largest gap still meeting BOTH conditions,
because either alone is misleading: a handful of lucky solves can keep a median
respectable while the system is useless, and a high solve rate at 80 m median
is not a working system either.

    solved >= half the frames   AND   median error <= `--usable-m`

A control row (gap 0.0) that does not solve invalidates its whole area, and is
reported as such rather than being averaged in with the rest.
"""
from __future__ import annotations

import argparse
import glob
import json
import re
from datetime import date
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DATE_RE = re.compile(r"(\d{4})-(\d{2})-(\d{2})")


def parse_date(s):
    m = DATE_RE.search(s)
    return date(*map(int, m.groups())) if m else None


def load(root):
    """{area: {"control": row|None, "rows": [(gap_years, capture, row)]}}"""
    areas = {}
    for f in sorted(glob.glob(str(REPO / root / "*.json"))):
        name = Path(f).stem                      # melbourne__control | melbourne__ref_tile_2023-06-29
        area, _, tail = name.partition("__")
        try:
            rows = json.load(open(f))
        except Exception:
            continue
        if not rows:
            continue
        r = rows[0]
        a = areas.setdefault(area, {"control": None, "rows": []})
        if tail == "control":
            a["control"] = r
            continue
        q, ref = parse_date(r.get("query", "")), parse_date(r.get("reference", ""))
        gap = (q - ref).days / 365.25 if q and ref else float("nan")
        a["rows"].append((gap, tail.replace("ref_tile_", ""), r))
    for a in areas.values():
        a["rows"].sort(key=lambda t: t[0])
    return areas


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results/crossdate_cities")
    ap.add_argument("--usable-m", type=float, default=15.0,
                    help="median error at or below which a gap counts as working")
    ap.add_argument("--md", action="store_true", help="markdown instead of a terminal table")
    a = ap.parse_args()

    areas = load(a.root)
    if not areas:
        print(f"nothing in {a.root} -- run scripts/crossdate_cities.sh first")
        return 1

    cliffs = {}
    for area, d in sorted(areas.items()):
        ctl = d["control"]
        frames = (ctl or (d["rows"][0][2] if d["rows"] else {})).get("frames", 0)
        print(f"\n## {area}" if a.md else f"\n\033[1m{area}\033[0m")
        if ctl is None:
            print("  NO CONTROL. Nothing from this area is interpretable.")
            continue
        if ctl.get("solved", 0) < 1:
            print(f"  CONTROL FAILED ({ctl.get('solved')}/{frames} solved). "
                  "A zero-fix result here is the harness or the tile, not the date.")
            continue
        print(f"  control: {ctl['solved']}/{frames} solved, "
              f"median {ctl.get('median_error_m')} m, "
              f"{ctl.get('inliers_median')} inliers -- area is testable")

        head = ("| capture | gap, yr | inliers | solved | median m | p90 m |"
                if a.md else
                f"  {'capture':12} {'gap yr':>7} {'inl':>5} {'solved':>7} "
                f"{'median m':>10} {'p90 m':>9}")
        print(head)
        if a.md:
            print("|---|---|---|---|---|---|")
        # Track EVERY working gap, not just the last one. The rows are sorted
        # by gap, so a monotonic area gives a working prefix and a failing
        # tail; anything else is non-monotonic and a "cliff at X years" is not
        # a description of it. An earlier version reported last_good and
        # first_bad blindly and printed "cliff between 9.1 and 1.6 years" for
        # Perth, which is not a statement about anything.
        good, bad = [], []
        for gap, cap, r in d["rows"]:
            solved, med = r.get("solved", 0), r.get("median_error_m")
            ok = solved >= frames / 2 and med is not None and med <= a.usable_m
            (good if ok else bad).append(gap)
            ms = "--" if med is None else f"{med:.2f}"
            ps = "--" if r.get("p90_error_m") is None else f"{r['p90_error_m']:.2f}"
            mark = "" if a.md else ("  " if ok else " x")
            if a.md:
                print(f"| {cap} | {gap:.1f} | {r.get('inliers_median')} | "
                      f"{solved}/{frames} | **{ms}** | {ps} |")
            else:
                print(f"  {cap:12} {gap:7.1f} {str(r.get('inliers_median')):>5} "
                      f"{solved:>3}/{frames:<3} {ms:>10} {ps:>9}{mark}")
        # Monotonic means every working gap is shorter than every failing one.
        monotonic = (not good or not bad) or max(good) < min(bad)
        cliffs[area] = (good, bad, monotonic)
        if not good:
            print(f"  No gap works at all -- even the shortest fails the "
                  f"{a.usable_m:g} m / half-the-frames test.")
        elif not bad:
            print(f"  Every measured gap works, out to {max(good):.1f} years. "
                  "No cliff inside this area's capture history.")
        elif monotonic:
            print(f"  **Cliff between {max(good):.1f} and {min(bad):.1f} years.**")
        else:
            print(f"  **NOT MONOTONIC -- there is no cliff to quote.** Works at "
                  f"{', '.join(f'{g:.1f}' for g in good)} yr; fails at "
                  f"{', '.join(f'{b:.1f}' for b in bad)} yr.")
            print(f"  A short gap failing while a longer one works means ELAPSED "
                  f"TIME IS NOT THE VARIABLE here. The likeliest reason is that "
                  f"the axis is\n  the Wayback RELEASE date, not the acquisition "
                  f"date -- see scripts/wayback_source_meta.py.")

    if len(cliffs) > 1:
        print("\n## Across areas" if a.md else "\n\033[1mAcross areas\033[0m")
        for area, (good, bad, mono) in sorted(cliffs.items()):
            if not good:
                where = "no working gap at all"
            elif not bad:
                where = f"works at every measured gap, out to {max(good):.1f} yr"
            elif mono:
                where = f"cliff between {max(good):.1f} and {min(bad):.1f} yr"
            else:
                where = (f"NOT MONOTONIC -- {len(good)} of "
                         f"{len(good) + len(bad)} gaps work, out to {max(good):.1f} yr")
            print(f"  {area:18} {where}")
        mono_areas = [a_ for a_, (g, b, m) in cliffs.items() if m and g and b]
        print("\n  A cliff in the same place everywhere would be a MATCHER property. "
              "One that moves with\n  the site is a property of what is on the ground, "
              "and then the flight area decides it.")
        if len(cliffs) - len(mono_areas) > 0:
            print("  Areas whose results are NOT monotonic cannot contribute a cliff "
                  "location at all:\n  a shorter gap failing while a longer one works "
                  "says the x-axis is not measuring age.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
