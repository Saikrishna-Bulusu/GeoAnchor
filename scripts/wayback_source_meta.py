#!/usr/bin/env python3
"""What imagery is actually under a point: when it was flown, by what, how fine.

    python scripts/wayback_source_meta.py --lat -37.818 --lon 144.967
    python scripts/wayback_source_meta.py --areas melbourne,sydney --json out.json

WHY THIS EXISTS. `fetch_wayback_tile.py` labels each capture with the Wayback
RELEASE date, taken from the service's `itemTitle`. That is the date Esri
published a version of the World Imagery basemap -- **not** the date the
imagery was acquired. A release can republish imagery flown years earlier, and
two releases months apart can carry acquisitions from completely different
years.

So `results/crossdate_wayback_curve.md`'s x-axis is a PUBLICATION gap that has
been read as map age. Over the Sydney CBD it happened to behave monotonically
and nobody noticed. Over Melbourne it does not: a 0.3-year publication gap
solves 5 of 24 frames while a 1.6-year gap solves 13, which is impossible if
the axis meant what it says.

Esri publishes the real thing. Each Wayback release has a companion metadata
service carrying per-footprint attributes:

    SRC_DATE   acquisition date, YYYYMMDD
    SRC_RES    NATIVE source resolution, metres -- not the tile's
    SRC_DESC   the sensor ("WV03", "Aerial Photography", ...)
    SRC_ACC    stated horizontal accuracy, metres

SRC_RES is the one that matters most for a cross-CITY comparison and is
invisible from the tiles themselves. Fetching at zoom 19 gives ~0.24 m/px
everywhere, but if the source under Melbourne is 0.31 m WorldView-3 and the
source under Sydney is 0.1 m aerial photography, those tiles carry very
different amounts of real detail at the same nominal pixel size -- and a
cross-date result compares sensors rather than dates.

SRC_ACC matters too: a stated 8 m horizontal accuracy is the same order as the
errors being measured, so part of any cross-date "error" is the two captures
simply being georeferenced differently.
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor

ROOT = "https://metadata.maptiles.arcgis.com/arcgis/rest/services"

# Same centres as scripts/crossdate_cities.sh.
AREAS = {
    "sydney":         (-33.86880, 151.20930),
    "melbourne":      (-37.81800, 144.96700),
    "brisbane":       (-27.47000, 153.02500),
    "perth":          (-31.95300, 115.86000),
    "rural_griffith": (-34.29000, 146.05000),
}


_SERVICES_CACHE = []


def services(timeout=60, retries=4):
    """The metadata service names, fetched ONCE and retried.

    This is ~200 names and it does not change during a run, but it was being
    re-fetched per area -- and the server rate-limits after a few hundred
    requests, so the third area died on `RemoteDisconnected` after the first
    two had succeeded. Cached and retried with a backoff.
    """
    if _SERVICES_CACHE:
        return _SERVICES_CACHE
    last = None
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(f"{ROOT}?f=json", timeout=timeout) as r:
                d = json.load(r)
            _SERVICES_CACHE.extend(
                s["name"] for s in d.get("services", [])
                if s["name"].startswith("World_Imagery_Metadata_"))
            return _SERVICES_CACHE
        except Exception as exc:
            last = exc
            time.sleep(2 ** attempt)
    raise RuntimeError(f"could not list metadata services after {retries} tries: {last}")


def identify(service, lat, lon, timeout=30):
    q = urllib.parse.urlencode({
        "geometry": json.dumps({"x": lon, "y": lat}),
        "geometryType": "esriGeometryPoint", "sr": 4326, "tolerance": 2,
        "mapExtent": f"{lon-0.05},{lat-0.05},{lon+0.05},{lat+0.05}",
        "imageDisplay": "400,400,96", "returnGeometry": "false",
        "layers": "all", "f": "json"})
    url = f"{ROOT}/{service}/MapServer/identify?{q}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            d = json.load(r)
    except Exception:
        return None
    for res in d.get("results", []):
        a = res.get("attributes", {})
        if a.get("SRC_DATE"):
            return {"service": service,
                    "src_date": str(a.get("SRC_DATE")),
                    "src_res_m": _f(a.get("SRC_RES")),
                    "src_acc_m": _f(a.get("SRC_ACC")),
                    "samp_res_m": _f(a.get("SAMP_RES")),
                    "sensor": a.get("SRC_DESC")}
    return None


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def history(lat, lon, workers=8):
    """Distinct acquisitions under a point, oldest first."""
    svcs = services()
    rows = []
    with ThreadPoolExecutor(max_workers=workers) as ex:
        for r in ex.map(lambda s: identify(s, lat, lon), svcs):
            if r:
                rows.append(r)
    seen, out = set(), []
    for r in sorted(rows, key=lambda r: r["src_date"]):
        key = (r["src_date"], r["src_res_m"], r["sensor"])
        if key not in seen:
            seen.add(key)
            out.append(r)
    return out, len(svcs), len(rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lat", type=float)
    ap.add_argument("--lon", type=float)
    ap.add_argument("--areas", default=None,
                    help="comma-separated names from the built-in table")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    targets = {}
    if a.areas:
        for n in a.areas.split(","):
            if n not in AREAS:
                print(f"unknown area '{n}'. Known: {', '.join(AREAS)}")
                return 2
            targets[n] = AREAS[n]
    elif a.lat is not None and a.lon is not None:
        targets["point"] = (a.lat, a.lon)
    else:
        targets = dict(AREAS)

    doc = {}
    for name, (lat, lon) in targets.items():
        # One area failing must not lose the ones already resolved: this takes
        # tens of minutes and the server rate-limits.
        try:
            rows, n_svc, n_hit = history(lat, lon)
        except Exception as exc:
            print(f"\n{name}: FAILED ({type(exc).__name__}: {exc})")
            doc[name] = {"lat": lat, "lon": lon, "error": str(exc)}
            continue
        doc[name] = {"lat": lat, "lon": lon, "services_probed": n_svc,
                     "services_answering": n_hit, "acquisitions": rows}
        coverage = n_hit / max(n_svc, 1)
        print(f"\n\033[1m{name}\033[0m  {lat:.5f}, {lon:.5f}   "
              f"{n_hit}/{n_svc} services answered, {len(rows)} distinct acquisitions")
        # A LOW ANSWER RATE IS A RATE LIMIT, NOT A SHORT HISTORY. Esri throttles
        # after a few hundred identify calls, and a throttled area returns a
        # handful of acquisitions that look like a complete record -- Perth came
        # back with 2 acquisitions on 7/196 services where Brisbane got 6 on
        # 101/196, and Perth demonstrably has captures back to 2014. Nothing in
        # the returned data says which it is; only this ratio does.
        if coverage < 0.25:
            print(f"  \033[1mINCOMPLETE\033[0m -- only {coverage:.0%} of services "
                  f"answered. This is throttling, not a short capture history.")
            print(f"  Re-run this area alone after a cooldown before using it.")
            doc[name]["incomplete"] = True
        print(f"  {'acquired':10} {'native m':>9} {'sampled m':>10} "
              f"{'stated acc m':>13}  sensor")
        for r in rows:
            d = r["src_date"]
            pretty = f"{d[:4]}-{d[4:6]}-{d[6:8]}" if len(d) == 8 else d
            print(f"  {pretty:10} {_s(r['src_res_m']):>9} {_s(r['samp_res_m']):>10} "
                  f"{_s(r['src_acc_m']):>13}  {r['sensor']}")
        res = [r["src_res_m"] for r in rows if r["src_res_m"]]
        if res:
            print(f"  native resolution spans {min(res):g} to {max(res):g} m here")
        acc = [r["src_acc_m"] for r in rows if r["src_acc_m"]]
        if acc:
            print(f"  stated horizontal accuracy {min(acc):g} to {max(acc):g} m "
                  f"-- the same order as the cross-date errors being measured")
        if a.json:                      # incremental, so a crash keeps this
            with open(a.json, "w") as f:
                json.dump(doc, f, indent=2)

    if len(doc) > 1:
        print("\n\033[1mNative source resolution by area\033[0m  "
              "-- a cross-CITY comparison at fixed zoom compares these, not dates")
        for name, d in doc.items():
            res = [r["src_res_m"] for r in d["acquisitions"] if r["src_res_m"]]
            sensors = sorted({r["sensor"] for r in d["acquisitions"] if r["sensor"]})
            if res:
                print(f"  {name:16} {min(res):.2f}-{max(res):.2f} m   "
                      f"{', '.join(sensors)[:60]}")

    if a.json:
        with open(a.json, "w") as f:
            json.dump(doc, f, indent=2)
        print(f"\nwrote {a.json}")
    return 0


def _s(v):
    return "--" if v is None else f"{v:g}"


if __name__ == "__main__":
    raise SystemExit(main())
