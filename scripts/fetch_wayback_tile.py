#!/usr/bin/env python3
"""Fetch a dated capture of the same ground from Esri's World Imagery Wayback.

    python scripts/fetch_wayback_tile.py --list --match data/sydney/ref_tile.tif
    python scripts/fetch_wayback_tile.py --match data/sydney/ref_tile.tif --release 3026
    python scripts/fetch_wayback_tile.py --match data/sydney/ref_tile.tif --all

WHY THIS AND NOT THE NSW SERVICE
--------------------------------
The demo feed cuts its frames out of the reference tile, so it matches a
picture against itself and reports 0.007 m. A real result needs the query and
the reference to be INDEPENDENT captures of the same ground.

NSW's own mosaic knows about four dated captures over the Sydney CBD -- 2013,
2018, 2020 and 2021 -- and will not render a chosen one: `layerDefs` is
rejected because the imagery layer is a raster, not a feature layer. Its only
separately-fetchable epoch is `sixmaps/sydney1943`, and 1943 against 2021 is a
78-year gap that returns zero fixes (see HISTORICAL_IMAGERY.md). Useful as a
failure boundary, useless as a benchmark.

Esri's Wayback archive solves it. It keeps every published version of the World
Imagery basemap -- 196 releases between 2014-02-20 and 2026-02-26 -- each one a
plain tile service with no token. Most are identical over any given place,
because a release only changes where Esri recaptured, so `--list` fetches one
tile from every release and reports the DISTINCT ones. Over the Sydney CBD that
is 13 captures spanning twelve years, at zoom 20 = 0.124 m/px, which is exactly
the resolution of `data/sydney/ref_tile.tif`.

That is a degradation CURVE -- 2, 5, 9, 12 years -- rather than the single
useless point 1943 gives.

LICENCE
-------
Esri World Imagery is not CC BY. It is redistributed under Esri's terms, which
permit use in Esri-published apps and for evaluation, and the Wayback archive
is published for change comparison. **Treat what this fetches as working data
for measurement, not as something to redistribute or to ship inside a
deliverable.** Numbers derived from it are yours; the pixels are not. For an
open-licence reference tile use the NSW service (CC BY), which is what
`step17` builds and what the pipeline actually flies against.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

CONFIG = ("https://s3-us-west-2.amazonaws.com/config.maptiles.arcgis.com"
          "/waybackconfig.json")
MAX_Z = 20                       # deepest level served over Sydney; 21 is 404
TILE_PX = 256
EARTH_CIRCUM = 40075016.685578488
ATTRIBUTION = ("Esri World Imagery (Wayback). Source: Esri, Maxar, Earthstar "
               "Geographics, and the GIS User Community.")


def deg2tile(lat, lon, z):
    n = 2.0 ** z
    return ((lon + 180.0) / 360.0 * n,
            (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)


def tile2merc(x, y, z):
    n = 2.0 ** z
    return (x / n * EARTH_CIRCUM - EARTH_CIRCUM / 2,
            EARTH_CIRCUM / 2 - y / n * EARTH_CIRCUM)


def releases():
    cfg = json.loads(urllib.request.urlopen(CONFIG, timeout=45).read())
    out = []
    for key, v in cfg.items():
        title = v.get("itemTitle", "")
        date = title.split("Wayback ")[-1].rstrip(")") if "Wayback " in title else "?"
        out.append({"release": key, "date": date, "url": v["itemURL"]})
    return sorted(out, key=lambda r: r["date"])


def tile_url(rel_url, z, x, y):
    return rel_url.replace("{level}", str(z)).replace("{row}", str(y)).replace("{col}", str(x))


def fetch(url, retries=3):
    for i in range(retries):
        try:
            with urllib.request.urlopen(url, timeout=40) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return None
            if i == retries - 1:
                raise
        except Exception:
            if i == retries - 1:
                raise
    return None


# ------------------------------------------------------------------ list ----
def list_distinct(lat, lon, z):
    """One tile per release, hashed. Identical hash = the same capture."""
    import hashlib
    rels = releases()
    xf, yf = deg2tile(lat, lon, z)
    x, y = int(xf), int(yf)
    print(f"probing {len(rels)} releases at z={z} over {lat:.4f},{lon:.4f}")

    def job(r):
        try:
            b = fetch(tile_url(r["url"], z, x, y))
            return r, (hashlib.md5(b).hexdigest() if b else None), (len(b) if b else 0)
        except Exception:
            return r, None, 0

    got = []
    with ThreadPoolExecutor(max_workers=16) as p:
        for n, (r, h, size) in enumerate(p.map(job, rels), 1):
            if h:
                got.append((r, h, size))
            if n % 40 == 0 or n == len(rels):
                print(f"\r  {n}/{len(rels)}", end="", flush=True)
    print()

    seen, distinct = set(), []
    for r, h, size in got:
        if h not in seen:
            seen.add(h)
            distinct.append((r, size))
    print(f"{len(got)} releases returned a tile; {len(distinct)} DISTINCT captures\n")
    print(f"  {'release':>8s}  {'date':12s}  bytes")
    for r, size in distinct:
        print(f"  {r['release']:>8s}  {r['date']:12s}  {size:7d}")
    print("\nfetch one with  --release <id>,  or every one with  --all")
    return distinct


# ----------------------------------------------------------------- fetch ----
def deepest_zoom(rel, lat, lon, want):
    """The finest level this RELEASE actually serves here.

    Coverage depth is per capture, not per service: the 2014 release 404s at
    zoom 20 over Sydney while 2026 serves it. Fetching at a fixed zoom
    therefore produced a completely black tile for the older half of the
    archive and reported 'every tile was absent', which reads as a broken
    script rather than as a real property of the data.
    """
    for z in range(want, 15, -1):
        x, y = deg2tile(lat, lon, z)
        try:
            if fetch(tile_url(rel["url"], z, int(x), int(y))):
                return z
        except Exception:
            pass
    return None


def build_mosaic(rel, west, south, east, north, z):
    x0f, _ = deg2tile(south, west, z)
    _, y0f = deg2tile(north, west, z)
    x1f, _ = deg2tile(south, east, z)
    _, y1f = deg2tile(south, east, z)
    x0, y0, x1, y1 = int(x0f), int(y0f), int(x1f), int(y1f)
    nx, ny = x1 - x0 + 1, y1 - y0 + 1
    total = nx * ny
    if total > 4096:
        raise SystemExit(f"  {total} tiles is too many -- narrow the bbox or lower --zoom")

    canvas = np.zeros((ny * TILE_PX, nx * TILE_PX, 3), np.uint8)
    missing = 0

    def job(i):
        ty, tx = divmod(i, nx)
        b = fetch(tile_url(rel["url"], z, x0 + tx, y0 + ty))
        return i, (cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR) if b else None)

    with ThreadPoolExecutor(max_workers=8) as p:
        for done, (i, img) in enumerate(p.map(job, range(total)), 1):
            ty, tx = divmod(i, nx)
            if img is None:
                missing += 1
            else:
                canvas[ty * TILE_PX:(ty + 1) * TILE_PX,
                       tx * TILE_PX:(tx + 1) * TILE_PX] = img
            if done % 25 == 0 or done == total:
                print(f"\r    {done}/{total} tiles", end="", flush=True)
    print()
    if missing == total:
        raise SystemExit("    every tile was absent for this release")
    if missing:
        print(f"    {missing}/{total} tiles absent (rendered black)")
    left, top = tile2merc(x0, y0, z)
    right, bottom = tile2merc(x1 + 1, y1 + 1, z)
    return canvas, (left, bottom, right, top)


def write_geotiff(img, merc, dst_crs, out, rel):
    import rasterio
    from rasterio.transform import from_bounds
    from rasterio.warp import calculate_default_transform, reproject, Resampling

    left, bottom, right, top = merc
    h, w = img.shape[:2]
    src_tf = from_bounds(left, bottom, right, top, w, h)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    dst_tf, dw, dh = calculate_default_transform("EPSG:3857", dst_crs, w, h,
                                                 left, bottom, right, top)
    out.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(out, "w", driver="GTiff", height=dh, width=dw, count=3,
                       dtype="uint8", crs=dst_crs, transform=dst_tf,
                       compress="DEFLATE", tiled=True,
                       blockxsize=256, blockysize=256) as dst:
        for b in range(3):
            reproject(source=rgb[:, :, b], destination=rasterio.band(dst, b + 1),
                      src_transform=src_tf, src_crs="EPSG:3857",
                      dst_transform=dst_tf, dst_crs=dst_crs,
                      resampling=Resampling.bilinear)
        dst.update_tags(source="Esri World Imagery Wayback",
                        release=rel["release"], epoch=rel["date"],
                        attribution=ATTRIBUTION)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--match", help="copy the footprint and CRS of this GeoTIFF")
    ap.add_argument("--bbox", help="west,south,east,north in degrees")
    ap.add_argument("--list", action="store_true", help="report the distinct captures")
    ap.add_argument("--release", help="release id to fetch")
    ap.add_argument("--all", action="store_true", help="fetch every distinct capture")
    ap.add_argument("--zoom", type=int, default=MAX_Z)
    ap.add_argument("--outdir", default="data/sydney/wayback")
    ap.add_argument("--crs", default=None)
    a = ap.parse_args()

    def rp(p):
        p = Path(p)
        return p if p.is_absolute() else REPO / p

    dst_crs = a.crs
    if a.match:
        import rasterio
        from rasterio.warp import transform_bounds
        with rasterio.open(rp(a.match)) as s:
            west, south, east, north = transform_bounds(s.crs, "EPSG:4326", *s.bounds)
            dst_crs = dst_crs or str(s.crs)
    elif a.bbox:
        west, south, east, north = (float(v) for v in a.bbox.split(","))
    else:
        raise SystemExit("give --match <tif> or --bbox w,s,e,n")
    # DERIVE THE UTM ZONE. This used to fall back to a hardcoded EPSG:32756,
    # which is zone 56S -- Sydney's. Every --bbox fetch anywhere else in the
    # country therefore landed in Sydney's projection: Melbourne 8 degrees
    # outside it, PERTH 37 DEGREES OUTSIDE IT. A tile and its own historical
    # captures all get the same warp, so matching between them is unaffected
    # and inlier counts are still comparable -- but distances in metres are
    # scaled by the UTM scale-factor error, which grows with the square of the
    # distance from the central meridian and is nonsense that far out.
    #
    # ap_vo2 and geoanchor both require a UTM tile and fail silently on a
    # wrong one, so this is not only a measurement issue.
    if not dst_crs:
        clon_, clat_ = (west + east) / 2, (south + north) / 2
        zone = int((clon_ + 180) / 6) + 1
        dst_crs = f"EPSG:{32700 + zone if clat_ < 0 else 32600 + zone}"
        print(f"CRS {dst_crs} (UTM zone {zone}{'S' if clat_ < 0 else 'N'}), "
              f"derived from the bbox centre")

    if a.list:
        list_distinct((south + north) / 2, (west + east) / 2, min(a.zoom, 18))
        return

    if not a.release and not a.all:
        raise SystemExit("give --release <id>, or --all, or --list to see them")

    if a.all:
        picks = [r for r, _ in list_distinct((south + north) / 2, (west + east) / 2,
                                             min(a.zoom, 18))]
    else:
        picks = [r for r in releases() if r["release"] == a.release]
        if not picks:
            raise SystemExit(f"no release {a.release} -- run --list")

    outdir = rp(a.outdir)
    clat, clon = (south + north) / 2, (west + east) / 2
    print(f"\nfetching {len(picks)} capture(s) -> {outdir}")
    written = []
    for rel in picks:
        out = outdir / f"ref_tile_{rel['date']}.tif"
        if out.exists():
            print(f"  {rel['date']}  already present, skipping")
            written.append(out)
            continue
        z = deepest_zoom(rel, clat, clon, a.zoom)
        if z is None:
            print(f"  {rel['date']}  no coverage here at any zoom -- skipped")
            continue
        gsd = (EARTH_CIRCUM / (2.0 ** z) / TILE_PX) * math.cos(math.radians(clat))
        print(f"  {rel['date']}  (release {rel['release']}, z={z}, ~{gsd:.3f} m/px)")
        img, merc = build_mosaic(rel, west, south, east, north, z)
        write_geotiff(img, merc, dst_crs, out, rel)
        print(f"    wrote {out.name}  ({out.stat().st_size / 1e6:.1f} MB)")
        written.append(out)

    print(f"\n{len(written)} tiles in {outdir}")
    print(f"\n{ATTRIBUTION}")
    print("Working data for measurement -- see the licence note in this file's header.")
    print("\nMeasure the cross-date degradation with:")
    print("  python scripts/crossdate_probe.py --query data/sydney/ref_tile.tif \\")
    print(f"      --reference {outdir.relative_to(REPO)}/<one>.tif")


if __name__ == "__main__":
    main()
