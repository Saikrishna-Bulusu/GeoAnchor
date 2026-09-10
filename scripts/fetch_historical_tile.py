#!/usr/bin/env python3
"""Fetch NSW's 1943 aerial survey as a georeferenced tile, matching an existing
reference tile's footprint.

    python scripts/fetch_historical_tile.py --match data/sydney/ref_tile.tif
    python scripts/fetch_historical_tile.py --bbox 151.2065,-33.8711,151.2121,-33.8665
    python scripts/fetch_historical_tile.py --match data/sydney/ref_tile.tif --check

WHY THIS EXISTS
---------------
The demo feed cuts its frames out of the reference tile, so the pipeline is
matching a picture against a copy of itself and reports 0.007 m. That is a
wiring test, not accuracy, and the README says so at length.

A real localisation result needs the query and the reference to be INDEPENDENT
captures of the same ground. NSW publishes exactly that: `sixmaps/sydney1943`
is a separate cached service holding a greyscale aerial survey of inner Sydney
circa 1943, on the same ground the current `LPI_Imagery_Best` mosaic covers.

So the two together give a genuine cross-date pair with nothing to fetch but
tiles, and no licence problem -- NSW imagery is CC BY.

WHAT THIS IS AND IS NOT
-----------------------
1943 against 2021 is a **78-year** gap. That is an upper bound on appearance
change, not a deployment number: in the Sydney CBD almost every building in the
frame has been replaced. What survives is road geometry, the coastline, parks
and a handful of landmarks.

Use it to answer "how far can this degrade before it breaks", which is a
covariance question and therefore on-topic for this project. Do NOT quote it as
the system's accuracy. The realistic cross-date number already exists and comes
from AnyVisLoc env80 -- real drone frames against a satellite basemap captured
at a different time -- at 2.5-3.8 m median.

RESOLUTION
----------
The service advertises 22 levels of detail but the cache is only built to
**zoom 19** over Sydney; 20 and 21 return 404. Zoom 19 is 0.2986 m/px in Web
Mercator units, which at Sydney's latitude is about **0.25 m/px** on the
ground -- roughly half the detail of the 0.124 m/px current tile. Checked, not
assumed: `--check` re-verifies it.
"""
from __future__ import annotations

import argparse
import io
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

SERVICE = ("https://maps.six.nsw.gov.au/arcgis/rest/services"
           "/sixmaps/sydney1943/MapServer/tile")
# The deepest level the cache actually serves here. The service metadata claims
# 21; 20 and 21 both 404 over the CBD. Verified 10 Sept 2026.
MAX_Z = 19
TILE_PX = 256
EARTH_CIRCUM = 40075016.685578488          # Web Mercator, metres
ATTRIBUTION = ("NSW Department of Customer Service, Sydney 1943 imagery. "
               "CC BY 4.0. https://maps.six.nsw.gov.au")


# ---------------------------------------------------------------- tiles ----
def deg2tile(lat: float, lon: float, z: int) -> tuple:
    n = 2.0 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def tile2merc(x: float, y: float, z: float) -> tuple:
    """Tile coordinate -> EPSG:3857 metres."""
    n = 2.0 ** z
    return (x / n * EARTH_CIRCUM - EARTH_CIRCUM / 2,
            EARTH_CIRCUM / 2 - y / n * EARTH_CIRCUM)


def fetch_tile(z: int, x: int, y: int, retries: int = 3):
    url = f"{SERVICE}/{z}/{y}/{x}"
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(url, timeout=30) as r:
                buf = np.frombuffer(r.read(), np.uint8)
            return cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return None                 # genuinely outside the cache
            if attempt == retries - 1:
                raise
        except Exception:
            if attempt == retries - 1:
                raise
    return None


def build_mosaic(west, south, east, north, z):
    """Fetch every tile covering the box and stitch them, north-up."""
    x0f, y0f = deg2tile(north if False else south, west, z)   # x from west
    x0f, _ = deg2tile(south, west, z)
    _, y0f = deg2tile(north, west, z)                          # y from north
    x1f, _ = deg2tile(south, east, z)
    _, y1f = deg2tile(south, east, z)

    x0, y0 = int(math.floor(x0f)), int(math.floor(y0f))
    x1, y1 = int(math.floor(x1f)), int(math.floor(y1f))
    nx, ny = x1 - x0 + 1, y1 - y0 + 1
    total = nx * ny
    print(f"  zoom {z}: {nx} x {ny} = {total} tiles")
    if total > 4096:
        raise SystemExit(f"  {total} tiles is too many -- narrow the bbox")

    canvas = np.zeros((ny * TILE_PX, nx * TILE_PX, 3), np.uint8)
    missing = 0

    def job(i):
        ty, tx = divmod(i, nx)
        return i, fetch_tile(z, x0 + tx, y0 + ty)

    with ThreadPoolExecutor(max_workers=8) as pool:
        for done, (i, img) in enumerate(pool.map(job, range(total)), 1):
            ty, tx = divmod(i, nx)
            if img is None:
                missing += 1
            else:
                canvas[ty * TILE_PX:(ty + 1) * TILE_PX,
                       tx * TILE_PX:(tx + 1) * TILE_PX] = img
            if done % 25 == 0 or done == total:
                print(f"\r  fetched {done}/{total}", end="", flush=True)
    print()
    if missing:
        print(f"  {missing}/{total} tiles absent from the cache (rendered black)")
    if missing == total:
        raise SystemExit("  every tile was absent -- this area is not in the "
                         "1943 survey. It covers inner Sydney and the highways.")

    # Web Mercator bounds of the stitched canvas, from the tile grid itself.
    left, top = tile2merc(x0, y0, z)
    right, bottom = tile2merc(x1 + 1, y1 + 1, z)
    return canvas, (left, bottom, right, top), missing / total


# ------------------------------------------------------------------ io ----
def write_geotiff(img, merc_bounds, dst_crs, out_path, clip_bounds=None):
    import rasterio
    from rasterio.transform import from_bounds
    from rasterio.warp import calculate_default_transform, reproject, Resampling

    left, bottom, right, top = merc_bounds
    h, w = img.shape[:2]
    src_tf = from_bounds(left, bottom, right, top, w, h)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    dst_tf, dw, dh = calculate_default_transform(
        "EPSG:3857", dst_crs, w, h, left, bottom, right, top)

    profile = dict(driver="GTiff", height=dh, width=dw, count=3,
                   dtype="uint8", crs=dst_crs, transform=dst_tf,
                   compress="DEFLATE", tiled=True,
                   blockxsize=256, blockysize=256)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(out_path, "w", **profile) as dst:
        for b in range(3):
            reproject(source=rgb[:, :, b], destination=rasterio.band(dst, b + 1),
                      src_transform=src_tf, src_crs="EPSG:3857",
                      dst_transform=dst_tf, dst_crs=dst_crs,
                      resampling=Resampling.bilinear)
        dst.update_tags(
            source="NSW sixmaps/sydney1943",
            epoch="1943",
            attribution=ATTRIBUTION,
            note=("Independent capture of the same ground as the current "
                  "reference. Cross-date pair; see docs/HISTORICAL_IMAGERY.md"))
    return out_path


def check(path: Path):
    import rasterio
    from rasterio.warp import transform_bounds
    with rasterio.open(path) as s:
        gsd = s.res[0]
        print(f"  crs     {s.crs}")
        print(f"  size    {s.width} x {s.height}, {s.count} bands")
        print(f"  gsd     {gsd:.4f} m/px")
        print(f"  bounds  {transform_bounds(s.crs, 'EPSG:4326', *s.bounds)}")
        print(f"  tags    {s.tags().get('epoch')} / {s.tags().get('source')}")
        band = s.read(1)
    ok = True
    if not str(s.crs).startswith("EPSG:326") and not str(s.crs).startswith("EPSG:327"):
        print("  FAIL    not a UTM CRS -- ap_vo2 and mapprep require one")
        ok = False
    dark = float((band == 0).mean())
    if dark > 0.25:
        print(f"  WARN    {dark:.0%} of the tile is black (absent cache tiles)")
    # rho: how many reference pixels one frame pixel covers at 50-100 m AGL,
    # the same figure step17 reports. fx_px 1421.48 measured on the C270.
    for alt in (50, 100):
        rho = (alt / 1421.48) / gsd
        print(f"  rho     {rho:.3f} at {alt} m AGL"
              f"{'   (below the 0.15 comfort target)' if rho < 0.15 else ''}")
    print("  " + ("PASS" if ok else "FAIL"))
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--match", help="copy the footprint and CRS of this GeoTIFF")
    ap.add_argument("--bbox", help="west,south,east,north in degrees")
    ap.add_argument("--zoom", type=int, default=MAX_Z)
    ap.add_argument("--out", default="data/sydney/ref_tile_1943.tif")
    ap.add_argument("--crs", default=None, help="target CRS, default UTM 56S")
    ap.add_argument("--check", action="store_true",
                    help="verify the output instead of fetching")
    a = ap.parse_args()

    out = Path(a.out)
    if not out.is_absolute():
        out = REPO / out

    if a.check and out.exists():
        print(f"checking {out}")
        raise SystemExit(0 if check(out) else 1)

    dst_crs = a.crs
    if a.match:
        import rasterio
        from rasterio.warp import transform_bounds
        src = Path(a.match)
        if not src.is_absolute():
            src = REPO / src
        with rasterio.open(src) as s:
            west, south, east, north = transform_bounds(s.crs, "EPSG:4326", *s.bounds)
            dst_crs = dst_crs or str(s.crs)
        print(f"matching {src.name}: {west:.5f},{south:.5f},{east:.5f},{north:.5f}")
    elif a.bbox:
        west, south, east, north = (float(v) for v in a.bbox.split(","))
    else:
        raise SystemExit("give --match <tif> or --bbox w,s,e,n")

    dst_crs = dst_crs or "EPSG:32756"
    if a.zoom > MAX_Z:
        print(f"note: zoom {a.zoom} is not in the cache over Sydney; "
              f"using {MAX_Z}")
        a.zoom = MAX_Z

    print(f"fetching NSW 1943 survey -> {dst_crs}")
    img, merc, frac_missing = build_mosaic(west, south, east, north, a.zoom)
    print(f"  mosaic  {img.shape[1]} x {img.shape[0]} px")
    write_geotiff(img, merc, dst_crs, out)
    print(f"  wrote   {out}  ({out.stat().st_size / 1e6:.1f} MB)")
    print()
    check(out)
    print()
    print(f"  {ATTRIBUTION}")
    print()
    print("  Build a feature store against it with:")
    print(f"    GEOANCHOR_SET=\"data_layer.map.source={out.relative_to(REPO)}\" \\")
    print("      python -m geoanchor.data_layer --build-map")


if __name__ == "__main__":
    main()
