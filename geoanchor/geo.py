"""Pixel to ground, and distances on the ground.

Deliberately depends on pyproj and not on rasterio. Reading a GeoTIFF is a
map-preprocessing job that happens once, on whatever machine has GDAL; the
flight-time path only ever needs the affine transform and the EPSG code that
preprocessing already wrote into the store manifest. That keeps the Jetson
install to wheels that are known good on aarch64.
"""
from __future__ import annotations

import math
from functools import lru_cache

WGS84 = 4326
_EARTH_R = 6371008.8  # mean radius, metres


def _spec(crs) -> str:
    """A CRS here is either an EPSG code or a PROJ string.

    AnyVisLoc ground truth is scene-local and SfM-refined, not geodetic, so its
    reference maps have no EPSG code at all. Rather than inventing one, the
    env80 adapter builds a transverse-Mercator PROJ string anchored at a
    declared origin: distances stay exact to the millimetre over a 300 m scene,
    and everything downstream that expects latitude and longitude keeps working
    without a second code path.
    """
    if isinstance(crs, int):
        return f"EPSG:{crs}"
    text = str(crs)
    return text if ("+" in text or "[" in text) else f"EPSG:{text}"


@lru_cache(maxsize=16)
def _transformer(src, dst):
    from pyproj import Transformer
    return Transformer.from_crs(_spec(src), _spec(dst), always_xy=True)


def pixel_to_crs(transform, col: float, row: float) -> tuple:
    """Affine in rasterio order (a, b, c, d, e, f). Pixel centres, not corners."""
    a, b, c, d, e, f = transform[:6]
    x = a * (col + 0.5) + b * (row + 0.5) + c
    y = d * (col + 0.5) + e * (row + 0.5) + f
    return x, y


def crs_to_pixel(transform, x: float, y: float) -> tuple:
    a, b, c, d, e, f = transform[:6]
    det = a * e - b * d
    if abs(det) < 1e-12:
        raise ValueError("degenerate affine transform")
    dx, dy = x - c, y - f
    col = (e * dx - b * dy) / det - 0.5
    row = (-d * dx + a * dy) / det - 0.5
    return col, row


def crs_to_wgs84(crs, x: float, y: float) -> tuple:
    if crs == WGS84:
        return x, y
    lon, lat = _transformer(crs, WGS84).transform(x, y)
    return lat, lon


def wgs84_to_crs(crs, lat: float, lon: float) -> tuple:
    if crs == WGS84:
        return lon, lat
    return _transformer(WGS84, crs).transform(lon, lat)


def local_tmerc(anchor_lat: float, anchor_lon: float) -> str:
    """A metre-true projected CRS centred on a declared anchor."""
    return (f"+proj=tmerc +lat_0={anchor_lat} +lon_0={anchor_lon} +k=1 "
            "+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs")


def pixel_to_wgs84(transform, crs, col: float, row: float) -> tuple:
    x, y = pixel_to_crs(transform, col, row)
    return crs_to_wgs84(crs, x, y)


def wgs84_to_pixel(transform, crs, lat: float, lon: float) -> tuple:
    x, y = wgs84_to_crs(crs, lat, lon)
    return crs_to_pixel(transform, x, y)


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Exact distance on the WGS84 ellipsoid. Use this for reported error.

    haversine_m below assumes a sphere of mean radius, which is out by about
    0.2% at Sydney's latitude -- 21 cm over 100 m. That is a systematic scale
    bias sitting underneath a ground-truth noise floor of 0.5 m, and it would
    quietly shift every error in every table. pyproj is already a dependency,
    so there is no reason to accept it.
    """
    try:
        return abs(_geod().inv(lon1, lat1, lon2, lat2)[2])
    except Exception:
        return haversine_m(lat1, lon1, lat2, lon2)


@lru_cache(maxsize=1)
def _geod():
    from pyproj import Geod
    return Geod(ellps="WGS84")


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Spherical approximation. Kept for cheap comparisons, not for reporting."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _EARTH_R * math.asin(min(1.0, math.sqrt(h)))


def ne_offset_m(lat_ref: float, lon_ref: float, lat: float, lon: float) -> tuple:
    """North and east offset in metres. Signed, and ellipsoidal."""
    north = distance_m(lat_ref, lon_ref, lat, lon_ref) * (1 if lat >= lat_ref else -1)
    east = distance_m(lat_ref, lon_ref, lat_ref, lon) * (1 if lon >= lon_ref else -1)
    return north, east


def gsd_m_px(height_m: float, pixel_pitch_m: float, focal_m: float) -> float:
    """Equation (3) of the review: GSD = h * p / f."""
    if focal_m <= 0:
        raise ValueError("focal length must be positive")
    return height_m * pixel_pitch_m / focal_m


def scale_ratio(gsd_frame: float, gsd_reference: float) -> float:
    """rho -- the scale gap the matcher has to absorb."""
    if gsd_reference <= 0:
        raise ValueError("reference GSD must be positive")
    return gsd_frame / gsd_reference
