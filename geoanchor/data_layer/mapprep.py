"""Turn a georeferenced image into a reusable feature store. Once per map.

rasterio is imported lazily and only here. Reading a GeoTIFF is a ground-side
job; the flight-time path needs nothing but the affine transform and the EPSG
code that this step already wrote into the manifest. Keeping GDAL out of the
runtime import graph is what makes the Jetson install a handful of pure wheels
instead of a build.
"""
from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

from .. import codes as C
from .. import methods as M
from .store import FeatureStore, Tile, store_id

# At 50-100 m AGL a useful reference sits roughly between 5 cm and 2 m per
# pixel. Finer than 5 cm and the scale gap to the frame inverts; coarser than
# 2 m and there is not enough structure left to match at this altitude.
GSD_MIN_M, GSD_MAX_M = 0.02, 2.0


class MapPrepError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def read_georeference(source: Path) -> dict:
    try:
        import rasterio
    except ImportError as exc:
        raise MapPrepError(
            "DLE-07",
            "rasterio is not installed, so a new GeoTIFF cannot be ingested here. "
            "Either `pip install rasterio`, or build the store on a machine that "
            "has it and copy the store directory across -- the runtime does not "
            "need rasterio, only the store.",
        ) from exc

    if not source.exists():
        raise MapPrepError("DLE-02", f"map file not found: {source}")

    with rasterio.open(source) as ds:
        if ds.crs is None or ds.transform is None or ds.transform.is_identity:
            raise MapPrepError(
                "DLE-03",
                f"{source.name} has no CRS or no affine transform. It is a picture, "
                "not a map, and nothing downstream can turn a pixel into a coordinate.",
            )
        epsg = ds.crs.to_epsg()
        if epsg is None:
            raise MapPrepError("DLE-03", f"CRS {ds.crs} has no EPSG code")
        t = ds.transform
        gsd = float(abs(t.a))
        if not (GSD_MIN_M <= gsd <= GSD_MAX_M):
            raise MapPrepError(
                "DLE-04",
                f"reference GSD {gsd:.4f} m/px is outside {GSD_MIN_M}-{GSD_MAX_M} m/px, "
                "which is the usable band for a 50-100 m AGL frame",
            )
        from rasterio.warp import transform_bounds
        bounds = list(transform_bounds(ds.crs, "EPSG:4326", *ds.bounds))
        return {
            "epsg": int(epsg),
            "transform": [t.a, t.b, t.c, t.d, t.e, t.f],
            "width": int(ds.width), "height": int(ds.height),
            "gsd_m_px": gsd, "bounds_wgs84": bounds,
            "bands": int(ds.count), "source": str(source),
        }


def _read_window(ds, x0: int, y0: int, w: int, h: int) -> np.ndarray:
    """Return a BGR uint8 window. Handles 1, 3 and 4 band sources."""
    import rasterio
    from rasterio.windows import Window
    n = min(ds.count, 3)
    arr = ds.read(list(range(1, n + 1)), window=Window(x0, y0, w, h))
    arr = np.transpose(arr, (1, 2, 0))
    if arr.dtype != np.uint8:
        finite = arr[np.isfinite(arr)] if arr.size else arr
        hi = float(finite.max()) if finite.size else 1.0
        arr = (arr.astype(np.float32) / (hi or 1.0) * 255.0).clip(0, 255).astype(np.uint8)
    if arr.shape[2] == 1:
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    else:
        arr = arr[:, :, ::-1].copy()          # rasterio gives RGB, OpenCV wants BGR
    return arr


def plan_tiles(width: int, height: int, tile_px: int, overlap_px: int) -> list:
    """Overlapping grid. Overlap exists so a frame landing on a tile seam
    still finds a tile that contains the whole of it."""
    stride = max(1, tile_px - overlap_px)
    xs = list(range(0, max(1, width - overlap_px), stride))
    ys = list(range(0, max(1, height - overlap_px), stride))
    plan = []
    for r, y0 in enumerate(ys):
        for c, x0 in enumerate(xs):
            w = min(tile_px, width - x0)
            h = min(tile_px, height - y0)
            if w < 32 or h < 32:
                continue
            plan.append((r, c, x0, y0, w, h))
    return plan


def build_store(source: Path, store_root: Path, method_name: str, tile_px: int = 1024,
                overlap_px: int = 128, max_keypoints: int = 2048,
                force: bool = False, log=None, progress=None) -> tuple:
    """Build (or reuse) the store. Returns (FeatureStore, manifest, cache_hit)."""
    source = Path(source)
    sid = store_id(source, method_name, tile_px, overlap_px, max_keypoints)
    store = FeatureStore(Path(store_root) / f"{source.stem}__{method_name}__{sid}")

    if store.exists() and not force:
        try:
            manifest = store.load()
            if log:
                log.step("DL-09", f"reusing {store.path.name}",
                         tiles=manifest["n_tiles"], keypoints=manifest["n_keypoints"])
            return store, manifest, True
        except ValueError as exc:
            code = "DLE-05" if "version" in str(exc) else "DLE-06"
            if log:
                log.error(code, f"{exc} -- rebuilding")

    geo = read_georeference(source)
    if log:
        log.step("DL-05", f"{source.name}: EPSG:{geo['epsg']}, {geo['gsd_m_px']:.4f} m/px",
                 size=f"{geo['width']}x{geo['height']}")

    method = M.build(method_name, max_keypoints=max_keypoints)
    ok, why = method.available()
    if not ok:
        raise MapPrepError("DLE-01", f"method '{method_name}' unavailable: {why}")

    plan = plan_tiles(geo["width"], geo["height"], tile_px, overlap_px)
    if log:
        log.step("DL-06", f"{len(plan)} tiles of {tile_px} px, {overlap_px} px overlap")

    import rasterio
    tmp = store.begin_write()
    entries, total_kp, t_start = [], 0, time.perf_counter()
    try:
        with rasterio.open(source) as ds:
            entries, total_kp = _tile_loop(
                plan, method, store, tmp, lambda x0, y0, w, h: _read_window(ds, x0, y0, w, h),
                progress)
    except MemoryError as exc:
        raise MapPrepError("DLDE-07", f"out of memory tiling {source.name}: {exc}") from exc
    except OSError as exc:
        if getattr(exc, "errno", None) == 28:
            raise MapPrepError("DLDE-06", f"disk full writing the feature store: {exc}") from exc
        raise

    # A downscaled PNG of the reference, written once, so the dashboard can
    # draw a basemap with no network and no GDAL. An offline aircraft and an
    # offline laptop both need this; OSM tiles are not available on either.
    try:
        _write_preview(source, tmp, geo)
    except Exception as exc:                     # a missing preview is cosmetic
        if log:
            log.error("DLE-06", f"map preview not written: {exc}")

    elapsed = time.perf_counter() - t_start
    manifest = {
        "store_id": sid, "source": str(source), "source_name": source.name,
        "method": method_name, "max_keypoints": max_keypoints,
        "tile_px": tile_px, "overlap_px": overlap_px,
        "n_tiles": len(entries), "n_keypoints": total_kp,
        "tile_grid": [max((e["row"] for e in entries), default=-1) + 1,
                      max((e["col"] for e in entries), default=-1) + 1],
        "build_seconds": round(elapsed, 2),
        "descriptor_dim": int(entries and np.load(tmp / "tiles" / entries[0]["file"])["desc"].shape[1]),
        "preview": "preview.png" if (tmp / "preview.png").exists() else None,
        "tiles": entries,
        **{k: geo[k] for k in ("epsg", "transform", "width", "height", "gsd_m_px", "bounds_wgs84")},
    }
    store.commit(tmp, manifest)
    if log:
        log.step("DL-08", f"{store.path.name}",
                 tiles=len(entries), keypoints=total_kp,
                 mb=round(store.size_bytes() / 1e6, 1), seconds=round(elapsed, 1))
    return store, manifest, False


def _tile_loop(plan, method, store, tmp, read_window, progress):
    entries, total_kp = [], 0
    for i, (r, c, x0, y0, w, h) in enumerate(plan):
        window = read_window(x0, y0, w, h)
        f = method.detect(window)
        kp = f.kpts.copy()
        if len(kp):
            kp[:, 0] += x0      # tile pixels -> map pixels, stored once
            kp[:, 1] += y0      # so the runtime never has to remember an offset
        entries.append(store.write_tile(tmp, Tile(
            row=r, col=c, x0=x0, y0=y0, width=w, height=h,
            kpts=kp, desc=f.desc, scores=f.scores)))
        total_kp += len(kp)
        if progress:
            progress(i + 1, len(plan), total_kp)
    return entries, total_kp


def build_store_from_image(image_path: Path, georef: dict, store_root: Path, method_name: str,
                           tile_px: int = 1024, overlap_px: int = 128, max_keypoints: int = 2048,
                           force: bool = False, log=None, progress=None) -> tuple:
    """Build a store from a plain raster plus an explicit georeference.

    The AnyVisLoc reference maps are PNGs with their affine in a sidecar JSON
    and no CRS at all, because the dataset's ground truth is scene-local rather
    than geodetic. Requiring a GeoTIFF here would mean writing one just to read
    it back, so the affine is passed in instead and rasterio stays out of the
    path entirely.
    """
    image_path = Path(image_path)
    if not image_path.exists():
        raise MapPrepError("DLE-02", f"map image not found: {image_path}")

    sid = store_id(image_path, method_name, tile_px, overlap_px, max_keypoints)
    store = FeatureStore(Path(store_root) / f"{image_path.stem}__{method_name}__{sid}")
    if store.exists() and not force:
        try:
            manifest = store.load()
            if log:
                log.step("DL-09", f"reusing {store.path.name}",
                         tiles=manifest["n_tiles"], keypoints=manifest["n_keypoints"])
            return store, manifest, True
        except ValueError as exc:
            if log:
                log.error("DLE-05" if "version" in str(exc) else "DLE-06", f"{exc} -- rebuilding")

    img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if img is None:
        raise MapPrepError("DLE-02", f"cannot decode {image_path}")
    h_img, w_img = img.shape[:2]
    if (w_img, h_img) != (georef["width"], georef["height"]):
        raise MapPrepError(
            "DLE-03",
            f"{image_path.name} is {w_img}x{h_img} but the georeference says "
            f"{georef['width']}x{georef['height']}. Every position would be scaled wrong.")

    method = M.build(method_name, max_keypoints=max_keypoints)
    ok, why = method.available()
    if not ok:
        raise MapPrepError("DLE-01", f"method '{method_name}' unavailable: {why}")

    plan = plan_tiles(w_img, h_img, tile_px, overlap_px)
    if log:
        log.step("DL-05", f"{image_path.name}: {georef['gsd_m_px']:.4f} m/px",
                 size=f"{w_img}x{h_img}", frame="local" if georef.get("local_frame") else "geodetic")
        log.step("DL-06", f"{len(plan)} tiles of {tile_px} px, {overlap_px} px overlap")

    tmp = store.begin_write()
    t_start = time.perf_counter()
    try:
        entries, total_kp = _tile_loop(
            plan, method, store, tmp,
            lambda x0, y0, w, h: img[y0:y0 + h, x0:x0 + w], progress)
    except MemoryError as exc:
        raise MapPrepError("DLDE-07", f"out of memory tiling {image_path.name}: {exc}") from exc

    k = 1400 / max(w_img, h_img)
    if k < 1.0:
        cv2.imwrite(str(tmp / "preview.png"),
                    cv2.resize(img, (int(w_img * k), int(h_img * k)), interpolation=cv2.INTER_AREA))
    else:
        cv2.imwrite(str(tmp / "preview.png"), img)

    elapsed = time.perf_counter() - t_start
    manifest = {
        "store_id": sid, "source": str(image_path), "source_name": image_path.name,
        "method": method_name, "max_keypoints": max_keypoints,
        "tile_px": tile_px, "overlap_px": overlap_px,
        "n_tiles": len(entries), "n_keypoints": total_kp,
        "tile_grid": [max((e["row"] for e in entries), default=-1) + 1,
                      max((e["col"] for e in entries), default=-1) + 1],
        "build_seconds": round(elapsed, 2),
        "descriptor_dim": int(entries and np.load(tmp / "tiles" / entries[0]["file"])["desc"].shape[1]),
        "preview": "preview.png", "tiles": entries,
        **{k2: georef[k2] for k2 in ("epsg", "transform", "width", "height", "gsd_m_px",
                                     "bounds_wgs84")},
        "crs": georef.get("crs"),
        "local_frame": bool(georef.get("local_frame", False)),
        "local_frame_note": georef.get("local_frame_note", ""),
    }
    store.commit(tmp, manifest)
    if log:
        log.step("DL-08", f"{store.path.name}", tiles=len(entries), keypoints=total_kp,
                 mb=round(store.size_bytes() / 1e6, 1), seconds=round(elapsed, 1))
    return store, manifest, False


def _write_preview(source: Path, tmp: Path, geo: dict, long_edge: int = 1400) -> None:
    import rasterio
    with rasterio.open(source) as ds:
        k = long_edge / max(ds.width, ds.height)
        w, h = max(1, int(ds.width * k)), max(1, int(ds.height * k))
        n = min(ds.count, 3)
        arr = ds.read(list(range(1, n + 1)), out_shape=(n, h, w))
    arr = np.transpose(arr, (1, 2, 0))
    if arr.dtype != np.uint8:
        hi = float(arr.max()) or 1.0
        arr = (arr.astype(np.float32) / hi * 255).clip(0, 255).astype(np.uint8)
    bgr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR) if arr.shape[2] == 1 else arr[:, :, ::-1]
    cv2.imwrite(str(tmp / "preview.png"), bgr)
