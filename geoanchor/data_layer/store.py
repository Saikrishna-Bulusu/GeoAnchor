"""The reference feature store: the map, preprocessed once, reused forever.

This is the piece that decides whether the system flies.

Measured on a Pi 5 on 2 Sept 2026, XFeat sparse cost 1017 ms per fix end to
end, four times over the 250 ms budget. 856 ms of that was running the
detector over the reference tile, which is roughly ten times the pixels of the
frame. A deployed aircraft matches against a STATIC georeferenced map, so
those descriptors can be computed on the ground, once, and the aircraft never
pays for them. With the tile precomputed the same matcher came in at 202 ms
and fits.

So the professor's rule -- preprocess each new map once and save it in a form
that can be reused -- is not housekeeping. It is the difference between a
pipeline that meets EKF3's delay budget and one that does not.

One condition, and it is load-bearing: the stored descriptors are only valid
while the reference stays north-up and unwarped. Stage 02 rectifies the FRAME
using vehicle attitude. If the tile ever has to be warped per frame, every
number above is void.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

STORE_VERSION = 2


@dataclass
class Tile:
    row: int
    col: int
    x0: int           # offset in map pixels of this tile's top-left corner
    y0: int
    width: int
    height: int
    kpts: np.ndarray  # (N, 2) in MAP pixel coordinates, already offset
    desc: np.ndarray
    scores: np.ndarray = None

    def __len__(self) -> int:
        return len(self.kpts)


def store_id(source: Path, method: str, tile_px: int, overlap_px: int,
             max_keypoints: int) -> str:
    """Cache key. Hashes the file's CONTENT, not its path or mtime.

    A path is not identity -- the same tile gets copied between machines and
    re-exported under new names. An mtime is not identity either: rsync and
    git both rewrite it without changing a byte. Content is.
    """
    h = hashlib.sha256()
    h.update(f"v{STORE_VERSION}|{method}|{tile_px}|{overlap_px}|{max_keypoints}".encode())
    with open(source, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


class FeatureStore:
    def __init__(self, path: Path | str):
        self.path = Path(path)
        self.manifest: dict = {}
        self._tiles: dict = {}
        self._merge_cache: tuple = None

    # -- reading -----------------------------------------------------------
    @property
    def manifest_path(self) -> Path:
        return self.path / "manifest.json"

    def exists(self) -> bool:
        return self.manifest_path.exists()

    def load(self) -> dict:
        """Raises ValueError if the store is unusable. The caller turns that
        into DLE-05 or DLE-06 and rebuilds rather than limping on."""
        if not self.exists():
            raise ValueError(f"no manifest at {self.manifest_path}")
        try:
            self.manifest = json.loads(self.manifest_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"manifest unreadable: {exc}") from exc
        if self.manifest.get("store_version") != STORE_VERSION:
            raise ValueError(
                f"store version {self.manifest.get('store_version')} != {STORE_VERSION}")
        missing = [t for t in self.manifest["tiles"]
                   if not (self.path / "tiles" / t["file"]).exists()]
        if missing:
            raise ValueError(f"{len(missing)} tile files missing from the store")
        return self.manifest

    def tile(self, row: int, col: int) -> Tile:
        key = (row, col)
        if key not in self._tiles:
            meta = next((t for t in self.manifest["tiles"]
                         if t["row"] == row and t["col"] == col), None)
            if meta is None:
                raise KeyError(f"tile {row},{col} is not in this store")
            with np.load(self.path / "tiles" / meta["file"]) as z:
                self._tiles[key] = Tile(
                    row=row, col=col, x0=meta["x0"], y0=meta["y0"],
                    width=meta["width"], height=meta["height"],
                    kpts=z["kpts"], desc=z["desc"],
                    scores=z["scores"] if "scores" in z else None,
                )
        return self._tiles[key]

    def tiles_covering(self, col: float, row_px: float, radius_px: float) -> list:
        """Which tiles could contain a point within radius of a map pixel.

        This is the search-space reduction: with a prior from the last fix,
        only a handful of tiles are candidates rather than the whole map.
        """
        out = []
        for t in self.manifest["tiles"]:
            x0, y0 = t["x0"], t["y0"]
            x1, y1 = x0 + t["width"], y0 + t["height"]
            nearest_x = min(max(col, x0), x1)
            nearest_y = min(max(row_px, y0), y1)
            if (nearest_x - col) ** 2 + (nearest_y - row_px) ** 2 <= radius_px ** 2:
                out.append((t["row"], t["col"]))
        return out

    def all_tile_keys(self) -> list:
        return [(t["row"], t["col"]) for t in self.manifest["tiles"]]

    def merged(self, keys=None):
        """Keypoints and descriptors of several tiles as one set.

        Tiles overlap, so the same physical corner appears in two tiles with
        two descriptors. That is harmless for matching -- both are valid
        observations of the same point and RANSAC sees a consistent model --
        but it does inflate the keypoint count, so overlap is kept modest.
        """
        keys = tuple(sorted(keys or self.all_tile_keys()))
        # The selected tile set changes only when the vehicle crosses a tile
        # boundary, so restacking 13 MB of descriptors on every frame is pure
        # waste. Caching the last set removes it from the latency budget.
        if self._merge_cache and self._merge_cache[0] == keys:
            return self._merge_cache[1]
        kp, ds, sc = [], [], []
        for r, c in keys:
            t = self.tile(r, c)
            if len(t) == 0:
                continue
            kp.append(t.kpts); ds.append(t.desc)
            sc.append(t.scores if t.scores is not None else np.zeros(len(t), np.float32))
        if not kp:
            out = (np.zeros((0, 2), np.float32), np.zeros((0, 1), np.float32), np.zeros(0, np.float32))
        else:
            out = (np.vstack(kp), np.vstack(ds), np.concatenate(sc))
        self._merge_cache = (keys, out)
        return out

    def unload(self) -> None:
        self._tiles.clear()
        self._merge_cache = None

    # -- writing -----------------------------------------------------------
    def begin_write(self) -> Path:
        tmp = self.path.with_name(self.path.name + ".partial")
        if tmp.exists():
            shutil.rmtree(tmp)
        (tmp / "tiles").mkdir(parents=True)
        return tmp

    def write_tile(self, tmp: Path, tile: Tile) -> dict:
        name = f"t_{tile.row:03d}_{tile.col:03d}.npz"
        payload = {"kpts": tile.kpts.astype(np.float32), "desc": tile.desc}
        if tile.scores is not None:
            payload["scores"] = tile.scores.astype(np.float32)
        np.savez_compressed(tmp / "tiles" / name, **payload)
        return {"row": tile.row, "col": tile.col, "x0": tile.x0, "y0": tile.y0,
                "width": tile.width, "height": tile.height,
                "n_keypoints": int(len(tile)), "file": name}

    def commit(self, tmp: Path, manifest: dict) -> None:
        """Write the manifest last, then swap the directory into place.

        A store is valid only once its manifest exists, so an interrupted
        build leaves a .partial directory that the next run discards. Without
        this, a build killed by Ctrl-C leaves a half-populated store that
        loads without complaint and silently drops a third of the map.
        """
        manifest["store_version"] = STORE_VERSION
        manifest["built_at"] = time.time()
        (tmp / "manifest.json").write_text(json.dumps(manifest, indent=2))
        if self.path.exists():
            shutil.rmtree(self.path)
        tmp.rename(self.path)
        self.manifest = manifest

    def size_bytes(self) -> int:
        return sum(f.stat().st_size for f in self.path.rglob("*") if f.is_file())
