"""Data layer process.

Two inputs preprocessed independently, plus the vehicle state, published on
one socket. The map is preprocessed once and cached; the feed is preprocessed
every frame. Nothing in here knows that a processing layer exists.

    python -m geoanchor.data_layer                 run it
    python -m geoanchor.data_layer --build-map     build the store and exit
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

from .. import config as cfgmod
from .. import contracts as K
from .. import device
from ..bus import BindError, CommandServer, Publisher
from ..contracts import FramePacket, MapPacket, StatusPacket, to_dict
from ..logbus import LayerLog, resolve_run_dir
from .feed import FeedError, Preprocessor, open_feed
from .gpsin import GpsError, ReplayGps, VehicleState, open_gps
from .mapprep import MapPrepError, build_store, build_store_from_image

LAYER = "data"
MAP_REPUBLISH_S = 3.0
STATUS_S = 1.0


class DataLayer:
    def __init__(self, cfg: cfgmod.Config, run_dir: Path):
        self.cfg = cfg
        self.run_dir = run_dir
        self.running = True
        self.paused = False
        self.seq = 0
        self.state = VehicleState()
        self.board = device.detect()
        self.store = self.manifest = self.map_packet = None
        self.feed = self.gps = self.pre = None
        self._last_map_pub = 0.0
        self._last_status = 0.0
        self._rate_window: list = []
        self._dropped = 0

        try:
            self.pub = Publisher(cfg.get("bus.data_pub"))
        except BindError as exc:
            # Log without a publisher: the socket is exactly what failed, so
            # the code still has to reach the JSONL and the operator.
            self.log = LayerLog(LAYER, run_dir, None)
            self.log.device("DLDE-08", str(exc))
            self.log.close()
            raise SystemExit(3)
        self.log = LayerLog(LAYER, run_dir, self.pub)
        self.ctl = CommandServer(cfg.get("bus.data_ctl", "ipc:///tmp/geoanchor/data.ctl"))

    # -- setup -------------------------------------------------------------
    def start(self) -> None:
        self.log.step("DL-01", f"{self.board.model or self.board.arch}, {self.board.cores} cores",
                      run=self.run_dir.name)
        for note in self.board.notes:
            self.log.step("DL-01", note)

        problems = self.cfg.validate()
        if problems:
            for p in problems:
                self.log.error("DLE-01", p)
            raise SystemExit(2)
        self.log.step("DL-02", f"config {self.cfg.path}")
        self.log.step("DL-03", self.cfg.get("bus.data_pub"))

        self.build_map()
        self.open_feed()
        self.open_gps()

    def build_map(self) -> None:
        m = self.cfg.section("data_layer").get("map", {})
        source = self.cfg.resolve("data_layer.map.source")
        store_root = self.cfg.resolve("data_layer.map.store_dir", "stores")
        self.log.step("DL-04", f"{source}")

        def progress(i, n, kp):
            if i == 1 or i % 5 == 0 or i == n:
                self.publish_status(ready=False, note=f"preprocessing map {i}/{n} tiles")

        common = dict(tile_px=m.get("tile_px", 1024), overlap_px=m.get("overlap_px", 128),
                      max_keypoints=m.get("max_keypoints", 2048),
                      force=bool(m.get("rebuild", False)), log=self.log, progress=progress)
        try:
            if m.get("kind", "geotiff") == "anyvisloc":
                # The scene's reference JSON supplies the affine, so the map is
                # a plain PNG and rasterio is not involved.
                from . import anyvisloc as A
                scene = self.cfg.resolve("data_layer.map.scene_dir") or source
                georef = A.load_georeference(scene, m.get("mode", "satellite"),
                                             m.get("anchor"))
                self.log.step("DL-04", f"{scene} ({m.get('mode', 'satellite')} reference)")
                self.store, self.manifest, hit = build_store_from_image(
                    georef["map_path"], georef, store_root, m.get("method", "xfeat_mnn"), **common)
            else:
                self.store, self.manifest, hit = build_store(
                    source, store_root, m.get("method", "xfeat_mnn"), **common)
        except MapPrepError as exc:
            self.log.emit(exc.code, str(exc))
            raise SystemExit(2)

        mf = self.manifest
        self.map_packet = MapPacket(
            store_id=mf["store_id"], store_path=str(self.store.path), source_path=mf["source"],
            epsg=mf["epsg"], transform=mf["transform"], width=mf["width"], height=mf["height"],
            gsd_m_px=mf["gsd_m_px"], bounds_wgs84=mf["bounds_wgs84"], tile_px=mf["tile_px"],
            tile_grid=mf["tile_grid"], n_tiles=mf["n_tiles"], method=mf["method"],
            built_at=mf["built_at"], crs=mf.get("crs"),
            local_frame=bool(mf.get("local_frame", False)))
        self.publish_map()

    def open_feed(self) -> None:
        fc = dict(self.cfg.section("data_layer").get("feed", {}))
        for key in ("path", "sidecar", "scene_dir", "frames_dir"):
            if fc.get(key):
                resolved = self.cfg.resolve(f"data_layer.feed.{key}")
                if resolved:
                    fc[key] = str(resolved)
        try:
            self.feed = open_feed(fc)
        except FeedError as exc:
            self.log.emit(exc.code, str(exc))
            raise SystemExit(2)
        self.pre = Preprocessor(
            intrinsics=fc.get("intrinsics") or {},
            fallback_long_edge=fc.get("frame_px", 512),
            jpeg_quality=fc.get("jpeg_quality", 80))
        if not self.pre.fx_px:
            self.log.error("DLE-13", "no fx_px or focal_mm/pixel_pitch_um configured, so frames "
                                     "cannot be scaled to the reference GSD")
        self.log.step("DL-11", str(self.feed.describe()))

    def open_gps(self) -> None:
        gc = self.cfg.section("data_layer").get("gps", {})
        try:
            self.gps = open_gps(gc)
        except GpsError as exc:
            self.log.emit(exc.code, str(exc))
            self.gps = open_gps({"source": "none"})
            self.log.error("DLE-01", "continuing without actual GPS -- predicted fixes will be "
                                     "logged but cannot be scored")
        self.log.step("DL-15", str(self.gps.describe()))

    # -- publishing --------------------------------------------------------
    def publish_map(self) -> None:
        self.pub.send(K.T_MAP, to_dict(self.map_packet))
        self._last_map_pub = time.monotonic()
        self.log.throttled("DL-10", 30.0, f"{self.manifest['n_tiles']} tiles, "
                                          f"{self.manifest['n_keypoints']} reference keypoints")

    def publish_status(self, ready: bool = True, note: str = "") -> None:
        now = time.monotonic()
        cut = now - 10.0
        self._rate_window = [t for t in self._rate_window if t > cut]
        rate = len(self._rate_window) / 10.0 if self._rate_window else 0.0
        st = StatusPacket(
            layer=LAYER, t_unix=K.now_unix(), ready=ready, uptime_s=round(self.log.uptime(), 1),
            last_code=self.log.last_code, counts=self.log.tally(), rate_hz=round(rate, 2),
            config={
                "feed": self.feed.describe() if self.feed else None,
                "gps": self.gps.describe() if self.gps else None,
                "map": {"store_id": self.manifest["store_id"], "tiles": self.manifest["n_tiles"],
                        "gsd_m_px": self.manifest["gsd_m_px"], "method": self.manifest["method"]}
                if self.manifest else None,
                "paused": self.paused, "note": note, "dropped_frames": self._dropped,
                "device_errors": self.log.device_errors,
                "board": {"kind": self.board.kind, "model": self.board.model},
                "power_w": device.read_power_w(self.board), "temp_c": device.read_temp_c(),
            })
        self.pub.send(K.T_STATUS, to_dict(st))
        self._last_status = now

    # -- the loop ----------------------------------------------------------
    def run(self) -> int:
        self.start()
        ready_announced = False
        env = self.cfg.section("data_layer").get("envelope", {})
        agl_min, agl_max = env.get("agl_min_m", 50), env.get("agl_max_m", 100)
        stale_after = self.cfg.get("data_layer.gps.stale_after_s", 3.0)

        while self.running:
            self.handle_commands()
            if self.paused:
                self.publish_status(note="paused")
                time.sleep(0.2)
                continue

            try:
                frame, t_cap, meta = self.feed.read()
            except FeedError as exc:
                self.log.emit(exc.code, str(exc))
                break
            if frame is None:
                self.log.step("DL-19", "end of feed")
                break

            if isinstance(self.gps, ReplayGps):
                self.gps.offer(meta)
            self.gps.poll(self.state)
            if hasattr(self.gps, "link_lost") and self.gps.link_lost(self.state):
                self.log.throttled("DLDE-05", 10.0)
            if getattr(self.gps, "malformed", 0):
                self.log.throttled("DLE-10", 10.0, count=self.gps.malformed)
            if self.state.gps is not None and self.state.gps_age() > stale_after:
                self.log.throttled("DLE-11", 10.0, age_s=round(self.state.gps_age(), 1))

            alt = self.state.rel_alt_m if self.state.rel_alt_m is not None else meta.get("alt_agl_m")
            if alt is not None and not (agl_min <= alt <= agl_max):
                self.log.throttled("DLE-12", 10.0, altitude_m=round(float(alt), 1),
                                   envelope=f"{agl_min}-{agl_max}")

            try:
                _, jpeg, pm = self.pre.run(
                    frame, altitude_m=alt, map_gsd_m_px=self.manifest["gsd_m_px"],
                    # A dataset frame carries its own K, so its intrinsics beat
                    # whatever the config says about the flight camera.
                    fx_px=meta.get("fx_px"), pitch_deg=meta.get("pitch_deg"))
            except FeedError as exc:
                self.log.emit(exc.code, str(exc))
                continue
            if pm["note"]:
                self.log.throttled("DLE-13", 30.0, pm["note"])

            self.seq += 1
            if self.seq == 1:
                self.log.step("DL-12", f"{frame.shape[1]}x{frame.shape[0]}")
            self.log.throttled("DL-13", 30.0, f"scale {pm['scale']}", ms=round(pm["preprocess_ms"], 1))

            fp = FramePacket(
                seq=self.seq, t_capture_unix=t_cap, t_capture_mono=K.now_mono(),
                width=frame.shape[1], height=frame.shape[0], feed=self.feed.kind,
                altitude_m=float(alt) if alt is not None else None,
                roll_deg=self.state.roll_deg, pitch_deg=self.state.pitch_deg,
                yaw_deg=self.state.yaw_deg, preprocess_ms=round(pm["preprocess_ms"], 2))
            if self.pub.send(K.T_FRAME, to_dict(fp), jpeg):
                self.log.throttled("DL-14", 30.0, f"seq {self.seq}")
            else:
                self._dropped += 1
                self.log.throttled("DLE-09", 10.0,
                                   "the processing layer is not keeping up",
                                   dropped_total=self._dropped)
            self._rate_window.append(time.monotonic())

            if self.state.gps is not None:
                self.pub.send(K.T_GPS, to_dict(self.state.gps))
                if self.log.counts.get("DL-16", 0) == 0:
                    self.log.step("DL-16", f"{self.state.gps.lat:.6f}, {self.state.gps.lon:.6f}",
                                  source=self.state.gps.source)
                self.log.throttled("DL-17", 30.0)

            if not ready_announced and self.state.gps is not None:
                self.log.step("DL-18", "map, feed and GPS all live")
                ready_announced = True

            now = time.monotonic()
            if now - self._last_map_pub > MAP_REPUBLISH_S:
                self.publish_map()
            if now - self._last_status > STATUS_S:
                self.publish_status(ready=ready_announced)

        self.stop()
        return 0

    def handle_commands(self) -> None:
        for msg in self.ctl.poll():
            cmd = msg.get("cmd")
            if cmd == "stop":
                self.running = False
            elif cmd == "pause":
                self.paused = True
            elif cmd == "resume":
                self.paused = False
            elif cmd == "set" and "path" in msg:
                self.cfg.set(msg["path"], msg.get("value"))
                self.log.step("DL-02", f"{msg['path']} = {msg.get('value')}")
                if msg["path"].startswith("data_layer.feed"):
                    self.reopen_feed()
            elif cmd == "reopen_feed":
                if msg.get("feed"):
                    for k, v in msg["feed"].items():
                        self.cfg.set(f"data_layer.feed.{k}", v)
                self.reopen_feed()
            elif cmd == "rebuild_map":
                # A method change means new descriptors, which means a new
                # store. The store id hashes the method in, so switching back
                # to a method already built is a cache hit and costs nothing --
                # only a method never built here pays the preprocessing time.
                if msg.get("method"):
                    self.cfg.set("data_layer.map.method", msg["method"])
                force = bool(msg.get("force", False))
                self.cfg.set("data_layer.map.rebuild", force)
                self.build_map()
                self.cfg.set("data_layer.map.rebuild", False)

    def reopen_feed(self) -> None:
        try:
            self.feed.close()
        except Exception:
            pass
        self.seq = 0
        self.open_feed()

    def stop(self) -> None:
        for closer in (getattr(self, "feed", None), getattr(self, "gps", None)):
            try:
                closer and closer.close()
            except Exception:
                pass
        self.log.step("DL-20", f"{self.seq} frames published")
        self.publish_status(ready=False, note="stopped")
        time.sleep(0.2)
        self.log.close()
        self.pub.close()
        self.ctl.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="geoanchor.data_layer")
    ap.add_argument("--config", default=None)
    ap.add_argument("--build-map", action="store_true", help="build the feature store and exit")
    ap.add_argument("--force", action="store_true", help="rebuild even on a cache hit")
    args = ap.parse_args(argv)

    cfg = cfgmod.load(args.config)
    if args.force:
        cfg.set("data_layer.map.rebuild", True)
    run_dir = resolve_run_dir(cfg.resolve("session.runs_dir", "runs"),
                              cfg.get("session.tag", ""))

    layer = DataLayer(cfg, run_dir)
    signal.signal(signal.SIGINT, lambda *_: setattr(layer, "running", False))
    signal.signal(signal.SIGTERM, lambda *_: setattr(layer, "running", False))

    if args.build_map:
        layer.log.step("DL-01", "map build only")
        layer.build_map()
        layer.log.step("DL-20", "map built, exiting")
        return 0
    return layer.run()


if __name__ == "__main__":
    raise SystemExit(main())
