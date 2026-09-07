"""Processing layer process.

Subscribes to the data layer, produces a Predicted GPS, publishes it. Knows
nothing about the flight controller, the dashboard or the export.

Two properties are deliberate. It never blocks waiting for the data layer: a
missing map is PLE-01 and a retry, not a stall. And it always matches the
NEWEST frame, dropping any backlog, because at a 250 ms budget working through
stale frames means falling further behind on every iteration.

    python -m geoanchor.processing_layer
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from .. import config as cfgmod
from .. import contracts as K
from .. import device
from .. import methods as M
from ..bus import BindError, CommandServer, Publisher, Subscriber
from ..contracts import FixPacket, StatusPacket, to_dict
from ..data_layer.store import FeatureStore
from ..logbus import LayerLog, resolve_run_dir
from . import covariance as cov
from .rectify import Rectifier
from .solve import select_tiles, solve

LAYER = "processing"
STATUS_S = 1.0


class ProcessingLayer:
    def __init__(self, cfg: cfgmod.Config, run_dir: Path):
        self.cfg = cfg
        self.run_dir = run_dir
        self.running = True
        self.paused = False
        self.board = device.detect()
        self.store = self.manifest = None
        self.map_id = None
        self.method = None
        self.method_name = None
        self.prior = None            # (lat, lon, t_unix) of the last accepted fix
        self.n_fix = self.n_accept = 0
        self._last_status = 0.0
        self._rate_window: list = []
        self._skipped = 0
        self._last_rx = time.monotonic()
        self._feed_ended = False
        self._cold_sweeps = 0
        self._hot_since = None

        try:
            self.pub = Publisher(cfg.get("bus.processing_pub"))
        except BindError as exc:
            # Log without a publisher: the socket is exactly what failed, so
            # the code still has to reach the JSONL and the operator.
            self.log = LayerLog(LAYER, run_dir, None)
            self.log.device("PLDE-06", str(exc))
            self.log.close()
            raise SystemExit(3)
        self.log = LayerLog(LAYER, run_dir, self.pub)
        self.ctl = CommandServer(cfg.get("bus.processing_ctl", "ipc:///tmp/geoanchor/processing.ctl"))
        # T_LOG is subscribed for exactly one thing -- hearing the data layer say
        # its feed is finished. See note_data_log; it is not a liveness signal.
        self.sub = Subscriber([cfg.get("bus.data_pub")], [K.T_MAP, K.T_FRAME, K.T_LOG])
        self.rect = Rectifier(enabled=bool(cfg.get("processing_layer.rectify", True)))
        self.cov = cov.build(cfg.section("processing_layer").get("covariance", {}))

    # -- setup -------------------------------------------------------------
    def start(self) -> None:
        self.log.step("PL-01", f"{self.board.model or self.board.arch}, {self.board.cores} cores",
                      run=self.run_dir.name)
        problems = self.cfg.validate()
        if problems:
            for p in problems:
                self.log.error("PLE-02", p)
            raise SystemExit(2)
        self.log.step("PL-02", f"config {self.cfg.path}")
        self.log.step("PL-03", self.cfg.get("bus.processing_pub"))
        self.log.step("PL-04", ", ".join(sorted(M.REGISTRY)))
        self.load_method(self.cfg.get("processing_layer.method", "xfeat_mnn"))
        self.log.step("PL-06", self.cfg.get("bus.data_pub"))

    def load_method(self, name: str) -> bool:
        try:
            m = M.build(name, max_keypoints=self.cfg.get("processing_layer.max_keypoints", 4096),
                        threads=self.cfg.get("processing_layer.threads", 0))
        except KeyError as exc:
            self.log.error("PLE-02", str(exc))
            return False
        ok, why = m.available()
        if not ok:
            self.log.error("PLE-03", f"{name}: {why}")
            return False
        t0 = time.perf_counter()
        try:
            m.detect(np.zeros((64, 64, 3), np.uint8))     # force the weight load now
        except Exception as exc:
            self.log.error("PLE-03", f"{name} failed to initialise: {exc}")
            return False
        self.method, self.method_name = m, name
        self.log.step("PL-05", name, warmup_ms=round((time.perf_counter() - t0) * 1000, 1))
        accel = str(self.cfg.get("processing_layer.accelerator", "cpu")).lower()
        if accel not in ("", "cpu", "none"):
            # No accelerator backend exists: the pipeline is CUDA-free by
            # design so that one binary covers every board. Asking for one has
            # to be answered, not ignored.
            self.log.device("PLDE-04", f"'{accel}' requested; this build is CPU-only by design")
        return True

    def attach_map(self, header: dict) -> None:
        if self.map_id == header["store_id"] and self.store is not None:
            return
        store = FeatureStore(header["store_path"])
        try:
            manifest = store.load()
        except ValueError as exc:
            self.log.error("PLDE-05", f"{header['store_path']}: {exc}")
            return
        if manifest["method"] != self.method_name:
            self.log.error("PLE-14", f"store method '{manifest['method']}' vs layer method "
                                     f"'{self.method_name}'")
            # The store holds descriptors from ONE extractor. Matching XFeat
            # against ORB descriptors produces confident nonsense, so this is a
            # hard stop rather than a warning.
            self.log.error("PLE-02",
                           f"store was built with '{manifest['method']}' but this layer runs "
                           f"'{self.method_name}'. Rebuild the store or switch method.")
            self.store = None
            return
        self.store, self.manifest, self.map_id = store, manifest, header["store_id"]
        self.log.step("PL-07", f"{manifest['n_tiles']} tiles, {manifest['n_keypoints']} keypoints, "
                               f"{manifest['gsd_m_px']:.4f} m/px")
        self.log.step("PL-08", "ready")

    # A finite feed ends: 60 frames of replay, or a file that runs out. From
    # that moment silence is the expected state rather than a broken link, but
    # ZeroMQ cannot tell the two apart -- so without this the watchdog reports
    # "cannot connect to the data layer endpoint" every 30 s for as long as the
    # process is left up, which on a replay run is nearly the whole session and
    # buries any real device error under a rising count of false ones.
    #
    # The data layer already announces it on the bus. This listens rather than
    # inferring, because inferring is what got it wrong in the first place.
    def note_data_log(self, row: dict) -> None:
        if self._feed_ended or row.get("layer") != "data":
            return
        if row.get("code") in ("DL-19", "DL-20"):
            self._feed_ended = True
            # Standing down is not the same as never having faulted, but a
            # finished feed is not a fault at all, so the code is cleared and
            # the layer card goes back to green.
            self.log.clear_device_error("PLDE-01")
            self.log.step("PL-19", f"data layer reported {row.get('code')} -- "
                                   "link watchdog stood down until frames resume")

    # -- the loop ----------------------------------------------------------
    def run(self) -> int:
        self.start()
        while self.running:
            self.handle_commands()
            msgs, dropped = self.sub.drain(200, keep_latest_of=[K.T_FRAME])
            # Liveness is a question about frames, not about chatter. Counting
            # log packets here would let a data layer that is still talking but
            # has stopped publishing frames look healthy, which is the stall
            # this watchdog exists to catch.
            if any(topic in (K.T_FRAME, K.T_MAP) for topic, _, _ in msgs):
                self._last_rx = time.monotonic()
                if self._feed_ended:
                    # A looping feed came round, or the data layer was
                    # restarted. Either way the stand-down no longer applies.
                    self._feed_ended = False
                    self.log.step("PL-19", "frames resumed -- link watchdog armed again")
            elif not self._feed_ended and time.monotonic() - self._last_rx > 15.0:
                # ZeroMQ connect() never fails loudly -- a wrong endpoint or a
                # dead publisher looks exactly like an idle one. Silence past
                # the point where a running data layer would have said
                # something is the only signal available.
                self.log.throttled("PLDE-01", 30.0,
                                   f"nothing from {self.cfg.get('bus.data_pub')} for "
                                   f"{time.monotonic() - self._last_rx:.0f}s")
            self._skipped += dropped
            if dropped:
                self.log.throttled("PLE-10", 10.0, f"{dropped} stale frames dropped",
                                   total_skipped=self._skipped)
            for topic, header, payload in msgs:
                if topic == K.T_LOG:
                    self.note_data_log(header)
                elif topic == K.T_MAP:
                    self.attach_map(header)
                elif topic == K.T_FRAME and not self.paused:
                    try:
                        self.on_frame(header, payload)
                    except MemoryError:
                        self.log.throttled("PLDE-02", 10.0, "out of memory during inference")
                    except Exception as exc:
                        # One bad frame must not end the layer. A descriptor
                        # mismatch, a corrupt JPEG or an OpenCV assertion is a
                        # skipped frame, not a dead process -- the whole point
                        # of running the three layers separately is that a
                        # fault stays inside the layer that caused it.
                        self.log.throttled("PLE-13", 5.0,
                                           f"{type(exc).__name__}: {exc}", seq=header.get("seq"))
            now = time.monotonic()
            if now - self._last_status > STATUS_S:
                self.publish_status()
        self.stop()
        return 0

    def on_frame(self, header: dict, payload: bytes) -> None:
        if self.store is None:
            self.log.throttled("PLE-01", 5.0, "waiting for a map packet")
            return
        if self.method is None:
            self.log.throttled("PLE-03", 5.0, "no usable method loaded")
            return

        t_start = time.perf_counter()
        frame = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.log.error("PLE-04", "frame did not decode")
            return
        decode_ms = (time.perf_counter() - t_start) * 1000.0
        self.log.throttled("PL-09", 30.0, f"seq {header['seq']}")

        pl = self.cfg.section("processing_layer")
        env = self.cfg.section("data_layer").get("envelope", {})
        alt = header.get("altitude_m")
        if alt is not None and not (env.get("agl_min_m", 50) <= alt <= env.get("agl_max_m", 100)):
            self.log.throttled("PLE-11", 10.0, altitude_m=round(alt, 1))

        t0 = time.perf_counter()
        rect, rinfo = self.rect.run(frame, header.get("yaw_deg"),
                                    header.get("roll_deg"), header.get("pitch_deg"))
        rect_ms = (time.perf_counter() - t0) * 1000.0
        if rinfo["applied"]:
            self.log.throttled("PL-10", 30.0, f"yaw {header.get('yaw_deg')}")
        elif rinfo["note"]:
            self.log.throttled("PLE-12", 30.0, rinfo["note"])

        search = pl.get("search", {})
        prior_lat = prior_lon = None
        if search.get("use_prior", True) and self.prior:
            # A PRIOR HAS A SHELF LIFE. It bounds where the vehicle can be only
            # because the vehicle was there recently; the timestamp was being
            # unpacked and thrown away, so a prior stayed authoritative forever.
            # That is the failure mode that bites hardest when matching is
            # already struggling: one accepted fix, then a run of failures while
            # the aircraft keeps flying, and every later frame is searched
            # against a position it has long left. Nothing recovers, because
            # only an accepted fix refreshes the prior and the prior is why they
            # stop being accepted.
            #
            # The bound is geometric: the prior is worth trusting while the
            # vehicle cannot yet have left the circle being searched, i.e. for
            # about prior_radius_m / max ground speed. 100 m at 20 m/s is 5 s.
            max_age = float(search.get("prior_max_age_s", 5.0) or 0.0)
            age = K.now_unix() - self.prior[2]
            if max_age > 0 and age > max_age:
                self.prior = None
                self.log.throttled("PLE-16", 10.0,
                                   f"prior is {age:.1f}s old (max {max_age:.1f}s)",
                                   dropped=True)
            else:
                prior_lat, prior_lon, _ = self.prior

        def _prior_missed(plat, plon, col, row):
            # Drop it. Keeping a prior that covers no tile means consulting the
            # same wrong position on every subsequent frame, which is how the
            # solver latches out of a map it is still flying over.
            self.prior = None
            self.log.throttled("PLE-15", 10.0,
                               f"prior {plat:.6f}, {plon:.6f} maps to pixel "
                               f"({col:.0f}, {row:.0f}), which covers no tile",
                               dropped=True)

        keys = select_tiles(self.store, self.manifest, prior_lat, prior_lon,
                            radius_m=search.get("prior_radius_m", 150.0),
                            footprint_px=max(rect.shape[:2]),
                            cold_start_max_tiles=search.get("cold_start_max_tiles", 0),
                            cold_start_offset=self._cold_sweeps,
                            on_prior_miss=_prior_missed)
        if prior_lat is None or self.prior is None:
            # This frame searched cold -- either there was no prior, or the
            # prior missed and was dropped. Either way the next cold frame
            # starts where this one left off rather than repeating the window.
            self._cold_sweeps += 1
        self.log.throttled("PL-11", 30.0, f"{len(keys)}/{self.manifest['n_tiles']} tiles",
                           prior="yes" if prior_lat else "cold")

        res = solve(self.method, rect, self.store, self.manifest, keys,
                    min_keypoints=pl.get("min_keypoints", 30),
                    min_matches=pl.get("min_matches", 12),
                    inlier_gate=pl.get("inlier_gate", 25),
                    ransac_reproj_px=pl.get("ransac_reproj_px", 3.0),
                    tile_ransac=pl.get("tile_ransac", True),
                    frame_centre=rinfo["centre"])
        res.stage_ms["decode"] = decode_ms
        res.stage_ms["rectify"] = rect_ms
        if res.keypoints:
            self.log.throttled("PL-12", 30.0, f"{res.keypoints} keypoints")
        if res.matches:
            self.log.throttled("PL-13", 30.0, f"{res.matches} matches")
        if res.inliers:
            self.log.throttled("PL-14", 30.0, f"{res.inliers} inliers",
                               reproj_px=round(res.reproj_err_px, 2) if res.reproj_err_px else None)
        if res.lat is not None:
            self.log.throttled("PL-15", 30.0, f"{res.lat:.6f}, {res.lon:.6f}")

        sigma = None
        if res.lat is not None:
            sigma = self.cov.sigma_m(cov.features_from(res, alt))
            self.log.throttled("PL-16", 30.0, f"sigma {sigma:.2f} m",
                               backend=self.cov.name)

        t_fix = K.now_unix()
        latency_ms = (t_fix - header["t_capture_unix"]) * 1000.0
        budget = pl.get("latency_budget_ms", 250)
        if latency_ms > budget:
            # Not rejected. ArduPilot does not reject a late fix either -- it
            # stamps it as current and fuses it at the wrong time, silently.
            # Recording it is the only way that ever becomes visible.
            self.log.throttled("PLE-09", 5.0, f"{latency_ms:.0f} ms over the {budget} ms budget",
                               injected_m_at_5ms=round((latency_ms - budget) / 1000.0 * 5.0, 2))

        if res.reject_code:
            self.log.throttled(res.reject_code, 5.0, res.reject_reason)

        fix = FixPacket(
            seq=header["seq"], t_capture_unix=header["t_capture_unix"], t_fix_unix=t_fix,
            latency_ms=round(latency_ms, 2), method=self.method_name, accepted=res.ok,
            lat=res.lat, lon=res.lon, sigma_m=sigma, covariance_source=self.cov.name,
            inliers=res.inliers, matches=res.matches, keypoints=res.keypoints,
            inlier_ratio=round(res.inlier_ratio, 4),
            reproj_err_px=round(res.reproj_err_px, 3) if res.reproj_err_px else None,
            reject_code=res.reject_code, altitude_m=alt,
            stage_ms={k: round(v, 2) for k, v in res.stage_ms.items()})
        self.pub.send(K.T_FIX, to_dict(fix))
        self.log.throttled("PL-17", 30.0, f"seq {header['seq']} accepted={res.ok}")

        self.n_fix += 1
        self._rate_window.append(time.monotonic())
        if res.ok:
            self.n_accept += 1
            self.prior = (res.lat, res.lon, t_fix)

    def check_thermal(self) -> None:
        """Above this, clocks are being cut and a timing number is not a board
        result any more. Reporting it is the difference between a slow run and
        an invalid one."""
        t = device.read_temp_c()
        limit = self.cfg.get("processing_layer.thermal_warn_c", 80.0)
        if t is not None and t >= limit:
            if self._hot_since is None:
                self._hot_since = time.monotonic()
            self.log.throttled("PLDE-03", 30.0, f"{t:.1f} C at or above {limit} C",
                               hot_for_s=round(time.monotonic() - self._hot_since, 1))
        else:
            self._hot_since = None

    def publish_status(self) -> None:
        self.check_thermal()
        now = time.monotonic()
        cut = now - 10.0
        self._rate_window = [t for t in self._rate_window if t > cut]
        st = StatusPacket(
            layer=LAYER, t_unix=K.now_unix(), ready=self.store is not None and self.method is not None,
            uptime_s=round(self.log.uptime(), 1), last_code=self.log.last_code,
            counts=self.log.tally(), rate_hz=round(len(self._rate_window) / 10.0, 2),
            config={
                "method": self.method_name,
                "methods_available": {k: v["available"] for k, v in M.survey().items()},
                "inlier_gate": self.cfg.get("processing_layer.inlier_gate"),
                "latency_budget_ms": self.cfg.get("processing_layer.latency_budget_ms"),
                "rectify": self.rect.enabled,
                "covariance": self.cov.describe(),
                "map": self.map_id, "paused": self.paused,
                "fixes": self.n_fix, "accepted": self.n_accept,
                "accept_rate": round(self.n_accept / self.n_fix, 3) if self.n_fix else None,
                "skipped_frames": self._skipped,
                "device_errors": self.log.device_errors,
                "power_w": device.read_power_w(self.board), "temp_c": device.read_temp_c(),
            })
        self.pub.send(K.T_STATUS, to_dict(st))
        self._last_status = now

    def handle_commands(self) -> None:
        for msg in self.ctl.poll():
            cmd = msg.get("cmd")
            if cmd == "stop":
                self.running = False
            elif cmd == "pause":
                self.paused = True
            elif cmd == "resume":
                self.paused = False
            elif cmd == "set_method":
                if self.load_method(msg.get("method", "")):
                    # Drop the store as well as the id. A store holds
                    # descriptors from ONE extractor; keeping the old one
                    # attached for the few hundred milliseconds until the next
                    # map packet arrives lets a frame reach a matcher whose
                    # descriptor type does not match, which OpenCV answers with
                    # an assertion rather than a bad result.
                    self.store = None
                    self.map_id = None
                    self.prior = None
            elif cmd == "reset_prior":
                self.prior = None
                self.log.step("PL-11", "prior cleared, next fix is a cold start")
            elif cmd == "set" and "path" in msg:
                self.cfg.set(msg["path"], msg.get("value"))
                self.log.step("PL-02", f"{msg['path']} = {msg.get('value')}")
                if msg["path"].startswith("processing_layer.covariance"):
                    self.cov = cov.build(self.cfg.section("processing_layer").get("covariance", {}))
                if msg["path"] == "processing_layer.rectify":
                    self.rect.enabled = bool(msg.get("value"))

    def stop(self) -> None:
        self.log.step("PL-18", f"{self.n_fix} fixes, {self.n_accept} accepted")
        self.publish_status()
        time.sleep(0.2)
        self.log.close(); self.pub.close(); self.ctl.close(); self.sub.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="geoanchor.processing_layer")
    ap.add_argument("--config", default=None)
    ap.add_argument("--method", default=None)
    args = ap.parse_args(argv)
    cfg = cfgmod.load(args.config)
    if args.method:
        cfg.set("processing_layer.method", args.method)
    run_dir = resolve_run_dir(cfg.resolve("session.runs_dir", "runs"), cfg.get("session.tag", ""))
    layer = ProcessingLayer(cfg, run_dir)
    signal.signal(signal.SIGINT, lambda *_: setattr(layer, "running", False))
    signal.signal(signal.SIGTERM, lambda *_: setattr(layer, "running", False))
    return layer.run()


if __name__ == "__main__":
    raise SystemExit(main())
