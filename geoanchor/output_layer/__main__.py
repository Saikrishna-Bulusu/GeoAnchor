"""Output layer process.

Pairs a predicted fix with the actual GPS taken at the same instant, scores it,
writes the export, and decides what -- if anything -- reaches the flight
controller. It is the only layer allowed to write to the vehicle, and it
refuses to do so on its own initiative: closed loop requires an explicit mode,
a fix above the quality floor, an altitude inside the envelope, and no live
device error anywhere in the system.

    python -m geoanchor.output_layer
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from collections import deque
from pathlib import Path

from .. import codes as C
from .. import config as cfgmod
from .. import contracts as K
from .. import device
from ..bus import BindError, CommandServer, Publisher, Subscriber
from ..contracts import RecordPacket, StatusPacket, to_dict
from ..logbus import LayerLog, resolve_run_dir
from . import metrics
from .fcout import FcError, FlightControllerLink, QgcLink, quality_from
from .recorder import Recorder, basemap_data_uri

LAYER = "output"
STATUS_S = 1.0


class OutputLayer:
    def __init__(self, cfg: cfgmod.Config, run_dir: Path):
        self.cfg = cfg
        self.run_dir = run_dir
        self.running = True
        self.board = device.detect()
        self.time_step = 0
        self.gps_hist: deque = deque(maxlen=200)
        self.map_info: dict = None
        self.fc: FlightControllerLink = None
        self.fc_error = None
        self.qgc: QgcLink = None
        self.qgc_error = None
        self._last_status = 0.0
        self._upstream_device_errors: list = []
        self._last_rx = time.monotonic()

        ol = cfg.section("output_layer")
        self.loop_mode = ol.get("fc", {}).get("loop_mode", "open")
        self.pair_window = float(ol.get("pair_window_s", 0.5))
        self.loss_kind = ol.get("loss", "nll")
        self.alarm_m = float(ol.get("alarm_error_m", 50.0))
        self.stats = metrics.Running()

        try:
            self.pub = Publisher(cfg.get("bus.output_pub"))
        except BindError as exc:
            # Log without a publisher: the socket is exactly what failed, so
            # the code still has to reach the JSONL and the operator.
            self.log = LayerLog(LAYER, run_dir, None)
            self.log.device("OLDE-06", str(exc))
            self.log.close()
            raise SystemExit(3)
        self.log = LayerLog(LAYER, run_dir, self.pub)
        self.ctl = CommandServer(cfg.get("bus.output_ctl", "ipc:///tmp/geoanchor/output.ctl"))
        self.sub = Subscriber([cfg.get("bus.processing_pub"), cfg.get("bus.data_pub")],
                              [K.T_FIX, K.T_GPS, K.T_MAP, K.T_STATUS])
        self.rec: Recorder = None

    # -- setup -------------------------------------------------------------
    def start(self) -> None:
        self.log.step("OL-01", f"{self.board.model or self.board.arch}", run=self.run_dir.name)
        problems = self.cfg.validate()
        if problems:
            for p in problems:
                self.log.error("OLE-05", p)
            raise SystemExit(2)
        self.log.step("OL-02", f"config {self.cfg.path}")
        self.log.step("OL-03", self.cfg.get("bus.output_pub"))
        self.log.step("OL-04", self.cfg.get("bus.processing_pub"))
        self.log.step("OL-05", self.cfg.get("bus.data_pub"))

        feed_path = str(self.cfg.get("data_layer.feed.path", ""))
        header = {
            "session_tag": self.cfg.get("session.tag", ""),
            "run_dir": str(self.run_dir),
            "config": self.cfg.snapshot(),
            "board": device.summary(),
            "loss": self.loss_kind,
            "warnings": [],
        }
        if "demo/flight" in feed_path:
            header["synthetic_from_reference"] = True
            header["warnings"].append(
                "Frames were cut from the same image used as the reference map. This session "
                "proves the pipeline is wired correctly and is NOT an accuracy result.")
        ex = self.cfg.section("output_layer").get("export", {}) or {}
        self.rec = Recorder(self.run_dir, header,
                            flush_every=ex.get("flush_every", 20),
                            include_logs=ex.get("include_logs", True),
                            max_log_rows=ex.get("max_log_rows", 4000))
        # The step-code registry travels with the session. It is 113 short rows,
        # and without it the replayed "steps completed" view has nothing to name
        # the codes it is drawing -- it would have to ask a board that is not
        # there.
        self.rec.set_extra("codes", [{"code": c, "kind": C.kind(c), "layer": C.layer(c),
                                      "description": d} for c, d in C.REGISTRY.items()])
        self.log.step("OL-06", str(self.rec.jsonl))
        self.open_fc()
        self.open_qgc()
        self.log.step("OL-07", f"loop mode: {self.loop_mode}")

    def open_fc(self) -> None:
        fcfg = self.cfg.section("output_layer").get("fc", {})
        if not fcfg.get("enabled", False) or self.loop_mode == "off":
            self.log.step("OL-15", "flight controller link disabled -- logging only")
            return
        try:
            self.fc = FlightControllerLink(
                fcfg.get("endpoint", "udpout:127.0.0.1:14551"),
                baud=fcfg.get("baud", 57600), message=fcfg.get("message", "ODOMETRY"),
                firmware=fcfg.get("firmware", "ardupilot"), origin=fcfg.get("origin"),
                send_origin=fcfg.get("send_origin", False))
            self.log.step("OL-13", str(self.fc.describe()))
        except FcError as exc:
            self.fc, self.fc_error = None, str(exc)
            self.log.emit(exc.code, str(exc))

    def open_qgc(self) -> None:
        q = self.cfg.section("output_layer").get("qgc", {}) or {}
        if not q.get("enabled", False):
            return
        try:
            self.qgc = QgcLink(q.get("endpoint", "udpout:127.0.0.1:14550"),
                               system_id=q.get("system_id", 2),
                               label=q.get("label", "GeoAnchor"))
            self.qgc.heartbeat(force=True)
            self.log.step("OL-18", str(self.qgc.describe()))
        except FcError as exc:
            self.qgc, self.qgc_error = None, str(exc)
            self.log.emit(exc.code, str(exc))

    # -- the loop ----------------------------------------------------------
    def run(self) -> int:
        self.start()
        while self.running:
            self.handle_commands()
            msgs, _ = self.sub.drain(200)
            if msgs:
                self._last_rx = time.monotonic()
            elif time.monotonic() - self._last_rx > 15.0:
                self.log.throttled("OLDE-05", 30.0,
                                   "nothing from either upstream layer for "
                                   f"{time.monotonic() - self._last_rx:.0f}s")
            self.check_fc_link()
            for topic, header, _ in msgs:
                if topic == K.T_GPS:
                    self.gps_hist.append(header)
                elif topic == K.T_MAP:
                    if self.map_info is None and header.get("local_frame") and self.rec:
                        self.rec.header["local_frame"] = True
                        self.rec.header["warnings"].append(
                            "Coordinates are a declared anchor plus a true metric offset, not "
                            "real positions: this dataset's ground truth is scene-local. Every "
                            "distance and therefore every error is exact.")
                    first_map = self.map_info is None
                    self.map_info = header
                    if self.rec is not None and first_map:
                        # Carry the georeference, and the tile itself, into the
                        # export. A replayed session otherwise draws its tracks
                        # against nothing: the map packet lives only on the bus,
                        # and the basemap image only on the board.
                        self.rec.set_extra("map", header)
                        if (self.cfg.section("output_layer").get("export", {}) or {}) \
                                .get("embed_basemap", True):
                            uri = basemap_data_uri(header.get("store_path"))
                            if uri:
                                self.rec.set_extra("basemap", uri)
                            else:
                                self.log.step("OL-16", "no basemap embedded; replay will draw "
                                                       "tracks on a blank ground")
                elif topic == K.T_STATUS:
                    de = (header.get("config") or {}).get("device_errors") or []
                    if de:
                        self._upstream_device_errors = sorted(set(self._upstream_device_errors) | set(de))
                    # Keep the newest status per layer so the exported session
                    # can show the layer cards and the steps-completed view,
                    # both of which read counts.seen off exactly this packet.
                    if self.rec is not None and header.get("layer"):
                        self.rec.layers[header["layer"]] = header
                elif topic == K.T_FIX:
                    self.on_fix(header)
            now = time.monotonic()
            if now - self._last_status > STATUS_S:
                self.publish_status()
        self.stop()
        return 0

    def pair(self, t_capture: float):
        """Nearest actual GPS in time, within the pairing window."""
        best, best_dt = None, self.pair_window
        for g in reversed(self.gps_hist):
            dt = abs(g["t_unix"] - t_capture)
            if dt <= best_dt:
                best, best_dt = g, dt
        return best, (best_dt if best else None)

    def on_fix(self, fix: dict) -> None:
        self.time_step += 1
        actual, dt = self.pair(fix["t_capture_unix"])
        codes: list = []

        if actual is None:
            if self.gps_hist:
                self.log.throttled("OLE-02", 10.0,
                                   f"nearest GPS is more than {self.pair_window}s from the frame")
                codes.append("OLE-02")
            else:
                self.log.throttled("OLE-01", 10.0)
                codes.append("OLE-01")
        else:
            self.log.throttled("OL-08", 30.0, f"dt {dt*1000:.0f} ms")
            codes.append("OL-08")

        predicted = ({"lat": fix["lat"], "lon": fix["lon"]}
                     if fix.get("lat") is not None else None)
        actual_gps = ({"lat": actual["lat"], "lon": actual["lon"],
                       "alt_amsl_m": actual.get("alt_amsl_m"),
                       "rel_alt_m": actual.get("rel_alt_m"),
                       "fix_type": actual.get("fix_type"), "source": actual.get("source")}
                      if actual else None)

        error_m = metrics.position_error_m(actual_gps, predicted)
        if error_m is not None:
            self.log.throttled("OL-09", 30.0, f"{error_m:.2f} m")
            codes.append("OL-09")
            if error_m > self.alarm_m:
                self.log.throttled("OLE-04", 5.0, f"{error_m:.1f} m exceeds {self.alarm_m} m")
                codes.append("OLE-04")

        loss = metrics.compute_loss(self.loss_kind, error_m, fix.get("sigma_m"))
        if loss is not None:
            self.log.throttled("OL-10", 30.0, f"{self.loss_kind} {loss:.3f}")
            codes.append("OL-10")

        sent, why = self.maybe_send(fix, actual_gps, codes)
        self.push_qgc(fix)

        record = RecordPacket(
            time_step=self.time_step, t_unix=fix["t_capture_unix"],
            actual_gps=actual_gps, predicted_gps=predicted,
            error_m=round(error_m, 4) if error_m is not None else None,
            loss=round(loss, 6) if loss is not None else None,
            accepted=bool(fix["accepted"]), sigma_m=fix.get("sigma_m"),
            inliers=fix.get("inliers", 0), latency_ms=fix.get("latency_ms"),
            method=fix.get("method", ""), loop_mode=self.loop_mode,
            sent_to_fc=sent, codes=codes, stage_ms=fix.get("stage_ms") or {})
        try:
            self.rec.append(record)
        except OSError as exc:
            code = "OLDE-04" if getattr(exc, "errno", None) == 28 else "OLE-05"
            self.log.throttled(code, 10.0, str(exc))
        else:
            self.log.throttled("OL-11", 30.0, f"step {self.time_step}")

        self.stats.add(accepted=bool(fix["accepted"]), error_m=error_m, loss=loss,
                       sigma_m=fix.get("sigma_m"), latency_ms=fix.get("latency_ms"))
        self.pub.send(K.T_RECORD, to_dict(record))
        self.log.throttled("OL-12", 30.0)

    def maybe_send(self, fix: dict, actual_gps, codes: list) -> tuple:
        """The only place anything reaches the vehicle. Every gate is explicit."""
        if self.fc is None or self.loop_mode == "off":
            self.log.throttled("OL-15", 30.0, "logged, not sent")
            return False, "link disabled"

        fcfg = self.cfg.section("output_layer").get("fc", {})
        if self.loop_mode == "open":
            if actual_gps is None:
                return False, "no actual GPS to echo"
            try:
                self.fc.send(actual_gps["lat"], actual_gps["lon"], sigma_m=1.0,
                             t_capture_unix=fix["t_capture_unix"], quality=100)
            except FcError as exc:
                self.log.throttled(exc.code, 10.0, str(exc))
                return False, str(exc)
            self.log.throttled("OL-14", 30.0, "actual GPS echoed over ExternalNav (open loop)")
            return True, ""

        # closed loop from here down
        if not fix["accepted"]:
            self.log.throttled("OLE-03", 10.0, f"rejected upstream ({fix.get('reject_code')})")
            codes.append("OLE-03")
            return False, "fix rejected upstream"
        alt = fix.get("altitude_m")
        cap = self.cfg.get("data_layer.envelope.agl_max_m", 100)
        if alt is not None and alt > cap:
            self.log.throttled("OLE-07", 10.0, f"{alt:.0f} m above the {cap} m cap")
            codes.append("OLE-07")
            return False, "above the altitude cap"
        floor = fcfg.get("min_quality_inliers", 40)
        if fix.get("inliers", 0) < floor:
            self.log.throttled("OLE-06", 10.0, f"{fix.get('inliers')} inliers < closed-loop floor {floor}")
            codes.append("OLE-06")
            return False, "below the closed-loop quality floor"
        if fcfg.get("require_no_device_errors", True):
            live = sorted(set(self._upstream_device_errors) | set(self.log.device_errors))
            if live:
                self.log.throttled("OLE-08", 10.0, f"live device errors: {', '.join(live)}")
                codes.append("OLE-08")
                return False, "device errors are live"
        if fix.get("sigma_m") is None:
            self.log.throttled("OLE-06", 10.0, "no covariance on this fix")
            return False, "no covariance"

        try:
            self.fc.send(fix["lat"], fix["lon"], sigma_m=fix["sigma_m"],
                         t_capture_unix=fix["t_capture_unix"],
                         quality=quality_from(fix.get("inliers", 0),
                                              self.cfg.get("processing_layer.inlier_gate", 25)))
        except FcError as exc:
            self.log.throttled(exc.code, 10.0, str(exc))
            return False, str(exc)
        self.log.throttled("OL-14", 30.0, "predicted GPS sent (closed loop)")
        codes.append("OL-14")
        return True, ""

    def check_fc_link(self) -> None:
        """A write-only MAVLink link gives no delivery confirmation, so the
        vehicle's own heartbeat is the only evidence anything is listening."""
        if self.fc is None:
            return
        age = self.fc.heartbeat_age()
        if age is not None and age > self.cfg.get("output_layer.fc.heartbeat_timeout_s", 5.0):
            self.log.throttled("OLDE-02", 15.0, f"no heartbeat for {age:.0f}s")

    def push_qgc(self, fix: dict) -> None:
        """Display only. Independent of loop_mode, and a failure here is never
        allowed to matter."""
        if self.qgc is None:
            return
        try:
            if fix.get("lat") is not None:
                self.qgc.send(fix["lat"], fix["lon"], alt_m=fix.get("altitude_m"),
                              sigma_m=fix.get("sigma_m"),
                              t_capture_unix=fix["t_capture_unix"],
                              accepted=bool(fix["accepted"]))
                self.log.throttled("OL-19", 30.0, f"seq {fix['seq']}")
            else:
                self.qgc.heartbeat()
        except FcError as exc:
            self.log.throttled(exc.code, 30.0, str(exc))

    def publish_status(self) -> None:
        st = StatusPacket(
            layer=LAYER, t_unix=K.now_unix(), ready=self.rec is not None,
            uptime_s=round(self.log.uptime(), 1), last_code=self.log.last_code,
            counts=self.log.tally(), rate_hz=0.0,
            config={
                "loop_mode": self.loop_mode, "loss": self.loss_kind,
                "pair_window_s": self.pair_window, "alarm_error_m": self.alarm_m,
                "fc": self.fc.describe() if self.fc else {"enabled": False, "error": self.fc_error},
                "qgc": self.qgc.describe() if self.qgc else {"enabled": False, "error": self.qgc_error},
                "summary": self.stats.summary(),
                "records": self.time_step,
                "export": str(self.rec.session) if self.rec else None,
                "map": {"store_id": self.map_info["store_id"],
                        "bounds_wgs84": self.map_info["bounds_wgs84"]} if self.map_info else None,
                "device_errors": self.log.device_errors,
                "upstream_device_errors": self._upstream_device_errors,
            })
        self.pub.send(K.T_STATUS, to_dict(st))
        # This layer does not subscribe to its own bus, so its card would be the
        # one missing from its own export. Record it here alongside the two it
        # hears from upstream.
        if self.rec is not None:
            self.rec.layers[LAYER] = to_dict(st)
        self._last_status = time.monotonic()

    def handle_commands(self) -> None:
        for msg in self.ctl.poll():
            cmd = msg.get("cmd")
            if cmd == "stop":
                self.running = False
            elif cmd == "set_loop_mode":
                mode = msg.get("mode", "off")
                if mode not in ("off", "open", "closed"):
                    self.log.error("OLE-05", f"unknown loop mode '{mode}'")
                    continue
                self.loop_mode = mode
                self.cfg.set("output_layer.fc.loop_mode", mode)
                if self.fc:
                    self.fc.bump_reset()
                elif mode != "off":
                    self.open_fc()
                self.log.step("OL-07", f"loop mode -> {mode}")
            elif cmd == "flush":
                self.rec.flush(self.stats.summary())
                self.log.step("OL-16", str(self.rec.session))
            elif cmd == "set" and "path" in msg:
                self.cfg.set(msg["path"], msg.get("value"))
                if msg["path"] == "output_layer.loss":
                    self.loss_kind = msg.get("value", "nll")
                self.log.step("OL-02", f"{msg['path']} = {msg.get('value')}")

    def stop(self) -> None:
        summary = self.stats.summary()
        if self.rec:
            path = self.rec.close(summary)
            self.log.step("OL-16", str(path), records=self.time_step)
        if self.fc:
            self.fc.close()
        if self.qgc:
            self.qgc.close()
        self.log.step("OL-17", f"{self.time_step} records | "
                               f"median {summary['median_error_m']} m | "
                               f"accepted {summary['n_accepted']}/{summary['n_fixes']}")
        self.publish_status()
        time.sleep(0.2)
        self.log.close(); self.pub.close(); self.ctl.close(); self.sub.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="geoanchor.output_layer")
    ap.add_argument("--config", default=None)
    ap.add_argument("--loop-mode", choices=["off", "open", "closed"], default=None)
    args = ap.parse_args(argv)
    cfg = cfgmod.load(args.config)
    if args.loop_mode:
        cfg.set("output_layer.fc.loop_mode", args.loop_mode)
    run_dir = resolve_run_dir(cfg.resolve("session.runs_dir", "runs"), cfg.get("session.tag", ""))
    layer = OutputLayer(cfg, run_dir)
    signal.signal(signal.SIGINT, lambda *_: setattr(layer, "running", False))
    signal.signal(signal.SIGTERM, lambda *_: setattr(layer, "running", False))
    return layer.run()


if __name__ == "__main__":
    raise SystemExit(main())
