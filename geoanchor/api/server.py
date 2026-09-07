"""Dashboard API. An observer, never a participant.

It subscribes to all three layers and fans the traffic out over a WebSocket.
It publishes nothing back onto the bus except explicit operator commands, and
it holds no state any layer depends on -- kill it mid-flight and the pipeline
does not notice, which is the property the three-layer requirement asks for.

Runs on the board. A browser anywhere on the network opens it directly, so
live telemetry never leaves the LAN. The same dashboard also builds as a static
bundle for Vercel, where it reads an exported session.json instead of this API,
because a public host cannot reach a Jetson sitting behind a home router.
"""
from __future__ import annotations

import asyncio
import json
import threading
import time
from collections import deque
from pathlib import Path

from .. import codes as C
from .. import config as cfgmod
from .. import contracts as K
from .. import device
from .. import methods as M
from ..bus import CommandClient, Subscriber

# These are imported at MODULE scope on purpose, and it is not a style choice.
# This file uses `from __future__ import annotations`, so every annotation is a
# string that FastAPI must resolve with get_type_hints() against the module
# globals. With `WebSocket` imported inside create_app() it is not in those
# globals, resolution fails, FastAPI decides the handler's `sock` parameter is
# an unresolvable body field, and dependency solving errors out. Its handler
# for that closes the socket instead of raising, so the browser sees a bare
# HTTP 403 on the upgrade with no traceback anywhere. Cost an hour once.
try:
    from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.responses import FileResponse, JSONResponse, Response
    from fastapi.staticfiles import StaticFiles
except ImportError as _exc:                      # only the API process needs these
    raise ImportError("fastapi is not installed -- run bootstrap.sh") from _exc

MAX_LOGS = 400
MAX_RECORDS = 5000


def _frame_summary(header: dict) -> dict:
    """The frame fields the dashboard reads, in ONE shape.

    The incremental push and the connect-time snapshot both describe the same
    frame, so they have to describe it identically. They did not: the push sent
    w/h while the snapshot passed the raw packet through with width/height, and
    a panel can only read one of those. The symptom is a camera caption reading
    "undefinedxundefined" until the next frame happens to arrive -- permanently,
    if the feed has already ended.
    """
    if not header:
        return None
    return {
        "seq": header.get("seq"),
        "t": header.get("t_capture_unix"),
        "w": header.get("width"),
        "h": header.get("height"),
        "source_w": header.get("source_width"),
        "source_h": header.get("source_height"),
        "capture_age_ms": header.get("capture_age_ms"),
        "preprocess_ms": header.get("preprocess_ms"),
        "feed": header.get("feed"),
        "altitude_m": header.get("altitude_m"),
        "yaw_deg": header.get("yaw_deg"),
    }


class Hub:
    """Owns the bus connection and every buffer the dashboard reads."""

    def __init__(self, cfg: cfgmod.Config):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.logs: deque = deque(maxlen=MAX_LOGS)
        self.records: deque = deque(maxlen=MAX_RECORDS)
        self.status: dict = {}
        self.fixes: deque = deque(maxlen=200)
        self.frame_jpeg: bytes = None
        self.frame_header: dict = None
        self.map: dict = None
        self.started = time.time()
        self.listeners: list = []
        self.loop: asyncio.AbstractEventLoop = None
        self._stop = threading.Event()
        self._ctl: dict = {}

    # -- bus ---------------------------------------------------------------
    def start(self) -> None:
        threading.Thread(target=self._pump, name="hub", daemon=True).start()

    def _pump(self) -> None:
        sub = Subscriber(
            [self.cfg.get("bus.data_pub"), self.cfg.get("bus.processing_pub"),
             self.cfg.get("bus.output_pub")],
            [K.T_LOG, K.T_STATUS, K.T_FIX, K.T_RECORD, K.T_FRAME, K.T_MAP], rcvhwm=32)
        while not self._stop.is_set():
            msgs, _ = sub.drain(200, keep_latest_of=[K.T_FRAME])
            for topic, header, payload in msgs:
                self._absorb(topic, header, payload)
        sub.close()

    def _absorb(self, topic: str, header: dict, payload) -> None:
        push = None
        with self.lock:
            if topic == K.T_LOG:
                header["kind"] = C.kind(header["code"])
                header["description"] = C.describe(header["code"])
                self.logs.append(header)
                push = ("log", header)
            elif topic == K.T_STATUS:
                self.status[header["layer"]] = header
                push = ("status", header)
            elif topic == K.T_FIX:
                self.fixes.append(header)
                push = ("fix", header)
            elif topic == K.T_RECORD:
                self.records.append(header)
                push = ("record", header)
            elif topic == K.T_FRAME:
                self.frame_jpeg, self.frame_header = payload, header
                push = ("frame", _frame_summary(header))
            elif topic == K.T_MAP:
                self.map = header
                push = ("map", header)
        if push and self.loop:
            asyncio.run_coroutine_threadsafe(self._broadcast(push[0], push[1]), self.loop)

    async def _broadcast(self, kind: str, data: dict) -> None:
        msg = json.dumps({"type": kind, "data": data}, default=str)
        for q in list(self.listeners):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                pass          # a slow browser tab must not stall the hub

    # -- control -----------------------------------------------------------
    def command(self, layer: str, msg: dict) -> bool:
        key = {"data": "bus.data_ctl", "processing": "bus.processing_ctl",
               "output": "bus.output_ctl"}.get(layer)
        if not key:
            return False
        if layer not in self._ctl:
            self._ctl[layer] = CommandClient(self.cfg.get(key))
        return self._ctl[layer].send(msg)

    # -- snapshot ----------------------------------------------------------
    def state(self) -> dict:
        with self.lock:
            live = {n: (time.time() - s["t_unix"] < 4.0) for n, s in self.status.items()}
            return {
                "server_time": time.time(),
                "uptime_s": round(time.time() - self.started, 1),
                "layers": {n: {"status": s, "live": live.get(n, False)}
                           for n, s in self.status.items()},
                "map": self.map,
                "logs": list(self.logs)[-120:],
                "records": list(self.records)[-600:],
                "fixes": list(self.fixes)[-60:],
                "frame": _frame_summary(self.frame_header),
                "board": device.summary(),
                "codes": {"total": len(C.REGISTRY)},
                "config": self.cfg.snapshot(),
            }


def create_app(cfg: cfgmod.Config):
    hub = Hub(cfg)
    app = FastAPI(title="GeoAnchor", version="1.0")
    app.add_middleware(
        CORSMiddleware, allow_origins=cfg.get("api.cors_origins", ["*"]),
        allow_methods=["*"], allow_headers=["*"])

    @app.on_event("startup")
    async def _startup():
        hub.loop = asyncio.get_running_loop()
        hub.start()

    @app.get("/api/health")
    def health():
        return {"ok": True, "uptime_s": round(time.time() - hub.started, 1)}

    @app.get("/api/state")
    def state():
        return hub.state()

    @app.get("/api/methods")
    def methods():
        return M.survey(max_keypoints=cfg.get("processing_layer.max_keypoints", 4096))

    @app.get("/api/codes")
    def codes():
        return [{"code": c, "kind": C.kind(c), "layer": C.layer(c), "description": d}
                for c, d in C.REGISTRY.items()]

    @app.get("/api/frame.jpg")
    def frame():
        with hub.lock:
            buf = hub.frame_jpeg
        if not buf:
            raise HTTPException(404, "no frame yet")
        return Response(buf, media_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    @app.get("/api/map.png")
    def map_png():
        with hub.lock:
            m = hub.map
        if not m:
            raise HTTPException(404, "no map packet yet")
        p = Path(m["store_path"]) / "preview.png"
        if not p.exists():
            raise HTTPException(404, "this store has no preview")
        return FileResponse(p, media_type="image/png")

    @app.get("/api/records")
    def records(since: int = 0):
        with hub.lock:
            rows = [r for r in hub.records if r["time_step"] > since]
        return {"since": since, "count": len(rows), "records": rows}

    @app.get("/api/runs")
    def runs():
        root = cfg.resolve("session.runs_dir", "runs")
        if not root or not root.is_dir():
            return []
        out = []
        for d in root.iterdir():
            s = d / "session.json"
            if s.exists():
                out.append({"name": d.name, "size": s.stat().st_size,
                            "modified": s.stat().st_mtime})
        # Newest first BY TIME, not by name. Sorting the directory names put
        # every verify_* run ahead of every dated session -- 'v' outranks '2'
        # -- so the dashboard, which shows the first eight, listed three-day-old
        # verification runs and never the session flushed a minute earlier.
        out.sort(key=lambda r: r["modified"], reverse=True)
        return out

    @app.get("/api/runs/{name}")
    def run_session(name: str):
        root = cfg.resolve("session.runs_dir", "runs")
        p = (root / name / "session.json").resolve()
        if root not in p.parents or not p.exists():
            raise HTTPException(404, "no such run")
        return FileResponse(p, media_type="application/json")

    @app.get("/api/export")
    def export():
        """The JSON the professor asked for: time step, actual, predicted, error."""
        root = cfg.resolve("session.runs_dir", "runs")
        latest = None
        if root and root.is_dir():
            cands = [d for d in root.iterdir() if (d / "session.json").exists()]
            latest = max(cands, key=lambda d: d.name) if cands else None
        if latest is None:
            with hub.lock:
                rows = list(hub.records)
            return JSONResponse({"schema": 1, "header": {"source": "live buffer"},
                                 "records": rows})
        return FileResponse(latest / "session.json", media_type="application/json",
                            filename=f"{latest.name}_session.json")

    @app.post("/api/control")
    async def control(body: dict):
        layer = body.pop("layer", "")
        if not body.get("cmd"):
            raise HTTPException(400, "cmd is required")
        if not hub.command(layer, body):
            raise HTTPException(400, f"unknown layer '{layer}'")
        return {"sent": True, "layer": layer, "cmd": body["cmd"],
                "note": "the layer echoes its applied configuration in its next status"}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        q: asyncio.Queue = asyncio.Queue(maxsize=256)
        hub.listeners.append(q)
        try:
            await sock.send_text(json.dumps({"type": "snapshot", "data": hub.state()},
                                            default=str))
            while True:
                await sock.send_text(await q.get())
        except (WebSocketDisconnect, RuntimeError, asyncio.CancelledError):
            pass
        finally:
            if q in hub.listeners:
                hub.listeners.remove(q)

    # The built dashboard, if it has been built. Mounted last so it never
    # shadows /api or /ws.
    for candidate in (cfgmod.REPO_ROOT / "dashboard" / "out",
                      cfgmod.REPO_ROOT / "dashboard" / "dist"):
        if candidate.is_dir():
            app.mount("/", StaticFiles(directory=str(candidate), html=True), name="dashboard")
            break

    app.state.hub = hub
    return app
