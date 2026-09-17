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
from .. import log_layout
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
    from fastapi import FastAPI, File, HTTPException, UploadFile, WebSocket, WebSocketDisconnect
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

    # ------------------------------------------------------------ fleet ----
    # Sessions from OTHER devices, read out of a clone of the logs repo. The
    # layout is <fleet_dir>/board/<board>/<method>/<run>/session.json, which is
    # exactly what scripts/sync_logs.sh pushes. geoanchor.log_layout.iter_runs
    # owns the walk, and still reads the old <device>/<run>/ layout too, so a
    # clone that has not been migrated keeps showing up here.
    #
    # Read-only and entirely separate from `runs`: this API never writes into
    # the clone and never runs git. Syncing is a scheduled job on each device,
    # so a board that is off, or a laptop with no network, degrades to "its
    # runs are not listed yet" rather than to a failed request here.

    def _fleet_index(root):
        """{run name -> (board, method, dir)} for a logs clone."""
        return {name: (board, method, d)
                for board, method, name, d in log_layout.iter_runs(root)}

    @app.get("/api/fleet")
    def fleet():
        root = cfg.resolve("session.fleet_dir", "fleet")
        if not root or not root.is_dir():
            return {"available": False, "devices": [], "boards": [],
                    "methods": [], "runs": []}
        out = []
        for name, (board, method, d) in _fleet_index(root).items():
            st = (d / "session.json").stat()
            # `device` is kept as an alias of `board` so an older dashboard
            # build keeps rendering against this endpoint rather than showing
            # an empty fleet the moment the API is updated first.
            out.append({"board": board, "device": board, "method": method,
                        "name": name, "size": st.st_size,
                        "modified": st.st_mtime})
        out.sort(key=lambda r: r["modified"], reverse=True)
        return {"available": True,
                "boards": sorted({r["board"] for r in out}),
                "devices": sorted({r["board"] for r in out}),
                "methods": sorted({r["method"] for r in out}),
                "runs": out}

    @app.get("/api/fleet/{device}/{name}")
    def fleet_session(device: str, name: str):
        # A run name is unique across the whole logs repo (it is a UTC stamp),
        # so the lookup is by name and `device` is accepted only to keep the
        # URL shape stable. Resolving by name rather than by path is also what
        # stops a ../.. in either segment from meaning anything at all.
        root = cfg.resolve("session.fleet_dir", "fleet")
        if not root or not root.is_dir():
            raise HTTPException(404, "no fleet directory")
        hit = _fleet_index(root).get(name)
        if not hit:
            raise HTTPException(404, "no such run")
        board, _method, d = hit
        if device not in (board, "_", ""):
            raise HTTPException(404, "no such run")
        return FileResponse(d / "session.json", media_type="application/json")

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

    # ------------------------------------------------------- covariance ----
    # The covariance backend is LIVE-switchable: the processing layer rebuilds
    # the estimator whenever a `set processing_layer.covariance.*` arrives, so
    # this needs no restart. What it does need is for the operator to see WHICH
    # MATCHER each model was trained on, because applying an XFeat-trained
    # model to EdgePoint2 is exactly the failure this project documents -- a
    # system that swaps matchers silently inherits a covariance model that no
    # longer works. Hence `matches_current`, and hence the models advertise
    # their own provenance rather than being an opaque list of filenames.

    @app.get("/api/covariance")
    def covariance():
        import pickle
        root = cfgmod.REPO_ROOT / "models"
        cur = (cfg.section("processing_layer") or {}).get("covariance", {}) or {}
        cur_method = (cfg.section("processing_layer") or {}).get("method")
        models = []
        for f in sorted(root.glob("*.pkl")) if root.is_dir() else []:
            entry = {"path": f"models/{f.name}", "name": f.stem,
                     "size_kb": round(f.stat().st_size / 1024)}
            try:
                with open(f, "rb") as fh:
                    blob = pickle.load(fh)
                meta = blob.get("meta", {}) if isinstance(blob, dict) else {}
                trained_on = meta.get("matcher_filter") or "unstated"
                entry.update(
                    trained_on=trained_on,
                    features=(blob.get("features") if isinstance(blob, dict) else None),
                    validation=meta.get("validation", "unstated"),
                    scenes=meta.get("scenes"), n_train=meta.get("n_train"),
                    source=meta.get("source"),
                    # A string compare, deliberately loose: the harness names a
                    # matcher XFEAT_MNN and the runtime names it xfeat_mnn.
                    matches_current=bool(
                        cur_method and trained_on
                        and trained_on.lower().replace("-", "_") == str(cur_method).lower()))
            except Exception as exc:
                entry["error"] = f"unreadable: {exc}"
            models.append(entry)
        return {
            "backends": ["gate_only", "learned"],
            "current": {"backend": cur.get("backend", "gate_only"),
                        "model_path": cur.get("model_path"),
                        "fixed_sigma_m": cur.get("fixed_sigma_m"),
                        "clamp_m": cur.get("clamp_m")},
            "current_method": cur_method,
            "models": models,
            "note": ("gate_only emits a fixed sigma and labels itself a placeholder "
                     "in every record. `learned` needs a model trained on the SAME "
                     "matcher -- see matches_current."),
        }

    # ------------------------------------------------------------- map ----
    # Uploading a reference map is what a real operator does on the ground
    # before a flight: point the system at imagery of where it is about to fly.
    # The file is written into data/uploads and the data layer is told to
    # rebuild its feature store from it, which is the expensive step and the
    # reason it happens once, on the ground, rather than per frame.

    @app.post("/api/map")
    async def upload_map(file: UploadFile = File(...)):
        name = Path(file.filename or "map.tif").name          # no path from the wire
        if not name.lower().endswith((".tif", ".tiff", ".png", ".jpg", ".jpeg")):
            raise HTTPException(400, "expected a GeoTIFF, or a PNG/JPG with a sidecar")
        dest_dir = cfgmod.REPO_ROOT / "data" / "uploads"
        dest_dir.mkdir(parents=True, exist_ok=True)
        dest = dest_dir / name
        size = 0
        with open(dest, "wb") as fh:
            while chunk := await file.read(1 << 20):
                size += len(chunk)
                fh.write(chunk)
        info = {"path": f"data/uploads/{name}", "bytes": size}
        # Report the georeference back, because a map WITHOUT one is the
        # failure mode that matters: ap_vo2 and this pipeline both require a
        # projected CRS and fail silently on a plain image with a .tif suffix.
        try:
            import rasterio
            with rasterio.open(dest) as ds:
                info.update(width=ds.width, height=ds.height,
                            crs=str(ds.crs) if ds.crs else None,
                            gsd_m_px=abs(ds.transform.a) if ds.crs else None,
                            georeferenced=bool(ds.crs),
                            projected=bool(ds.crs and not ds.crs.is_geographic))
        except Exception as exc:
            info.update(georeferenced=False, error=str(exc))
        if not info.get("projected"):
            info["warning"] = ("no projected CRS. The pipeline needs a UTM (or "
                               "similar) GeoTIFF; a plain image with a .tif "
                               "extension loads and matches against nothing.")
        return info

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
