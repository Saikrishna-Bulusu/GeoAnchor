"""The export. One JSONL row per fix, one session.json at the end.

The row shape is the one the professor asked for -- time step, actual GPS,
predicted GPS, error -- with the fields needed to interpret it kept alongside
rather than in a separate file. Written as JSONL while running so that a run
killed at minute forty still has thirty-nine minutes of data, and assembled
into session.json on flush so there is a single file to hand over.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import asdict
from pathlib import Path


def _json_safe(obj):
    """Replace NaN and +/-inf with null, everywhere, before serialising.

    json.dumps defaults to allow_nan=True and emits bare NaN / Infinity tokens.
    Python reads those back happily, which is exactly why this survived: the
    file looks fine from the side that wrote it. Nothing else accepts them --
    JSON.parse throws, and jq throws -- and they are not valid JSON. The failure
    is total rather than partial: one NaN loss value in one row costs the entire
    session export and blanks the dashboard that reads it.

    null is the honest encoding. A NaN here means "not computed", which is what
    every consumer already understands null to mean.
    """
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    return obj


def basemap_data_uri(store_path, long_edge: int = 1400, quality: int = 78,
                     max_bytes: int = 3_000_000):
    """The reference tile, small enough to live inside session.json.

    Without this a replayed session has no map at all: MapView asks the board
    for /api/map.png, and the whole point of the export is that there is no
    board. Embedding makes one file that opens anywhere, which is what an
    internet-denied workflow actually needs -- carry the file, not the aircraft.

    The store's own preview is 3.6 MB for the Sydney tile and would be about
    4.9 MB once base64'd, for a canvas that renders it around 600 px wide. So it
    is resampled to a long edge that still oversamples any panel it will be
    drawn in, and encoded as JPEG: this is a backdrop for a track overlay, not
    an image anything is measured from. Returns None rather than a broken
    session if anything is missing or the result is still too big.
    """
    try:
        import base64
        import cv2
        p = Path(store_path) / "preview.png"
        if not p.exists():
            return None
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            return None
        h, w = img.shape[:2]
        k = long_edge / max(h, w)
        if k < 1.0:
            img = cv2.resize(img, (max(1, int(w * k)), max(1, int(h * k))),
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok or buf.nbytes > max_bytes:
            return None
        return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")
    except Exception:
        # A missing basemap degrades the replay to tracks on a blank ground.
        # It must never be the reason an export fails to write.
        return None


class Recorder:
    #: Log files written by the three layers, in the order they appear on screen.
    LAYER_LOGS = (("data", "data.jsonl"), ("processing", "processing.jsonl"),
                  ("output", "output.jsonl"))

    def __init__(self, run_dir: Path | str, header: dict = None, flush_every: int = 20,
                 include_logs: bool = True, max_log_rows: int = 4000,
                 min_flush_interval_s: float = 20.0, max_records: int = 200_000):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.jsonl = self.run_dir / "records.jsonl"
        self.session = self.run_dir / "session.json"
        self.header = dict(header or {})
        self.header.setdefault("created_at", time.time())
        self.flush_every = max(1, int(flush_every))
        self.include_logs = bool(include_logs)
        self.max_log_rows = int(max_log_rows)
        # Ceiling on the wall-clock cost of rebuilding session.json, and on the
        # memory held to do it. See flush() and append() for why both exist.
        self.min_flush_interval_s = max(0.0, float(min_flush_interval_s))
        self.max_records = max(1, int(max_records))
        self.rows: list = []
        self.layers: dict = {}        # last status per layer, for the layer cards
        self.extras: dict = {}        # map packet, basemap, code registry
        self._since_flush = 0
        self._rows_dropped = 0        # only ever nonzero past max_records
        self._last_flush_t = 0.0
        self._log_state: dict = {}    # per layer: byte offset, kept rows, full tally
        self._fh = open(self.jsonl, "a", buffering=1)

    def set_extra(self, key: str, value) -> None:
        """Attach something that makes the file readable without the board."""
        self.extras[key] = value

    def _collect_logs(self) -> dict:
        """Fold each layer's step codes into the session document.

        These are written per layer as JSONL and, until now, stayed there: a
        session handed to anyone else carried the records and nothing about how
        the run actually went. That is the wrong way round for a system meant to
        fly GNSS- and internet-denied, where the file brought home is the only
        artefact anybody gets to look at, and where "which step did the data
        layer stop at" is the first question worth asking.

        Under the cap, error and device-error rows are kept in full and steps are
        trimmed from the front: a step code repeats thousands of times and its
        tally carries the same information, whereas a fault fires once and is
        the whole reason to open the file.
        """
        out = {}
        for name, fname in self.LAYER_LOGS:
            path = self.run_dir / fname
            if not path.exists():
                continue
            st = self._log_state.setdefault(name, {"offset": 0, "rows": [], "tally": {}})
            try:
                size = path.stat().st_size
            except OSError:
                continue
            # A shrunk file means it was rotated or rewritten under us; the
            # offset no longer points where we think, so start it over.
            if size < st["offset"]:
                st.update(offset=0, rows=[], tally={})
            if size > st["offset"]:
                with open(path, "r", errors="replace") as fh:
                    fh.seek(st["offset"])
                    fresh = fh.read()
                    st["offset"] = fh.tell()
                # A trailing partial line is a row still being written. Push the
                # offset back so the next pass re-reads it whole.
                if fresh and not fresh.endswith("\n"):
                    cut = fresh.rfind("\n")
                    if cut == -1:
                        st["offset"] -= len(fresh.encode("utf-8", "replace"))
                        fresh = ""
                    else:
                        st["offset"] -= len(fresh[cut + 1:].encode("utf-8", "replace"))
                        fresh = fresh[:cut + 1]
                tally = st["tally"]
                for line in fresh.splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        r = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    st["rows"].append(r)
                    code = r.get("code")
                    if code:
                        tally[code] = tally.get(code, 0) + 1
                # Trim here rather than at read time: the tally above has already
                # counted every row the file has ever held, so what is dropped is
                # only the copy carried in the document.
                rows = st["rows"]
                if len(rows) > self.max_log_rows:
                    faults = [r for r in rows if r.get("level") in ("error", "warn")]
                    steps = [r for r in rows if r.get("level") not in ("error", "warn")]
                    keep = max(0, self.max_log_rows - len(faults))
                    st["rows"] = sorted(faults + steps[-keep:],
                                        key=lambda r: r.get("t_unix", 0))
            out[name] = {"rows": list(st["rows"]), "counts": dict(st["tally"]),
                         "total": sum(st["tally"].values())}
        return out

    def append(self, record) -> None:
        row = _json_safe(asdict(record) if hasattr(record, "__dataclass_fields__") else dict(record))
        self.rows.append(row)
        self._fh.write(json.dumps(row, default=str, allow_nan=False) + "\n")
        # records.jsonl above is the complete record and is the crash-safe path:
        # one line per fix, append-only, O(1). The list held here exists only to
        # assemble session.json, so it is the thing that has to be bounded. A
        # run is normally a flight and ends; a looping replay feed does not, and
        # at max_records the oldest rows leave the document rather than the
        # process growing until it is killed. records.jsonl still has them all.
        if len(self.rows) > self.max_records:
            drop = len(self.rows) - self.max_records
            del self.rows[:drop]
            self._rows_dropped += drop
        self._since_flush += 1
        if self._since_flush >= self.flush_every:
            self.flush()

    def flush(self, summary: dict = None, force: bool = False) -> Path:
        """Rebuild session.json. Rate-limited, because it is a whole-run document.

        Every call serialises the entire run and re-reads the layer logs, so the
        cost is proportional to how long the run has been going. Firing that
        every flush_every records makes the total work quadratic in run length:
        measured on a 4k-record run it is already 6 ms per record and climbing,
        and over two days of a looping feed it reached seven saturated cores and
        1.6 TB of rewritten bytes for a file nothing was reading.

        flush_every alone cannot express the intent, because it counts records
        instead of the work they imply. So the record counter proposes and the
        clock decides: a rebuild happens at most once per min_flush_interval_s,
        which bounds the overhead to a fraction of wall-clock no matter how long
        the run lasts or how fast fixes arrive.

        Nothing is lost by waiting. The durability promise belongs to
        records.jsonl, which is written per record and already holds everything.
        A skipped rebuild costs a stale session.json for a few seconds, and
        close() and an explicit operator flush both pass force=True.
        """
        now = time.time()
        if not force and self.min_flush_interval_s > 0.0 \
                and (now - self._last_flush_t) < self.min_flush_interval_s:
            return self.session
        doc = {
            "schema": 1,
            "header": self.header,
            "summary": summary or {},
            "records": self.rows,
        }
        if self._rows_dropped:
            # Say so in the file rather than letting a truncated export read as
            # a complete one.
            doc["records_truncated"] = {
                "dropped_oldest": self._rows_dropped,
                "kept": len(self.rows),
                "cap": self.max_records,
                "complete_record": self.jsonl.name,
            }
        if self.layers:
            doc["layers"] = self.layers
        if self.include_logs:
            doc["logs"] = self._collect_logs()
        doc.update(self.extras)
        tmp = self.session.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(_json_safe(doc), indent=2, default=str, allow_nan=False))
        tmp.replace(self.session)          # atomic, so a reader never sees half a file
        self._since_flush = 0
        self._last_flush_t = now
        return self.session

    def close(self, summary: dict = None) -> Path:
        path = self.flush(summary, force=True)
        try:
            self._fh.close()
        except OSError:
            pass
        return path
