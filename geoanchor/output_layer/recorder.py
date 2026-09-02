"""The export. One JSONL row per fix, one session.json at the end.

The row shape is the one the professor asked for -- time step, actual GPS,
predicted GPS, error -- with the fields needed to interpret it kept alongside
rather than in a separate file. Written as JSONL while running so that a run
killed at minute forty still has thirty-nine minutes of data, and assembled
into session.json on flush so there is a single file to hand over.
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict
from pathlib import Path


class Recorder:
    def __init__(self, run_dir: Path | str, header: dict = None, flush_every: int = 20):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.jsonl = self.run_dir / "records.jsonl"
        self.session = self.run_dir / "session.json"
        self.header = dict(header or {})
        self.header.setdefault("created_at", time.time())
        self.flush_every = max(1, int(flush_every))
        self.rows: list = []
        self._since_flush = 0
        self._fh = open(self.jsonl, "a", buffering=1)

    def append(self, record) -> None:
        row = asdict(record) if hasattr(record, "__dataclass_fields__") else dict(record)
        self.rows.append(row)
        self._fh.write(json.dumps(row, default=str) + "\n")
        self._since_flush += 1
        if self._since_flush >= self.flush_every:
            self.flush()

    def flush(self, summary: dict = None) -> Path:
        doc = {
            "schema": 1,
            "header": self.header,
            "summary": summary or {},
            "records": self.rows,
        }
        tmp = self.session.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(doc, indent=2, default=str))
        tmp.replace(self.session)          # atomic, so a reader never sees half a file
        self._since_flush = 0
        return self.session

    def close(self, summary: dict = None) -> Path:
        path = self.flush(summary)
        try:
            self._fh.close()
        except OSError:
            pass
        return path
