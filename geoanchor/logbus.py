"""Every layer logs the same way: a step code, to three places at once.

    stderr      so a bare `python -m geoanchor.data_layer` is readable
    JSONL       so a finished run can be re-read without the dashboard
    the bus     so the dashboard shows it live

A log line without a code is not allowed. That is the point of the exercise:
the professor's requirement is that each layer reports which step it reached,
so `log.step("DL-07", ...)` is the normal way to say anything, and the free
text is the detail hanging off the code rather than the message itself.
"""
from __future__ import annotations

import json
import os
import sys
import time
from collections import Counter, deque
from pathlib import Path

from . import codes as C
from . import contracts as K

_LEVEL_OF = {C.STEP: "info", C.ERROR: "warn", C.DEVICE: "error"}
_COLOUR = {"info": "\033[32m", "warn": "\033[33m", "error": "\033[31m"}
_RESET = "\033[0m"


class LayerLog:
    def __init__(self, layer: str, run_dir: Path | str, publisher=None, echo: bool = True):
        self.layer = layer
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.run_dir / f"{layer}.jsonl"
        self._fh = open(self.path, "a", buffering=1)
        self.pub = publisher
        self.echo = echo and sys.stderr.isatty() or echo
        self.counts: Counter = Counter()
        self.last_code = ""
        self.recent: deque = deque(maxlen=200)
        self.started = time.monotonic()
        self._device_errors: set = set()
        self._throttle: dict = {}
        self._suppressed: dict = {}

    # -- the one entry point ----------------------------------------------
    def emit(self, code: str, message: str = "", **detail) -> None:
        k = C.kind(code)
        level = _LEVEL_OF[k]
        if not message:
            message = C.describe(code)
        pkt = K.LogPacket(
            t_unix=K.now_unix(), layer=self.layer, code=code,
            level=level, message=message, detail=detail,
        )
        row = K.to_dict(pkt)
        self.counts[k] += 1
        self.counts[code] += 1
        self.last_code = code
        self.recent.append(row)
        if k == C.DEVICE:
            self._device_errors.add(code)

        try:
            self._fh.write(json.dumps(row, default=str) + "\n")
        except OSError:
            pass  # a full disk must not take the layer down; the DE code covers it

        if self.pub is not None:
            self.pub.send(K.T_LOG, row)

        if self.echo:
            col = _COLOUR.get(level, "")
            extra = " ".join(f"{k2}={v}" for k2, v in detail.items() if v is not None)
            sys.stderr.write(
                f"{col}{code:9s}{_RESET} {self.layer:10s} {message}"
                + (f"  [{extra}]" if extra else "") + "\n"
            )

    def throttled(self, code: str, min_interval_s: float, message: str = "", **d) -> bool:
        """Emit at most once every min_interval_s.

        A stale GPS link or an out-of-envelope altitude is true on every frame,
        and at 4 Hz that is 14000 identical lines an hour. Throttling keeps the
        condition visible without burying the codes that fired once.
        """
        import time as _t
        now = _t.monotonic()
        last = self._throttle.get(code, -1e9)
        if now - last < min_interval_s:
            self._suppressed[code] = self._suppressed.get(code, 0) + 1
            return False
        self._throttle[code] = now
        n = self._suppressed.pop(code, 0)
        if n:
            d = dict(d, suppressed=n)
        self.emit(code, message, **d)
        return True

    # sugar, purely for readability at the call sites
    def step(self, code: str, message: str = "", **d): self.emit(code, message, **d)
    def error(self, code: str, message: str = "", **d): self.emit(code, message, **d)
    def device(self, code: str, message: str = "", **d): self.emit(code, message, **d)

    # -- state the heartbeat carries --------------------------------------
    @property
    def device_errors(self) -> list:
        return sorted(self._device_errors)

    def clear_device_error(self, code: str) -> None:
        """A recovered link is not a permanent fault. Clearing is explicit so
        that nothing silently forgets a fault that is still present."""
        self._device_errors.discard(code)

    def tally(self) -> dict:
        return {
            "steps": self.counts[C.STEP],
            "errors": self.counts[C.ERROR],
            "device_errors": self.counts[C.DEVICE],
            "live_device_errors": len(self._device_errors),
            # Per-code counts, so the dashboard can show WHICH steps a layer has
            # completed rather than only how many. Bounded by the registry, so
            # about forty entries per layer.
            "seen": {k: v for k, v in self.counts.items() if isinstance(k, str) and "-" in k},
        }

    def uptime(self) -> float:
        return time.monotonic() - self.started

    def close(self) -> None:
        try:
            self._fh.close()
        except OSError:
            pass


def new_run_dir(root: Path | str, tag: str = "") -> Path:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    name = f"{stamp}_{tag}" if tag else stamp
    d = Path(root) / name
    d.mkdir(parents=True, exist_ok=True)
    return d


def latest_run_dir(root: Path | str) -> Path | None:
    root = Path(root)
    if not root.is_dir():
        return None
    runs = sorted((p for p in root.iterdir() if p.is_dir()), key=lambda p: p.name)
    return runs[-1] if runs else None


def resolve_run_dir(root: Path | str, tag: str = "") -> Path:
    """Layers started separately must land in the SAME run directory.

    The supervisor exports GEOANCHOR_RUN_DIR; a layer started by hand makes
    its own. Without this the three JSONL files scatter across three
    directories and the session cannot be reassembled afterwards.
    """
    env = os.environ.get("GEOANCHOR_RUN_DIR")
    if env:
        p = Path(env)
        p.mkdir(parents=True, exist_ok=True)
        return p
    return new_run_dir(root, tag)
