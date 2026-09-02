"""One YAML file is the single source of truth for all three layers.

Each layer loads the same file and reads only its own section, so there is no
shared mutable state between processes. Runtime changes from the dashboard
arrive as control messages carrying a dotted path and a value; a layer applies
them to its own copy and echoes the result in its status heartbeat. Nothing
writes back to the file unless you ask it to, so a restart returns to a known
configuration.
"""
from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "configs" / "system.yaml"

REQUIRED = [
    "bus.data_pub", "bus.processing_pub", "bus.output_pub",
    "data_layer.map.source", "data_layer.feed.type",
    "processing_layer.method", "processing_layer.inlier_gate",
    "output_layer.fc.loop_mode",
]


class ConfigError(ValueError):
    pass


class Config:
    def __init__(self, data: dict, path: Path | None = None):
        self.data = data
        self.path = path

    # -- access ------------------------------------------------------------
    def get(self, dotted: str, default: Any = None) -> Any:
        node: Any = self.data
        for part in dotted.split("."):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    def set(self, dotted: str, value: Any) -> None:
        parts = dotted.split(".")
        node = self.data
        for part in parts[:-1]:
            node = node.setdefault(part, {})
            if not isinstance(node, dict):
                raise ConfigError(f"{dotted}: {part} is not a section")
        node[parts[-1]] = value

    def section(self, name: str) -> dict:
        return self.get(name, {}) or {}

    def snapshot(self) -> dict:
        return copy.deepcopy(self.data)

    # -- paths -------------------------------------------------------------
    def resolve(self, dotted: str, default: Any = None) -> Path | None:
        """Resolve a configured path relative to the repo, not the cwd.

        Layers are started from a supervisor, from systemd and by hand, and
        each has a different working directory. Anchoring on the repo means
        the same config file works from all three.
        """
        raw = self.get(dotted, default)
        if raw in (None, ""):
            return None
        p = Path(str(raw)).expanduser()
        return p if p.is_absolute() else (REPO_ROOT / p).resolve()

    # -- validation --------------------------------------------------------
    def validate(self) -> list[str]:
        missing = [k for k in REQUIRED if self.get(k) is None]
        problems = [f"missing required key: {k}" for k in missing]

        loop = self.get("output_layer.fc.loop_mode")
        if loop not in ("open", "closed"):
            problems.append("output_layer.fc.loop_mode must be 'open' or 'closed'")

        feed = self.get("data_layer.feed.type")
        if feed not in ("file", "uvc", "rtsp", "env80"):
            problems.append("data_layer.feed.type must be file, uvc, rtsp or env80")

        lo = self.get("data_layer.envelope.agl_min_m", 50)
        hi = self.get("data_layer.envelope.agl_max_m", 100)
        if not (0 < lo < hi <= 100):
            problems.append(
                f"envelope {lo}-{hi} m is invalid: the project cap is 100 m AGL and min must be below max"
            )

        budget = self.get("processing_layer.latency_budget_ms", 250)
        if budget > 250:
            problems.append(
                f"latency_budget_ms={budget} exceeds 250: ArduPilot clamps VISO_DELAY_MS "
                "to 0-250 and overrunning it is silent, not rejected"
            )
        return problems


def load(path: str | os.PathLike | None = None) -> Config:
    p = Path(path) if path else Path(os.environ.get("GEOANCHOR_CONFIG", DEFAULT_CONFIG))
    if not p.exists():
        raise ConfigError(f"config not found: {p}")
    with open(p) as fh:
        data = yaml.safe_load(fh) or {}
    cfg = Config(data, p)
    _apply_env_overrides(cfg)
    return cfg


def _apply_env_overrides(cfg: Config) -> None:
    """GEOANCHOR_SET="a.b.c=value;d.e=value" -- for scripted runs and CI."""
    raw = os.environ.get("GEOANCHOR_SET", "").strip()
    for item in filter(None, (s.strip() for s in raw.split(";"))):
        key, _, val = item.partition("=")
        cfg.set(key.strip(), yaml.safe_load(val))
