#!/usr/bin/env python3
"""Where a session lives in the logs repo: board, then method.

    board/<board>/<method>/<stamp>_<tag>/

That is the comparison the paper makes -- joules and metres per fix across
compute classes, each running the same matcher -- so it is the axis the
directory tree should hand you for free. The old layout was
`<hostname>/<run>/`, which sorts by the accident of what someone called the
machine and leaves the method buried inside a JSON file.

Both facts come out of `session.json.header`, so the sort is mechanical rather
than a judgement call:

    header.board   {kind, model, arch, cores, ...}   -> board slug
    header.config.processing_layer.method            -> method

Imported by `geoanchor/api/server.py` (to read the layout) and shelled out to
by `scripts/sync_logs.sh` (to write it), so the two can never disagree about
where a run goes. Run it directly to classify a tree:

    python -m geoanchor.log_layout <dir>
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

# Board slugs are matched against `header.board.model` in order, first hit
# wins. The model string is what the board reports about itself -- "Raspberry
# Pi 4 Model B Rev 1.5", "NVIDIA Jetson AGX Xavier" -- so this survives a
# hostname change, which is exactly what the old layout did not.
_MODEL_SLUGS = (
    ("legion", "legion"),
    ("raspberry pi 5", "pi5"),
    ("raspberry pi 4", "pi4"),
    ("raspberry pi 3", "pi3"),
    ("agx xavier", "xavier"),
    ("xavier nx", "xavier-nx"),
    ("orin nano", "orin-nano"),
    ("orin nx", "orin-nx"),
    ("jetson nano", "jetson-nano"),
)


def board_slug(board: dict) -> str:
    """A stable directory name for the board that produced a run.

    Falls back through model -> jetpack -> arch, and finally to "unknown-board"
    rather than to the hostname: a hostname in the tree is what this layout
    exists to get rid of, and an honest "unknown" is easier to notice and fix
    than a plausible-looking wrong answer.
    """
    board = board or {}
    model = str(board.get("model") or "").lower()
    for needle, slug in _MODEL_SLUGS:
        if needle in model:
            return slug
    if board.get("jetpack") or board.get("l4t"):
        return "jetson-unknown"
    # No model, or one nothing above matched: fall back to the architecture
    # rather than to a machine name. Mapping an ARCHITECTURE to a name was what
    # this did before -- x86_64 meant "legion" -- and it silently filed a
    # second x86 machine's runs under the laptop's directory, on exactly the
    # axis this tree exists to separate. device.detect() now reads DMI on x86,
    # so a real model string is available there too and the table above hits.
    arch = str(board.get("arch") or "").lower()
    return f"unknown-{arch}" if arch else "unknown-board"


def method_slug(header: dict) -> str:
    """The matcher the processing layer ran, from the config the run recorded."""
    cfg = (header or {}).get("config") or {}
    m = (cfg.get("processing_layer") or {}).get("method")
    return str(m) if m else "unknown-method"


def classify(session_json: Path) -> tuple[str, str]:
    """(board, method) for one session.json. Raises on an unreadable file."""
    header = json.loads(Path(session_json).read_text()).get("header") or {}
    return board_slug(header.get("board")), method_slug(header)


def iter_runs(root: Path):
    """Every run in a logs-repo clone, new layout or old.

    Yields (board, method, run_name, run_dir). The old `<device>/<run>/` layout
    is still read so a clone that has not been migrated -- or a board that has
    not pulled the new sync script yet -- keeps showing up rather than silently
    vanishing from the fleet view.
    """
    root = Path(root)
    new = root / "board"
    if new.is_dir():
        for board in sorted(p for p in new.iterdir() if p.is_dir()):
            for method in sorted(p for p in board.iterdir() if p.is_dir()):
                for run in sorted(p for p in method.iterdir() if p.is_dir()):
                    if (run / "session.json").exists():
                        yield board.name, method.name, run.name, run
    for dev in sorted(p for p in root.iterdir()
                      if p.is_dir() and p.name not in ("board", ".git")
                      and not p.name.startswith(".")):
        for run in sorted(p for p in dev.iterdir() if p.is_dir()):
            s = run / "session.json"
            if s.exists():
                try:
                    board, method = classify(s)
                except Exception:
                    board, method = "unknown-board", "unknown-method"
                yield board, method, run.name, run


def local_board_slug() -> str:
    """This machine's board slug, without needing a session.json to hand."""
    try:
        from . import device
        b = device.detect()
        return board_slug({"model": b.model, "arch": b.arch,
                           "jetpack": b.jetpack_hint, "l4t": b.l4t})
    except Exception:
        import platform
        return board_slug({"arch": platform.machine()})


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--board-slug":
        print(local_board_slug())
    elif args and args[0] == "--dest":
        # --dest <session.json> <run-name>  ->  the path, relative to the logs
        # repo root, that this run belongs at. Used by sync_logs.sh.
        print("/".join(("board",) + classify(Path(args[1])) + (args[2],)))
    else:
        target = Path(args[0] if args else ".")
        n = 0
        for board, method, name, path in iter_runs(target):
            print(f"{board:14} {method:16} {name}")
            n += 1
        print(f"\n{n} runs")
