"""python -m geoanchor.api"""
from __future__ import annotations

import argparse

from .. import config as cfgmod
from .server import create_app


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="geoanchor.api")
    ap.add_argument("--config", default=None)
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    args = ap.parse_args(argv)

    cfg = cfgmod.load(args.config)
    host = args.host or cfg.get("api.host", "0.0.0.0")
    port = args.port or int(cfg.get("api.port", 8000))

    try:
        import uvicorn
    except ImportError:
        print("uvicorn is not installed -- run bootstrap.sh")
        return 2
    uvicorn.run(create_app(cfg), host=host, port=port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
