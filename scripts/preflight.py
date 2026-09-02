#!/usr/bin/env python3
"""Does this board have what the three layers need?

Reports against the same step-code vocabulary the layers use, so a failure here
names the code you will see at runtime rather than a different error for the
same problem.

    python scripts/preflight.py
    python scripts/preflight.py --json
"""
from __future__ import annotations

import argparse
import importlib
import json
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from geoanchor import config as cfgmod    # noqa: E402
from geoanchor import device              # noqa: E402

OK, WARN, FAIL = "ok", "warn", "fail"
GLYPH = {OK: "  ok  ", WARN: " note ", FAIL: " FAIL "}


def check(results, name, status, detail="", code=""):
    results.append({"check": name, "status": status, "detail": detail, "code": code})


def run() -> list:
    r: list = []

    b = device.detect()
    check(r, "board", OK, f"{b.model} | {b.arch} | {b.cores} cores | {b.ram_gb} GB")
    if b.jetpack_hint:
        check(r, "jetpack", OK, b.jetpack_hint)
    for note in b.notes:
        check(r, "board note", WARN, note)

    pw = device.read_power_w(b)
    if pw is not None:
        check(r, "power probe", OK, f"{b.power_source} reading {pw:.2f} W")
    else:
        check(r, "power probe", WARN,
              f"no power reading here ({b.power_source}). Joules per fix is the headline "
              "measurement of this study, so on a board that cannot self-report, use an "
              "inline meter and record it by hand.")
    t = device.read_temp_c()
    if t is not None:
        check(r, "temperature", OK if t < 75 else WARN, f"{t:.1f} C")

    for mod, why, fatal in [
        ("numpy", "arrays", True), ("cv2", "capture, RANSAC, classical matchers", True),
        ("yaml", "configuration", True), ("zmq", "the layer bus", True),
        ("pyproj", "pixel to geodetic", True), ("pymavlink", "GPS in, position out", True),
        ("fastapi", "dashboard API", False), ("uvicorn", "dashboard API", False),
        ("websockets", "live telemetry to the browser", False),
        ("torch", "XFeat", False), ("kornia", "LighterGlue", False),
        ("rasterio", "ingesting a NEW GeoTIFF", False),
    ]:
        try:
            m = importlib.import_module(mod)
            v = getattr(m, "__version__", "?")
            check(r, mod, OK, f"{v} -- {why}")
            if mod == "cv2" and str(v).startswith("5"):
                check(r, "opencv major", FAIL,
                      "OpenCV 5 moved the 2D-features constructors; cv2.AKAZE_create is gone. "
                      "pip install 'opencv-python-headless>=4.8,<5'")
            if mod == "torch" and getattr(m.version, "cuda", None) is not None:
                check(r, "torch build", WARN,
                      f"CUDA build ({m.version.cuda}). This pipeline is CUDA-free; on ARM "
                      "this wheel is dead weight and on a Jetson it targets the wrong GPU.")
        except ImportError:
            check(r, mod, FAIL if fatal else WARN, f"missing -- needed for {why}",
                  "DLE-01" if fatal else "")

    try:
        from geoanchor import methods as M
        survey = M.survey()
        avail = [k for k, v in survey.items() if v["available"]]
        check(r, "methods", OK if avail else FAIL, ", ".join(avail) or "none available")
        for k, v in survey.items():
            if not v["available"]:
                check(r, f"method {k}", WARN, v["reason"][:110], "PLE-03")
    except Exception as exc:
        check(r, "methods", FAIL, f"{type(exc).__name__}: {exc}", "PLE-02")

    try:
        cfg = cfgmod.load()
        problems = cfg.validate()
        check(r, "config", OK if not problems else FAIL,
              str(cfg.path) if not problems else "; ".join(problems),
              "" if not problems else "DLE-01")

        src = cfg.resolve("data_layer.map.source")
        if src and src.exists():
            check(r, "reference map", OK, f"{src.name} ({src.stat().st_size/1e6:.1f} MB)")
        else:
            check(r, "reference map", FAIL, f"not found: {src}", "DLE-02")

        stores = cfg.resolve("data_layer.map.store_dir", "stores")
        built = sorted(stores.glob("*/manifest.json")) if stores and stores.is_dir() else []
        if built:
            for m in built:
                d = json.loads(m.read_text())
                check(r, "feature store", OK,
                      f"{m.parent.name}: {d['n_tiles']} tiles, {d['n_keypoints']} keypoints, "
                      f"{d['method']}")
        else:
            check(r, "feature store", WARN,
                  "not built yet. python -m geoanchor.data_layer --build-map", "DL-09")

        feed = cfg.section("data_layer").get("feed", {})
        if feed.get("type") == "file":
            p = cfg.resolve("data_layer.feed.path")
            check(r, "feed", OK if p and p.exists() else FAIL,
                  str(p) if p and p.exists() else f"not found: {p}",
                  "" if p and p.exists() else "DLE-02")
        elif feed.get("type") == "uvc":
            dev = Path(f"/dev/video{feed.get('device', 0)}")
            check(r, "feed", OK if dev.exists() else FAIL,
                  str(dev) if dev.exists() else f"{dev} not present -- v4l2-ctl --list-devices",
                  "" if dev.exists() else "DLDE-01")

        if not cfg.section("data_layer").get("feed", {}).get("intrinsics", {}).get("fx_px"):
            check(r, "intrinsics", WARN,
                  "no fx_px, so frames cannot be scaled to the reference GSD and the matcher "
                  "sees the full scale gap. Calibrate before any real flight.", "DLE-13")
    except Exception as exc:
        check(r, "config", FAIL, f"{type(exc).__name__}: {exc}", "DLE-01")

    out = REPO / "dashboard" / "out"
    if out.is_dir():
        check(r, "dashboard", OK, "built -- the API serves it at /")
    elif shutil.which("npm"):
        check(r, "dashboard", WARN, "not built. cd dashboard && npm install && npm run build")
    else:
        check(r, "dashboard", WARN, "npm not found. The API and the layers work without it.")

    return r


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    r = run()
    if args.json:
        print(json.dumps(r, indent=2))
    else:
        for c in r:
            tag = f" [{c['code']}]" if c["code"] else ""
            print(f"  {GLYPH[c['status']]}  {c['check']:16s} {c['detail']}{tag}")
        fails = sum(1 for c in r if c["status"] == FAIL)
        warns = sum(1 for c in r if c["status"] == WARN)
        print(f"\n  {len(r)} checks, {fails} failed, {warns} notes")
    return 1 if any(c["status"] == FAIL for c in r) else 0


if __name__ == "__main__":
    raise SystemExit(main())
