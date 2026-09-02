#!/usr/bin/env python3
"""Measure OVERHEAD_MS: everything in the loop that is not the matcher.

This is the number the whole deployment question rests on and nobody has
measured it. The Pi 5 result of 2 Sept 2026 was that XFeat sparse fits the
250 ms budget warm with 48 ms of margin, against an assumed overhead of 40 ms
that was a guess. If capture, ISP, copy, encode and the MAVLink hop come to
more than 88 ms, it misses. If less, it fits.

    python scripts/measure_overhead.py --device 0 --n 200

Measures four things separately, because they have different fixes:

  capture     the driver handing over a frame. Includes exposure and ISP.
  preprocess  undistort and the rescale to reference GSD.
  encode      JPEG, which is what crosses the bus.
  bus         publish to the processing layer receiving it.

Run it on the real rig with the real camera at the real resolution. A number
from a laptop webcam is not this number.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from geoanchor import config as cfgmod          # noqa: E402
from geoanchor import contracts as K            # noqa: E402
from geoanchor import device                    # noqa: E402
from geoanchor.bus import Publisher, Subscriber  # noqa: E402
from geoanchor.data_layer.feed import Preprocessor, open_feed  # noqa: E402


def pct(v, q):
    if not v:
        return float("nan")
    s = sorted(v)
    i = min(len(s) - 1, int(round(q * (len(s) - 1))))
    return s[i]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--device", default=None, help="override the v4l2 index")
    ap.add_argument("--n", type=int, default=200,
                    help="N=30 under-samples the tail badly; 200 is the canonical run")
    ap.add_argument("--altitude", type=float, default=75.0)
    ap.add_argument("--warmup", type=int, default=20)
    args = ap.parse_args()

    cfg = cfgmod.load(args.config)
    board = device.detect()
    print(f"board      {board.model or board.arch} | {board.cores} cores | {board.ram_gb} GB")
    if board.jetpack_hint:
        print(f"jetpack    {board.jetpack_hint}")
    if board.kind == "generic":
        print("           NOTE this is not an embedded board; the numbers are not board results")

    fc = dict(cfg.section("data_layer").get("feed", {}))
    fc["type"] = "uvc"
    if args.device is not None:
        fc["device"] = int(args.device) if str(args.device).isdigit() else args.device
    feed = open_feed(fc)
    print(f"camera     {feed.describe()}")

    map_gsd = None
    stores = cfg.resolve("data_layer.map.store_dir", "stores")
    if stores and stores.is_dir():
        import json
        for m in sorted(stores.glob("*/manifest.json")):
            map_gsd = json.loads(m.read_text())["gsd_m_px"]
            break
    pre = Preprocessor(intrinsics=fc.get("intrinsics") or {},
                       fallback_long_edge=fc.get("frame_px", 512),
                       jpeg_quality=fc.get("jpeg_quality", 80))
    print(f"reference  {map_gsd if map_gsd else 'no store built -- using the fallback resize'}")
    print(f"fx_px      {pre.fx_px or 'not configured -- the rescale is not being measured'}")

    pub = Publisher("ipc:///tmp/geoanchor/overhead.sock")
    sub = Subscriber(["ipc:///tmp/geoanchor/overhead.sock"], [K.T_FRAME])
    time.sleep(0.3)

    cap, prep, enc, bus, total = [], [], [], [], []
    t_start = time.perf_counter()
    for i in range(args.n + args.warmup):
        t0 = time.perf_counter()
        frame, t_capture, _ = feed.read()
        t1 = time.perf_counter()
        if frame is None:
            print("camera stopped returning frames")
            break
        out, jpeg, pm = pre.run(frame, altitude_m=args.altitude, map_gsd_m_px=map_gsd)
        t2 = time.perf_counter()
        pub.send(K.T_FRAME, {"seq": i, "t": time.time()}, jpeg)
        got = sub.recv(500)
        t3 = time.perf_counter()
        if i < args.warmup:
            continue
        cap.append((t1 - t0) * 1000)
        # run() times its own work; the difference is the JPEG encode
        prep.append(pm["preprocess_ms"])
        enc.append((t2 - t1) * 1000 - pm["preprocess_ms"])
        bus.append((t3 - t2) * 1000 if got else float("nan"))
        total.append((t3 - t0) * 1000)

    feed.close(); pub.close(); sub.close()
    if not total:
        print("no samples collected")
        return 1

    print(f"\nN={len(total)} in {time.perf_counter()-t_start:.1f}s | "
          f"frame {out.shape[1]}x{out.shape[0]} | jpeg {len(jpeg)//1024} KB")
    if pm["note"]:
        print(f"NOTE {pm['note']}")
    print(f"\n  {'stage':12s} {'median':>9s} {'p95':>9s} {'max':>9s}")
    for name, v in (("capture", cap), ("preprocess", prep), ("encode", enc), ("bus", bus)):
        vv = [x for x in v if x == x]
        print(f"  {name:12s} {statistics.median(vv):9.2f} {pct(vv,0.95):9.2f} {max(vv):9.2f}")
    print(f"  {'-'*12} {'-'*9} {'-'*9} {'-'*9}")
    print(f"  {'OVERHEAD':12s} {statistics.median(total):9.2f} {pct(total,0.95):9.2f} {max(total):9.2f}")

    o95 = pct(total, 0.95)
    budget = cfg.get("processing_layer.latency_budget_ms", 250)
    print(f"\nOVERHEAD_MS (p95) = {o95:.0f} ms")
    print(f"Leaves {budget - o95:.0f} ms of the {budget} ms budget for detect + match + solve.")
    print("Compare against the warm p95 for your matcher. On a Pi 5, xfeat_cpu warm was")
    print("161.6 ms, so it fits only while OVERHEAD_MS stays under 88 ms.")
    t = device.read_temp_c()
    if t:
        print(f"\nboard temperature {t:.1f} C at the end of the run")
    if board.kind == "jetson":
        print("Did you run `sudo nvpmodel -m 0 && sudo jetson_clocks` first? "
              "Without it this measures the governor, not the board.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
