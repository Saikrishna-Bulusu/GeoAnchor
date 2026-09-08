#!/usr/bin/env python3
"""Sweep data_layer.feed.fps and report latency/staleness/accuracy per arm.

    bash scripts/feed_fps_sweep.sh                  100s per arm, fps 2,4,8,16,30
    bash scripts/feed_fps_sweep.sh 60 2,4,8          override duration and fps list

Same methodology as CLAUDE.md's "the feed rate curve is not monotonic" table:
one run.sh per arm, feed.loop=true, first 5 fixes dropped as warm-up (frame 1
pays the lazy model-load cost). staleness = latency_ms - sum(stage_ms),
excluding tiles_fitted, which is a count, not a duration.

CLAUDE.md is explicit that the Xavier's fps:2 choice is board-specific and
must be re-measured, not inherited -- this script is that re-measurement,
run fresh on whatever board it executes on.
"""
from __future__ import annotations

import json
import os
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

DEFAULT_FPS = [2, 4, 8, 16, 30]
DEFAULT_DURATION = 100
WARMUP_DROP = 5


def pctl(vals: list[float], p: float) -> float:
    if not vals:
        return float("nan")
    s = sorted(vals)
    k = (len(s) - 1) * p
    f = int(k)
    c = min(f + 1, len(s) - 1)
    if f == c:
        return s[f]
    return s[f] + (s[c] - s[f]) * (k - f)


def run_arm(fps: float, duration: int) -> dict:
    tag = f"fpssweep_{str(fps).replace('.', 'p')}"
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    run_dir = REPO / "runs" / f"{stamp}_{tag}"
    env = {
        **os.environ,
        "GEOANCHOR_RUN_DIR": str(run_dir),
        "GEOANCHOR_SET": f"data_layer.feed.fps={fps};data_layer.feed.loop=true",
    }
    proc = subprocess.Popen(
        ["bash", "run.sh", "--no-api", "--tag", tag],
        cwd=REPO, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    time.sleep(duration)
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=10)

    recs_path = run_dir / "records.jsonl"
    records = []
    if recs_path.exists():
        for line in recs_path.read_text().splitlines():
            line = line.strip()
            if line:
                records.append(json.loads(line))
    records = records[WARMUP_DROP:]

    latencies, computes, stalenesses, errs = [], [], [], []
    for r in records:
        stage = r.get("stage_ms") or {}
        compute = sum(v for k, v in stage.items() if k != "tiles_fitted")
        computes.append(compute)
        if r.get("latency_ms") is not None:
            latencies.append(r["latency_ms"])
            stalenesses.append(r["latency_ms"] - compute)
        if r.get("error_m") is not None:
            errs.append(r["error_m"])

    return {
        "fps": fps,
        "run_dir": str(run_dir),
        "n": len(records),
        "fixes_per_s": len(records) / duration if duration else float("nan"),
        "compute_med": statistics.median(computes) if computes else float("nan"),
        "latency_med": statistics.median(latencies) if latencies else float("nan"),
        "latency_p95": pctl(latencies, 0.95),
        "staleness_med": statistics.median(stalenesses) if stalenesses else float("nan"),
        "staleness_p95": pctl(stalenesses, 0.95),
        "err_m_med": statistics.median(errs) if errs else float("nan"),
    }


def main() -> None:
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DURATION
    fps_list = [float(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else DEFAULT_FPS

    header = f"{'fps':>5} {'n':>4} {'fix/s':>6} {'compute':>8} {'latency':>8} {'lat_p95':>8} {'staleness':>10} {'stl_p95':>8} {'err_m_med':>10}"
    print(header, flush=True)

    rows = []
    for fps in fps_list:
        print(f"--- fps={fps}, {duration}s, loop=true ---", flush=True)
        row = run_arm(fps, duration)
        rows.append(row)
        print(
            f"{row['fps']:>5} {row['n']:>4} {row['fixes_per_s']:>6.2f} "
            f"{row['compute_med']:>8.1f} {row['latency_med']:>8.1f} {row['latency_p95']:>8.1f} "
            f"{row['staleness_med']:>10.1f} {row['staleness_p95']:>8.1f} {row['err_m_med']:>10.4f}",
            flush=True,
        )

    out = REPO / "results" / "feed_fps_sweep.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(rows, indent=2))
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
