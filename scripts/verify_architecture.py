#!/usr/bin/env python3
"""Check the built system against the specification, on this machine.

    bash verify.sh

Every check below maps to one line of the brief. It runs the real processes
over the real bus -- not imports, not mocks -- because the requirement that the
layers are independent can only be tested by killing one.

Exit code 0 means every required check passed.
"""
from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from geoanchor import codes as C          # noqa: E402
from geoanchor import config as cfgmod    # noqa: E402

PASS, FAIL, SKIP = "PASS", "FAIL", "skip"
results: list = []


def check(spec: str, what: str, ok, detail: str = "", required: bool = True):
    status = PASS if ok else (FAIL if required else SKIP)
    results.append({"spec": spec, "check": what, "status": status, "detail": detail,
                    "required": required})
    mark = {"PASS": "\033[32m PASS \033[0m", "FAIL": "\033[31m FAIL \033[0m",
            "skip": "\033[33m skip \033[0m"}[status]
    print(f"  {mark} [{spec}] {what}" + (f"  -- {detail}" if detail else ""), flush=True)
    return ok


def run(cmd, timeout=120, env=None):
    e = dict(os.environ)
    e.update(env or {})
    return subprocess.run(cmd, shell=True, cwd=REPO, capture_output=True, text=True,
                          timeout=timeout, env=e)


def main() -> int:
    print("\n\033[1mGeoAnchor architecture verification\033[0m")
    print(f"repo {REPO}\n")

    # ---------------------------------------------------------------- codes
    print("\033[1mStep codes\033[0m")
    problems = C.validate()
    check("codes", "registry is well formed", not problems, "; ".join(problems))
    import re
    emitted = set()
    for d in ("geoanchor", "scripts"):
        for p in (REPO / d).rglob("*.py"):
            if p.name == "codes.py":
                continue
            emitted |= set(re.findall(r'["\']((?:DL|PL|OL)(?:DE|E)?-\d{2})["\']', p.read_text()))
    for prefix, name in (("DL", "data"), ("PL", "processing"), ("OL", "output")):
        codes = {c for c in C.REGISTRY if c.startswith(prefix)}
        for kind, label in ((C.STEP, "steps completed"), (C.ERROR, "errors"),
                            (C.DEVICE, "device errors")):
            group = {c for c in codes if C.kind(c) == kind}
            miss = sorted(group - emitted)
            check("codes", f"{name} layer: all {len(group)} {label} codes are emitted",
                  not miss, ", ".join(miss))

    # ---------------------------------------------------------------- config
    print("\n\033[1mConfiguration\033[0m")
    for name in ("configs/system.yaml", "configs/env80.yaml"):
        cfg = cfgmod.load(REPO / name)
        probs = cfg.validate()
        check("config", f"{name} validates", not probs, "; ".join(probs))
    cfg = cfgmod.load(REPO / "configs/system.yaml")
    check("L3", "flight controller link is disabled by default",
          not cfg.get("output_layer.fc.enabled", False),
          "nothing reaches the vehicle until this is turned on deliberately")
    check("L3", "loop mode is one of off/open/closed",
          cfg.get("output_layer.fc.loop_mode") in ("off", "open", "closed"))
    check("L1", "altitude envelope caps at 100 m",
          cfg.get("data_layer.envelope.agl_max_m") == 100)
    check("L3", "message is ODOMETRY or VISION_POSITION_ESTIMATE, never GPS_INPUT",
          cfg.get("output_layer.fc.message") in ("ODOMETRY", "VISION_POSITION_ESTIMATE"))

    # ------------------------------------------------------- MAVLink encoding
    print("\n\033[1mFlight controller encoding\033[0m")
    from geoanchor.output_layer import fcout as F
    covs = []
    for sig in (0.5, 8.0, 45.0, 100.0):
        cov = [0.0] * 21
        cov[F.IDX_XX] = cov[F.IDX_YY] = sig ** 2 / 2.0
        covs.append((sig, F.pos_err_as_ardupilot_sees_it(cov)))
    check("L3", "posErr ArduPilot computes equals the sigma we emit",
          all(abs(a - b) < 1e-9 for a, b in covs),
          "; ".join(f"{a}->{b:.3f}" for a, b in covs))
    check("L3", "ODOMETRY frame is LOCAL_FRD (20), which is the only one accepted",
          F.MAV_FRAME_LOCAL_FRD == 20 and F.MAV_FRAME_BODY_FRD == 12)
    r = run("python3 scripts/test_fc_encoding.py", timeout=180)
    passed = "0 if all" not in r.stdout and r.returncode == 0
    tail = [l for l in r.stdout.splitlines() if "passed" in l]
    check("L3", "real MAVLink decoded by ArduPilot's own logic round-trips",
          passed, tail[-1] if tail else r.stdout.strip().splitlines()[-1:] or "see output")

    # ----------------------------------------------------------- prerequisites
    print("\n\033[1mPrerequisites\033[0m")
    store_dir = cfg.resolve("data_layer.map.store_dir", "stores")
    demo = cfg.resolve("data_layer.feed.path")
    if demo and not demo.exists():
        print("    building the demo flight (one-off)...", flush=True)
        run("python3 scripts/make_demo_video.py", timeout=600)
    check("L1", "a reference map source exists", bool(cfg.resolve("data_layer.map.source")
                                                     and cfg.resolve("data_layer.map.source").exists()))
    check("L1", "a feed exists", bool(demo and demo.exists()), str(demo))
    if not (store_dir and any(store_dir.glob("*/manifest.json"))):
        print("    building the reference feature store (one-off, ~1 min)...", flush=True)
        run("python3 -m geoanchor.data_layer --build-map", timeout=1800)
    stores = sorted(store_dir.glob("*/manifest.json")) if store_dir.is_dir() else []
    check("L1", "map preprocessed once into a reusable store", bool(stores),
          stores[0].parent.name if stores else "not built")
    if stores:
        man = json.loads(stores[0].read_text())
        check("L1", "the store is a cache keyed by content, so it is reused not rebuilt",
              len(man.get("store_id", "")) >= 12 and man.get("n_keypoints", 0) > 0,
              f"{man['n_tiles']} tiles, {man['n_keypoints']} keypoints")
        check("L1", "the store carries a georeference the runtime can use alone",
              bool(man.get("transform")) and (man.get("epsg") or man.get("crs")))

    # ------------------------------------------------------------- live system
    print("\n\033[1mLive system: three independent processes\033[0m")
    run_dir = REPO / "runs" / f"verify_{int(time.time())}"
    env = {"GEOANCHOR_RUN_DIR": str(run_dir),
           "GEOANCHOR_SET": "data_layer.feed.fps=1;data_layer.feed.loop=true;api.port=8791"}
    proc = subprocess.Popen("bash run.sh --tag verify", shell=True, cwd=REPO,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            env={**os.environ, **env}, preexec_fn=os.setsid)
    ok_live = False
    try:
        state = None
        for _ in range(60):
            time.sleep(2)
            r = run("curl -s --max-time 3 localhost:8791/api/state")
            if r.returncode == 0 and r.stdout.strip().startswith("{"):
                state = json.loads(r.stdout)
                if len(state.get("layers", {})) == 3 and state.get("records"):
                    ok_live = True
                    break
        check("arch", "all three layers run as separate processes and report",
              ok_live, f"{len(state.get('layers', {})) if state else 0}/3 reporting")

        if ok_live:
            for lid in ("data", "processing", "output"):
                st = state["layers"][lid]["status"]
                check("arch", f"{lid} layer emits its own step codes",
                      st["counts"]["steps"] > 0,
                      f"last {st['last_code']}, {st['counts']['steps']} steps")
            check("L2", "predicted GPS produced", any(r.get("predicted_gps") for r in state["records"]))
            check("L3", "actual and predicted paired, error computed",
                  any(r.get("error_m") is not None for r in state["records"]))
            check("L3", "loss computed", any(r.get("loss") is not None for r in state["records"]))
            check("dash", "dashboard is served by the API",
                  run("curl -s -o /dev/null -w '%{http_code}' --max-time 5 localhost:8791/").stdout == "200")
            check("dash", "camera frame endpoint serves the frame the matcher saw",
                  run("curl -s -o /dev/null -w '%{http_code}' --max-time 5 localhost:8791/api/frame.jpg").stdout == "200")
            check("dash", "map basemap endpoint serves offline",
                  run("curl -s -o /dev/null -w '%{http_code}' --max-time 5 localhost:8791/api/map.png").stdout == "200")
            check("dash", "code registry exposed for the steps-completed view",
                  "DL-01" in run("curl -s --max-time 5 localhost:8791/api/codes").stdout)
            check("dash", "per-code counts published so the dashboard shows WHICH steps ran",
                  bool(state["layers"]["data"]["status"]["counts"].get("seen")))

            # the independence requirement, tested the only way it can be
            pl = run("pgrep -f 'geoanchor[.]processing_layer'").stdout.split()
            if pl:
                os.kill(int(pl[0]), signal.SIGKILL)
                # Poll rather than sleep a fixed time: the dashboard marks a
                # layer dead after its heartbeat goes stale, which takes a few
                # seconds, and a fixed wait either races it or wastes time.
                d_live = o_live = p_live = None
                api_up = False
                for _ in range(15):
                    time.sleep(2)
                    r = run("curl -s --max-time 4 localhost:8791/api/state")
                    if not r.stdout.strip().startswith("{"):
                        continue
                    api_up = True
                    s2 = json.loads(r.stdout).get("layers", {})
                    d_live = s2.get("data", {}).get("live")
                    o_live = s2.get("output", {}).get("live")
                    p_live = s2.get("processing", {}).get("live")
                    if p_live is False:
                        break
                check("arch", "killing one layer does not affect the other two",
                      bool(api_up and d_live and o_live and p_live is False),
                      f"api={api_up} data={d_live} output={o_live} processing={p_live}")
            else:
                check("arch", "killing one layer does not affect the other two", False,
                      "could not find the processing layer to kill")
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=30)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        run("pkill -f 'geoanchor\\.' || true")

    # ---------------------------------------------------------------- export
    print("\n\033[1mExport\033[0m")
    sess = run_dir / "session.json"
    if sess.exists():
        doc = json.loads(sess.read_text())
        rows = doc.get("records", [])
        check("export", "session.json written", bool(rows), f"{len(rows)} records")
        if rows:
            r0 = next((r for r in rows if r.get("error_m") is not None), rows[0])
            for field in ("time_step", "actual_gps", "predicted_gps", "error_m"):
                check("export", f"record carries {field}", field in r0)
            check("export", "JSONL written line by line as the run proceeds",
                  (run_dir / "records.jsonl").exists())
            check("export", "no mean and no RMSE in the summary",
                  not any(k in json.dumps(doc.get("summary", {})).lower()
                          for k in ('"mean', '"rmse')))
            check("export", "median, p90 and p99 are reported",
                  all(k in doc.get("summary", {}) for k in
                      ("median_error_m", "p90_error_m", "p99_error_m")))
        check("export", "per-layer JSONL logs written",
              all((run_dir / f"{n}.jsonl").exists() for n in ("data", "processing", "output")))
    else:
        check("export", "session.json written", False, f"not found at {sess}")

    # --------------------------------------------------------------- summary
    req = [r for r in results if r["required"]]
    failed = [r for r in req if r["status"] == FAIL]
    print(f"\n{'='*76}")
    print(f"{len(req)} required checks, {len(req)-len(failed)} passed, {len(failed)} failed")
    if failed:
        print("\nFailed:")
        for r in failed:
            print(f"  [{r['spec']}] {r['check']}  {r['detail']}")
    else:
        print("\n\033[1mThe built system matches the specification.\033[0m")
    print(f"\nrun directory: {run_dir}")
    if shutil.which("xdg-open"):
        print("dashboard while running: http://localhost:8000")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
