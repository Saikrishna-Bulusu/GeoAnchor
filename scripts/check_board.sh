#!/usr/bin/env bash
#
# Can THIS board run GeoAnchor? Run it on the board and read the verdict.
#
#     bash scripts/check_board.sh
#     bash scripts/check_board.sh --overhead 132   # if you have measured it
#     bash scripts/check_board.sh --quick          # skip the matcher benchmark
#
# It answers three questions in order, and stops at the first hard no:
#
#   1. Does the runtime FIT IN RAM here?      hard limit, no way around it
#   2. Does a matcher RUN here at all?        needs torch + the weights
#   3. Does a matcher that WORKS fit the      the deployment question
#      250 ms budget once overhead is paid?
#
# Question 3 is the one that has been got wrong repeatedly, in both directions.
# A board that fails it is still useful: the joules-per-fix curve across compute
# classes is this project's headline contribution, and "this class cannot do it"
# is a measurement, not a failure. The script says so rather than just failing.
#
set -uo pipefail
cd "$(dirname "$0")/.."
REPO="$PWD"

OVERHEAD_MS=""
QUICK=0
REPS=9
while [ $# -gt 0 ]; do
  case "$1" in
    --overhead) OVERHEAD_MS="$2"; shift 2 ;;
    --quick)    QUICK=1; shift ;;
    --reps)     REPS="$2"; shift 2 ;;
    -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done

PY="$REPO/.venv/bin/python"
[ -x "$PY" ] || PY="$(command -v python3)"
[ -x "$PY" ] || { echo "no python3 and no .venv -- run bootstrap.sh first"; exit 1; }

echo "==============================================================="
echo " GeoAnchor board viability"
echo " $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "==============================================================="
echo

# ---------------------------------------------------------------- board ----
"$PY" - <<'PYEOF'
import sys, os, shutil
sys.path.insert(0, os.getcwd())
try:
    from geoanchor import device
    b = device.detect()
    print(f"  board        {b.model}")
    print(f"  arch         {b.arch}, {b.cores} cores, {b.ram_gb} GB RAM")
    if b.jetpack_hint:
        print(f"  jetpack      {b.jetpack_hint}")
except Exception as e:
    print(f"  board        detection failed: {e}")

# Clock pinning. The governor string is NOT the test -- `performance` can still
# sit over a min<max range and step down between reps. This has cost this
# project two runs, once on each board it has been measured on.
def rd(p):
    try:    return open(p).read().strip()
    except Exception: return None
gov = rd("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
lo  = rd("/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq")
hi  = rd("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq")
if gov:
    pinned = (lo is not None and lo == hi)
    print(f"  governor     {gov}   min={lo} max={hi}   {'PINNED' if pinned else '*** NOT PINNED ***'}")
    if not pinned:
        print("               timings below will drift. Pin with:")
        print(f"               echo {hi} | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_min_freq")
try:
    la = os.getloadavg()[0]; nc = os.cpu_count() or 1
    flag = "" if la < nc/4 else "   *** BUSY -- torch paths will read slow ***"
    print(f"  loadavg      {la:.2f} on {nc} cores{flag}")
except Exception:
    pass
PYEOF
echo

# ------------------------------------------------------------------ RAM ----
# The only hard wall. Measured, not assumed: import the real chain and read
# peak RSS. A Pi Zero 2W (512 MB) cannot hold the processing layer alone.
echo "---------------------------------------------------------------"
echo " 1. RAM"
echo "---------------------------------------------------------------"
"$PY" - <<'PYEOF'
import resource, sys, os
sys.path.insert(0, os.getcwd())
def rss(): return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024

total_mb = None
try:
    for line in open("/proc/meminfo"):
        if line.startswith("MemTotal:"):
            total_mb = int(line.split()[1])/1024; break
except Exception: pass

import numpy, cv2                      # noqa
try:
    import torch; torch.set_num_threads(1)
    peak = rss()
    have_torch = True
except Exception as e:
    peak = rss(); have_torch = False
    print(f"  torch        NOT IMPORTABLE: {e}")

print(f"  peak RSS     {peak:.0f} MB for one layer's imports"
      f"{' (torch loaded)' if have_torch else ' (NO torch)'}")

# Four processes: data, processing, output, api. Only processing pays for the
# model and the detect buffers; the others are opencv/fastapi-sized. This is
# the measured shape on x86, and it is the shape that decides a 512 MB board.
est = peak + 120 + 60 + 40 + 90       # processing + detect headroom + data + output + api
if total_mb:
    print(f"  MemTotal     {total_mb:.0f} MB")
    print(f"  estimate     ~{est:.0f} MB for all four layers + OS headroom")
    if est > total_mb * 0.92:
        print()
        print("  VERDICT      *** WILL NOT FIT ***")
        print("               The runtime needs more memory than this board has.")
        print("               This is a hard limit -- no matcher choice changes it.")
        print("               Use this board as a MAVLink bridge or log shipper.")
        sys.exit(3)
    print(f"  headroom     {total_mb - est:.0f} MB")
    print("  VERDICT      fits")
PYEOF
RAM_RC=$?
echo
if [ $RAM_RC -eq 3 ]; then
  echo "Stopping here: RAM is a hard wall and the questions below are moot."
  exit 3
fi

# -------------------------------------------------------------- matchers ---
echo "---------------------------------------------------------------"
echo " 2. Which matchers are available"
echo "---------------------------------------------------------------"
"$PY" scripts/preflight.py 2>/dev/null | grep -E "methods|method edge|torch|cv2" || true
echo

if [ "$QUICK" = "1" ]; then
  echo "--quick given: skipping the benchmark. Re-run without it for a verdict."
  exit 0
fi

# ---------------------------------------------------------------- timing ---
echo "---------------------------------------------------------------"
echo " 3. Timing, at one tile / 2048 reference keypoints"
echo "---------------------------------------------------------------"
echo " Comparing anything to another board REQUIRES this geometry --"
echo " match cost is linear in reference keypoints, and a 25-tile store"
echo " against a 1-tile store is a 15x difference that is not a board result."
echo
OUT="results/board_check_$(hostname -s)_$(date -u +%Y%m%dT%H%M%SZ).json"
mkdir -p results
"$PY" scripts/bench_matchers.py --tiles 1 --reps "$REPS" --json "$OUT" 2>&1 | tail -40
echo
echo "  raw: $OUT"
echo

# --------------------------------------------------------------- verdict ---
echo "---------------------------------------------------------------"
echo " VERDICT"
echo "---------------------------------------------------------------"
OVERHEAD_MS="$OVERHEAD_MS" "$PY" - "$OUT" <<'PYEOF'
import json, os, sys

path = sys.argv[1]
try:
    data = json.load(open(path))
except Exception as e:
    print(f"  could not read {path}: {e}"); sys.exit(1)

rows = data.get("rows") or []
if not rows:
    print(f"  no rows in {path} -- did the benchmark find any stores?"); sys.exit(1)

# p95 is the number the budget is judged against, not the median: a fix that
# lands late is silently stamped as current and fused at the wrong time, so the
# tail is the thing that hurts.
def total_of(r):
    for k in ("total_p95_ms", "total_ms"):
        if isinstance(r.get(k), (int, float)): return r[k]
    return None

env = os.environ.get("OVERHEAD_MS") or ""
if env.strip():
    overhead, src = float(env), "given on the command line"
else:
    overhead, src = 45.7, "NOT MEASURED HERE -- the Xavier's 45.7 ms, used as a placeholder"

BUDGET = 250.0
avail = BUDGET - overhead

# Which matchers actually produce fixes against a real satellite reference.
# From env80, 326 frames, both scenes: ORB and SIFT returned ZERO plausible
# fixes on Scene_10; the few they return on Scene_09 are wrong by 62-127 m.
# A matcher that fits the budget and cannot localise is not a solution.
WORKS = {"xfeat_mnn", "xfeat_lg", "edgepoint2_s64", "edgepoint2_s32", "edgepoint2_t32"}
BROKEN_NOTE = {
    "orb":   "zero plausible fixes in 192 Scene_10 frames",
    "sift":  "zero plausible fixes in 192 Scene_10 frames",
    "akaze": "6 plausible fixes of 192 Scene_10 frames",
}

print(f"  budget       {BUDGET:.0f} ms (ArduPilot VISO_DELAY_MS cap)")
print(f"  overhead     {overhead:.1f} ms  ({src})")
print(f"  available    {avail:.1f} ms for detect + match + solve")
print()

fits_working, fits_broken, over = [], [], []
for r in rows:
    name, t = r.get("method"), total_of(r)
    if not name or t is None:
        continue
    if t > avail:
        over.append((name, t))
    elif name in WORKS:
        fits_working.append((name, t))
    else:
        fits_broken.append((name, t))

for name, t in sorted(fits_working, key=lambda x: x[1]):
    print(f"  FITS    {name:18s} {t:8.1f} ms   and localises on real reference")
for name, t in sorted(fits_broken, key=lambda x: x[1]):
    print(f"  fits*   {name:18s} {t:8.1f} ms   * {BROKEN_NOTE.get(name,'not validated on real reference')}")
for name, t in sorted(over, key=lambda x: x[1]):
    tag = "" if name in WORKS else "  (and does not localise anyway)"
    print(f"  over    {name:18s} {t:8.1f} ms   by {t-avail:+.0f} ms{tag}")

# A store is built per (map, method), so a matcher with no store at this
# geometry is simply absent from the table -- silently. That is how the
# project's own recommended matcher can vanish from a verdict, so say it.
seen = {r.get("method") for r in rows}
missing = sorted(WORKS - seen)
if missing:
    print()
    print(f"  NOT MEASURED: {', '.join(missing)}")
    print("               No 1-tile store exists for these on this board, so they")
    print("               are absent from the table above -- not fast, not slow.")
    print("               Build one and re-run, or the verdict is incomplete:")
    print()
    print("                   python -m geoanchor.data_layer --build-map \\")
    print("                       --set data_layer.map.method=edgepoint2_s64")

print()
if fits_working:
    best = min(fits_working, key=lambda x: x[1])
    print(f"  ==> CLOSED LOOP IS REACHABLE on this board with {best[0]}.")
    print(f"      Set processing_layer.method to it and re-run verify.sh.")
elif fits_broken:
    print("  ==> The only matchers that fit are ones that do not localise against")
    print("      a real satellite reference. This board CANNOT close the loop.")
    print("      It is still a valid joules-per-fix curve point -- run the")
    print("      benchmark, record the energy, and report it as a measured")
    print("      limit of this compute class. Do not fly it.")
else:
    print("  ==> Nothing fits the budget on this board.")
    print("      Still useful as a curve point. Do not fly it.")

if not env.strip():
    print()
    print("  *** OVERHEAD_MS WAS NOT MEASURED ON THIS BOARD. ***")
    print("      Everything above shifts by however wrong the placeholder is,")
    print("      and it has already varied 3x between two boards in this")
    print("      project (45.7 ms Xavier, 132.1 ms Pi 5 -- the camera, not the")
    print("      board). Measure it, then re-run with --overhead <ms>:")
    print()
    print("          python scripts/measure_overhead.py")
PYEOF
