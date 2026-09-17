#!/usr/bin/env bash
#
# Is the 3-year cross-date cliff a property of the MATCHER, or of the Sydney CBD?
#
#     bash scripts/crossdate_cities.sh
#     bash scripts/crossdate_cities.sh --areas melbourne,perth
#
# results/crossdate_wayback_curve.md measured one place: a 510 x 500 m tile over
# the Sydney CBD. It found a sharp cliff between 2.8 and 3.6 years -- 20-31
# inliers and 3.3-7.9 m median inside it, 5 inliers and 0-2 of 24 solved past
# it. A single site cannot tell a matcher property from a site property, and the
# Sydney CBD is an unusually bad site to generalise from: dense high-rise, so
# the dominant change between captures is facade parallax and shadow rather than
# genuine ground change.
#
# This runs the SAME probe over Melbourne, Brisbane and Perth CBDs and one rural
# area, on tiles of the same ground span, so the only thing varying is where.
#
# THREE THINGS THAT WOULD OTHERWISE INVALIDATE IT, all handled:
#
#  * THE CONTROL IS NOT OPTIONAL. Each area is first matched against ITSELF. A
#    zero-fix result is otherwise indistinguishable from a broken harness, a bad
#    tile, or an area with no texture -- and the rural area is exactly where
#    that ambiguity would bite. The control runs first and its failure aborts
#    that area rather than producing a table of zeros.
#  * RESOLUTION IS HELD CONSTANT at zoom 19 = 0.2476 m/px, the same zoom as the
#    Sydney curve. Resolution is a confound that hides the cliff: a coarser tile
#    has less high-frequency detail to lose, so it degrades more gracefully and
#    the cliff moves.
#  * THE QUERY IS ALWAYS THE NEWEST CAPTURE, matched against each older one, so
#    "gap" means the same thing everywhere.
#
# Wayback is global and needs no token. Esri imagery here is measurement-only
# working data and is not redistributed.
set -uo pipefail
cd "$(dirname "$0")/.."

AREAS="melbourne,brisbane,perth,rural_griffith"
METHOD="edgepoint2_s64"
FRAMES=24
GATE=8
MAXGAP=""

while [ $# -gt 0 ]; do
  case "$1" in
    --areas)  AREAS="$2"; shift 2 ;;
    --method) METHOD="$2"; shift 2 ;;
    --frames) FRAMES="$2"; shift 2 ;;
    --gate)   GATE="$2"; shift 2 ;;
    --max-gap) MAXGAP="$2"; shift 2 ;;
    -h|--help) sed -n '2,35p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done

mkdir -p results/crossdate_cities

IFS=',' read -ra LIST <<< "$AREAS"
for area in "${LIST[@]}"; do
  dir="data/crossdate/$area"
  if [ ! -d "$dir" ]; then
    echo "no tiles for $area -- fetch them first with scripts/fetch_wayback_tile.py"
    continue
  fi
  newest="$(ls "$dir"/ref_tile_*.tif 2>/dev/null | sort | tail -1)"
  if [ -z "$newest" ]; then
    echo "no tiles in $dir"; continue
  fi
  echo
  echo "================================================================"
  echo "  $area   query $(basename "$newest")"
  echo "================================================================"

  # CONTROL FIRST. Same tile both sides. If this does not solve, nothing else
  # from this area means anything and the run stops here rather than producing
  # a column of zeros that reads like a cross-date result.
  echo "--- control: same tile both sides"
  .venv/bin/python scripts/crossdate_probe.py --query "$newest" \
      --methods "$METHOD" --frames "$FRAMES" --gate "$GATE" \
      --json "results/crossdate_cities/${area}__control.json" 2>&1 | tail -6
  solved=$(.venv/bin/python - "results/crossdate_cities/${area}__control.json" <<'PY'
import json, sys
# crossdate_probe.py writes a top-level LIST, one entry per method.
try:
    rows = json.load(open(sys.argv[1]))
except Exception:
    rows = []
print(max([r.get("solved", 0) for r in rows] or [0]))
PY
)
  if [ "${solved:-0}" -lt 1 ]; then
    echo "  CONTROL FAILED for $area -- skipping its cross-date runs."
    echo "  A zero-fix result here means the harness or the tile, not the date."
    continue
  fi
  echo "  control solved $solved/$FRAMES -- cross-date results from here are meaningful"

  for ref in $(ls "$dir"/ref_tile_*.tif | sort -r | tail -n +2); do
    [ "$ref" = "$newest" ] && continue
    echo "--- vs $(basename "$ref")"
    .venv/bin/python scripts/crossdate_probe.py --query "$newest" --reference "$ref" \
        --methods "$METHOD" --frames "$FRAMES" --gate "$GATE" \
        --json "results/crossdate_cities/${area}__$(basename "$ref" .tif).json" 2>&1 | tail -5
  done
done
echo
echo "JSON in results/crossdate_cities/. Summarise with scripts/crossdate_table.py"
