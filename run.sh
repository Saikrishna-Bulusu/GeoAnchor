#!/usr/bin/env bash
# run.sh -- start the three layers and the dashboard API as independent processes.
#
#     bash run.sh                 everything
#     bash run.sh --no-api        layers only
#     bash run.sh --layers data,processing
#     bash run.sh --config configs/xavier.yaml
#
# Each layer is its own process with its own exit code. If one dies the others
# keep going, which is the point of the architecture -- so this script reports
# the death rather than tearing the system down. Ctrl-C stops all of them
# cleanly, in reverse order, so the output layer flushes its session file last.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

CONFIG="${GEOANCHOR_CONFIG:-configs/system.yaml}"
LAYERS="data,processing,output"
WITH_API=1
TAG=""

while [ $# -gt 0 ]; do
  case "$1" in
    --config) CONFIG="$2"; shift 2 ;;
    --layers) LAYERS="$2"; shift 2 ;;
    --tag)    TAG="$2"; shift 2 ;;
    --no-api) WITH_API=0; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
done

[ -f "$CONFIG" ] || { echo "config not found: $CONFIG"; exit 2; }
export GEOANCHOR_CONFIG="$HERE/$CONFIG"

if [ -d .venv ]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi
PY="${PYTHON:-python3}"

# One run directory shared by all three layers, so the session can be
# reassembled afterwards. Without this each layer makes its own and the three
# JSONL files scatter.
# An inherited GEOANCHOR_RUN_DIR wins, so a caller -- verify.sh, a sweep, a
# scheduled run -- can pin the directory it will read results back from.
# Overriding it here unconditionally meant the caller looked in one place while
# the layers wrote to another, and the run appeared to produce nothing.
if [ -z "${GEOANCHOR_RUN_DIR:-}" ]; then
  RUN_TAG="${TAG:-$($PY -c "import yaml,sys;print((yaml.safe_load(open('$CONFIG')) or {}).get('session',{}).get('tag',''))" 2>/dev/null)}"
  STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
  export GEOANCHOR_RUN_DIR="$HERE/runs/${STAMP}${RUN_TAG:+_$RUN_TAG}"
fi
mkdir -p "$GEOANCHOR_RUN_DIR"

echo "config   $CONFIG"
echo "run dir  $GEOANCHOR_RUN_DIR"
echo "layers   $LAYERS$([ "$WITH_API" = 1 ] && echo ', api')"
echo

PIDS=(); NAMES=()
start() {
  local name="$1"; shift
  "$@" > "$GEOANCHOR_RUN_DIR/$name.out" 2>&1 &
  PIDS+=("$!"); NAMES+=("$name")
  printf '  %-11s pid %-7s -> %s\n' "$name" "$!" "$GEOANCHOR_RUN_DIR/$name.out"
}

shutdown() {
  echo
  echo "stopping (reverse order, so the session file flushes last)"
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    kill -TERM "${PIDS[$i]}" 2>/dev/null && printf '  TERM %s\n' "${NAMES[$i]}"
  done
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    for _ in $(seq 1 50); do kill -0 "${PIDS[$i]}" 2>/dev/null || break; sleep 0.1; done
    kill -0 "${PIDS[$i]}" 2>/dev/null && kill -KILL "${PIDS[$i]}" 2>/dev/null
  done
  echo
  echo "session: $GEOANCHOR_RUN_DIR"
  ls -1 "$GEOANCHOR_RUN_DIR" 2>/dev/null | sed 's/^/  /'
}
trap shutdown EXIT INT TERM

# The output layer starts first so it is subscribed before the first fix
# exists; the data layer starts last because it is the one that produces work.
case ",$LAYERS," in *,output,*)     start output     $PY -m geoanchor.output_layer ;; esac
case ",$LAYERS," in *,processing,*) start processing $PY -m geoanchor.processing_layer ;; esac
[ "$WITH_API" = 1 ] && start api $PY -m geoanchor.api
sleep 1
case ",$LAYERS," in *,data,*)       start data       $PY -m geoanchor.data_layer ;; esac

echo
echo "running. Ctrl-C to stop.  Dashboard API: http://$(hostname -I 2>/dev/null | awk '{print $1}'):$($PY -c "import yaml;print((yaml.safe_load(open('$CONFIG')) or {}).get('api',{}).get('port',8000))" 2>/dev/null || echo 8000)"
echo

# Watch for a layer dying. Report it; do not kill the others.
while true; do
  alive=0
  for i in "${!PIDS[@]}"; do
    if kill -0 "${PIDS[$i]}" 2>/dev/null; then
      alive=$((alive+1))
    elif [ "${NAMES[$i]}" != "_done" ]; then
      wait "${PIDS[$i]}" 2>/dev/null; rc=$?
      printf '  %s exited (rc %s) -- the other layers are still running\n' "${NAMES[$i]}" "$rc"
      NAMES[$i]="_done"
    fi
  done
  [ "$alive" -eq 0 ] && break
  sleep 1
done
