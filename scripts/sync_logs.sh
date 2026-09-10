#!/usr/bin/env bash
#
# Push this device's session transcripts to the shared logs repo, and pull
# everyone else's back.
#
#     bash scripts/sync_logs.sh              push mine, pull theirs
#     bash scripts/sync_logs.sh --pull-only  just refresh the fleet view
#     bash scripts/sync_logs.sh --dry-run    show what would be copied
#
# Layout in the logs repo, which is what /api/fleet reads:
#
#     <device>/<run-id>/session.json
#     <device>/<run-id>/records.jsonl
#     <device>/<run-id>/*.jsonl
#     <device>/board.json                    what this device is
#
# WHY A SEPARATE REPO. Session transcripts are ~100 KB per run and every board
# writes them continuously. In the code repo they would bloat history forever
# and every device would be committing to the same branch on every run, which
# is a merge conflict per sync. Here each device owns its own directory, so two
# devices can never touch the same file and the merge is always trivial.
#
# WHAT IS NOT SYNCED. `stores/` (rebuildable, and 16-41 MB each), `.venv`,
# `data/` and the raw `*.out` console logs. The JSONL transcripts and
# session.json are the record; everything else is derivable or huge.
#
set -uo pipefail
cd "$(dirname "$0")/.."
REPO="$PWD"

LOGS_REMOTE="${GEOANCHOR_LOGS_REMOTE:-git@github.com:Saikrishna-Bulusu/geoanchor-logs.git}"
FLEET_DIR="${GEOANCHOR_FLEET_DIR:-$REPO/fleet}"
DEVICE="${GEOANCHOR_DEVICE:-$(hostname -s)}"
PULL_ONLY=0
DRY=0

while [ $# -gt 0 ]; do
  case "$1" in
    --pull-only) PULL_ONLY=1; shift ;;
    --dry-run)   DRY=1; shift ;;
    --remote)    LOGS_REMOTE="$2"; shift 2 ;;
    --device)    DEVICE="$2"; shift 2 ;;
    -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done

command -v git >/dev/null || { echo "git not found"; exit 1; }

echo "device : $DEVICE"
echo "remote : $LOGS_REMOTE"
echo "clone  : $FLEET_DIR"
echo

# ---------------------------------------------------------------- clone ----
if [ ! -d "$FLEET_DIR/.git" ]; then
  echo "no clone yet -- cloning"
  if ! git clone "$LOGS_REMOTE" "$FLEET_DIR" 2>&1 | sed 's/^/  /'; then
    echo
    echo "Clone failed. If the repo does not exist yet, create it once:"
    echo "    gh repo create Saikrishna-Bulusu/geoanchor-logs --private"
    echo "Then re-run. Until then this device keeps its runs locally and"
    echo "nothing is lost -- runs/ is the source of truth, fleet/ is a mirror."
    exit 1
  fi
fi

cd "$FLEET_DIR"

# Pull first, always. A device that has been off for a week must not push a
# branch that is behind and then force anything.
#
# Except on the very first sync, when the remote has no commits at all: there
# is no ref to merge with and `git pull` fails rather than no-opping. That is
# the FIRST run of the FIRST device, so failing there would mean the sync could
# never bootstrap itself.
if git rev-parse --verify -q HEAD >/dev/null 2>&1 || \
   [ -n "$(git ls-remote --heads origin 2>/dev/null)" ]; then
  echo "pulling"
  git pull --rebase --autostash 2>&1 | sed 's/^/  /' || {
    echo "  pull failed -- resolve by hand in $FLEET_DIR, then re-run"
    exit 1
  }
else
  echo "remote is empty -- this is the first sync, nothing to pull"
fi

if [ "$PULL_ONLY" = "1" ]; then
  echo
  echo "pull-only: done. $(find . -name session.json -not -path './.git/*' | wc -l) sessions from $(find . -maxdepth 1 -type d -not -name .git -not -name . | wc -l) devices."
  exit 0
fi

# ----------------------------------------------------------------- push ----
mkdir -p "$DEVICE"

# A one-line record of what this device IS, so a session's numbers can be read
# against the hardware that produced them without guessing from the hostname.
"$REPO/.venv/bin/python" - "$DEVICE/board.json" <<'PYEOF' 2>/dev/null || true
import json, os, sys, platform
sys.path.insert(0, os.environ.get("REPO", "."))
info = {"hostname": platform.node(), "arch": platform.machine(),
        "python": platform.python_version()}
try:
    from geoanchor import device
    b = device.detect()
    info.update(model=b.model, cores=b.cores, ram_gb=b.ram_gb,
                jetpack=b.jetpack_hint)
except Exception as e:
    info["detect_error"] = str(e)
try:
    for line in open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq"):
        info["max_freq_khz"] = int(line.strip()); break
except Exception:
    pass
open(sys.argv[1], "w").write(json.dumps(info, indent=2) + "\n")
PYEOF

COPIED=0
for d in "$REPO"/runs/*/; do
  [ -d "$d" ] || continue
  name="$(basename "$d")"
  [ -f "$d/session.json" ] || continue          # unfinished run, skip it
  dest="$DEVICE/$name"

  # Skip a run already pushed and unchanged, so a sync after a quiet hour is
  # a no-op rather than a rewrite of every file.
  if [ -f "$dest/session.json" ] && \
     [ "$d/session.json" -ot "$dest/session.json" ]; then
    continue
  fi

  if [ "$DRY" = "1" ]; then
    echo "  would copy $name"
    COPIED=$((COPIED+1))
    continue
  fi
  mkdir -p "$dest"
  cp "$d/session.json" "$dest/" 2>/dev/null || true
  for f in "$d"/*.jsonl; do
    [ -f "$f" ] && cp "$f" "$dest/"
  done
  COPIED=$((COPIED+1))
done

if [ "$DRY" = "1" ]; then
  echo
  echo "dry run: $COPIED runs would be copied. Nothing written."
  exit 0
fi

echo "staged $COPIED runs from $DEVICE"

if [ -z "$(git status --porcelain)" ]; then
  echo "nothing changed -- already in sync"
  exit 0
fi

git add -A
git -c user.name="$DEVICE" \
    -c user.email="$DEVICE@geoanchor.local" \
    commit -q -m "$DEVICE: $COPIED run(s), $(date -u +%Y-%m-%dT%H:%M:%SZ)"

echo "pushing"
if git push 2>&1 | sed 's/^/  /'; then
  echo
  echo "done. $(find . -name session.json -not -path './.git/*' | wc -l) sessions from $(ls -d */ 2>/dev/null | wc -l) devices."
else
  echo
  echo "Push failed. The commit is local in $FLEET_DIR and nothing is lost;"
  echo "re-run when the network is back."
  exit 1
fi
