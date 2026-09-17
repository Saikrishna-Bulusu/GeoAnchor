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
#     board/<board>/<method>/<run-id>/session.json
#     board/<board>/<method>/<run-id>/records.jsonl
#     board/<board>/<method>/<run-id>/*.jsonl
#     board/<board>/board.json               what this class of board is
#
# Board then method, because that is the comparison the paper makes. Both
# facts are read out of session.json by geoanchor/log_layout.py, so the sort is
# mechanical and a renamed host does not fork the tree.
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

# HTTPS, not SSH: `gh auth login` sets up an HTTPS credential helper, and that
# is what is present on these boards. An SSH default fails on a headless board
# with "Host key verification failed", which reads as a permissions problem
# rather than as a missing key. Override with GEOANCHOR_LOGS_REMOTE.
LOGS_REMOTE="${GEOANCHOR_LOGS_REMOTE:-https://github.com/Saikrishna-Bulusu/geoanchor-logs.git}"
FLEET_DIR="${GEOANCHOR_FLEET_DIR:-$REPO/fleet}"
DEVICE="${GEOANCHOR_DEVICE:-$(hostname -s)}"
PULL_ONLY=0
DRY=0

# A transcript is meant to be ~100 KB. A looping feed left running for days
# produces one per FIX, and one such run reached 402 MB across 337k JSONL
# lines -- which in a git repo is permanent, for a synthetic replay whose only
# content is that the wiring works. Runs above this are skipped with a line
# saying so; raise it with --max-run-mb when a big one is genuinely wanted.
MAX_RUN_MB="${GEOANCHOR_MAX_RUN_MB:-32}"

while [ $# -gt 0 ]; do
  case "$1" in
    --pull-only) PULL_ONLY=1; shift ;;
    --dry-run)   DRY=1; shift ;;
    --remote)    LOGS_REMOTE="$2"; shift 2 ;;
    --device)    DEVICE="$2"; shift 2 ;;
    --max-run-mb) MAX_RUN_MB="$2"; shift 2 ;;
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
  echo "pull-only: done. $(find . -name session.json -not -path './.git/*' | wc -l) sessions from $(ls -d board/*/ 2>/dev/null | wc -l) boards."
  exit 0
fi

# ----------------------------------------------------------------- push ----
# A one-line record of what this device IS, so a session's numbers can be read
# against the hardware that produced them without guessing from the hostname.
#
# The repo path is passed as argv[2], NOT read from the environment. It used to
# be `os.environ.get("REPO", ".")`, and `REPO` is a plain shell variable that
# was never exported -- so the lookup always missed, fell back to ".", and "."
# at this point is $FLEET_DIR rather than the repo. Every board.json ever
# written by this script therefore carried
#     "detect_error": "No module named 'geoanchor'"
# and none of them recorded model, cores or RAM. That is the entire reason
# board.json exists, and it silently did not do it on any device.
BOARD_SLUG="$(PYTHONPATH="$REPO" "$REPO/.venv/bin/python" -m geoanchor.log_layout --board-slug 2>/dev/null || echo unknown-board)"
mkdir -p "board/$BOARD_SLUG"
"$REPO/.venv/bin/python" - "board/$BOARD_SLUG/board.json" "$REPO" <<'PYEOF' 2>/dev/null || true
import json, os, sys, platform
sys.path.insert(0, sys.argv[2])
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
SKIPPED=0
for d in "$REPO"/runs/*/; do
  [ -d "$d" ] || continue
  name="$(basename "$d")"
  [ -f "$d/session.json" ] || continue          # unfinished run, skip it

  # board/<board>/<method>/<run>/ -- see geoanchor/log_layout.py. The device
  # name is NOT part of the path: two boards of the same kind belong in the
  # same directory, and the hostname is already recorded inside session.json.
  dest="$(PYTHONPATH="$REPO" "$REPO/.venv/bin/python" -m geoanchor.log_layout --dest "$d/session.json" "$name" 2>/dev/null)"
  if [ -z "$dest" ]; then
    echo "  skipping $name -- cannot classify its session.json"
    continue
  fi

  # Skip a run already pushed and unchanged, so a sync after a quiet hour is
  # a no-op rather than a rewrite of every file.
  if [ -f "$dest/session.json" ] && \
     [ "$d/session.json" -ot "$dest/session.json" ]; then
    continue
  fi

  # Size guard, measured over exactly the files that would be copied.
  mb=$(du -cm "$d/session.json" "$d"/*.jsonl 2>/dev/null | tail -1 | cut -f1)
  if [ -n "$mb" ] && [ "$mb" -gt "$MAX_RUN_MB" ]; then
    echo "  skipping $name -- ${mb} MB exceeds --max-run-mb $MAX_RUN_MB"
    SKIPPED=$((SKIPPED+1))
    continue
  fi

  if [ "$DRY" = "1" ]; then
    echo "  would copy $name (${mb} MB)"
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

echo "staged $COPIED runs from $DEVICE${SKIPPED:+, skipped $SKIPPED as oversized}"

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
  echo "done. $(find . -name session.json -not -path './.git/*' | wc -l) sessions from $(ls -d board/*/ 2>/dev/null | wc -l) boards."
else
  echo
  echo "Push failed. The commit is local in $FLEET_DIR and nothing is lost;"
  echo "re-run when the network is back."
  exit 1
fi
