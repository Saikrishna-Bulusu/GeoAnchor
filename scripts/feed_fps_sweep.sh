#!/usr/bin/env bash
# feed.fps sweep -- reproduce CLAUDE.md's "the feed rate curve is not
# monotonic" table on THIS board. Never inherit fps:2 from the Xavier numbers;
# re-run this and read the row for this board's own compute cost.
#
#     bash scripts/feed_fps_sweep.sh                  100s per arm, fps 2,4,8,16,30
#     bash scripts/feed_fps_sweep.sh 60 2,4,8          override duration and fps list
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
if [ -d .venv ]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi
exec python3 scripts/feed_fps_sweep.py "$@"
