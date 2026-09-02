#!/usr/bin/env bash
# sweep.sh -- run every env80 frame through the pipeline and print the table.
#
#     bash sweep.sh                       both scenes, five matchers, satellite
#     bash sweep.sh --modes satellite,aerial
#     bash sweep.sh --no-rectify          the A/B that shows stage 02 matters
#     FORCE=1 bash sweep.sh               redo combinations already on disk
#
# Idempotent: a combination whose CSV exists is reused. Safe to interrupt and
# re-run; only the unfinished combinations cost anything.
#
# RUNTIME: roughly 0.3-1.5 s per frame per matcher on a laptop CPU, plus about
# 10 s to build each reference store the first time. 326 frames across five
# matchers is tens of minutes. xfeat_lg is the slow one; the classical matchers
# are several times faster than XFeat.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
if [ -d .venv ]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi
exec "${PYTHON:-python3}" scripts/env80_sweep.py "$@"
