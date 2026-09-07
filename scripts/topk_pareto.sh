#!/usr/bin/env bash
# top_k Pareto front on this board: latency here, accuracy on env80.
# CLAUDE.md "Open, in order" item 3.
#
#     bash scripts/topk_pareto.sh
#
# Ascending k on purpose. LighterGlue's attention is quadratic in keypoints, so
# k=4096 costs ~30 s/frame on the Xavier -- about 135 min for the single point
# CLAUDE.md already calls unusable ("XFEAT_LG does not fit even warm"). Running
# it last means the informative part of the front exists within the hour and
# the expensive tail can be killed without losing it.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source .venv/bin/activate
for K in 512 1024 2048 4096; do
  echo "===================== top_k=$K ====================="
  date -u +'start %H:%M:%SZ'
  python3 scripts/env80_sweep.py \
      --methods xfeat_mnn,xfeat_lg \
      --frame-keypoints "$K" --ref-keypoints "$K" \
      --out "results/topk_sweep/k${K}" 2>&1
  date -u +'done  %H:%M:%SZ'
done
echo "ALL DONE"
