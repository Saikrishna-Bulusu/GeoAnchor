#!/usr/bin/env bash
# pack_for_board.sh -- tar up everything a second board needs that git does NOT carry.
#
#     bash scripts/pack_for_board.sh              minimal: run the sydney replay
#     bash scripts/pack_for_board.sh --with-env80 add the AnyVisLoc scenes (2.4 GB)
#
# The code goes over git. This covers the four things .gitignore excludes that
# the board cannot regenerate cheaply, or at all:
#
#   stores/   the built feature stores. THE POINT OF COPYING THESE is that the
#             board then never needs rasterio/GDAL -- see the note below.
#   data/     the source imagery. Needed even with the store already built,
#             because store_id() hashes the source file's BYTES to find the
#             store. It reads them as raw bytes, so no GeoTIFF reader is
#             involved on the cache-hit path.
#   demo/     flight.mp4 and its ground-truth sidecar: the only feed that
#             exists without a camera.
#   results/  this board's numbers, so the other board can diff rather than
#             start from nothing.
#
# xfeat/, edgepoint2/ and .venv/ are deliberately NOT included: bootstrap.sh
# clones and builds those, and a venv is not portable across architectures or
# python versions anyway.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

WITH_ENV80=0
OUT="${OUT:-$HERE/geoanchor-payload-$(hostname -s)-$(date -u +%Y%m%d).tar.gz}"
while [ $# -gt 0 ]; do
  case "$1" in
    --with-env80) WITH_ENV80=1; shift ;;
    -o|--out) OUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
done

# data/ is a symlink here (-> /data/datasets). -h makes tar follow it and store
# real files; without it the other board unpacks a dangling link to a path that
# does not exist there.
ITEMS=(demo results stores)
[ -e data/sydney ] && ITEMS+=(data/sydney)
if [ "$WITH_ENV80" = 1 ]; then
  [ -e data/AnyVisLoc ]       && ITEMS+=(data/AnyVisLoc)
  [ -e data/AnyVisLoc_env80 ] && ITEMS+=(data/AnyVisLoc_env80)
fi

echo "packing:"
for i in "${ITEMS[@]}"; do printf '  %-22s %s\n' "$i" "$(du -shL "$i" 2>/dev/null | cut -f1)"; done
echo
tar -czhf "$OUT" "${ITEMS[@]}"
echo "wrote $OUT  ($(du -sh "$OUT" | cut -f1))"
echo
echo "on the other board:"
echo "  git clone <repo> geoanchor-rt && cd geoanchor-rt"
echo "  tar -xzf $(basename "$OUT")          # into the repo root"
echo "  bash bootstrap.sh"
echo "  bash run.sh"
