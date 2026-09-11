#!/usr/bin/env bash
# bootstrap.sh -- run ONCE on the board. Idempotent: safe to re-run any time.
#
#     cd ~/geoanchor-rt && bash bootstrap.sh
#
# Fails loudly rather than half-installing, and prints exactly what is missing.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
FAIL=0
step(){ printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok(){   printf '   ok    %s\n' "$*"; }
warn(){ printf '   note  %s\n' "$*"; }
bad(){  printf '   FAIL  %s\n' "$*"; FAIL=1; }

step "1/9  board"
MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
ARCH="$(uname -m)"
echo "   ${MODEL}  |  ${ARCH}  |  $(nproc) cores"
free -h 2>/dev/null | awk '/Mem:/{print "   ram   "$2" total, "$7" available"}'
df -h "$HERE" | awk 'NR==2{print "   disk  "$4" free on "$6}'

L4T=""
if [ -f /etc/nv_tegra_release ]; then
  L4T="$(sed -n 's/.*# R\([0-9]*\).*REVISION: \([0-9.]*\).*/\1.\2/p' /etc/nv_tegra_release)"
  echo "   L4T   ${L4T:-unknown}"
fi
IS_JETSON=0; IS_XAVIER=0
case "$MODEL" in *Jetson*|*NVIDIA*|*tegra*) IS_JETSON=1 ;; esac
case "$MODEL" in *Xavier*) IS_XAVIER=1 ;; esac

if [ "$IS_XAVIER" = 1 ]; then
  ok "AGX Xavier"
  warn "Xavier tops out at JetPack 5.1.x (L4T 35.x, Ubuntu 20.04, Python 3.8, CUDA 11.4)."
  warn "JetPack 6 is Orin-only, so do NOT follow Orin instructions for wheels or TensorRT."
elif [ "$IS_JETSON" = 1 ]; then
  ok "Jetson: $MODEL"
else
  warn "not a Jetson. Timing and energy measured here are not board results."
fi

step "2/9  python"
PYBIN="${PYTHON:-python3}"
PYV="$($PYBIN -V 2>&1 | cut -d' ' -f2)"
echo "   $PYBIN is $PYV"
PYMINOR="$(echo "$PYV" | cut -d. -f2)"
if [ "$PYMINOR" -lt 8 ]; then
  bad "Python $PYV is too old. Nothing here works below 3.8."
elif [ "$PYMINOR" -eq 8 ]; then
  warn "Python 3.8 (the JetPack 5 system Python). torch publishes cp38 aarch64 wheels"
  warn "only up to 2.4.x, so that is what gets pinned below. If you would rather have a"
  warn "newer stack, install python3.10 from deadsnakes and re-run with PYTHON=python3.10."
else
  ok "Python $PYV"
fi

step "3/9  system packages"
NEED=""
for p in python3-venv python3-dev git build-essential; do
  dpkg -s "$p" >/dev/null 2>&1 || NEED="$NEED $p"
done
if [ -n "$NEED" ]; then
  echo "   installing:$NEED"
  sudo apt-get update -qq && sudo apt-get install -y -qq $NEED && ok "installed" || bad "apt install failed"
else
  ok "python3-venv, python3-dev, git, build-essential present"
fi

step "4/9  virtualenv"
[ -d .venv ] || $PYBIN -m venv .venv || bad "venv create failed"
# shellcheck disable=SC1091
source .venv/bin/activate || { bad "venv activate failed"; exit 1; }
python -m pip install -q --upgrade pip wheel setuptools
ok "$(python -V) in $HERE/.venv"

step "5/9  torch (CPU only -- this is the step that goes wrong)"
# ---------------------------------------------------------------------------
# The whole pipeline is CUDA-free on purpose: the same code then runs on the
# Pi 5, the Xavier, the Orin Nano and the 2019 Nano, so joules per fix compares
# the BOARDS rather than four different implementations.
#
# Getting a CPU wheel on aarch64 is not the default. PyTorch 2.11.0 dropped the
# `platform_machine == "x86_64"` guard on its CUDA dependencies and kept only
# `platform_system == "Linux"`, because ARM now also means Jetson and Grace.
# Nothing checks whether the board can use them, so an unpinned install on any
# ARM Linux board drags in cudnn, cublas, cusparselt and triton -- over a
# gigabyte that will never execute. It cost 15 minutes on a Pi 5 on 2 Sept 2026.
#
# On a Jetson there is a second trap: PyPI's CUDA wheels are built for discrete
# and SBSA GPUs, not for Jetson's integrated iGPU. Installing one appears to
# work and then fails at the first kernel launch. NVIDIA's own Jetson wheels are
# the only CUDA-capable option here, and this project does not want one.
# ---------------------------------------------------------------------------
if python -c "import torch" 2>/dev/null; then
  ok "torch $(python -c 'import torch;print(torch.__version__)') already installed"
else
  if [ "$PYMINOR" -eq 8 ]; then
    TORCH_SPEC="torch==2.4.1"
  elif [ "$ARCH" = "aarch64" ] && ! grep -qw asimddp /proc/cpuinfo 2>/dev/null; then
    # Cortex-A72 and older (Pi 4 and earlier: ARMv8.0-A, no FEAT_DotProd /
    # asimddp) SIGILLs on a bare `import torch` from 2.10.0 onward -- oneDNN's
    # ACL backend emits SDOT/UDOT unconditionally there rather than dispatching
    # on runtime CPU features. 2.9.0 is the newest wheel confirmed clean.
    # Caught on a Pi 4B, 10 Sept 2026 -- see CLAUDE.md "Board notes: Pi 4".
    TORCH_SPEC="torch<2.10,>=2.6"
  else
    TORCH_SPEC="torch<2.11"
  fi
  echo "   installing $TORCH_SPEC -- 100-200 MB, several minutes, and it looks stalled."
  echo "   Watch from another terminal with:  watch -n 5 'du -sh ~/.cache/pip'"
  PIPI=(python -m pip install --progress-bar on --timeout 120 --retries 3 --only-binary=:all:)
  "${PIPI[@]}" "$TORCH_SPEC" \
    || "${PIPI[@]}" "$TORCH_SPEC" --index-url https://download.pytorch.org/whl/cpu \
    || bad "torch install failed -- see the notes at the bottom"
fi
if python -c "import torch" 2>/dev/null; then
  python - <<'PY'
import sys, torch
cuda = torch.version.cuda
print(f"   torch {torch.__version__}  cuda_build={cuda}  threads={torch.get_num_threads()}")
if cuda is not None:
    print("   FAIL  this is a CUDA build. On an ARM board that is the wrong wheel:")
    print("         it is either dead weight or, on a Jetson, built for the wrong GPU.")
    print("         Remove it and re-run:  pip uninstall -y torch")
    sys.exit(1)
PY
  [ $? -ne 0 ] && FAIL=1
fi

step "6/9  python packages"
PIPI=(python -m pip install --progress-bar on --timeout 120 --retries 3)
"${PIPI[@]}" -r requirements.txt && ok "runtime requirements" || bad "requirements.txt"
# kornia carries LighterGlue. XFeat pins 0.7.2; newer versions have changed the
# shapes its LightGlue path expects more than once.
"${PIPI[@]}" "kornia==0.7.2" >/dev/null 2>&1 && ok "kornia 0.7.2 (XFeat's pin)" \
  || { "${PIPI[@]}" kornia >/dev/null 2>&1 && warn "kornia unpinned -- if xfeat_lg misbehaves, pin 0.7.2" \
       || warn "kornia not installed -- xfeat_lg will be unavailable, xfeat_mnn still works"; }
"${PIPI[@]}" -r requirements-mapprep.txt >/dev/null 2>&1 && ok "rasterio (GeoTIFF ingest)" \
  || warn "rasterio not installed -- this board cannot ingest a NEW GeoTIFF. Build the store
          on a machine that has it and copy the stores/ directory across; the runtime does
          not need GDAL."

step "7/9  XFeat"
XR=""
for c in "$HERE/xfeat" "$HERE/../third_party/accelerated_features" "$HOME/GeoAnchor/third_party/accelerated_features"; do
  [ -f "$c/modules/xfeat.py" ] && { XR="$c"; break; }
done
if [ -z "$XR" ]; then
  echo "   cloning accelerated_features (Apache 2.0, ~20 MB with weights)"
  git clone -q --depth 1 https://github.com/verlab/accelerated_features.git "$HERE/xfeat" \
    && XR="$HERE/xfeat" || warn "clone failed -- set XFEAT_ROOT by hand, or use a classical method"
fi
if [ -n "$XR" ] && [ -f "$XR/weights/xfeat.pt" ]; then
  ok "XFeat at $XR"
  export XFEAT_ROOT="$XR"
else
  warn "XFeat unavailable. orb, sift and akaze still work and need no weights."
fi

step "8/9  EdgePoint2"
# Optional. Faster than XFeat on this class of hardware and its failures are far
# less wild (see CLAUDE.md), but every pipeline default still runs without it.
ER=""
for c in "$HERE/edgepoint2" "$HERE/../third_party/EdgePoint2" "$HOME/GeoAnchor/third_party/EdgePoint2"; do
  [ -f "$c/edgepoint2.py" ] && [ -f "$c/model/model.py" ] && { ER="$c"; break; }
done
if [ -z "$ER" ]; then
  echo "   cloning EdgePoint2 (MIT, ~11 MB with all 14 weight sets)"
  git clone -q --depth 1 https://github.com/HITCSC/EdgePoint2.git "$HERE/edgepoint2" \
    && ER="$HERE/edgepoint2" || warn "clone failed -- set EDGEPOINT2_ROOT by hand; edgepoint2_* stays unavailable"
fi
if [ -n "$ER" ] && [ -f "$ER/weights/S64.pth" ]; then
  ok "EdgePoint2 at $ER"
  export EDGEPOINT2_ROOT="$ER"
else
  warn "EdgePoint2 unavailable. Every other method is unaffected."
fi

step "9/9  verify"
python scripts/preflight.py
[ $? -ne 0 ] && FAIL=1

if [ "$IS_JETSON" = 1 ]; then
  step "jetson: clocks"
  echo "   Timing numbers are only comparable at a fixed power mode and clock."
  echo "   Before any benchmark run, do this once per boot:"
  echo "       sudo nvpmodel -m 0 && sudo jetson_clocks"
  echo "   and record 'sudo nvpmodel -q' in the run notes."
fi

echo
if [ "$FAIL" = 0 ]; then
  printf '\033[1mReady.\033[0m\n'
  echo "  build the reference store:  source .venv/bin/activate && python -m geoanchor.data_layer --build-map"
  echo "  run everything:             bash run.sh"
  echo "  dashboard:                  http://$(hostname -I 2>/dev/null | awk '{print $1}'):8000"
else
  printf '\033[1mFinished with failures above.\033[0m Fix them before trusting any number.\n'
  echo
  echo "torch failed?   Check python is 3.8-3.12 and the arch is aarch64 or x86_64."
  echo "                A 32-bit OS (armv7l) has no torch wheel at all."
  echo "rasterio failed? Not fatal. Build the store elsewhere and copy stores/ across."
fi
exit "$FAIL"
