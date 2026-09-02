#!/usr/bin/env bash
# verify.sh -- check the built system against the specification, on this machine.
#
#     bash verify.sh
#
# Runs the real processes over the real bus, produces a session, kills a layer
# to prove the other two survive, and checks the export. Builds the demo flight
# and the reference store first if they are missing.
#
# RUNTIME: about 3 minutes once the store exists, 5-6 the first time.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
if [ -d .venv ]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi
exec "${PYTHON:-python3}" scripts/verify_architecture.py "$@"
