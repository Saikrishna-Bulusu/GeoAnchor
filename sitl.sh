#!/usr/bin/env bash
# sitl.sh -- open-loop validation against ArduPilot SITL.
#
#     bash sitl.sh                  check parameters, then run a session
#     bash sitl.sh --set-params     write the ExternalNav parameters first
#     bash sitl.sh --skip-session   parameters only
#
# Start SITL first, in another terminal:
#     sim_vehicle.py -v ArduCopter --console --out=udp:127.0.0.1:14550
#
# --set-params WRITES to the autopilot. SITL only unless you are certain.
#
# RUNTIME: about two minutes for the default 90 s session.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
if [ -d .venv ]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi
exec "${PYTHON:-python3}" scripts/sitl_openloop.py "$@"
