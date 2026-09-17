#!/usr/bin/env bash
#
# The PX4 half of the flight-stack validation, end to end in one invocation.
#
#     bash scripts/px4_sitl.sh                      boot, configure, check, watch
#     bash scripts/px4_sitl.sh --px4 /path/to/PX4-Autopilot
#     bash scripts/px4_sitl.sh --keep               leave SITL running afterwards
#
# The ArduPilot equivalent is sitl.sh. This answers the same two questions
# against EKF2 instead of EKF3: are the parameters right, and does the
# estimator actually WALK to a position we inject once GNSS is taken away.
#
# WHY SIH AND NOT GAZEBO. PX4's simulation-in-hardware runs the vehicle model
# inside the PX4 process, with simulated GPS, baro and mag. No Gazebo, no
# gz-transport, no render. The estimator, the MAVLink handler and the parameter
# set are the real ones, which is the entire scope of this test -- nothing here
# depends on the flight dynamics being pretty.
#
# WHY IT BOOTS TWICE. EKF2_EV_DELAY and EKF2_HGT_REF are reboot_required in
# PX4's own parameter metadata: set them at runtime and they are accepted,
# stored and IGNORED until the next boot. SITL persists parameters into
# parameters.bson in its working directory, so the second boot comes up
# configured. This is the same "set them and reboot the autopilot" that
# check_extnav.py tells you to do on hardware, done without a human.
#
# WHAT IT DOES NOT COVER. SITL shares PX4's estimator and MAVLink handler but
# not the serial link, the timing or the camera. Same limit as the ArduPilot
# run in results/sitl_ardupilot_2026-09-17.md.
set -uo pipefail
cd "$(dirname "$0")/.."
REPO="$PWD"

PX4_DIR="${PX4_DIR:-/home/sai/thesis_2.0/PX4-Autopilot}"
WORK="${PX4_SITL_WORK:-$REPO/.px4_sitl}"
# PX4 streams continuously to 14540 (its Onboard instance's configured remote)
# and accepts inbound on 14580. It will NOT re-target 14580 to a second peer,
# so listening and writing have to be different sockets -- see the docstring in
# scripts/check_extnav.py.
LISTEN="udpin:0.0.0.0:14540"
WRITE="udpout:127.0.0.1:14580"
KEEP=0

while [ $# -gt 0 ]; do
  case "$1" in
    --px4)  PX4_DIR="$2"; shift 2 ;;
    --keep) KEEP=1; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done

BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
ETC="$PX4_DIR/build/px4_sitl_default/etc"
if [ ! -x "$BIN" ]; then
  echo "no PX4 SITL binary at $BIN"
  echo "build it with:  cd $PX4_DIR && make px4_sitl_default"
  exit 1
fi

PX4PID=""
cleanup() {
  [ "$KEEP" = "0" ] && stop_px4
}
trap cleanup EXIT

# Killing the SUBSHELL is not killing PX4. The first version of this wrapped
# the launch in `( cd ... ) &`, so $! was the subshell and `kill` left the px4
# process holding /tmp/px4_lock-0 -- the second boot then died on "PX4 server
# already running for instance 0", which reads as a stale lock and is not one.
# -w already sets the working directory, so no cd and no subshell is needed.
stop_px4() {
  [ -n "$PX4PID" ] || return 0
  kill "$PX4PID" 2>/dev/null
  for _ in $(seq 1 20); do
    kill -0 "$PX4PID" 2>/dev/null || break
    sleep 0.5
  done
  kill -9 "$PX4PID" 2>/dev/null
  wait "$PX4PID" 2>/dev/null
  PX4PID=""
  # The lock is what the next boot trips over, so wait for it rather than
  # sleeping a guessed number of seconds.
  for _ in $(seq 1 20); do
    [ -e /tmp/px4_lock-0 ] || break
    sleep 0.5
  done
}

boot() {
  # $1 = label for the log
  PX4_SYS_AUTOSTART=10040 PX4_SIMULATOR=sihsim PX4_SIM_MODEL=quadx \
      "$BIN" -i 0 -d "$ETC" -s etc/init.d-posix/rcS -w "$WORK" \
      > "$WORK/px4_$1.log" 2>&1 &
  PX4PID=$!
  for _ in $(seq 1 40); do
    grep -q "Ready for takeoff" "$WORK/px4_$1.log" 2>/dev/null && return 0
    kill -0 "$PX4PID" 2>/dev/null || { echo "PX4 exited during boot:"; tail -20 "$WORK/px4_$1.log"; return 1; }
    sleep 1
  done
  echo "PX4 did not reach 'Ready for takeoff' in 40s:"; tail -20 "$WORK/px4_$1.log"; return 1
}

# START FROM DEFAULTS. PX4 SITL persists parameters into parameters.bson in
# its working directory, so a previous run's state leaks into this one -- and
# one of the parameters this test touches is EKF2_GPS_CTRL. An autosaved 0
# there boots the vehicle permanently GNSS-denied: it never reaches "Ready for
# takeoff", never takes an origin, and this test fails reporting "SITL never
# got GPS lock" against a firmware that is fine. watch_extnav.py now restores
# it, and this makes the run reproducible regardless.
rm -rf "$WORK"
mkdir -p "$WORK"
echo "PX4     : $PX4_DIR"
echo "work    : $WORK"
echo "listen  : $LISTEN"
echo "write   : $WRITE"
echo

echo "=== 1/4  boot, and apply configs/px4_extnav.params ======================"
boot first || exit 1
"$REPO/.venv/bin/python" "$REPO/scripts/px4_set_params.py" \
    --endpoint "$WRITE" --file "$REPO/configs/px4_extnav.params" || exit 1
stop_px4

echo
echo "=== 2/4  reboot, so the reboot_required parameters take ================="
boot second || exit 1

echo
echo "=== 3/4  check_extnav.py ==============================================="
"$REPO/.venv/bin/python" "$REPO/scripts/check_extnav.py" \
    --endpoint "$LISTEN" --write-endpoint "$WRITE" --seconds 6
CHECK=$?

echo
echo "=== 4/4  watch_extnav.py -- GNSS off, 20 m injected ===================="
"$REPO/.venv/bin/python" "$REPO/scripts/watch_extnav.py" \
    --endpoint "$LISTEN" --write-endpoint "$WRITE" --offset-m 20 --seconds 30
WATCH=$?

echo
if [ "$CHECK" = "0" ] && [ "$WATCH" = "0" ]; then
  echo "both stages returned 0. Read the drift column: walking toward 20 m is"
  echo "EKF2 fusing our fix; holding at 0 is it ignoring one."
else
  echo "check_extnav exited $CHECK, watch_extnav exited $WATCH"
fi
[ "$KEEP" = "1" ] && echo "SITL left running as pid $PX4PID"
exit $(( CHECK != 0 || WATCH != 0 ))
