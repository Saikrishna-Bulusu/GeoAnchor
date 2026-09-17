#!/usr/bin/env bash
#
# The whole system, on a fixed wing, in Gazebo, end to end.
#
#     bash scripts/sim_fixedwing.sh              Gazebo + PX4 + pipeline + dashboard
#     bash scripts/sim_fixedwing.sh --headless   no GUI (CI, or a remote box)
#     bash scripts/sim_fixedwing.sh --no-pipeline  just the simulator, to look at it
#
# WHAT THIS IS FOR. Every other rig in this repo tests one seam: sitl.sh the
# MAVLink write path, px4_sitl.sh the PX4 estimator, verify.sh the layer
# boundaries, the replay the matcher. This is the only one where a camera on a
# moving aircraft produces frames that become fixes that reach a flight
# controller's estimator, with nothing stubbed anywhere in between.
#
# THE GROUND IS NOT THE REFERENCE MAP, and that is the point. The world is
# textured with one Esri Wayback capture of Griffith, NSW; the pipeline matches
# against a DIFFERENT capture of the same ground, 2.2 years apart. Texturing
# both sides with one image is the hard rule in CLAUDE.md -- it makes the
# pipeline match a picture to itself and report millimetres that mean nothing.
# So this run is also a genuine cross-date test.
#
# WHY GRIFFITH. results/crossdate_cities_2026-09-17.md measured this exact
# area against four high-rise CBDs: it solves 24/24 at every gap out to 8.8
# years while the CBDs fail at months, because facade parallax and shadow both
# scale with building height and Griffith's centre is one and two storeys. A
# demonstration flown over a CBD would fail for a reason that has nothing to do
# with this pipeline.
#
# IT IS A TOWN, NOT FARMLAND. The study labels this area `rural_griffith` and
# called it farmland; looking at the tile shows 2.17 km of suburban streets,
# shops and industrial sheds. The label was wrong and is corrected in that
# document. Nothing in this project has measured open country.
set -uo pipefail
cd "$(dirname "$0")/.."
REPO="$PWD"

PX4_DIR="${PX4_DIR:-/home/sai/thesis_2.0/PX4-Autopilot}"
WORLD="${WORLD:-geoanchor_rural}"
MODEL="${MODEL:-geoanchor_cessna}"
# The reference map. A DIFFERENT capture from the one texturing the ground.
REF="${REF:-data/sim_rural/ref_tile_2024-06-06.tif}"
GROUND_TIF="${GROUND_TIF:-data/sim_rural/ref_tile_2026-08-05.tif}"
CAM_TOPIC="${CAM_TOPIC:-/camera}"
HEADLESS=0
PIPELINE=1
CLOSED=1
FLY=1


say () { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }
PIDS=()
# STOP EVERYTHING, and note what "everything" means here.
#
#   * `gz sim` is a RUBY WRAPPER. The process is called `ruby`, so `pkill -x gz`
#     matches nothing and every previous run's Gazebo survives. Three servers
#     stepping the same world drove the load average to 81, Gazebo's real-time
#     factor to 0.30, and PX4 to `Accel #0 fail: TIMEOUT!` -- which surfaces as
#     a mission upload being rejected, a symptom that points nowhere near the
#     cause. Match on the COMMAND LINE.
#   * the layer processes are run.sh's grandchildren, so killing the PIDs this
#     script collected leaves them running and holding their sockets.
#   * agp_bridge runs under ROS via `bash -c ... exec python3`, so it does not
#     answer to any name this script would think to look for. It was missing
#     from this list and SEVEN of them accumulated across runs, each still
#     subscribed and each still willing to publish to /fmu/in/aux_global_position
#     -- so the next run's flight controller could have been fed by a previous
#     run's pipeline. Nothing visible would have said so.
#   * NEVER `pkill -f` a pattern that matches this script. It matches its own
#     shell and kills the cleanup mid-way, which shows up as a bare exit 144.
cleanup() {
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  sleep 1
  for p in $(ps -eo pid,args | grep -E "gz sim -r|geoanchor\.(data|processing|output)_layer|geoanchor\.api|scripts/agp_bridge\.py" | grep -v grep | awk "{print \$1}"); do
    kill -9 "$p" 2>/dev/null
  done
  for p in "${PIDS[@]:-}" $(pgrep -x px4) $(pgrep -x MicroXRCEAgent); do
    kill -9 "$p" 2>/dev/null
  done
  rm -f /tmp/px4-sock-0 /tmp/px4_lock-0
}
trap cleanup EXIT

while [ $# -gt 0 ]; do
  case "$1" in
    --headless)    HEADLESS=1; shift ;;
    --no-pipeline) PIPELINE=0; shift ;;
    --open-loop)   CLOSED=0; shift ;;
    --no-fly)      FLY=0; shift ;;
    --px4)         PX4_DIR="$2"; shift 2 ;;
    --ref)         REF="$2"; shift 2 ;;
    --kill)        PIDS=(); cleanup; echo "stopped"; trap - EXIT; exit 0 ;;
    -h|--help)     sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done

# ---------------------------------------------------------------- checks ----
say "0/7  preconditions"
STALE="$(ps -eo pid,args | grep -E "gz sim -r|geoanchor\.(data|processing|output)_layer|scripts/agp_bridge\.py" | grep -v grep | wc -l)"
if [ "$STALE" -gt 0 ] || pgrep -x px4 >/dev/null; then
  echo "  REFUSING: a previous run is still up ($STALE gz/layer processes)."
  echo "  Two Gazebo servers stepping one world starve PX4's IMU and the"
  echo "  failure looks like anything but that. Stop it first:"
  echo "    bash scripts/sim_fixedwing.sh --kill"
  exit 1
fi
[ -f "$REPO/sim/gz/worlds/$WORLD.sdf" ] || {
  echo "no world at sim/gz/worlds/$WORLD.sdf"
  echo "build it:  python scripts/make_gz_world.py --tif $GROUND_TIF --name $WORLD"
  exit 1; }
[ -f "$REPO/$REF" ] || { echo "no reference map at $REF"; exit 1; }
[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || {
  echo "no PX4 SITL binary. cd $PX4_DIR && make px4_sitl_default"; exit 1; }

# ROS 2 puts a vendored `gz` shim on PATH that shadows the real binary and
# reports "I cannot find any available 'gz' command" on a machine where Gazebo
# is installed and working. Resolve the real one rather than trusting PATH.
GZ_BIN="$(command -v gz)"
if ! "$GZ_BIN" sim --versions >/dev/null 2>&1; then
  if [ -x /usr/bin/gz ]; then GZ_BIN=/usr/bin/gz
  else echo "gz sim not usable. apt install gz-harmonic"; exit 1; fi
fi
# AND PUT THE REAL ONE FIRST ON PATH, because PX4 calls `gz` itself. Its rcS
# polls for the world with a bare `gz topic -l`, gets ROS's shim, is told
# "I cannot find any available 'gz' command", and sits in "Waiting for Gazebo
# world..." forever against a world that is up and publishing. Resolving
# GZ_BIN for our own use is not enough -- every child needs the same PATH.
export PATH="$(dirname "$GZ_BIN"):$PATH"
echo "  gz        $GZ_BIN ($("$GZ_BIN" sim --version 2>/dev/null | head -1))"
echo "  world     sim/gz/worlds/$WORLD.sdf"
echo "  ground    $GROUND_TIF   (textures the world)"
echo "  reference $REF   (what the pipeline matches against)"
[ "$GROUND_TIF" = "$REF" ] && {
  echo
  echo "  REFUSING: the ground texture and the reference map are the same file."
  echo "  That makes the pipeline match a picture against itself. See CLAUDE.md."
  exit 1; }

# The world's origin IS the ground image's centre, and PX4 has to agree or its
# simulated GPS and this ground are offset by a constant nobody can see.
eval "$(.venv/bin/python - "$REPO/sim/gz/worlds/$WORLD.sdf" <<'PY'
import re, sys
t = open(sys.argv[1]).read()
lat = re.search(r"<latitude_deg>([-\d.]+)", t).group(1)
lon = re.search(r"<longitude_deg>([-\d.]+)", t).group(1)
print(f"export PX4_HOME_LAT={lat}")
print(f"export PX4_HOME_LON={lon}")
PY
)"
export PX4_HOME_ALT=0
echo "  origin    $PX4_HOME_LAT, $PX4_HOME_LON  (world, and PX4's home)"

# ------------------------------------------------------------- gazebo ------
say "1/7  Gazebo"
export GZ_SIM_RESOURCE_PATH="$REPO/sim/gz/models:$REPO/sim/gz/worlds:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
GZ_ARGS="-r"
[ "$HEADLESS" = "1" ] && GZ_ARGS="-r -s --headless-rendering"
"$GZ_BIN" sim $GZ_ARGS "$REPO/sim/gz/worlds/$WORLD.sdf" > /tmp/gz_sim.log 2>&1 &
PIDS+=($!)
echo "  starting, log /tmp/gz_sim.log"
for _ in $(seq 1 60); do
  "$GZ_BIN" topic -l 2>/dev/null | grep -q . && break
  sleep 1
done

# ---------------------------------------------------------------- px4 ------
say "2/7  PX4 SITL, fixed wing, spawning $MODEL"
export PX4_SYS_AUTOSTART=4003              # rc_cessna: fixed-wing gz airframe
export PX4_SIM_MODEL="$MODEL"
export PX4_GZ_WORLD="$WORLD"
mkdir -p "$REPO/.px4_sim"
( cd "$REPO/.px4_sim" && "$PX4_DIR/build/px4_sitl_default/bin/px4" \
    -i 0 -d "$PX4_DIR/build/px4_sitl_default/etc" -s etc/init.d-posix/rcS -w . \
    > /tmp/px4_sim.log 2>&1 ) &
PIDS+=($!)
# NOT "Ready for takeoff". That line waits on every arming check including
# "No connection to the ground control station", which cannot pass until
# sim_fly.py connects and heartbeats -- so waiting for it here deadlocks for
# the full 90 s on a PX4 that is perfectly healthy. Wait for the boot instead
# and let the arming checks be sim_fly.py's problem.
for _ in $(seq 1 90); do
  grep -q "Startup script returned successfully" /tmp/px4_sim.log 2>/dev/null && break
  sleep 1
done
if grep -q "Startup script returned successfully" /tmp/px4_sim.log 2>/dev/null; then
  echo "  PX4 booted"
  grep -q "home set" /tmp/px4_sim.log && echo "  GPS lock, home set"
  # A preflight failure other than the GCS link is a real problem and is worth
  # seeing now rather than as an unexplained arm refusal seven steps later.
  grep "Preflight Fail" /tmp/px4_sim.log | grep -v "ground control station" \
    | sort -u | sed 's/^/  /'
else
  echo "  PX4 did not boot:"; tail -15 /tmp/px4_sim.log
fi

say "3/7  the camera topic"
TOPIC="$("$GZ_BIN" topic -l 2>/dev/null | grep -E "/image$" | head -1)"
if [ -n "$TOPIC" ]; then
  CAM_TOPIC="$TOPIC"
  echo "  $CAM_TOPIC"
else
  echo "  NO IMAGE TOPIC. The model may not carry a camera, or Gazebo is still"
  echo "  loading. \`gz topic -l\` lists what is published."
fi

# READ THE CAMERA'S REAL fx, do not trust the config's.
#
# GSD = altitude / fx_px is the whole scale path, so an fx that belongs to a
# different camera silently rescales every frame by the ratio of the two. The
# config ships fx_px 1200 for the demo video's camera; mono_cam's horizontal
# FOV of 1.74 rad over 1280 px makes the real value 540. Using 1200 shrinks
# each frame to 14% instead of 27%, leaving ~176 px of image, under 30
# keypoints, and a matcher that looks broken. Gazebo publishes the intrinsics
# on the camera_info topic beside the image, so ask.
FX_PX=""
if [ -n "${CAM_TOPIC:-}" ]; then
  INFO_TOPIC="${CAM_TOPIC%/image}/camera_info"
  FX_PX="$("$GZ_BIN" topic -e -n 1 -t "$INFO_TOPIC" 2>/dev/null \
           | awk '/^intrinsics/{f=1} f&&/k: /{print $2; exit}')"
fi
if [ -n "$FX_PX" ]; then
  echo "  fx_px     $FX_PX  (read from $INFO_TOPIC)"
else
  echo "  fx_px     NOT READABLE from camera_info -- falling back to the config,"
  echo "            which describes a different camera. Scale will be wrong."
fi

if [ "$PIPELINE" = "0" ]; then
  say "simulator only (--no-pipeline). Ctrl-C to stop."
  wait; exit 0
fi

# ------------------------------------------------------------ pipeline -----
say "4/7  the three layers, on the Gazebo camera"
# NOTE: no PYTHONPATH here. gz-transport's bindings are APT packages, but
# exporting the system dist-packages puts it AHEAD of the venv and the system
# typing_extensions then shadows the venv's, killing pydantic and fastapi and
# with them the dashboard. GzFeed appends the path itself, at import.
# GPS ENDPOINT: udpOUT to PX4's listening port, NOT udpin on the port PX4
# names as its remote. `udpin` binds and waits, and pymavlink cannot transmit
# on it until a packet has arrived to reveal a peer -- while PX4, for its part,
# does not know where to send until something contacts it. Two listeners, no
# traffic, no error: every frame logs "no altitude or no fx_px", the data layer
# scales to a fixed long edge instead of to the reference GSD, and the
# processing layer matches an unrotated frame. Connecting outward breaks the
# deadlock, and PX4 then streams back to this socket's source port.
GEOANCHOR_SET="\
data_layer.feed.type=gz;\
data_layer.feed.topic=$CAM_TOPIC;\
data_layer.map.source=$REF;\
${FX_PX:+data_layer.feed.intrinsics.fx_px=$FX_PX;data_layer.feed.intrinsics.fy_px=$FX_PX;}\
data_layer.gps.source=mavlink;\
data_layer.gps.endpoint=udpout:127.0.0.1:14580;\
output_layer.fc.firmware=px4;\
output_layer.fc.enabled=false" \
  bash run.sh &
PIDS+=($!)

# ------------------------------------------------------------- AGP path ----
# How a fix reaches THIS flight controller. ArduPilot takes VISION_POSITION_
# ESTIMATE over MAVLink; PX4's equivalent external-vision path has a 200 ms
# rate floor (EV_MAX_INTERVAL) that this pipeline cannot clear on a Pi. Aux
# Global Position has no rate check at all -- only a 5 s timeout -- so it is
# the path that works. It is reachable ONLY over uXRCE-DDS; there is no
# MAVLink message for it. See docs/px4_ekf2_extnav_2026-09-17.md.
if [ "$CLOSED" = "1" ]; then
  say "5/7  the AGP write path"
  # ITS OWN LINK, not the pipeline's. PX4's UDP mavlink locks to one peer per
  # instance, so a second client on 14580 steals the data layer's stream and
  # then exits, leaving PX4 sending telemetry to a socket nobody holds. The
  # symptom is the data layer receiving nothing at all, for the rest of the
  # run. 14280/14030 is a separate instance and costs nothing.
  # ONE invocation for both files. Two would be one too many: PX4 locks the
  # instance to the first client's socket, so the second gets "no heartbeat"
  # and applies nothing while the first still reports success.
  .venv/bin/python scripts/px4_set_params.py \
      --file configs/px4_agp.params configs/px4_sim_fixedwing.params \
      --endpoint udpout:127.0.0.1:14280 \
      2>&1 | sed -u 's/^/  /'
  if command -v MicroXRCEAgent >/dev/null 2>&1; then
    MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1 &
    PIDS+=($!)
    echo "  MicroXRCEAgent on 8888, log /tmp/xrce_agent.log"
    sleep 3
    # ROS, NOT THE VENV. agp_bridge is the one piece of this system that needs
    # rclpy and px4_msgs, which are ROS packages built into a workspace and are
    # not pip-installable into .venv. Running it with .venv/bin/python fails on
    # "No module named 'rclpy'" -- in a background process whose log nobody
    # reads, so the visible symptom is simply cs_aux_gpos staying False while
    # everything upstream looks healthy.
    ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
    WS_SETUP="${WS_SETUP:-$HOME/GeoAnchor/ros2_ws/install/setup.bash}"
    if [ -f "$ROS_SETUP" ] && [ -f "$WS_SETUP" ]; then
      bash -c "set +u; source '$ROS_SETUP'; source '$WS_SETUP';
               exec python3 '$REPO/scripts/agp_bridge.py'" \
           > /tmp/agp_bridge.log 2>&1 &
      PIDS+=($!)
      echo "  agp_bridge -> /fmu/in/aux_global_position, log /tmp/agp_bridge.log"
    else
      echo "  NO ROS 2 WORKSPACE ($ROS_SETUP / $WS_SETUP). AGP is the only path"
      echo "  into PX4's EKF2 and it is ROS-only, so nothing will fuse. Set"
      echo "  ROS_SETUP and WS_SETUP, or run with --open-loop."
    fi
  else
    echo "  NO MicroXRCEAgent on PATH. The pipeline still runs and the dashboard"
    echo "  still shows fixes, but nothing reaches EKF2. Build it:"
    echo "    Micro-XRCE-DDS-Agent/  -- cmake -B build && cmake --build build"
  fi
fi

# ---------------------------------------------------------------- fly -----
if [ "$FLY" = "1" ]; then
  say "6/7  launching"
  .venv/bin/python scripts/sim_fly.py 2>&1 | sed -u 's/^/  /'
fi

say "7/7  running"
cat <<EOF
  Dashboard   http://localhost:8000
  Gazebo GUI  $([ "$HEADLESS" = "1" ] && echo "headless" || echo "open")

  The aircraft is flying a $(printf %.0f 400) m box at 80 m AGL. --no-fly leaves it
  on the ground for QGroundControl to take over.

  Fixes reach EKF2 over AGP. Watch them land:
      grep ' cs_aux_gpos' -r /tmp/px4_sim.log ; or in the PX4 console:
      listener estimator_aid_src_aux_global_position
  --open-loop runs everything except the write to the flight controller.

  Ctrl-C stops everything.
EOF
wait
