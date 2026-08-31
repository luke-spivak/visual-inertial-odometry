#!/usr/bin/env bash
# run_sim_vio.sh -- OpenVINS against Gazebo sim sensors, compared to Gazebo truth.
#
# Orchestrates five processes:
#   Gazebo (NO ros sourced -- ROS's vendored gz libs hide the `sim` subcommand)
#   ArduPilot SITL          (flies the model via ardupilot_gazebo)
#   ros_gz_bridge           (gz-transport -> ROS 2)
#   OpenVINS                (mono, sim config)
#   ros2 bag record         (estimate + ground truth)
#
# Then flies a scripted mission and records both trajectories.
#
# Usage: ./run_sim_vio.sh [RUN_TAG]
set -eo pipefail

# Self-contained PATH: this runs under `bash -c`, which reads neither
# .profile (login) nor .bashrc (interactive), so ArduPilot's and pipx's
# PATH additions do not apply.
export PATH="$PATH:$HOME/.local/bin:$HOME/ardupilot/Tools/autotest"
TAG="${1:-$(date +%H%M%S)}"
# WORLD is the flat-vs-structured A/B. iris_runway_vio.sdf puts every feature on
# the runway plane (a monocular degeneracy); iris_field_vio.sdf adds the
# generated obstacle field. Both declare <world name="iris_runway">, so the
# bridge's /world/iris_runway/... topic names hold for either.
WORLD="${WORLD:-iris_runway_vio.sdf}"
RUN="/tmp/simvio_$TAG"
HERE="$(cd "$(dirname "$0")" && pwd)"

rm -rf "$RUN"; mkdir -p "$RUN"
echo "==> artifacts: $RUN"

# SITL recompiles on first run with a new frame (3-4 min), so poll for
# readiness rather than guessing a sleep.
wait_for_port() {
  local port="$1" limit="${2:-420}"
  for _ in $(seq 1 "$limit"); do
    (exec 3<>/dev/tcp/127.0.0.1/"$port") 2>/dev/null && { exec 3<&- 3>&-; return 0; }
    sleep 1
  done
  return 1
}

cleanup() {
  echo "==> stopping"
  pkill -f parameter_bridge 2>/dev/null || true
  pkill -f run_subscribe_msckf 2>/dev/null || true
  pkill -f "bag record" 2>/dev/null || true
  pkill -f arducopter 2>/dev/null || true
  pkill -f sim_vehicle 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "Xvfb :77" 2>/dev/null || true
  pkill -f gz-sim-server 2>/dev/null || true
}
trap cleanup EXIT

# Stale processes from a previous run (or from manual poking) will fight the
# new ones: a leftover gz sim holding :77 makes the fresh Xvfb collide, and
# Gazebo dies mid-run with "XIO: fatal IO error on X server".
echo "==> clearing stale processes"
cleanup
sleep 3

# --- 1. Xvfb + Gazebo ---------------------------------------------------
# Ogre2 needs an OpenGL 3.3+ core profile. On Apple Silicon under UTM, virgl
# exposes only 2.1 with no core profile, and Ogre2's EGL path explicitly
# selects a hardware device -- which makes Mesa refuse software rendering
# ("Not allowed to force software rendering...") and then segfault.
#
# Xvfb provides a display with no hardware GL, so GLX resolves to llvmpipe,
# which gives OpenGL 4.5 core. Combined with <headless_rendering>false</>
# in the world (forcing GLX over EGL), Ogre2 initialises and renders lit
# images. Slow -- everything is on CPU -- but correct.
#
# `env -i` because ROS 2 sets GZ_CONFIG_PATH to its own vendored gz packages,
# which contain no gz-sim, making the `sim` subcommand vanish.
echo "==> Xvfb"
pkill -f "Xvfb :77" 2>/dev/null || true
sleep 1
Xvfb :77 -screen 0 1280x1024x24 > "$RUN/xvfb.log" 2>&1 &
for _ in $(seq 1 20); do
  DISPLAY=:77 xdpyinfo >/dev/null 2>&1 && break
  sleep 1
done
if ! DISPLAY=:77 xdpyinfo >/dev/null 2>&1; then
  echo "Xvfb :77 never came up; see $RUN/xvfb.log" >&2; exit 1
fi
echo "    Xvfb serving :77"

echo "==> gazebo ($WORLD)"
GZRES="$HOME/vio_gazebo/models:$HOME/vio_gazebo/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds"
nohup env -i \
  HOME="$HOME" USER="${USER:-$(id -un)}" \
  PATH=/usr/local/bin:/usr/bin:/bin \
  DISPLAY=:77 \
  GZ_SIM_RESOURCE_PATH="$GZRES" \
  GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build" \
  gz sim -s -r -v2 "$WORLD" > "$RUN/gazebo.log" 2>&1 &
sleep 30
if grep -qE "cannot find any available|Segmentation" "$RUN/gazebo.log" 2>/dev/null; then
  echo "Gazebo failed to start; see $RUN/gazebo.log" >&2
  exit 1
fi

# --- 2. ArduPilot SITL ---------------------------------------------------
echo "==> SITL"
nohup bash -c "export PATH=\$PATH:\$HOME/ardupilot/Tools/autotest; cd \$HOME/ardupilot && \
  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy \
  --out=udp:127.0.0.1:14551" > "$RUN/sitl.log" 2>&1 &
echo "    (first run recompiles SITL, may take several minutes)"
if ! wait_for_port 5760 420; then
  echo "SITL never opened 5760; see $RUN/sitl.log" >&2
  tail -20 "$RUN/sitl.log" >&2
  exit 1
fi
echo "    SITL up"
sleep 8

# --- 3/4/5. ROS side -----------------------------------------------------
source /opt/ros/jazzy/setup.bash
source "$HOME/ws_ov/install/setup.bash"

echo "==> ros_gz_bridge"
nohup ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="$HOME/vio_gazebo/ros_gz_bridge_vio.yaml" \
  > "$RUN/bridge.log" 2>&1 &
sleep 8

# Sanity-check the render before spending a mission on it. Near-black images
# (max ~14 of 255) mean no GL context, and OpenVINS will track zero features
# whichever way the camera points.
echo "==> camera check"
python3 "$HERE/check_camera.py" --n 2 2>&1 | tee "$RUN/camera_check.log" || true
if grep -q "LOOKS BLACK" "$RUN/camera_check.log" 2>/dev/null; then
  echo "    WARNING: camera is rendering black -- no GL context." >&2
  echo "    Check $RUN/gazebo.log for XIO errors or a dead Xvfb." >&2
fi

echo "==> OpenVINS"
nohup ros2 launch ov_msckf subscribe.launch.py \
  config_path:="$HOME/vio_openvins/gz_sim/estimator_config.yaml" \
  max_cameras:=1 use_stereo:=false rviz_enable:=false \
  > "$RUN/openvins.log" 2>&1 &
sleep 12

echo "==> recording"
nohup ros2 bag record -s sqlite3 -o "$RUN/bag" \
  /ov_msckf/odomimu /gz/pose_info > "$RUN/record.log" 2>&1 &
RECPID=$!
sleep 4

# --- 6. Fly --------------------------------------------------------------
echo "==> mission"
python3 "$HERE/fly_sim_mission.py" --conn tcp:127.0.0.1:5760 2>&1 | tee "$RUN/mission.log"

sleep 6
kill -INT "$RECPID" 2>/dev/null || true
for _ in $(seq 1 15); do kill -0 "$RECPID" 2>/dev/null || break; sleep 1; done
kill -9 "$RECPID" 2>/dev/null || true

[ -f "$RUN/bag/metadata.yaml" ] || ros2 bag reindex "$RUN/bag" -s sqlite3 || true

echo
echo "==> recorded:"
ros2 bag info "$RUN/bag" 2>/dev/null | grep -E "Duration|Topic:" || echo "(no bag)"
echo
echo "artifacts in $RUN"
