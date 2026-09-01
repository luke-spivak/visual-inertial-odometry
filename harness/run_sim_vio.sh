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

PROCS=(parameter_bridge run_subscribe_msckf "bag record" arducopter
       sim_vehicle "gz sim" gz-sim-server "Xvfb :77")

# pkill only DELIVERS a signal; it does not wait. That distinction cost two
# runs. A dying Xvfb removes /tmp/.X11-unix/X77 on its way out, so starting a
# replacement one second later leaves the NEW server holding a socket the OLD
# one then deletes -- Gazebo's X connection dies mid-run with "XIO: fatal IO
# error on X server", the camera produces nothing, and the log blames X.
# So kill, then wait for the processes to actually be gone.
cleanup() {
  echo "==> stopping"
  for p in "${PROCS[@]}"; do pkill -f "$p" 2>/dev/null || true; done
  for _ in $(seq 1 20); do
    local alive=0
    for p in "${PROCS[@]}"; do pgrep -f "$p" >/dev/null && alive=1; done
    [ "$alive" = "0" ] && break
    sleep 1
  done
  for p in "${PROCS[@]}"; do pkill -9 -f "$p" 2>/dev/null || true; done
  # Whatever the old server left behind; a stale lock makes a new Xvfb refuse
  # the display outright.
  rm -f /tmp/.X77-lock /tmp/.X11-unix/X77 2>/dev/null || true
}
trap cleanup EXIT

# Stale processes from a previous run (or from manual poking) will fight the
# new ones: a leftover gz sim holding :77 makes the fresh Xvfb collide, and
# Gazebo dies mid-run with "XIO: fatal IO error on X server".
echo "==> clearing stale processes"
cleanup

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

# --- 2. ROS bridge + render gate -----------------------------------------
# ORDER MATTERS, and not for the obvious reason. The ArduPilot plugin runs the
# sim in lockstep with SITL: from the moment SITL connects until it is actually
# driving the motors, Gazebo's update loop is blocked -- measured, /world/.../
# stats publishes NOTHING for 10 s straight, and no camera frames come out
# either. Checking the camera in that window reports a dead camera on a sim
# that is merely paused. Gate the render BEFORE SITL, while Gazebo is still
# free-running at RTF ~0.4.
source /opt/ros/jazzy/setup.bash
source "$HOME/ws_ov/install/setup.bash"

echo "==> ros_gz_bridge"
nohup ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="$HOME/vio_gazebo/ros_gz_bridge_vio.yaml" \
  > "$RUN/bridge.log" 2>&1 &
sleep 8

# Gate on the render before spending a mission on it: a featureless image
# gives OpenVINS nothing to track, and the run is wasted wall time. This is a
# HARD stop rather than a warning -- a previous run flew a full mission and
# recorded exactly zero odometry messages because the ground was a flat grey.
# ALLOW_BLACK=1 overrides it, for deliberately diagnosing the renderer.
echo "==> camera check"
python3 "$HERE/check_camera.py" --n 2 --wait 120 2>&1 | tee "$RUN/camera_check.log" || true
if grep -qE "UNTRACKABLE|LOOKS BLACK|NO IMAGES" "$RUN/camera_check.log" 2>/dev/null; then
  echo "    camera is not producing a usable image." >&2
  echo "    Diagnose with the standalone probe world, which renders known" >&2
  echo "    albedo patches and needs no SITL:" >&2
  echo "      gz sim -s -r -v4 render_probe.sdf   (see gazebo/worlds/)" >&2
  echo "    Set ALLOW_BLACK=1 to fly anyway." >&2
  [ "${ALLOW_BLACK:-0}" = "1" ] || exit 1
fi

# --- 3. ArduPilot SITL ---------------------------------------------------
# env -i so ROS's Python environment cannot leak into ArduPilot's build and
# run, the same reason Gazebo gets one. SITL recompiles on first use with a new
# frame (3-4 min), so poll for the port rather than guessing a sleep.
#
# Leave this at 1. Real lockstep (see <no_time_sync>0</no_time_sync> in the
# model SDF) means ArduPilot WAITS for each Gazebo frame, so a slow sim costs
# wall-clock time and nothing else -- the mission is paced in sim time and does
# not care. Slowing ArduPilot instead was an earlier, wrong fix: SIM_SPEEDUP
# != 1 makes ArduPilot disable time sync, which silently switches it to
# AHRS_EKF_TYPE=10 (the fake SITL EKF) and starves it of FDM entirely
# ("No JSON sensor message received, resending servos").
# Kept as a knob only because sim_vehicle.py --speedup takes integers.
SPEEDUP="${SIM_SPEEDUP:-1}"
printf 'SIM_SPEEDUP %s\n' "$SPEEDUP" > "$RUN/sim_speed.parm"
echo "==> SITL (SIM_SPEEDUP $SPEEDUP)"
nohup env -i \
  HOME="$HOME" USER="${USER:-$(id -un)}" \
  PATH="/usr/local/bin:/usr/bin:/bin:$HOME/.local/bin:$HOME/ardupilot/Tools/autotest" \
  bash -c "cd \$HOME/ardupilot && \
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy \
    --add-param-file=$RUN/sim_speed.parm \
    --out=udp:127.0.0.1:14551" > "$RUN/sitl.log" 2>&1 &
echo "    (first run recompiles SITL, may take several minutes)"
if ! wait_for_port 5760 420; then
  echo "SITL never opened 5760; see $RUN/sitl.log" >&2
  tail -20 "$RUN/sitl.log" >&2
  exit 1
fi
echo "    SITL up"
sleep 8

echo "==> OpenVINS"
nohup ros2 launch ov_msckf subscribe.launch.py \
  config_path:="$HOME/vio_openvins/gz_sim/estimator_config.yaml" \
  max_cameras:=1 use_stereo:=false rviz_enable:=false \
  > "$RUN/openvins.log" 2>&1 &
sleep 12

# RECORD_SENSORS=1 also bags the raw camera and IMU. That is ~5 GB for a
# 10-minute flight, and worth it: with the sensors on disk the estimator can be
# re-run offline in seconds instead of re-flying for 25 minutes, and replaying
# identical input removes the run-to-run timing variance that PROJECT.md
# measured at 1.7x on EuRoC. Set to 0 for routine runs once tuning is settled.
TOPICS="/ov_msckf/odomimu /gz/ground_truth"
if [ "${RECORD_SENSORS:-1}" = "1" ]; then
  TOPICS="$TOPICS /cam0/image_raw /cam0/camera_info /imu0"
fi
echo "==> recording ($TOPICS)"
# shellcheck disable=SC2086
nohup ros2 bag record -s sqlite3 -o "$RUN/bag" $TOPICS > "$RUN/record.log" 2>&1 &
RECPID=$!
sleep 4

# --- 6. Fly --------------------------------------------------------------
# Sample trackability throughout the flight, not just on the ground. The gate
# above proves the scene renders; this says whether the imagery the estimator
# actually consumed was any good, which is what explains a bad ATE afterwards.
nohup python3 "$HERE/check_camera.py" --n 999 --every 10 --wait 900 \
  --save-dir "$RUN/frames" > "$RUN/camera_inflight.log" 2>&1 &
CAMPID=$!

echo "==> mission"
python3 "$HERE/fly_sim_mission.py" --conn tcp:127.0.0.1:5760 2>&1 | tee "$RUN/mission.log"
kill "$CAMPID" 2>/dev/null || true

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
