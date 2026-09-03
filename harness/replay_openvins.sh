#!/usr/bin/env bash
# replay_openvins.sh -- re-run the estimator on a recorded flight, offline.
#
#   ./replay_openvins.sh /tmp/simvio_flight9 [RATE]
#
# Why this exists. Flying takes ~25 minutes of wall clock for ~2 minutes of sim,
# which is a terrible loop to tune an estimator in. It is also not reproducible:
# PROJECT.md measured 1.7x run-to-run ATE variance on identical EuRoC input,
# caused by timing-dependent frame drops. Replaying a bag fixes both -- the
# input is byte-identical every time, so a change in the output is a change you
# made rather than jitter.
#
# RATE is the point of the exercise. Live, OpenVINS competes with Gazebo's
# software renderer for CPU and drops frames; KLT then sees large inter-frame
# jumps and tracks die, which looks like a featureless scene but is not. Replay
# below 1.0 gives the estimator the time it never had.
set -eo pipefail   # not -u: ROS's setup.bash references unbound variables

RUN="${1:?usage: replay_openvins.sh RUN_DIR [RATE]}"
RATE="${2:-0.5}"
# CONFIG lets an ablation point the estimator at a variant config without
# editing the canonical one. OUT_TAG keeps the variants' outputs apart.
CONFIG="${CONFIG:-$HOME/vio_openvins/gz_sim/estimator_config.yaml}"
# START_OFFSET skips the head of the bag. The estimator should not be running
# through a long stationary period: with no motion there is no parallax, MSCKF
# features cannot be triangulated, and the filter propagates on IMU alone and
# drifts before the flight even begins. This vehicle sits for 45 s waiting on
# ArduPilot's EKF; a lead-in of a few seconds is all the initialiser needs.
START_OFFSET="${START_OFFSET:-0}"
BAG="$RUN/bag"
OUT="$RUN/replay_${OUT_TAG:-$RATE}"

source /opt/ros/jazzy/setup.bash
source "$HOME/ws_ov/install/setup.bash"

ros2 bag info "$BAG" 2>/dev/null | grep -q "/cam0/image_raw" || {
  echo "no /cam0/image_raw in $BAG -- re-record with RECORD_SENSORS=1" >&2; exit 1; }

rm -rf "$OUT"; mkdir -p "$OUT"
echo "==> replaying $BAG at ${RATE}x from +${START_OFFSET}s -> $OUT"

cleanup() {
  pkill -f run_subscribe_msckf 2>/dev/null || true
  pkill -f "bag record" 2>/dev/null || true
  pkill -f "bag play" 2>/dev/null || true
  for _ in $(seq 1 15); do
    pgrep -f run_subscribe_msckf >/dev/null || break
    sleep 1
  done
}
trap cleanup EXIT
cleanup

# save_total_state MUST be passed here, not left to the YAML. subscribe.launch.py
# declares it as a launch argument defaulting to "false" and passes it as a ROS
# parameter, and ROS parameters beat config-file values -- so `save_total_state:
# true` in estimator_config.yaml is inert and the state file is silently never
# written. The launch file also HARDCODES the output paths, ignoring
# filepath_est/filepath_std/filepath_gt from the YAML entirely:
#
#     {"filepath_est": "/tmp/ov_estimate.txt"},
#
# so the file is at /tmp/ov_estimate.txt, not the /tmp/ov_sim_estimate.txt the
# config asks for. `verbosity` is overridden the same way, which is why VERBOSITY
# is passed here rather than set in the YAML. That makes four settings this
# launch file quietly takes ownership of: max_cameras, use_stereo,
# save_total_state and verbosity.
nohup ros2 launch ov_msckf subscribe.launch.py \
  config_path:="$CONFIG" \
  max_cameras:=1 use_stereo:=false rviz_enable:=false \
  save_total_state:=true verbosity:="${VERBOSITY:-INFO}" \
  > "$OUT/openvins.log" 2>&1 &

# Verify the estimator is actually up and stayed up. These scripts kill by
# process-name pattern, so a second invocation overlapping a first will shoot
# the new run's estimator: observed once as OpenVINS logging
# "signal_handler(SIGINT/SIGTERM)" thirteen seconds BEFORE playback began, then
# an empty result bag that looked like an estimator failure.
for _ in $(seq 1 40); do
  ros2 node list 2>/dev/null | grep -q ov_msckf && break
  sleep 1
done
ros2 node list 2>/dev/null | grep -q ov_msckf || {
  echo "OpenVINS never appeared in the node graph; see $OUT/openvins.log" >&2
  tail -5 "$OUT/openvins.log" >&2; exit 1; }
if grep -q "signal_handler" "$OUT/openvins.log" 2>/dev/null; then
  echo "OpenVINS was signalled during startup -- is another replay running?" >&2
  exit 1
fi
echo "    estimator up"

nohup ros2 bag record -s sqlite3 -o "$OUT/est" /ov_msckf/odomimu \
  > "$OUT/record.log" 2>&1 &
RECPID=$!
sleep 4

# Re-check immediately before spending the playback on it.
ros2 node list 2>/dev/null | grep -q ov_msckf || {
  echo "OpenVINS died before playback started; see $OUT/openvins.log" >&2; exit 1; }

# Foreground, so the script blocks for exactly as long as the data lasts.
ros2 bag play "$BAG" --rate "$RATE" --start-offset "$START_OFFSET" \
  --topics /cam0/image_raw /cam0/camera_info /imu0 2>&1 | tail -5

sleep 5
# Stop the estimator BEFORE copying its state file: it streams the total state
# as it runs, and a copy taken while it is still writing is short.
pkill -f run_subscribe_msckf 2>/dev/null || true
for _ in $(seq 1 15); do pgrep -f run_subscribe_msckf >/dev/null || break; sleep 1; done
kill -INT "$RECPID" 2>/dev/null || true
for _ in $(seq 1 15); do kill -0 "$RECPID" 2>/dev/null || break; sleep 1; done
kill -9 "$RECPID" 2>/dev/null || true
[ -f "$OUT/est/metadata.yaml" ] || ros2 bag reindex "$OUT/est" -s sqlite3 || true

# save_total_state writes to the FIXED path in estimator_config.yaml, so every
# replay overwrites the last one. Snapshot it here: it carries the gyro and
# accel bias estimates, which is where a constant-velocity runaway shows up as
# a cause rather than a symptom.
for f in /tmp/ov_estimate.txt /tmp/ov_estimate_std.txt; do
  [ -f "$f" ] && cp "$f" "$OUT/$(basename "$f")"
done
# turn_diagnostic.py looks for this name; keep one stable alias.
[ -f "$OUT/ov_estimate.txt" ] && cp "$OUT/ov_estimate.txt" "$OUT/ov_sim_estimate.txt"

echo
echo "==> estimator output:"
ros2 bag info "$OUT/est" 2>/dev/null | grep -E "Duration|Count" || echo "(none)"
echo
echo "==> initialisation:"
grep -cE "not enough feats" "$OUT/openvins.log" 2>/dev/null | sed 's/^/    init retries: /'
sed 's/\x1b\[[0-9;]*m//g' "$OUT/openvins.log" | grep -iE "successful|initialized|init]: " | tail -5
