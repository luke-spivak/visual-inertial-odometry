#!/usr/bin/env bash
# run_euroc_eval.sh — run OpenVINS against a EuRoC sequence and report ATE/RPE.
#
# Usage:  ./run_euroc_eval.sh [SEQUENCE] [BAG_DIR]
#   e.g.  ./run_euroc_eval.sh V1_01_easy
#
# Expects (see docs/development-log.md "Validated baseline"):
#   - ROS 2 Jazzy at /opt/ros/jazzy
#   - OpenVINS workspace at ~/ws_ov, built, with .h -> .hpp includes patched
#   - A ROS 2 bag of the sequence at ~/datasets/<SEQUENCE>_ros2
#   - evo on PATH (pipx install evo)
#
# Trajectory is taken from /ov_msckf/odomimu, NOT save_total_state: the state
# file stores q before p in JPL convention (q_GtoI), which a TUM reader
# silently misinterprets into ~135 deg of orientation error.
set -eo pipefail   # not -u: ROS's setup.bash references unbound variables

export PATH="$PATH:$HOME/.local/bin"   # pipx-installed evo

SEQ="${1:-V1_01_easy}"
BAG="${2:-$HOME/datasets/${SEQ}_ros2}"
WS="$HOME/ws_ov"
GT="$WS/src/open_vins/ov_data/euroc_mav/${SEQ}.txt"
RUN="/tmp/ovrun_${SEQ}"
REC="$RUN/odom"

command -v evo_ape >/dev/null || { echo "evo not on PATH (pipx install evo)"; exit 1; }
[ -d "$BAG" ] || { echo "bag not found: $BAG"; exit 1; }
[ -f "$GT" ]  || { echo "ground truth not found: $GT"; exit 1; }

rm -rf "$RUN"; mkdir -p "$RUN"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export MPLBACKEND=Agg

echo "==> estimator (headless)"
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav rviz_enable:=false \
  > "$RUN/estimator.log" 2>&1 &
sleep 12

# sqlite3, not mcap: an mcap killed before finalizing cannot be reopened.
echo "==> recording /ov_msckf/odomimu"
ros2 bag record -s sqlite3 -o "$REC" /ov_msckf/odomimu > "$RUN/record.log" 2>&1 &
RECPID=$!
sleep 3

echo "==> playing $SEQ"
ros2 bag play "$BAG" > "$RUN/play.log" 2>&1
sleep 8

# The `ros2` python wrapper does not reliably forward SIGINT, so bound the
# wait and escalate. Safe because sqlite3 survives a hard kill (mcap does not),
# and a missing metadata.yaml is rebuilt by `ros2 bag reindex` below.
kill -INT "$RECPID" 2>/dev/null || true
for _ in $(seq 1 15); do kill -0 "$RECPID" 2>/dev/null || break; sleep 1; done
kill -9 "$RECPID" 2>/dev/null || true
wait "$RECPID" 2>/dev/null || true
# -x cannot match: /proc comm truncates to 15 chars, name is 19.
# -f is safe from inside a script file (pattern is not in this process's cmdline).
pkill -f run_subscribe_msckf 2>/dev/null || true
sleep 3
pkill -9 -f run_subscribe_msckf 2>/dev/null || true

[ -f "$REC/metadata.yaml" ] || ros2 bag reindex "$REC" -s sqlite3

echo "==> converting to TUM"
( cd "$RUN" && evo_traj bag2 "$REC" /ov_msckf/odomimu --save_as_tum >/dev/null )
EST="$RUN/ov_msckf_odomimu.tum"

echo
echo "===================== ATE ====================="
evo_ape tum "$GT" "$EST" -va --save_plot "$RUN/ape.pdf" --plot_mode xyz 2>&1 \
  | sed -n '/APE w.r.t./,/^[[:space:]]*std/p'

echo "===================== RPE (10 m) ====================="
evo_rpe tum "$GT" "$EST" -va --delta 10 --delta_unit m --pose_relation trans_part 2>&1 \
  | sed -n '/RPE w.r.t./,/^[[:space:]]*std/p'

echo
echo "artifacts: $RUN  (ape.pdf, *.tum, estimator.log)"
