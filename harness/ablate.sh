#!/usr/bin/env bash
# ablate.sh -- run several estimator configurations against ONE recorded flight.
#
#   ./ablate.sh ~/vio_runs/simvio_alt10tall
#
# The input is byte-identical across variants, so a difference in the output is
# the change and not the 1.7x run-to-run timing variance. That is the only way
# any of the estimator conclusions in PROJECT.md were separable at all.
#
# Each variant is a copy of the canonical config with specific keys overwritten,
# written to /tmp so the canonical file is never edited by a script.
set -eo pipefail

RUN="${1:?usage: ablate.sh RUN_DIR}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BASE="$HOME/vio_openvins/gz_sim/estimator_config.yaml"
RATE="${RATE:-1.0}"
VARIANTS="${VARIANTS:-nocalib gravity nocalib_gravity}"

# set_key FILE KEY VALUE -- replace a top-level scalar, keeping the file's
# OpenCV-YAML shape. Appending instead of replacing would give a duplicate key,
# and OpenCV's parser takes the FIRST, so the edit would silently do nothing.
set_key() {
  local f="$1" k="$2" v="$3"
  grep -qE "^${k}:" "$f" || { echo "no key '$k' in $f" >&2; exit 1; }
  sed -i -E "s|^${k}:[^#]*|${k}: ${v} |" "$f"
}

# Variant configs live BESIDE the canonical one, not in /tmp. OpenVINS resolves
# relative_config_imu / relative_config_imucam against the config file's own
# directory and always treats them as relative -- an absolute path there comes
# out as "/tmp//home/luke/..." and the estimator dies with "unable to open the
# configuration file". The kalibr chains have to be reachable by the same
# relative name, so the copy has to sit in the same directory.
mk() {
  local name="$1"; shift
  local f="$HOME/vio_openvins/gz_sim/ablate_${name}.yaml"
  cp "$BASE" "$f"
  while [ $# -gt 0 ]; do set_key "$f" "$1" "$2"; shift 2; done
  echo "$f"
}

declare -A CFG
# Online calibration off. In simulation the intrinsics are exact by
# construction (perfect pinhole, zero distortion), the extrinsic is exact, and
# the time offset is zero -- there is nothing to calibrate, and calibrating it
# anyway adds 11 weakly-observable states that can absorb inconsistency during
# a degenerate segment. Measured on the 10 m structured flight: fx drifted from
# an exactly-correct 381.347 to 385.82 and cx from 320 to 313.7.
CFG[nocalib]="$(mk nocalib calib_cam_extrinsics false calib_cam_intrinsics false calib_cam_timeoffset false)"
# gz's world gravity is 9.8, but the stationary accelerometer on THIS vehicle
# measures 9.7914. The filter should be told what its sensor reports.
CFG[gravity]="$(mk gravity gravity_mag 9.7914)"
CFG[nocalib_gravity]="$(mk nocalib_gravity calib_cam_extrinsics false calib_cam_intrinsics false calib_cam_timeoffset false gravity_mag 9.7914)"

# The sliding window against the mission's stop-and-turn.
#
# The camera runs at 30.3 Hz and track_frequency 21 makes OpenVINS keep every
# second frame, so the filter updates at 15.2 Hz -- measured, 982 updates over
# 64.6 s. With max_clones 11 the window therefore spans 11/15.2 = 0.72 s.
# The mission settles at every waypoint, and the measured dwell is 0.5-0.8 s.
#
# So at each corner the ENTIRE clone window collapses onto stationary poses:
# every MSCKF feature marginalised there is triangulated from a zero-baseline
# geometry, which is the monocular degeneracy in its purest form. That is
# exactly where the divergence starts -- the climb and the whole first leg are
# clean, and the accelerometer bias slams from 0.01 to -0.47 m/s^2 in the 1.6 s
# after the first stop.
#
# Widening the window makes it straddle the stop, so there is always motion at
# one end of it. Same data, same update rate, one number changed.
CFG[clones20]="$(mk clones20 max_clones 20)"
CFG[clones30]="$(mk clones30 max_clones 30)"
# The opposite direction, as a control: a faster tracker shortens the window in
# TIME (11/30 = 0.37 s), so it should sit even more completely inside the stop
# and be worse. If it is not, the window/dwell story is wrong.
CFG[track30]="$(mk track30 track_frequency 30.0)"

# The chi-squared gate, which is the prime suspect for the PERMANENCE of the
# failure rather than its onset. Measured: MSCKF features used per update go to
# ZERO at t~17 s and stay there for the remaining 47 s (695 of 709 updates use
# no features at all), while an independent KLT tracker on the same images
# finds 387 trackable features throughout. So the images are fine and the
# filter is refusing them.
#
# That is what a too-tight gate does once a state becomes even slightly
# inconsistent: every residual looks like an outlier, every feature is
# rejected, no update can correct the state, and the inconsistency is therefore
# permanent. A looser gate lets the filter recover from a transient instead of
# locking itself out. OpenVINS ships 1 for EuRoC, where nothing ever goes
# wrong; 5 is common in configs meant for less forgiving data.
CFG[chi2_5]="$(mk chi2_5 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5)"
CFG[chi2_20]="$(mk chi2_20 up_msckf_chi2_multipler 20 up_slam_chi2_multipler 20)"
# More features to lose, so a corner that costs the SLAM set is not fatal.
CFG[morepts]="$(mk morepts num_pts 400 max_slam 75 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5)"
# With the gate opened, the update RATE becomes worth buying back. The camera
# runs at 30.3 Hz and track_frequency 21 makes OpenVINS keep only every second
# frame (measured: 15.2 Hz), so half the imagery is being discarded. 31 keeps
# every frame.
CFG[chi2_fast]="$(mk chi2_fast up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5 track_frequency 31.0)"
CFG[chi2_full]="$(mk chi2_full up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5 track_frequency 31.0 num_pts 400 max_slam 75 max_msckf_in_update 60)"

# The measurement noise itself, which is the more honest version of the same
# fix. up_*_sigma_px says how well a feature's reprojection is expected to
# match; 1 px is what OpenVINS ships for EuRoC, where the vehicle moves slowly
# and the imagery is real. Here the vehicle crosses 15 px of image between
# tracked frames at 10 m and 5.8 m/s, over textures sized near the 2.6 cm
# ground sample distance, so the true reprojection scatter is larger than a
# pixel. Declaring 1 px makes the filter over-confident, which shows up FIRST
# as a chi-squared gate that rejects everything.
#
# Raising sigma_px is better than inflating the gate alone: the gate multiplier
# only changes what is admitted, while sigma_px also correctly reduces how hard
# each admitted measurement pulls the state.
CFG[sigma2]="$(mk sigma2 up_msckf_sigma_px 2 up_slam_sigma_px 2 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5)"
CFG[sigma3]="$(mk sigma3 up_msckf_sigma_px 3 up_slam_sigma_px 3 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5)"
CFG[sigma2_plain]="$(mk sigma2_plain up_msckf_sigma_px 2 up_slam_sigma_px 2)"
CFG[sigma4]="$(mk sigma4 up_msckf_sigma_px 4 up_slam_sigma_px 4 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5)"
CFG[sigma5]="$(mk sigma5 up_msckf_sigma_px 5 up_slam_sigma_px 5 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5)"
CFG[sigma8]="$(mk sigma8 up_msckf_sigma_px 8 up_slam_sigma_px 8 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5)"
CFG[sigma4_pts]="$(mk sigma4_pts up_msckf_sigma_px 4 up_slam_sigma_px 4 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5 num_pts 400 max_slam 75)"
# Halve the inter-frame motion without touching the vehicle. track_frequency 21
# against a 30.3 Hz camera keeps every SECOND frame, so the tracker sees 15 px
# of displacement between views at 10 m and 5.8 m/s. Keeping every frame halves
# that to 7.4 px, which is the same benefit as flying half as fast and costs
# nothing but CPU.
CFG[sigma4_t31]="$(mk sigma4_t31 up_msckf_sigma_px 4 up_slam_sigma_px 4 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5 track_frequency 31.0)"
CFG[sigma2_t31]="$(mk sigma2_t31 up_msckf_sigma_px 2 up_slam_sigma_px 2 up_msckf_chi2_multipler 5 up_slam_chi2_multipler 5 track_frequency 31.0)"

OFF="$(cat "$RUN/start_offset.txt" 2>/dev/null || echo 0)"
SUM="$RUN/ablation.txt"
: > "$SUM"

for v in $VARIANTS; do
  f="${CFG[$v]}"
  [ -n "$f" ] || { echo "unknown variant $v" >&2; exit 1; }
  echo "############ $v ############"
  grep -E "^(calib_cam_|gravity_mag|try_zupt|max_slam|feat_rep_msckf)" "$f" | sed 's/^/    /'
  START_OFFSET="$OFF" CONFIG="$f" OUT_TAG="$v" \
    bash "$HERE/replay_openvins.sh" "$RUN" "$RATE" 2>&1 | tail -6
  bash "$HERE/eval_sim_run.sh" "$RUN" "$RUN/replay_$v/est" 2>&1 \
    | tee "$RUN/replay_$v/eval.txt" \
    | grep -E "overlap|PARTIAL|DRIFT" || true
  cp "$RUN/est.tum" "$RUN/replay_$v/est.tum" 2>/dev/null || true
  cp "$RUN/gt.tum" "$RUN/replay_$v/gt.tum" 2>/dev/null || true
  D=$(grep -oE "DRIFT: [0-9.]+" "$RUN/replay_$v/eval.txt" | awk '{print $2}' | tail -1)
  C=$(grep -oE "= [0-9.]+ % of the flight" "$RUN/replay_$v/eval.txt" | awk '{print $2}' | tail -1)
  printf '%-18s drift %-8s coverage %-6s\n' "$v" "${D:-nan}" "${C:-nan}" >> "$SUM"
  echo
done

echo "############ summary ############"
cat "$SUM"
echo
echo "Compare against the baseline distribution in $RUN/repeats_${RATE}.txt."
echo "A single ablation number is NOT a result on its own -- rerun the winner"
echo "through replay_repeats.sh before believing it."
