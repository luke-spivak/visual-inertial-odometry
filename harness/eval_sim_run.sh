#!/usr/bin/env bash
# eval_sim_run.sh -- turn one run_sim_vio.sh bag into the number PROJECT.md asks
# for: drift as a percentage of distance travelled.
#
#   ./eval_sim_run.sh /tmp/simvio_flight2
#
# Two numbers come out, and they answer different questions:
#
#   ATE (absolute)  -- how far the estimate is from truth after one global
#                      alignment. Dominated by whatever happened worst, and it
#                      grows with how long you flew. Good for "did it diverge".
#   RPE over 10 m   -- error accumulated per 10 m of travel, as a percent. This
#                      is the drift rate, and it is what transfers to a flight
#                      of a different length. This is the headline number.
#
# Alignment is -a (rotation + translation), NOT -as: scale is observable from
# the IMU, so letting evo fit a scale factor would hide exactly the error we
# are trying to measure.
set -eo pipefail

RUN="${1:?usage: eval_sim_run.sh RUN_DIR}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BAG="$RUN/bag"
[ -d "$BAG" ] || { echo "no bag at $BAG" >&2; exit 1; }

source /opt/ros/jazzy/setup.bash

[ -f "$BAG/metadata.yaml" ] || ros2 bag reindex "$BAG" -s sqlite3

echo "==> ground truth"
python3 "$HERE/gt_to_tum.py" --bag "$BAG" --child vio_link --out "$RUN/gt.tum"

echo "==> estimate"
evo_traj bag2 "$BAG" /ov_msckf/odomimu --save_as_tum >/dev/null
mv ov_msckf_odomimu.tum "$RUN/est.tum"

echo "==> path length"
python3 - "$RUN/gt.tum" <<'PY'
import sys, math
p = [list(map(float, l.split()))[1:4] for l in open(sys.argv[1]) if l.strip()]
d = sum(math.dist(a, b) for a, b in zip(p, p[1:]))
print(f"    ground-truth path: {d:.1f} m over {len(p)} poses")
open(sys.argv[1] + ".len", "w").write(f"{d}\n")
PY

echo
echo "==> ATE (aligned, no scale)"
evo_ape tum "$RUN/gt.tum" "$RUN/est.tum" -va --t_max_diff 0.05 2>&1 | tail -15

echo
echo "==> RPE over 10 m segments  -> drift rate"
evo_rpe tum "$RUN/gt.tum" "$RUN/est.tum" -va --t_max_diff 0.05 \
  --pose_relation trans_part --delta 10 --delta_unit m 2>&1 | tee "$RUN/rpe.txt" | tail -15

echo
python3 - "$RUN/rpe.txt" <<'PY'
import re, sys
m = re.search(r"^\s*mean\s+([0-9.eE+-]+)", open(sys.argv[1]).read(), re.M)
if m:
    print(f"    DRIFT: {float(m.group(1)) / 10.0 * 100:.2f} % of distance travelled")
else:
    print("    (could not parse RPE mean; see rpe.txt)")
PY
