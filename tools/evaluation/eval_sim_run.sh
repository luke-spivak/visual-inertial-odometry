#!/usr/bin/env bash
# eval_sim_run.sh -- turn one run_sim_vio.sh bag into the number docs/development-log.md asks
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
set -eo pipefail   # not -u: ROS's setup.bash references unbound variables

# EST_BAG lets the estimate come from an offline replay while ground truth
# still comes from the flight that produced it. Ground truth is a property of
# the flight; the estimate is a property of whatever estimator config you last
# tried, and those two now change independently.
RUN="${1:?usage: eval_sim_run.sh RUN_DIR [EST_BAG]}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BAG="$RUN/bag"
EST_BAG="${2:-$BAG}"
[ -d "$BAG" ] || { echo "no bag at $BAG" >&2; exit 1; }
[ -d "$EST_BAG" ] || { echo "no estimate bag at $EST_BAG" >&2; exit 1; }

source /opt/ros/jazzy/setup.bash
export PATH="$PATH:$HOME/.local/bin"   # pip --user installed evo
command -v evo_ape >/dev/null || { echo "evo not on PATH" >&2; exit 1; }

[ -f "$BAG/metadata.yaml" ] || ros2 bag reindex "$BAG" -s sqlite3
[ -f "$EST_BAG/metadata.yaml" ] || ros2 bag reindex "$EST_BAG" -s sqlite3

# Both trajectories go through the same reader. evo_traj bag2 could pull the
# estimate, but then a format bug would hit only one side of the comparison and
# look like estimator error.
# evo PROMPTS before overwriting an existing plot file, on stdin. From a
# non-interactive shell that either blocks forever (observed: evo_ape at 0.0%
# CPU for three hours) or dies with EOFError. Clear the outputs first.
rm -f "$RUN/ape.pdf" "$RUN/rpe.txt" "$RUN/gt.tum" "$RUN/est.tum"

echo "==> ground truth"
python3 "$HERE/gt_to_tum.py" --bag "$BAG" --topic /gz/ground_truth \
  --lever 0.10,0,0.02 --out "$RUN/gt.tum"

echo "==> estimate"
python3 "$HERE/gt_to_tum.py" --bag "$EST_BAG" --topic /ov_msckf/odomimu --out "$RUN/est.tum"

echo "==> path length"
python3 - "$RUN/gt.tum" <<'PY'
import sys, math
p = [list(map(float, l.split()))[1:4] for l in open(sys.argv[1]) if l.strip()]
d = sum(math.dist(a, b) for a, b in zip(p, p[1:]))
print(f"    ground-truth path: {d:.1f} m over {len(p)} poses")
open(sys.argv[1] + ".len", "w").write(f"{d}\n")
PY

# TRAP: ATE over different trajectory spans is not comparable, and this has
# invalidated conclusions here before. A run that diverges and stops early
# scores BETTER, because it ends before the worst of it. So the span check is
# automatic rather than something to remember: any comparison between two
# numbers below is only meaningful if both cover the same flight.
echo
echo "==> span"
python3 - "$RUN/gt.tum" "$RUN/est.tum" <<'PY'
import sys
def span(path):
    t = [float(l.split()[0]) for l in open(path) if l.strip()]
    return (t[0], t[-1], t[-1] - t[0], len(t))
g0, g1, gd, gn = span(sys.argv[1])
e0, e1, ed, en = span(sys.argv[2])
ov = max(0.0, min(g1, e1) - max(g0, e0))
print(f"    ground truth : {gd:7.2f} s  ({gn} poses)")
print(f"    estimate     : {ed:7.2f} s  ({en} poses)   starts +{e0 - g0:.2f} s")
print(f"    overlap      : {ov:7.2f} s  = {100 * ov / gd:.1f} % of the flight")
if ov < 0.9 * gd:
    print(f"    *** PARTIAL COVERAGE: the estimate spans {100 * ov / gd:.0f} % of the")
    print("        flight. ATE and drift below are NOT comparable to a run that")
    print("        covered more of it -- an early stop hides the worst error. ***")
PY

echo
# No --t_max_diff: evo's default 0.01 s is what run_euroc_eval.sh used for the
# baseline, so the two numbers stay comparable. Loosening it to 0.05 s would
# admit up to 50 ms of association error, which at 5 m/s is 0.25 m -- larger
# than the ATE being measured, and it would look like estimator error.
echo "==> ATE (aligned, no scale)"
evo_ape tum "$RUN/gt.tum" "$RUN/est.tum" -va \
  --save_plot "$RUN/ape.pdf" --plot_mode xyz < /dev/null 2>&1 | tail -15

echo
echo "==> RPE over 10 m segments  -> drift rate"
evo_rpe tum "$RUN/gt.tum" "$RUN/est.tum" -va \
  --pose_relation trans_part --delta 10 --delta_unit m < /dev/null 2>&1 \
  | tee "$RUN/rpe.txt" | tail -15

echo
python3 - "$RUN/rpe.txt" <<'PY'
import re, sys
m = re.search(r"^\s*mean\s+([0-9.eE+-]+)", open(sys.argv[1]).read(), re.M)
if m:
    print(f"    DRIFT: {float(m.group(1)) / 10.0 * 100:.2f} % of distance travelled")
else:
    print("    (could not parse RPE mean; see rpe.txt)")
PY
