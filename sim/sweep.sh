#!/usr/bin/env bash
# sweep.sh -- milestone 6. Fly a set of configurations headlessly and put ATE
# and RPE in one table.
#
#   ./sweep.sh                 # the default sweep
#   ./sweep.sh my_configs.txt  # one "name key=value ..." per line
#
# Each line is: NAME then any of WORLD / MISSION_ALT / MISSION_SPEED / FIXED_YAW.
#
# Three things this does that a for-loop would not:
#
# ONE SCENE FOR ALL ALTITUDES. The obstacle field carries collision geometry, so
# a scene generated for 10 m flight is not safe at 6 m -- and regenerating per
# altitude would confound altitude with scene, which is exactly the comparison
# the sweep exists to make. So the field is generated ONCE, capped for the
# LOWEST altitude in the sweep, and every run flies over the identical scene.
# Altitude then varies alone.
#
# DISTRIBUTIONS, NOT SINGLE NUMBERS. Every configuration is flown once with the
# sensors recorded, then the estimator is replayed over that recording three
# times. Replay-to-replay spread is ~1.02x and flight-to-flight is ~1.3x, so a
# single live number could not distinguish a real effect from either.
#
# BOUNDED DISK. A recorded flight is ~1.5 GB and the VM has ~16 GB, so the bag
# is deleted once its replays are done; the trajectories and eval output stay.
# Guest disk and the host's VM image are the same resource -- a sweep that fills
# one kills the machine running it.
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
CFG="${1:-}"
OUT="${SWEEP_OUT:-$HOME/vio_runs/sweep}"
REPEATS="${REPEATS:-3}"
mkdir -p "$OUT"

# name  key=value ...
read -r -d '' DEFAULT_CONFIGS <<'EOF' || true
base_a10_v8    MISSION_ALT=10
alt6           MISSION_ALT=6
speed25        MISSION_ALT=10 MISSION_SPEED=2.5
flat_runway    MISSION_ALT=10 WORLD=iris_runway_vio.sdf
fixedyaw       MISSION_ALT=10 FIXED_YAW=1
EOF

if [ -n "$CFG" ]; then
  CONFIGS="$(grep -vE '^\s*(#|$)' "$CFG")"
else
  CONFIGS="$(printf '%s\n' "$DEFAULT_CONFIGS" | grep -vE '^\s*(#|$)')"
fi

# The scene, generated once for the lowest altitude anyone flies.
# SCENE_ALT pins the scene so a LATER sweep can be compared with an earlier one.
# The corridor cap is a function of altitude, so letting each sweep pick its own
# minimum silently changes the world between batches and makes their numbers
# incomparable -- which is the same span/scene mistake as comparing ATE over
# different trajectory lengths.
MIN_ALT="${SCENE_ALT:-}"
if [ -z "$MIN_ALT" ]; then
  MIN_ALT="$(printf '%s\n' "$CONFIGS" | grep -oE 'MISSION_ALT=[0-9.]+' \
             | cut -d= -f2 | sort -g | head -1)"
fi
MIN_ALT="${MIN_ALT:-10}"
echo "=============================================================="
echo "sweep: $(printf '%s\n' "$CONFIGS" | wc -l) configurations, ${REPEATS} replays each"
echo "scene generated once for the lowest altitude in the sweep: ${MIN_ALT} m"
echo "=============================================================="
bash "$HERE/gazebo/worlds/make_scene.sh" "$MIN_ALT"
echo

RESULTS="$OUT/results.tsv"
# n is NOT optional. A replay can fail -- OpenVINS initialises on a background
# thread and "failed static init: platform moving too much" happens
# nondeterministically on perfectly good data -- and replay_repeats.sh correctly
# reports "n=2" when it does. Dropping that column here is how a two-replay row
# gets read, written up and committed as three. Observed on fixedyaw_c.
[ -f "$RESULTS" ] || printf 'name\tsettings\tpath_m\tn\tdrift_med\tdrift_min\tdrift_max\tate_med\tcoverage\n' > "$RESULTS"

printf '%s\n' "$CONFIGS" | while read -r NAME REST; do
  [ -n "$NAME" ] || continue
  RUN="$HOME/vio_runs/simvio_sweep_$NAME"
  if grep -qP "^\Q$NAME\E\t" "$RESULTS" 2>/dev/null; then
    echo "== $NAME: already in $RESULTS, skipping"
    continue
  fi

  FREE="$(df -BG --output=avail / | tail -1 | tr -dc '0-9')"
  if [ "${FREE:-0}" -lt 9 ]; then
    echo "== stopping: only ${FREE}G free, need 9G for another flight" >&2
    break
  fi

  echo
  echo "############################################################"
  echo "== $NAME   $REST   (${FREE}G free)"
  echo "############################################################"
  rm -rf "$RUN"
  # $REST comes LAST so a per-config WORLD= overrides the default rather than
  # being overridden by it -- env applies assignments left to right, and the
  # flat-scene A/B is precisely a config that sets WORLD.
  # shellcheck disable=SC2086
  if ! env WORLD=iris_field_vio.sdf RECORD_SENSORS=1 RUN_BRIDGE=0 $REST \
       bash "$HERE/run_sim_vio.sh" "sweep_$NAME" > "$OUT/$NAME.flight.log" 2>&1; then
    echo "   FLIGHT FAILED -- see $OUT/$NAME.flight.log" >&2
    tail -5 "$OUT/$NAME.flight.log" >&2
    continue
  fi

  if ! bash "$HERE/../tools/evaluation/analyze_run.sh" "$RUN" "$REPEATS" 1.0 > "$OUT/$NAME.eval.log" 2>&1; then
    echo "   ANALYSIS FAILED -- see $OUT/$NAME.eval.log" >&2
    continue
  fi

  python3 - "$NAME" "$REST" "$RUN" "$OUT/$NAME.eval.log" "$RESULTS" <<'PY'
import re, sys, statistics as st
name, rest, run, log, results = sys.argv[1:6]
txt = open(log, errors="replace").read()
rows = []
try:
    rows = [l.split() for l in open(f"{run}/repeats_1.0.txt") if l.strip()]
except OSError:
    pass
d = [float(r[0]) for r in rows if r[0] != "nan"]
a = [float(r[1]) for r in rows if r[1] != "nan"]
c = [float(r[3]) for r in rows if r[3] != "nan"]
m = re.search(r"ground-truth path:\s*([\d.]+)\s*m", txt)
path = m.group(1) if m else "nan"
def f(v, fmt="{:.2f}"):
    return fmt.format(v) if v else "nan"
with open(results, "a") as fh:
    fh.write("\t".join([
        name, rest or "-", path, str(len(d)),
        f(st.median(d) if d else 0), f(min(d) if d else 0), f(max(d) if d else 0),
        f(st.median(a) if a else 0),
        f(min(c) if c else 0, "{:.0f}"),
    ]) + "\n")
if d:
    print(f"   {name}: drift median {st.median(d):.2f} % "
          f"(min {min(d):.2f}, max {max(d):.2f}) over n={len(d)}, "
          f"ATE {st.median(a):.2f} m")
    if len(d) < len(rows):
        print(f"   *** {len(rows) - len(d)} of {len(rows)} replays produced no "
              f"result -- this row is backed by {len(d)}, not {len(rows)} ***")
else:
    print(f"   {name}: no drift numbers parsed")
PY

  # Reclaim the recording; the trajectories and eval output are what matter.
  rm -rf "$RUN/bag" "$RUN"/rep*/est
  echo "   bag deleted, $(df -BG --output=avail / | tail -1 | tr -dc '0-9')G free"
done

echo
echo "=============================================================="
column -t -s "$(printf '\t')" "$RESULTS"
echo "=============================================================="
echo "raw: $RESULTS   logs: $OUT/*.log"
