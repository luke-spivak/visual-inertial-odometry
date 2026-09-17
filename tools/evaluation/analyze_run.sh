#!/usr/bin/env bash
# analyze_run.sh -- everything between "the flight landed" and "here is the
# drift distribution", in one command.
#
#   ./analyze_run.sh /tmp/simvio_alt10tall [N_REPEATS] [RATE]
#
# The steps are individually cheap and individually easy to get wrong, and
# three of them are documented traps:
#
#   rebase   ros2 bag play replays at the WALL-CLOCK intervals a sim bag was
#            recorded at, which is whatever the machine managed -- one flight
#            held 96.5 s of sim time inside 31562 s of wall time. Rebasing puts
#            the bag on its sensor clock so 1.0x means 1.0x.
#   offset   measured from the bag, not guessed. Running the estimator through
#            the long stationary wait cost 623 m of ATE against 10.1 m.
#   repeats  OpenVINS initialises on a background thread, so one number from
#            one replay is not a result. Distribution or nothing.
#
# The original bag is kept alongside the rebased one. A flight costs ~25 minutes
# of wall clock and the rebase is a full rewrite; if it ever produces a bad bag,
# throwing away the only copy of the input would be expensive. At ~1 GB per
# flight that is affordable -- delete bag.orig by hand when disk gets tight.
set -eo pipefail

RUN="${1:?usage: analyze_run.sh RUN_DIR [N_REPEATS] [RATE]}"
N="${2:-3}"
RATE="${3:-1.0}"
LEAD="${LEAD:-8}"
HERE="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash
[ -d "$RUN/bag" ] || { echo "no bag at $RUN/bag" >&2; exit 1; }

if [ ! -f "$RUN/.rebased" ]; then
  echo "==> rebasing $RUN/bag onto its sensor clock"
  rm -rf "$RUN/bag.rebased"
  python3 "$HERE/rebase_bag_time.py" --in "$RUN/bag" --out "$RUN/bag.rebased"
  rm -rf "$RUN/bag.orig"
  mv "$RUN/bag" "$RUN/bag.orig"
  mv "$RUN/bag.rebased" "$RUN/bag"
  touch "$RUN/.rebased"
else
  echo "==> already rebased (remove $RUN/.rebased to redo)"
fi

echo
echo "==> start offset"
OFF="$(python3 "$HERE/find_takeoff.py" --bag "$RUN/bag" --lead "$LEAD" --verbose)"
echo "    START_OFFSET=$OFF  (${LEAD}s of pad time kept before takeoff)"
echo "$OFF" > "$RUN/start_offset.txt"

echo
START_OFFSET="$OFF" bash "$HERE/replay_repeats.sh" "$RUN" "$N" "$RATE"

# The distribution says how bad; this says where. Run it on the first repeat --
# they differ only by the estimator's own nondeterminism.
REP="$RUN/rep1_${RATE}"
if [ -f "$REP/gt.tum" ] && [ -f "$REP/est.tum" ]; then
  echo
  echo "############ per-leg breakdown (repeat 1) ############"
  STATE_ARG=()
  [ -f "$REP/ov_sim_estimate.txt" ] && STATE_ARG=(--state "$REP/ov_sim_estimate.txt")
  python3 "$HERE/turn_diagnostic.py" --gt "$REP/gt.tum" --est "$REP/est.tum" \
    "${STATE_ARG[@]}"
fi
