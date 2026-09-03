#!/usr/bin/env bash
# replay_repeats.sh -- run the estimator N times on ONE recorded flight and
# report the distribution of drift, not a single number.
#
#   START_OFFSET=48 ./replay_repeats.sh /tmp/simvio_alt10tall 3 1.0
#
# Why a distribution is mandatory here. OpenVINS initialises on a background
# thread, so identical input does NOT give identical output: PROJECT.md measured
# a 1.7x spread in ATE across two EuRoC runs of the same bag. A single drift
# number from this setup carries roughly +-40 % of noise, which is more than
# enough to "confirm" a fix that did nothing. Replaying the SAME bag isolates
# that estimator nondeterminism from flight-to-flight variation, because the
# input really is byte-identical.
#
# The span of each estimate is printed with its drift on purpose. A run that
# diverges and stops early scores BETTER on ATE than one that flew the whole
# mission, so a mean taken over runs of differing spans is meaningless.
set -eo pipefail

RUN="${1:?usage: replay_repeats.sh RUN_DIR [N] [RATE]}"
N="${2:-3}"
RATE="${3:-1.0}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SUM="$RUN/repeats_${RATE}.txt"
: > "$SUM"

for i in $(seq 1 "$N"); do
  echo "############ repeat $i/$N ############"
  bash "$HERE/replay_openvins.sh" "$RUN" "$RATE" 2>&1 | tail -12
  # Keep each repeat's estimate; replay_openvins.sh reuses replay_<rate>/.
  KEEP="$RUN/rep${i}_${RATE}"
  rm -rf "$KEEP"; cp -r "$RUN/replay_${RATE}" "$KEEP"

  bash "$HERE/eval_sim_run.sh" "$RUN" "$KEEP/est" 2>&1 | tee "$KEEP/eval.txt" \
    | grep -E "ground-truth path|ground truth :|estimate  |overlap|PARTIAL|rmse|DRIFT" || true
  # Per-repeat trajectories, so a later comparison is not overwritten.
  cp "$RUN/est.tum" "$KEEP/est.tum" 2>/dev/null || true
  cp "$RUN/gt.tum"  "$KEEP/gt.tum"  2>/dev/null || true

  # Parsed in python, not with greps. evo prints both ATE and RPE blocks with
  # an identically-named "rmse" row, and the two must not be confused; the
  # section header is the only thing that separates them.
  python3 - "$KEEP/eval.txt" >> "$SUM" <<'PY'
import re, sys
txt = open(sys.argv[1], errors="replace").read()

def section(start, end=None):
    i = txt.find(start)
    if i < 0:
        return ""
    j = txt.find(end, i) if end else len(txt)
    return txt[i:j if j > 0 else len(txt)]

def rmse_of(sec):
    m = re.search(r"^\s*rmse\s+([0-9.eE+-]+)", sec, re.M)
    return m.group(1) if m else "nan"

def one(pat, src=txt, grp=1):
    m = re.search(pat, src, re.M)
    return m.group(grp) if m else "nan"

drift = one(r"DRIFT:\s*([0-9.]+)")
ate = rmse_of(section("==> ATE (aligned", "==> RPE"))
span = one(r"^\s*overlap\s*:\s*([0-9.]+)\s*s")
cov = one(r"=\s*([0-9.]+)\s*% of the flight")
print(drift, ate, span, cov)
PY
  echo
done

echo "############ distribution over $N repeats of $RUN ############"
python3 - "$SUM" <<'PY'
import sys, statistics as st
rows = [l.split() for l in open(sys.argv[1]) if l.strip()]
def col(i):
    return [float(r[i]) for r in rows if r[i] != "nan"]
d, a, s, p = col(0), col(1), col(2), col(3)
if not d:
    print("  no drift numbers parsed -- check the per-repeat eval.txt files")
    raise SystemExit(1)
def fmt(name, v, unit):
    line = (f"  {name:<12} n={len(v)}  min {min(v):8.2f}  median {st.median(v):8.2f}"
            f"  max {max(v):8.2f}  {unit}")
    if len(v) > 1:
        line += f"   (spread {max(v) / max(min(v), 1e-9):.2f}x)"
    print(line)
fmt("drift", d, "%")
fmt("ATE rmse", a, "m")
fmt("span", s, "s")
if p and (max(p) - min(p)) > 5:
    print(f"  *** spans differ by {max(p) - min(p):.0f} percentage points of the "
          "flight; these drift numbers are NOT comparable to each other ***")
elif p:
    print(f"  all repeats covered {min(p):.0f}-{max(p):.0f} % of the flight")
PY
echo
echo "raw: $SUM   (drift% ATE_rmse span_s coverage%)"
