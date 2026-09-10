#!/usr/bin/env python3
"""
vio_closure.py -- loop-closure error of a replayed walk: the rig started and
ended on the same marked spot, so the true displacement is zero.

    python3 vio_closure.py ~/vio/walk1_run/replay_0.5/ov_estimate.txt

Reads OpenVINS's save_total_state file (timestamp, q[4], p[3], v[3], ...).
Start/end positions are means over the first/last --still seconds, when the
rig is resting on the mark. Reports closure error in m and as % of path length.
"""
import argparse
import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("estimate")
ap.add_argument("--still", type=float, default=1.0, help="seconds averaged at each end (default 1)")
a = ap.parse_args()

s = np.loadtxt(a.estimate, comments="#", ndmin=2)
if len(s) < 2:
    raise SystemExit(f"FAIL: {a.estimate} has {len(s)} states -- the estimator never initialized "
                     f"(static init needs the rig still for ~2 s, then moved)")
t, p, v = s[:, 0], s[:, 5:8], s[:, 8:11]
p0 = p[t <= t[0] + a.still].mean(axis=0)
p1 = p[t >= t[-1] - a.still].mean(axis=0)
path = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
err = np.linalg.norm(p1 - p0)
print(f"  estimate  {len(t)} states over {t[-1] - t[0]:.1f} s")
print(f"  path      {path:.2f} m; max |v| {np.linalg.norm(v, axis=1).max():.2f} m/s; "
      f"extent {np.ptp(p, axis=0).round(2)} m (x y z)")
print(f"  closure   {err:.3f} m = {100 * err / path:.2f} % of path  "
      f"(dx dy dz {np.round(p1 - p0, 3)})")
print(f"  end speed {np.linalg.norm(v[t >= t[-1] - a.still], axis=1).mean():.3f} m/s (0 if the rig ended resting)")
