#!/usr/bin/env python3
"""
imu_profile.py -- find bench disturbances in an Allan capture before trusting it.

    ./imu_profile.py ~/imu/run1.json

Splits the record into 30 s blocks and reports the worst-axis standard deviation
of each against the median, plus a sparkline of the whole run.

This exists because of what it caught. A three-hour stationary run reported
100 % contiguous, zero FIFO overruns and no timestamp gaps -- and was still
wrong, because someone walked past the bench once for thirty seconds. A
disturbance is not a gap: every check in the capture path passes straight
through it. That block came out at 11.2x the median on both sensors against 360
flat ones, and 0.3 % of the record was enough to put the accelerometer random
walk out by a factor of ten and break the short-tau slope from -0.46 to -0.22.

So: run this first, then trim with `allan.py --start/--end`, then believe the
numbers. Not the other way round.
"""
import json, sys, numpy as np

side = json.load(open(sys.argv[1]))
BLOCK = 30.0   # seconds

def load(kind):
    m = side["devices"][kind]
    ch = m["channels"]
    names = [c["name"] for c in ch]
    fmts = [("<" + ("i" if c["signed"] else "u") + str(c["storage"])) for c in ch]
    offs = [c["offset"] for c in ch]
    dt = np.dtype({"names": names, "formats": fmts, "offsets": offs,
                   "itemsize": m["record_bytes"]})
    return m, np.fromfile(m["file"], dtype=dt)

out = {}
for kind, pre in (("accel", "in_accel_"), ("gyro", "in_anglvel_")):
    m, a = load(kind)
    ts = a["in_timestamp"].astype(np.float64) * 1e-9
    span = ts[-1] - ts[0]
    n_blocks = int(span // BLOCK)
    per = int(len(a) // n_blocks)
    xyz = np.stack([a[pre + ax].astype(np.float64) * m["scale"] for ax in "xyz"], 1)
    xyz = xyz[:n_blocks * per].reshape(n_blocks, per, 3)
    sd = xyz.std(axis=1).max(axis=1)          # worst axis per block
    out[kind] = (sd, span, n_blocks)
    print(f"{kind}: {n_blocks} blocks of {BLOCK:.0f} s, worst-axis stdev per block")
    med = np.median(sd)
    print(f"  median {med:.6g}   max {sd.max():.6g} at block {int(sd.argmax())} "
          f"(t = {sd.argmax()*BLOCK/60:.1f} min)")
    bad = np.flatnonzero(sd > 3 * med)
    if len(bad):
        print(f"  blocks over 3x median: {len(bad)}")
        for b in bad:
            print(f"    block {b:3d}  t = {b*BLOCK/60:6.1f} min  stdev {sd[b]:.6g} "
                  f"({sd[b]/med:.1f}x median)")
    else:
        print("  no block exceeds 3x the median")
    print()

# a compact sparkline over the whole run, so a slow drift is visible too
for kind in ("accel", "gyro"):
    sd, span, nb = out[kind]
    lo, hi = sd.min(), sd.max()
    bars = " .:-=+*#%@"
    line = "".join(bars[min(9, int(9 * (v - lo) / (hi - lo + 1e-30)))] for v in sd)
    print(f"{kind:5} |{line}|  0 -> {span/60:.0f} min")
