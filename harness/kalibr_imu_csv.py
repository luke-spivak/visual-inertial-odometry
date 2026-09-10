#!/usr/bin/env python3
"""
kalibr_imu_csv.py -- turn an imu_log.py capture into the imu0.csv Kalibr wants.

    ./kalibr_imu_csv.py ~/imu/run.json out/imu0.csv [--start NS --end NS]

imu_log.py writes accel and gyro as two IIO streams with their own timestamps.
Kalibr wants one row per sample: timestamp_ns, wx, wy, wz (rad/s), ax, ay, az
(m/s^2). Rows are laid on the gyro's timestamps and accel is linearly
interpolated onto them -- both come out of one hardware FIFO at the same ODR,
so the correction is sub-sample, but it is not zero and a nearest-neighbour
pairing would put up to half a period of time error into every accel sample.

Timestamps stay absolute CLOCK_MONOTONIC in ns, the clock the camera's
SensorTimestamp is on, so kalibr_bagcreater lines the two sensors up with no
offset applied here. Any residual offset is what Kalibr's time calibration is
for.

--start / --end trim to a window, normally the camera capture's span plus a
little margin: an IMU log started before the camera is only dead weight.
"""
import argparse, json, os, sys
import numpy as np


def load(side, kind):
    m = side["devices"][kind]
    path = m["file"]
    if not os.path.exists(path):
        path = os.path.join(os.path.dirname(os.path.abspath(side["_path"])), os.path.basename(path))
    names, fmts, offs = [], [], []
    for c in m["channels"]:
        names.append(c["name"])
        fmts.append(("<" if c["endian"] == "le" else ">") + ("i" if c["signed"] else "u") + str(c["storage"]))
        offs.append(c["offset"])
    a = np.fromfile(path, dtype=np.dtype({"names": names, "formats": fmts, "offsets": offs,
                                           "itemsize": m["record_bytes"]}))
    pre = "in_anglvel_" if kind == "gyro" else "in_accel_"
    xyz = np.stack([a[pre + ax].astype(np.float64) * m["scale"] for ax in "xyz"], axis=1)
    return a["in_timestamp"].astype(np.int64), xyz


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sidecar"); ap.add_argument("out")
    ap.add_argument("--start", type=int); ap.add_argument("--end", type=int)
    a = ap.parse_args()
    side = json.load(open(a.sidecar)); side["_path"] = a.sidecar
    tg, w = load(side, "gyro")
    ta, acc = load(side, "accel")
    for name, t in (("gyro", tg), ("accel", ta)):
        d = np.diff(t)
        if (d <= 0).any():
            sys.exit(f"FAIL: {name} has {int((d <= 0).sum())} non-increasing timestamps")
    lo, hi = max(tg[0], ta[0]), min(tg[-1], ta[-1])
    if a.start is not None: lo = max(lo, a.start)
    if a.end is not None: hi = min(hi, a.end)
    keep = (tg >= lo) & (tg <= hi)
    tg, w = tg[keep], w[keep]
    if len(tg) < 100:
        sys.exit(f"FAIL: only {len(tg)} gyro samples inside the window")
    acc_i = np.stack([np.interp(tg, ta, acc[:, k]) for k in range(3)], axis=1)
    d = np.diff(tg)
    med = float(np.median(d))
    gaps = int((d > 1.5 * med).sum())
    rows = np.column_stack([tg, w, acc_i])
    with open(a.out, "w") as fh:
        fh.write("timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z\n")
        for r in rows:
            fh.write(f"{int(r[0])},{r[1]:.9f},{r[2]:.9f},{r[3]:.9f},{r[4]:.9f},{r[5]:.9f},{r[6]:.9f}\n")
    g = np.linalg.norm(acc_i, axis=1)
    print(f"  imu0.csv    {len(tg)} rows, {1e9 / med:.2f} Hz, {gaps} gap(s), span {(tg[-1] - tg[0]) / 1e9:.1f} s")
    print(f"  first/last  {tg[0]} .. {tg[-1]} ns (CLOCK_MONOTONIC)")
    print(f"  |accel|     median {np.median(g):.3f} m/s^2 (9.81 at rest); |gyro| max {np.degrees(np.linalg.norm(w, axis=1).max()):.0f} dps")


if __name__ == "__main__":
    main()
