#!/usr/bin/env python3
"""
Milestone 5's number: how far does the AIRCRAFT go wrong once GPS is gone?

This measures something different from `eval_sim_run.sh`, and the difference is
the whole point of the milestone. That script scores OpenVINS' trajectory. This
one scores ArduPilot's EKF3 — the estimate the vehicle is actually flying on,
after vision has been through the bridge, the source-set switch, and the
autopilot's own fusion. A good VIO trajectory that EKF3 fails to use is not a
GPS-denied aircraft.

The alignment is the careful part. ArduPilot reports position in its own NED
frame, whose origin is wherever the EKF happened to initialise and whose yaw is
whatever the compass said; the simulator reports its own. Rather than guess the
mapping, the GPS-ON segment supplies it: while GPS is available EKF3 tracks
truth closely, so a rigid 2-D fit over that segment recovers the transform, and
everything after GPS is removed is measured through it. That makes the reported
error entirely post-denial and removes any frame-convention guesswork from the
answer.

Reported: horizontal error against distance travelled since denial, which is the
same "% of distance" the VIO is scored on, so the two are comparable.

    gps_denied_eval.py --run ~/vio_runs/simvio_m5
"""
import argparse
import csv
import math
import os
import sys

import numpy as np


def load_events(path):
    ev = {}
    if not os.path.exists(path):
        return ev
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                ev[row["event"]] = float(row["t_boot"])
            except (KeyError, ValueError):
                continue
    return ev


def load_ekf(path):
    t, p = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_boot"]))
            # NED -> ENU-ish planar (x=east, y=north) so both sides are compared
            # in the same handedness; z is flipped out of "down".
            p.append([float(row["e"]), float(row["n"]), -float(row["d"])])
    return np.array(t), np.array(p)


def load_gt(bag, topic="/gz/ground_truth"):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"), ConverterOptions("", ""))
    t, p = [], []
    while r.has_next():
        tp, data, _ = r.read_next()
        if tp != topic:
            continue
        m = deserialize_message(data, Odometry)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        q = m.pose.pose.position
        p.append([q.x, q.y, q.z])
    return np.array(t), np.array(p)


def rigid_2d(src, dst):
    """Least-squares rotation+translation taking src onto dst, in the plane."""
    sc, dc = src.mean(0), dst.mean(0)
    H = (src - sc).T @ (dst - dc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    return R, dc - R @ sc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--max-dt", type=float, default=0.05,
                    help="max seconds between an EKF sample and a truth sample")
    a = ap.parse_args()

    ev = load_events(os.path.join(a.run, "events.csv"))
    if "gps_off" not in ev:
        sys.exit("no 'gps_off' event -- was this flown with GPS_DENIED=1?")
    t_off = ev["gps_off"]

    te, pe = load_ekf(os.path.join(a.run, "ekf_pos.csv"))
    tg, pg = load_gt(os.path.join(a.run, "bag"))
    if len(te) < 10 or len(tg) < 10:
        sys.exit("not enough samples")

    # The two clocks: ekf_pos.csv is time_boot_ms, ground truth is the sim
    # clock. Both advance in sim time and both start at vehicle boot, so they
    # share an origin to within the bridge's latency.
    idx = np.clip(np.searchsorted(tg, te), 0, len(tg) - 1)
    keep = np.abs(tg[idx] - te) < a.max_dt
    te, pe, pt = te[keep], pe[keep], pg[idx[keep]]
    if len(te) < 10:
        sys.exit("EKF and ground-truth clocks do not overlap")

    on = te < t_off
    off = te >= t_off
    if on.sum() < 20 or off.sum() < 20:
        sys.exit(f"too few samples either side of denial "
                 f"(on={on.sum()}, off={off.sum()})")

    # Frame recovered from the GPS-ON segment only.
    R, t_ = rigid_2d(pe[on][:, :2], pt[on][:, :2])
    proj = (pe[:, :2] @ R.T) + t_
    err = np.linalg.norm(proj - pt[:, :2], axis=1)
    zerr = np.abs(pe[:, 2] - pt[:, 2])

    # Distance travelled since denial, so the number is comparable with the
    # VIO's "% of distance travelled".
    dist = np.concatenate([[0], np.cumsum(
        np.linalg.norm(np.diff(pt[:, :2], axis=0), axis=1))])
    d_off = dist[off] - dist[off][0]

    print(f"GPS available : {on.sum():5d} samples, "
          f"horizontal error mean {err[on].mean():.2f} m, max {err[on].max():.2f} m")
    print(f"                (this is the alignment segment -- it is small by "
          f"construction)")
    print(f"\nGPS DENIED    : {off.sum():5d} samples over "
          f"{te[off][-1] - t_off:.1f} s of flight")
    print(f"  distance flown on vision alone : {d_off[-1]:7.1f} m")
    print(f"  horizontal error  mean {err[off].mean():6.2f} m   "
          f"max {err[off].max():6.2f} m   final {err[off][-1]:6.2f} m")
    print(f"  vertical   error  mean {zerr[off].mean():6.2f} m   "
          f"max {zerr[off].max():6.2f} m")
    if d_off[-1] > 1.0:
        print(f"  final error as % of distance flown denied : "
              f"{100 * err[off][-1] / d_off[-1]:.2f} %")
        print(f"  max   error as % of distance flown denied : "
              f"{100 * err[off].max() / d_off[-1]:.2f} %")

    # A hold that never moved would score well and mean nothing.
    if d_off[-1] < 10.0:
        print("\n  *** The vehicle travelled less than 10 m while denied. Drift")
        print("      is a fraction of distance travelled, so this result is")
        print("      close to free and does not demonstrate GPS-denied")
        print("      navigation. Fly a translating mission. ***")


if __name__ == "__main__":
    main()
