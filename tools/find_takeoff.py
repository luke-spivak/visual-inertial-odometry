#!/usr/bin/env python3
"""
Print the START_OFFSET that puts the estimator on the pad a few seconds before
takeoff, measured from the bag rather than guessed.

This matters more than it looks. Running OpenVINS through the vehicle's whole
stationary wait costs accuracy, not just time: with no motion there is no
parallax, MSCKF features cannot be triangulated, and the filter propagates on
IMU alone. Cutting a 45 s wait down to an 8 s lead-in took ATE from 623 m to
10.1 m on this rig -- the single largest improvement measured so far. Too short
is equally bad: static initialisation needs a stationary window WITH trackable
features before the vehicle moves.

    find_takeoff.py --bag /tmp/alt10tall_rebased --lead 8

prints one number, for:  START_OFFSET=$(find_takeoff.py --bag ...)
"""
import argparse
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/gz/ground_truth")
    ap.add_argument("--lead", type=float, default=8.0,
                    help="seconds of stationary pad time to keep before takeoff")
    ap.add_argument("--climb", type=float, default=0.5,
                    help="metres above the start height that counts as airborne")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    r = SequentialReader()
    r.open(StorageOptions(uri=a.bag, storage_id="sqlite3"),
           ConverterOptions("", ""))

    t_first = None      # first message of ANY topic: bag play's offset origin
    z0 = None
    t_takeoff = None
    while r.has_next():
        topic, data, stamp = r.read_next()
        t = stamp / 1e9
        if t_first is None or t < t_first:
            t_first = t
        if topic != a.topic:
            continue
        msg = deserialize_message(data, Odometry)
        # The message's own stamp, not the bag's write stamp: --start-offset is
        # relative to the bag clock, but the two agree once rebase_bag_time.py
        # has put the bag on its sensor clock, and this is the one that is
        # meaningful if it has not.
        z = msg.pose.pose.position.z
        if z0 is None:
            z0 = z
        if t_takeoff is None and z - z0 > a.climb:
            t_takeoff = t

    if t_first is None:
        sys.exit("empty bag")
    if t_takeoff is None:
        sys.exit(f"never climbed {a.climb} m above the start height on {a.topic}"
                 " -- did the vehicle actually fly?")

    off = max(0.0, t_takeoff - t_first - a.lead)
    if a.verbose:
        print(f"bag starts   {t_first:.3f}", file=sys.stderr)
        print(f"takeoff at   {t_takeoff:.3f}  (+{t_takeoff - t_first:.2f} s)",
              file=sys.stderr)
        print(f"lead-in      {a.lead:.1f} s", file=sys.stderr)
    print(f"{off:.2f}")


if __name__ == "__main__":
    main()
