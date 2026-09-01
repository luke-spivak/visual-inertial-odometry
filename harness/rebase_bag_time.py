#!/usr/bin/env python3
"""
Rewrite a bag so each message's receive time equals its header stamp.

`ros2 bag play` replays at the WALL-CLOCK intervals the messages were recorded
at, which for a sim bag is meaningless: it is the rate Gazebo happened to
manage, not the rate the sensors report. A flight recorded while the sim
crawled held 96.5 s of sim time inside 31562 s of wall time, so replaying it
faithfully would take 8.8 hours to deliver 96 seconds of flight.

After rebasing, playing at 1.0x delivers the true sensor cadence -- 30 Hz
images, 200 Hz IMU -- and takes as long as the flight actually lasted. Replay
becomes reproducible in the way that matters for tuning: the estimator sees
the same stream at the same rate every time, regardless of what the machine
was doing during recording.

Messages without a usable header keep their original timestamp.

    python3 rebase_bag_time.py --in /tmp/simvio_flight10/bag --out /tmp/f10_rebased
"""
import argparse
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="src", required=True)
    ap.add_argument("--out", dest="dst", required=True)
    ap.add_argument("--storage", default="sqlite3")
    a = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.src, storage_id=a.storage),
                rosbag2_py.ConverterOptions("", ""))
    topics = reader.get_all_topics_and_types()

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.dst, storage_id=a.storage),
                rosbag2_py.ConverterOptions("", ""))
    for t in topics:
        writer.create_topic(t)

    classes = {}
    for t in topics:
        try:
            classes[t.name] = get_message(t.type)
        except Exception:
            classes[t.name] = None

    # Messages must be written in nondecreasing timestamp order, and rebasing
    # reorders them relative to how they were recorded, so buffer and sort.
    out, kept, n = [], 0, 0
    while reader.has_next():
        topic, data, recv = reader.read_next()
        stamp = recv
        cls = classes.get(topic)
        if cls is not None:
            try:
                msg = deserialize_message(data, cls)
                h = msg.header.stamp
                ns = h.sec * 1_000_000_000 + h.nanosec
                if ns > 0:
                    stamp = ns
                else:
                    kept += 1
            except AttributeError:
                kept += 1
        out.append((stamp, topic, data))
        n += 1

    out.sort(key=lambda r: r[0])
    for stamp, topic, data in out:
        writer.write(topic, data, stamp)

    span = (out[-1][0] - out[0][0]) / 1e9 if out else 0
    print(f"rebased {n} messages ({kept} had no header stamp) -> {a.dst}")
    print(f"new span {span:.1f} s; play at 1.0x for true sensor cadence")


if __name__ == "__main__":
    main()
