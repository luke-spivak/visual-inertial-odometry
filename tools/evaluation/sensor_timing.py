#!/usr/bin/env python3
"""
Are the recorded sensor streams actually uniform?

An estimator cannot tell a gap in the IMU stream from a period of no motion. It
integrates across whatever it is given, so a dropped burst of samples during a
dynamic manoeuvre injects a velocity error at exactly the moment the vehicle is
accelerating hardest -- and the result looks like a bad accelerometer bias,
because that is the only state the filter has to explain it with.

This rig runs Gazebo under software rendering at RTF ~0.2 with the estimator,
the bridge and the recorder competing for the same cores, so gaps are a real
possibility rather than a theoretical one. Checking is cheap; assuming is what
costs flights.

Reports, per topic: the sample interval distribution, the largest gaps, and
where they fall in the flight.

    sensor_timing.py --bag RUN/bag
"""
import argparse
from collections import defaultdict

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topics", nargs="*",
                    default=["/imu0", "/cam0/image_raw", "/gz/ground_truth"])
    ap.add_argument("--top", type=int, default=6, help="how many gaps to list")
    a = ap.parse_args()

    r = SequentialReader()
    r.open(StorageOptions(uri=a.bag, storage_id="sqlite3"),
           ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in types.items() if n in a.topics}

    # The HEADER stamp, not the bag write time. The write time is when the
    # recorder got round to it; the header stamp is when the sensor says the
    # sample was taken, and that is what the estimator integrates against.
    stamps = defaultdict(list)
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in cls:
            continue
        h = deserialize_message(data, cls[topic]).header.stamp
        stamps[topic].append(h.sec + h.nanosec * 1e-9)

    for topic in a.topics:
        ts = sorted(stamps.get(topic, []))
        if len(ts) < 3:
            print(f"\n{topic}: {len(ts)} messages -- nothing to check")
            continue
        t0 = ts[0]
        d = [b - a_ for a_, b in zip(ts, ts[1:])]
        d_sorted = sorted(d)
        n = len(d)
        med = d_sorted[n // 2]
        print(f"\n{topic}")
        print(f"  {len(ts)} messages over {ts[-1] - t0:.2f} s "
              f"= {len(ts) / (ts[-1] - t0):.2f} Hz")
        print(f"  interval  median {med * 1e3:.3f} ms   "
              f"p99 {d_sorted[int(0.99 * n)] * 1e3:.3f} ms   "
              f"max {d_sorted[-1] * 1e3:.3f} ms")
        # A "gap" is anything that swallowed at least one whole expected sample.
        gaps = [(dt, ts[i]) for i, dt in enumerate(d) if dt > 1.8 * med]
        missing = sum(round(dt / med) - 1 for dt, _ in gaps)
        print(f"  gaps > 1.8x median: {len(gaps)}   "
              f"samples implied missing: {missing} "
              f"({100 * missing / max(len(ts), 1):.2f} % of the stream)")
        for dt, when in sorted(gaps, reverse=True)[:a.top]:
            print(f"     {dt * 1e3:9.2f} ms at t+{when - t0:6.2f} s "
                  f"(~{round(dt / med) - 1} samples lost)")
        if not gaps:
            print("     none -- the stream is uniform")


if __name__ == "__main__":
    main()
