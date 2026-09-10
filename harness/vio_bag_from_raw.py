#!/usr/bin/env python3
"""
vio_bag_from_raw.py -- turn a camera+IMU capture into the ROS 2 bag that
replay_openvins.sh plays. Runs on the VM host (jazzy), not in the Kalibr image:

    source /opt/ros/jazzy/setup.bash
    python3 vio_bag_from_raw.py ~/vio/walk1 ~/vio/walk1_run

Reads <prefix>.y16 + <prefix>.meta.json (camera) and <prefix>.imu.json +
its .bin streams (IMU, via kalibr_imu_csv.load). Writes <run_dir>/bag
(sqlite3) with /cam0/image_raw (mono8) and /imu0, stamped on absolute
CLOCK_MONOTONIC -- the same clock for both sensors, as in calibration. The
residual 2.18 ms offset is in the OpenVINS config, not applied here.

Same refusals as kalibr_bag_from_raw.py: byte count must be whole 1280x800
16-bit frames, values must be the R8 mode's 8-bit-in-16 layout, frame count
must equal metadata count, stamps must increase, and the IMU must cover the
camera on the same clock.
"""
import argparse, json, os, shutil, sys

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, Imu

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kalibr_imu_csv import load  # noqa: E402


def fail(msg):
    sys.exit(f"FAIL: {msg}")


def stamp(msg, ns, frame):
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(ns), 1_000_000_000)
    msg.header.frame_id = frame


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prefix"); ap.add_argument("run_dir")
    ap.add_argument("--width", type=int, default=1280); ap.add_argument("--height", type=int, default=800)
    a = ap.parse_args()

    per = a.width * a.height
    y16 = a.prefix + ".y16"
    raw = np.memmap(y16, dtype="<u2", mode="r")
    if raw.size == 0 or raw.size % per:
        fail(f"{y16}: not a whole number of {a.width}x{a.height} 16-bit frames -- wrong mode?")
    frames = raw.reshape(-1, a.height, a.width)
    if (frames[0] % 256).any() or (frames[-1] % 256).any():
        fail("not the R8 mode's 8-bit-in-16 layout -- refusing to guess a conversion")
    recs = json.load(open(a.prefix + ".meta.json"))
    if len(recs) != len(frames):
        fail(f"{len(frames)} frames but {len(recs)} metadata records")
    tc = np.array([int(r["SensorTimestamp"]) for r in recs], dtype=np.int64)
    d = np.diff(tc)
    if (d <= 0).any():
        fail(f"{int((d <= 0).sum())} non-increasing camera timestamps")
    period = float(np.median(d))
    print(f"  camera  {len(tc)} frames, {1e9 / period:.2f} fps, "
          f"{int((d > 1.5 * period).sum())} gap(s), span {(tc[-1] - tc[0]) / 1e9:.1f} s")

    side = json.load(open(a.prefix + ".imu.json")); side["_path"] = a.prefix + ".imu.json"
    tg, w = load(side, "gyro")
    ta, acc = load(side, "accel")
    for name, t in (("gyro", tg), ("accel", ta)):
        if (np.diff(t) <= 0).any():
            fail(f"{name} has non-increasing timestamps")
    if tg[0] > tc[0] or tg[-1] < tc[-1]:
        fail(f"IMU {tg[0]}..{tg[-1]} does not cover camera {tc[0]}..{tc[-1]} -- same clock?")
    keep = (tg >= tc[0] - 1_000_000_000) & (tg <= tc[-1] + 1_000_000_000) & (tg >= ta[0]) & (tg <= ta[-1])
    tg, w = tg[keep], w[keep]
    acc = np.stack([np.interp(tg, ta, acc[:, k]) for k in range(3)], axis=1)
    g = np.linalg.norm(acc[:200], axis=1).mean()
    print(f"  imu     {len(tg)} samples, {1e9 / np.median(np.diff(tg)):.1f} Hz; "
          f"|accel| over the first 0.5 s {g:.2f} m/s^2 (9.81 if still)")

    os.makedirs(a.run_dir, exist_ok=True)
    bag = os.path.join(a.run_dir, "bag")
    if os.path.exists(bag):
        shutil.rmtree(bag)
    wr = rosbag2_py.SequentialWriter()
    wr.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
    for i, (name, typ) in enumerate((("/cam0/image_raw", "sensor_msgs/msg/Image"), ("/imu0", "sensor_msgs/msg/Imu"))):
        wr.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=typ, serialization_format="cdr"))

    # Merge by stamp so playback order is time order.
    ci = ii = 0
    while ci < len(tc) or ii < len(tg):
        if ii >= len(tg) or (ci < len(tc) and tc[ci] <= tg[ii]):
            m = Image(); stamp(m, tc[ci], "cam0")
            m.height, m.width, m.encoding, m.is_bigendian, m.step = a.height, a.width, "mono8", 0, a.width
            m.data = (frames[ci] >> 8).astype(np.uint8).tobytes()
            wr.write("/cam0/image_raw", serialize_message(m), int(tc[ci])); ci += 1
        else:
            m = Imu(); stamp(m, tg[ii], "imu0")
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, w[ii])
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, acc[ii])
            wr.write("/imu0", serialize_message(m), int(tg[ii])); ii += 1
    del wr

    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    n = {"/cam0/image_raw": 0, "/imu0": 0}
    while rd.has_next():
        n[rd.read_next()[0]] += 1
    if n["/cam0/image_raw"] != len(tc) or n["/imu0"] != len(tg):
        fail(f"bag read back {n}, expected {len(tc)} images and {len(tg)} imu")
    print(f"  bag     {bag}: {n['/cam0/image_raw']} images, {n['/imu0']} imu -- read back OK")


if __name__ == "__main__":
    main()
