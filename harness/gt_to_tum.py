#!/usr/bin/env python3
"""
Extract a trajectory from a rosbag2 and write it in TUM format for evo.

Handles both nav_msgs/Odometry (ground truth from Gazebo's OdometryPublisher,
and OpenVINS' own /ov_msckf/odomimu) so the two sides of a comparison go
through identical code -- a format bug that hits both equally is far easier to
spot than one that silently biases only the reference.

    python3 gt_to_tum.py --bag /tmp/simvio_flight4/bag --topic /gz/ground_truth --out gt.tum
    python3 gt_to_tum.py --bag /tmp/simvio_flight4/bag --list

WHY NOT /world/<w>/dynamic_pose/info, the obvious ground-truth source: gz fills
in entity names and a header stamp there, but ros_gz_bridge's Pose_V ->
TFMessage conversion delivers every transform with empty parent and child frame
ids and stamp 0. Nothing errors. The bag records thousands of well-formed
messages and the trajectory is unrecoverable. The world file attaches an
OdometryPublisher to the model instead; see gazebo/worlds/iris_field_vio.sdf.
"""
import argparse
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def reader_for(bag, storage_id="sqlite3"):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    return r


def rotate(q, v):
    """Rotate v by quaternion q=(x,y,z,w). Written out rather than pulled from
    scipy so this runs anywhere ROS does."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v);  v' = v + w*t + q_vec x t
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def row(msg, lever=(0.0, 0.0, 0.0)):
    """nav_msgs/Odometry -> (t, x, y, z, qx, qy, qz, qw), with `lever` applied.

    Gazebo's OdometryPublisher advertises <robot_base_frame>, but that string is
    only a frame LABEL on the outgoing message -- the pose it publishes is always
    the MODEL's, whatever you put there. So the offset from the model origin to
    vio_link has to be applied here.

    It matters: the offset is 0.102 m, against a EuRoC-baseline ATE of 0.067-
    0.115 m, and because it is a BODY-frame lever arm it is not removed by evo's
    Umeyama alignment (that takes out a single global rigid transform). It would
    show up as yaw-dependent error and read as estimator drift.
    """
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if lever != (0.0, 0.0, 0.0):
        dx, dy, dz = rotate((q.x, q.y, q.z, q.w), lever)
    else:
        dx = dy = dz = 0.0
    return (t, p.x + dx, p.y + dy, p.z + dz, q.x, q.y, q.z, q.w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/gz/ground_truth")
    ap.add_argument("--storage", default="sqlite3")
    ap.add_argument("--out")
    ap.add_argument("--lever", default="",
                    help="body-frame offset x,y,z from the published frame to "
                         "the one being compared. For /gz/ground_truth use "
                         "0.10,0,0.02 (vio_link in the model frame). Re-measure "
                         "with: gz topic -e -t /world/<w>/pose/info -n 1")
    ap.add_argument("--list", action="store_true",
                    help="print the bag's topics, types and message counts")
    a = ap.parse_args()

    r = reader_for(a.bag, a.storage)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}

    if a.list:
        counts = {}
        while r.has_next():
            topic, _, _ = r.read_next()
            counts[topic] = counts.get(topic, 0) + 1
        for name in sorted(types):
            print(f"{name:28s} {types[name]:28s} {counts.get(name, 0)}")
        return

    if a.topic not in types:
        sys.exit(f"{a.topic} not in bag. present: {sorted(types)}")
    if not types[a.topic].endswith("Odometry"):
        sys.exit(f"{a.topic} is {types[a.topic]}; this reads nav_msgs/msg/Odometry")
    msg_cls = get_message(types[a.topic])

    lever = (0.0, 0.0, 0.0)
    if a.lever:
        parts = [float(v) for v in a.lever.split(",")]
        if len(parts) != 3:
            sys.exit("--lever wants three comma-separated numbers")
        lever = tuple(parts)

    rows = []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == a.topic:
            rows.append(row(deserialize_message(data, msg_cls), lever))

    if not rows:
        sys.exit(f"no messages on {a.topic}")

    # evo requires strictly increasing timestamps; sim time can repeat a stamp
    # when the publisher runs faster than the clock ticks.
    rows.sort(key=lambda x: x[0])
    dedup = [rows[0]]
    for rr in rows[1:]:
        if rr[0] > dedup[-1][0]:
            dedup.append(rr)
    dropped = len(rows) - len(dedup)

    out = a.out or "-"
    f = sys.stdout if out == "-" else open(out, "w")
    for rr in dedup:
        f.write(" ".join(f"{v:.9f}" for v in rr) + "\n")
    if f is not sys.stdout:
        f.close()
        span = dedup[-1][0] - dedup[0][0]
        note = f", {dropped} duplicate stamps dropped" if dropped else ""
        print(f"wrote {len(dedup)} poses ({span:.1f} s){note} -> {out}")


if __name__ == "__main__":
    main()
