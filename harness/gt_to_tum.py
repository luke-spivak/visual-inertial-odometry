#!/usr/bin/env python3
"""
Extract Gazebo ground truth from a rosbag2 and write it in TUM format for evo.

The bridge maps gz's `dynamic_pose/info` (a Pose_V) onto tf2_msgs/TFMessage.
gz publishes MODEL poses relative to the world and LINK poses relative to
their parent model, so a link's world pose is the product of the chain --
this walks that chain per message rather than assuming a flat hierarchy.

The target is `vio_link`, not the airframe's base_link: OpenVINS estimates the
pose of the IMU, and vio_imu sits at the vio_link origin with no rotation
(see gazebo/models/iris_with_vio/model.sdf). Comparing against base_link would
add a body-frame lever arm, and a lever arm is NOT absorbed by evo's Umeyama
alignment -- that only removes one global rigid transform, so what is left
shows up as rotation-dependent error masquerading as drift.

    python3 gt_to_tum.py --bag /tmp/simvio_flight2/bag --list
    python3 gt_to_tum.py --bag /tmp/simvio_flight2/bag --out gt.tum
"""
import argparse
import sys

import numpy as np
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


def mat(t):
    """TransformStamped -> 4x4 homogeneous matrix."""
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])
    tr = t.transform.translation
    m[:3, 3] = (tr.x, tr.y, tr.z)
    return m


def quat_of(m):
    """Rotation part of a 4x4 -> (x, y, z, w). Shepperd's method: pick the
    branch with the largest denominator so no near-zero division is taken."""
    r = m[:3, :3]
    tr = np.trace(r)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/gz/pose_info")
    ap.add_argument("--child", default="vio_link")
    ap.add_argument("--storage", default="sqlite3")
    ap.add_argument("--out")
    ap.add_argument("--list", action="store_true",
                    help="print the parent -> child pairs present and exit")
    a = ap.parse_args()

    r = reader_for(a.bag, a.storage)
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if a.topic not in types:
        sys.exit(f"{a.topic} not in bag. present: {sorted(types)}")
    msg_cls = get_message(types[a.topic])

    seen, rows, chain_fail = set(), [], 0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic != a.topic:
            continue
        m = deserialize_message(data, msg_cls)

        # One TFMessage is a full snapshot of the world's dynamic poses, so the
        # chain can be resolved inside this message alone -- no TF buffer, no
        # interpolation across timestamps.
        by_child = {t.child_frame_id: t for t in m.transforms}
        seen.update((t.header.frame_id, t.child_frame_id) for t in m.transforms)
        if a.list:
            continue

        t = by_child.get(a.child)
        if t is None:
            continue

        acc, cur, hops = np.eye(4), t, 0
        while cur is not None and hops < 8:
            acc = mat(cur) @ acc
            parent = cur.header.frame_id
            if parent in ("", "world", "map", "default"):
                break
            cur = by_child.get(parent)
            hops += 1
        else:
            chain_fail += 1
            continue

        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        qx, qy, qz, qw = quat_of(acc)
        rows.append((stamp, *acc[:3, 3], qx, qy, qz, qw))

    if a.list:
        for p, c in sorted(seen):
            print(f"{p or '<empty>'} -> {c}")
        return

    if not rows:
        sys.exit(f"no '{a.child}' transforms found. Try --list to see what is there.")
    if chain_fail:
        print(f"warning: {chain_fail} messages had an unresolvable parent chain",
              file=sys.stderr)

    # Ground truth arrives at physics rate and is not guaranteed monotonic
    # across a reset; evo requires strictly increasing timestamps.
    rows.sort(key=lambda x: x[0])
    dedup = [rows[0]]
    for row in rows[1:]:
        if row[0] > dedup[-1][0]:
            dedup.append(row)

    out = a.out or "-"
    f = sys.stdout if out == "-" else open(out, "w")
    for row in dedup:
        f.write(" ".join(f"{v:.9f}" for v in row) + "\n")
    if f is not sys.stdout:
        f.close()
        span = dedup[-1][0] - dedup[0][0]
        print(f"wrote {len(dedup)} poses ({span:.1f} s) -> {out}")


if __name__ == "__main__":
    main()
