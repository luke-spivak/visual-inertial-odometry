#!/usr/bin/env python3
"""
Report pixel statistics for the simulated camera.

Distinguishes "the world has no features" from "the renderer produced a black
image" -- which look identical from OpenVINS, where both give zero tracks.

Healthy render:  max near 255, distinct_values in the hundreds.
Black render:    max in single/low double digits, distinct_values < 20.

Run while Gazebo and ros_gz_bridge are up:
    python3 check_camera.py [--topic /cam0/image_raw] [--n 3]
"""
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Stat(Node):
    def __init__(self, topic, want, save=None):
        super().__init__("check_camera")
        self.create_subscription(Image, topic, self.cb, 10)
        self.n = 0
        self.want = want
        self.save = save

    def cb(self, m):
        d = bytes(m.data)
        lo, hi, uniq = min(d), max(d), len(set(d))
        verdict = "LOOKS BLACK" if (hi < 40 or uniq < 20) else "ok"
        print(f"{m.width}x{m.height} enc={m.encoding} min={lo} max={hi} "
              f"mean={sum(d)/len(d):.1f} distinct={uniq}  -> {verdict}", flush=True)
        if self.save and self.n == 0:
            # PGM (P5) is a 3-line header plus raw bytes -- no dependencies,
            # and eog/GIMP open it directly.
            if m.encoding == "mono8":
                with open(self.save, "wb") as f:
                    f.write(b"P5\n%d %d\n255\n" % (m.width, m.height))
                    f.write(d)
                print(f"  wrote {self.save}", flush=True)
            else:
                print(f"  (not saving: encoding {m.encoding} is not mono8)", flush=True)
        self.n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/cam0/image_raw")
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--save", metavar="PATH",
                    help="write the first frame as a PGM you can actually look at")
    a = ap.parse_args()

    rclpy.init()
    node = Stat(a.topic, a.n, a.save)
    for _ in range(600):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.n >= a.n:
            break
    if node.n == 0:
        print(f"NO IMAGES on {a.topic} -- is Gazebo running and the bridge up?")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
