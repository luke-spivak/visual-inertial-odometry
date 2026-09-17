#!/usr/bin/env python3
"""
Report whether the simulated camera is producing a TRACKABLE image.

Brightness is the wrong question and answering it wasted three full missions.
A frame measured min=1 max=255 mean=14 -- max is saturated, so by any
brightness test the renderer is fine. It was: 954 pixels of blown highlight out
of 256000, with 97.6% of the frame sitting on two adjacent grey values. Zero
odometry messages came out.

What a KLT tracker needs is a local gradient, so that is what is measured:
the fraction of 16x16 blocks whose standard deviation is below `--flat-thresh`.
Those blocks can produce no corners, and a frame made mostly of them is
untrackable however bright it is.

    healthy   flat_blocks ~10-15%   (textured ground + 3D structure)
    useless   flat_blocks >50%      (a surface minifying to a constant)

Run while Gazebo and ros_gz_bridge are up:
    python3 check_camera.py [--topic /cam0/image_raw] [--n 3] [--save f.pgm]
"""
import argparse
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Stat(Node):
    def __init__(self, topic, want, save=None, flat_thresh=1.0, max_flat=0.40,
                 every=0.0, save_dir=None):
        super().__init__("check_camera")
        self.create_subscription(Image, topic, self.cb, 10)
        self.n = 0
        self.want = want
        self.save = save
        self.flat_thresh = flat_thresh
        self.max_flat = max_flat
        self.every = every
        self.last = None
        self.save_dir = save_dir

    def flat_fraction(self, d, w, h, step=16):
        """Fraction of step x step blocks with too little contrast to yield a
        corner. Rows are strided rather than copied -- this runs inside the
        harness gate and the frame is 256 kB."""
        flat = total = 0
        for by in range(0, h - step, step):
            for bx in range(0, w - step, step):
                v = [d[(by + y) * w + bx + x] for y in range(step) for x in range(step)]
                mean = sum(v) / len(v)
                var = sum((p - mean) ** 2 for p in v) / len(v)
                total += 1
                flat += var < self.flat_thresh ** 2
        return flat / total if total else 1.0

    def cb(self, m):
        # In --every mode most frames are skipped without being decoded: the
        # block statistics cost more than the sim can spare while it is flying.
        now = time.monotonic()
        if self.every and self.last is not None and now - self.last < self.every:
            return
        self.last = now

        d = bytes(m.data)
        lo, hi, uniq = min(d), max(d), len(set(d))
        if m.encoding == "mono8":
            frac = self.flat_fraction(d, m.width, m.height)
            verdict = "UNTRACKABLE" if frac > self.max_flat else "ok"
            extra = f"flat_blocks={100*frac:.0f}%"
        else:
            # Only mono8 is laid out one byte per pixel; anything else would
            # need unpacking before the block statistics mean anything.
            frac, extra = None, f"(no contrast check for {m.encoding})"
            verdict = "LOOKS BLACK" if (hi < 40 or uniq < 20) else "ok"
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        print(f"t={stamp:.2f} {m.width}x{m.height} enc={m.encoding} min={lo} max={hi} "
              f"mean={sum(d)/len(d):.1f} distinct={uniq} {extra}  -> {verdict}",
              flush=True)
        if self.save_dir:
            path = f"{self.save_dir}/f{self.n:04d}.pgm"
            if m.encoding == "mono8":
                with open(path, "wb") as f:
                    f.write(b"P5\n%d %d\n255\n" % (m.width, m.height))
                    f.write(d)
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
    ap.add_argument("--wait", type=float, default=90.0,
                    help="seconds to wait for the first frame")
    ap.add_argument("--flat-thresh", type=float, default=1.0,
                    help="block stddev below which a block yields no corners")
    ap.add_argument("--max-flat", type=float, default=0.40,
                    help="fail above this fraction of flat blocks")
    ap.add_argument("--every", type=float, default=0.0, metavar="SEC",
                    help="sample at most one frame every SEC (0 = every frame), "
                         "for tracing trackability across a whole flight")
    ap.add_argument("--save-dir", metavar="DIR",
                    help="write every sampled frame as DIR/fNNNN.pgm, so the "
                         "imagery the estimator actually saw can be measured "
                         "afterwards with count_features.py")
    ap.add_argument("--save", metavar="PATH",
                    help="write the first frame as a PGM you can actually look at")
    a = ap.parse_args()

    rclpy.init()
    if a.save_dir:
        os.makedirs(a.save_dir, exist_ok=True)
    node = Stat(a.topic, a.n, a.save, a.flat_thresh, a.max_flat, a.every, a.save_dir)
    # Ogre2 on llvmpipe compiles its shaders on the CPU, so the FIRST frame can
    # be a minute behind the rest. A short window made this report NO IMAGES on
    # a run whose camera was working perfectly by the time the mission started.
    deadline = time.monotonic() + a.wait
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.n >= a.n:
            break
    if node.n == 0:
        print(f"NO IMAGES on {a.topic} after {a.wait:.0f} s "
              f"-- is Gazebo running and the bridge up?")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
