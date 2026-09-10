#!/usr/bin/env python3
"""
kalibr_bag_from_raw.py -- turn a kalibr_capture.sh recording into a Kalibr bag.

Runs inside the Kalibr image, on the VM:

    docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp -e ROS_HOME=/tmp \\
        -v ~/cal:/data kalibr:noetic \\
        python3 /data/kalibr_bag_from_raw.py /data/cam0_20260910

Reads <prefix>.y16, <prefix>.pts and <prefix>.meta.json, writes <prefix>_frames/cam0/<ns>.png, builds
<prefix>.bag with kalibr_bagcreater, then reads the bag back and checks it --
message count, first and last stamps, and a pixel-exact round trip -- because
an exit code is not evidence. The PNGs are removed afterwards unless
--keep-png.

Refuses rather than guesses, and every refusal has an earlier failure behind
it in this repo:
  * a byte count that is not whole WxH 16-bit frames (default 1280x800, full
    resolution, the mode that flies) means the wrong mode
  * values not all multiples of 256 means this is not the R8 mode's
    8-bit-in-16 layout; mono8 is raw >> 8, and any other reading gives a
    plausible wrong image
  * frame count must equal timestamp count, and timestamps must increase

Also reports clipping. Pixels at 255 or 0 carry no gradient, so a clipped
region of the target contributes no corners, however sharp it looks.
"""
import argparse
import os
import shutil
import subprocess
import sys

import cv2
import numpy as np


def fail(msg):
    sys.exit(f"FAIL: {msg}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prefix", help="path without extension: <prefix>.y16 and <prefix>.pts")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--cam", default="cam0")
    ap.add_argument("--keep-png", action="store_true")
    ap.add_argument("--boost", type=float, default=1.0,
                    help="linear brightness gain applied after subtracting the black level "
                         "(default 1 = off). For captures too dark for Kalibr's detector: a "
                         "linear stretch moves no edge, so corner positions are preserved")
    a = ap.parse_args()

    y16, pts = a.prefix + ".y16", a.prefix + ".pts"
    for f in (y16, pts):
        if not os.path.isfile(f):
            fail(f"{f} not found")

    per = a.width * a.height
    raw = np.fromfile(y16, dtype="<u2")
    if raw.size == 0 or raw.size % per:
        fail(f"{y16}: {raw.size * 2} bytes is not a whole number of "
             f"{a.width}x{a.height} 16-bit frames -- wrong mode?")
    frames = raw.reshape(-1, a.height, a.width)
    if (frames % 256).any():
        fail("values are not all multiples of 256, so this is not the R8 mode's "
             "8-bit-in-16 layout -- refusing to guess a conversion")
    mono = (frames >> 8).astype(np.uint8)

    # Timestamps come from the per-frame metadata's SensorTimestamp: absolute
    # CLOCK_MONOTONIC, the IMU's clock. rpicam-apps' --save-pts file is
    # RELATIVE -- the first frame counts as a restart, which zeroes it -- so it
    # is used only to prove the metadata records pair one-to-one with frames.
    # Relative stamps would also break kalibr_bagcreater outright: it reads all
    # but the last 9 digits of a filename as seconds, and a stamp under 1 s has
    # none.
    import json
    meta = a.prefix + ".meta.json"
    if not os.path.isfile(meta):
        fail(f"{meta} not found -- capture with kalibr_capture.sh, which records it")
    recs = json.load(open(meta))
    rel_ms = []
    with open(pts) as fh:
        for line in fh:
            line = line.strip()
            if line and not line.startswith("#"):
                rel_ms.append(float(line))
    if not (len(mono) == len(rel_ms) == len(recs)):
        fail(f"{len(mono)} frames, {len(rel_ms)} pts, {len(recs)} metadata records "
             f"-- they must pair one to one")
    ts = np.array([int(r["SensorTimestamp"]) for r in recs], dtype=np.int64)
    worst_us = max(abs((t - ts[0]) / 1e6 - p) for t, p in zip(ts, rel_ms)) * 1000
    # 5 us: pts carries whole microseconds (a real capture differed by exactly
    # 1.000 us through rounding); a misaligned record is off by a whole frame.
    if worst_us > 5.0:
        fail(f"metadata and frames disagree by up to {worst_us:.1f} us "
             f"-- the metadata is not aligned with the frames")
    d = np.diff(ts)
    if len(d) and (d <= 0).any():
        fail(f"{int((d <= 0).sum())} non-increasing timestamps")
    black = recs[0].get("SensorBlackLevels", [0])[0] / 256.0
    if a.boost != 1.0:
        mono = np.clip((mono.astype(np.float32) - black) * a.boost, 0, 255).astype(np.uint8)
        print(f"  boost       (raw - {black:.0f}) x {a.boost:g}, clipped to 0..255")
        black = 0.0

    period = float(np.median(d)) if len(d) else float("nan")
    gaps = int((d > 1.5 * period).sum()) if len(d) else 0
    hi = (mono == 255).mean(axis=(1, 2))
    lo = (mono == 0).mean(axis=(1, 2))
    print(f"  frames      {len(mono)}, median period {period / 1e6:.2f} ms "
          f"({1e9 / period:.2f} fps), {gaps} gap(s) over 1.5x period")
    print(f"  span        {(ts[-1] - ts[0]) / 1e9:.2f} s")
    print(f"  brightness  mean {mono.mean():.1f} / 255, black level {black:.0f}, so ~{mono.mean() - black:.0f} above black")
    print(f"  clipped     at 255: median {100 * np.median(hi):.2f} %, worst {100 * hi.max():.2f} %;"
          f"  at 0: median {100 * np.median(lo):.2f} %")
    if np.median(hi) > 0.02:
        print("  WARNING: more than 2 % of a typical frame is at 255 -- corners there are lost")

    fdir = a.prefix + "_frames"
    cdir = os.path.join(fdir, a.cam)
    if os.path.isdir(fdir):
        shutil.rmtree(fdir)
    os.makedirs(cdir)
    for t, img in zip(ts, mono):
        if not cv2.imwrite(os.path.join(cdir, f"{t}.png"), img):
            fail(f"could not write PNG for {t}")

    bag = a.prefix + ".bag"
    if os.path.exists(bag):
        os.remove(bag)
    r = subprocess.run(["rosrun", "kalibr", "kalibr_bagcreater",
                        "--folder", fdir, "--output-bag", bag],
                       capture_output=True, text=True)
    if r.returncode != 0:
        fail(f"kalibr_bagcreater exited {r.returncode}:\n{r.stdout[-800:]}{r.stderr[-800:]}")

    import rosbag
    topic = f"/{a.cam}/image_raw"
    with rosbag.Bag(bag) as b:
        msgs = [m for _, m, _ in b.read_messages(topics=[topic])]
    if len(msgs) != len(mono):
        fail(f"bag holds {len(msgs)} messages on {topic}, expected {len(mono)}")
    stamps = [m.header.stamp.to_nsec() for m in msgs]
    if stamps[0] != ts[0] or stamps[-1] != ts[-1]:
        fail(f"bag stamps {stamps[0]}..{stamps[-1]} differ from timestamps {ts[0]}..{ts[-1]}")
    m0 = msgs[0]
    back = np.frombuffer(m0.data, np.uint8).reshape(m0.height, m0.step)[:, :m0.width]
    if not np.array_equal(back, mono[0]):
        fail("first frame's pixels did not survive the round trip into the bag")
    print(f"  bag         {bag}: {len(msgs)} messages on {topic} ({m0.encoding}), "
          f"stamps and pixels round-trip exactly")

    if not a.keep_png:
        shutil.rmtree(fdir)


if __name__ == "__main__":
    main()
