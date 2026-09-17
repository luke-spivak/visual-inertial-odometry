#!/usr/bin/env python3
"""
Measure how much PARALLAX a recorded scene actually offers.

The standing hypothesis for this rig's drift is the monocular planar
degeneracy: at 10 m the obstacle field subtends so little of the view that the
scene is effectively a plane, and a plane does not constrain structure and
motion independently. That hypothesis has been argued from scene geometry and
never measured from the imagery, which is backwards -- the estimator sees
pixels, not SDF files.

This measures it directly, and the trick is what to compare against. Between
two views of a PLANE, every pixel's motion is explained exactly by a homography,
whatever the camera did. So:

    fit the best homography to the tracked flow, then look at what is left over

The residual is, by construction, the part of the image motion that no plane can
account for -- which is parallax, and parallax is the only thing that makes
depth observable. A scene that is truly a plane gives residuals at the noise
floor no matter how fast you fly over it. A scene with real 3D structure gives a
heavy tail: features on tall objects move differently from the ground behind
them.

Reported per frame pair:

    flow_px       how far features moved (the raw motion; NOT the useful number)
    resid_p50     median homography residual -- the noise floor
    resid_p95     the tail. THIS is the parallax the estimator has to work with
    frac_gt_1px   fraction of features a plane mispredicts by more than a pixel

Comparing two altitudes or two scenes on resid_p95 is a fair comparison in a way
that comparing drift is not, because it needs no estimator and no ground truth.

    parallax_check.py --bag RUN/bag --t0 20 --t1 40
"""
import argparse
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("needs opencv (python3-opencv)")

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def to_gray(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "mono8":
        return buf.reshape(msg.height, msg.width)
    if msg.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.width, 3)
        code = cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(img, code)
    raise SystemExit(f"unhandled encoding {msg.encoding}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/cam0/image_raw")
    ap.add_argument("--t0", type=float, default=0.0,
                    help="seconds after the first image to start")
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--pairs", type=int, default=40,
                    help="how many frame pairs to measure")
    ap.add_argument("--gap", type=int, default=2,
                    help="frames between the two views of a pair. 1 is the "
                         "camera period; a slightly larger gap gives the "
                         "baseline that makes parallax visible at all")
    ap.add_argument("--min-flow", type=float, default=2.0,
                    help="skip pairs with less motion than this (a hovering "
                         "vehicle has no parallax to measure, and saying so is "
                         "not the same as the scene being flat)")
    a = ap.parse_args()

    r = SequentialReader()
    r.open(StorageOptions(uri=a.bag, storage_id="sqlite3"),
           ConverterOptions("", ""))
    frames, t_first = [], None
    while r.has_next():
        topic, data, stamp = r.read_next()
        if topic != a.topic:
            continue
        t = stamp / 1e9
        if t_first is None:
            t_first = t
        rel = t - t_first
        if rel < a.t0:
            continue
        if rel > a.t1:
            break
        frames.append((rel, to_gray(deserialize_message(data, Image))))
    if len(frames) < a.gap + 2:
        sys.exit(f"only {len(frames)} frames in [{a.t0}, {a.t1}] s")

    step = max(1, (len(frames) - a.gap) // a.pairs)
    rows = []
    for i in range(0, len(frames) - a.gap, step):
        t, img0 = frames[i]
        _, img1 = frames[i + a.gap]
        p0 = cv2.goodFeaturesToTrack(img0, maxCorners=400, qualityLevel=0.01,
                                     minDistance=8)
        if p0 is None or len(p0) < 30:
            continue
        p1, st, _ = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None,
                                             winSize=(21, 21), maxLevel=4)
        if p1 is None:
            continue
        ok = st.ravel() == 1
        a0, a1 = p0[ok].reshape(-1, 2), p1[ok].reshape(-1, 2)
        if len(a0) < 30:
            continue
        flow = np.linalg.norm(a1 - a0, axis=1)
        if np.median(flow) < a.min_flow:
            continue

        # RANSAC so a handful of tracking failures do not become "parallax".
        H, inl = cv2.findHomography(a0, a1, cv2.RANSAC, 3.0)
        if H is None:
            continue
        proj = cv2.perspectiveTransform(a0.reshape(-1, 1, 2), H).reshape(-1, 2)
        resid = np.linalg.norm(a1 - proj, axis=1)
        rows.append((t, len(a0), float(np.median(flow)),
                     float(np.percentile(resid, 50)),
                     float(np.percentile(resid, 95)),
                     float(np.mean(resid > 1.0)),
                     float(inl.mean()) if inl is not None else float("nan")))

    if not rows:
        sys.exit("no pair had enough motion or features -- widen --t0/--t1")

    arr = np.array([r[1:] for r in rows], dtype=float)
    print(f"{len(rows)} frame pairs, gap {a.gap} frames, "
          f"t = {rows[0][0]:.1f}..{rows[-1][0]:.1f} s\n")
    hdr = f"{'':14}{'median':>10}{'p10':>10}{'p90':>10}"
    print(hdr)
    print("-" * len(hdr))
    for name, col in (("features", 0), ("flow_px", 1), ("resid_p50", 2),
                      ("resid_p95", 3), ("frac_gt_1px", 4), ("H inliers", 5)):
        c = arr[:, col]
        print(f"{name:<14}{np.median(c):10.2f}{np.percentile(c, 10):10.2f}"
              f"{np.percentile(c, 90):10.2f}")

    p95 = float(np.median(arr[:, 3]))
    frac = float(np.median(arr[:, 4]))
    print(f"\n  resid_p95 = {p95:.2f} px, {100 * frac:.1f} % of features move "
          f"more than 1 px\n  away from where a plane says they should.")
    if p95 < 0.5:
        print("  -> The scene is effectively PLANAR at this altitude. Depth is")
        print("     poorly conditioned and the monocular degeneracy is real here.")
    elif p95 < 1.5:
        print("  -> Marginal parallax. There is some structure, but not much for")
        print("     the filter to separate depth from motion with.")
    else:
        print("  -> Genuine 3D structure in frame. If drift is still bad, the")
        print("     planar degeneracy is NOT the explanation and this rules it out.")


if __name__ == "__main__":
    main()
