#!/usr/bin/env python3
"""
Count the FAST corners OpenVINS would actually extract from a frame.

This exists because the earlier gate measured the wrong quantity. Block
standard deviation is GRADIENT, and a scene can be full of gradient while
being nearly empty of corners: smooth multi-octave noise shades continuously
in every direction, so every 16x16 block looks textured and FAST finds almost
nothing. A flight whose frames scored flat_blocks=0-3% still gave OpenVINS
"not enough feats to compute disp: 0,0 < 15" and never initialised.

FAST asks a different question -- is the centre pixel brighter or darker than
a contiguous arc of the 16 pixels around it, by more than `threshold` -- and
that is the question the front end asks, so it is the one to measure.

Mirrors ov_msckf's KLT front end: histogram equalisation, then extraction over
a grid_x by grid_y grid, `num_pts` total budget.

    python3 count_features.py frame.pgm
    python3 count_features.py frame.pgm --thresholds 5,10,15,20,25
"""
import argparse
import sys

import cv2
import numpy as np


def read_pgm(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"could not read {path}")
    return img


def grid_counts(img, thresh, gx, gy, num_pts):
    """FAST per grid cell, as ov_core::TrackKLT does. The grid matters: 200
    corners all in one bush is not the same as 200 spread over the frame, and
    only the spread version constrains the filter well."""
    h, w = img.shape
    fast = cv2.FastFeatureDetector_create(threshold=thresh, nonmaxSuppression=True)
    per_cell = max(1, num_pts // (gx * gy))
    total, occupied = 0, 0
    for iy in range(gy):
        for ix in range(gx):
            y0, y1 = iy * h // gy, (iy + 1) * h // gy
            x0, x1 = ix * w // gx, (ix + 1) * w // gx
            kps = fast.detect(img[y0:y1, x0:x1], None)
            n = min(len(kps), per_cell)
            total += n
            occupied += n > 0
    return total, occupied, gx * gy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames", nargs="+")
    ap.add_argument("--thresholds", default="5,10,15,20,30")
    ap.add_argument("--grid", default="5,5")
    ap.add_argument("--num-pts", type=int, default=200)
    ap.add_argument("--no-equalize", action="store_true",
                    help="skip the histogram equalisation ov_msckf applies")
    a = ap.parse_args()

    gx, gy = (int(v) for v in a.grid.split(","))
    ths = [int(v) for v in a.thresholds.split(",")]

    for path in a.frames:
        img = read_pgm(path)
        if not a.no_equalize:
            img = cv2.equalizeHist(img)
        print(f"{path}  {img.shape[1]}x{img.shape[0]}  "
              f"stddev={np.std(img):.1f}")
        for t in ths:
            n, occ, cells = grid_counts(img, t, gx, gy, a.num_pts)
            # OpenVINS needs >15 tracked features just to attempt initialisation,
            # and wants num_pts for a healthy update.
            verdict = ("unusable" if n < 15 else
                       "thin" if n < a.num_pts // 3 else "ok")
            print(f"   fast_threshold={t:3d}  corners={n:4d}  "
                  f"cells_with_features={occ}/{cells}  -> {verdict}")


if __name__ == "__main__":
    main()
