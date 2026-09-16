#!/usr/bin/env bash
# kalibr_capture.sh -- record the Aprilgrid for Kalibr, on the Pi.
#
#   ./kalibr_capture.sh [seconds] [shutter_us] [gain] [out_prefix]
#   ./kalibr_capture.sh 90 2000 4 ~/cal/cam0_$(date +%Y%m%d)
#   FPS=10 ./kalibr_capture.sh ...        # default 5 fps
#   W=640 H=400 ./kalibr_capture.sh ...   # default 1280x800
#
# Goes through rpicam-raw, per docs/development-log.md:1446 -- the ISP's processed output is
# all zeros on this camera, and sRGB's gamma would sit in front of the corner
# detector even if it worked. Writes <prefix>.y16 (frames concatenated, in the
# R8 mode's 8-bit-in-16 layout) and <prefix>.pts (one timestamp per frame, ms
# with microsecond decimals). tools/kalibr_bag_from_raw.py turns the pair
# into a Kalibr bag on the VM.
#
# Mode: 1280x800, full resolution -- the mode that flies. Intrinsics do not
# transfer between resolutions or binning modes, so calibrate exactly that.
#
# Exposure: short. At full resolution the focal length is ~600 px, so a target
# moving ~0.3 m/s at 0.6 m sweeps ~300 px/s: 2 ms of exposure is ~0.6 px of
# blur, while 15 ms -- what the AGC settles on indoors -- is ~4.5 px and costs
# corner accuracy. Buy the light back with gain and bright, even illumination,
# not with exposure.
#
# Rate: 5 fps. Kalibr samples a calibration bag at ~4 Hz anyway, so capturing
# faster only ships more data. A full-resolution frame is 2 MB in the Y16
# container: 90 s at 5 fps is ~0.9 GB.
#
# Timestamps: from the per-frame metadata (--metadata, JSON), whose
# SensorTimestamp is absolute CLOCK_MONOTONIC in ns -- the clock the IMU is
# pinned to by udev. NOT from --save-pts: that file is RELATIVE, because
# rpicam-apps treats the first frame after start as a restart
# (output/output.cpp, FLAG_RESTART sets time_offset_ to the first timestamp),
# so it begins at 0.000. That was misread from the source once and caught by
# the check below on real hardware. The pts file is still written, as a cross
# check: SensorTimestamp - first must equal every pts value, which proves the
# metadata records pair one-to-one with frames (measured: within 1.000 us
# over 432 frames -- the pts file's whole-microsecond rounding).
#
# Light: SensorBlackLevels is 4096 in the 16-bit container, i.e. 16/255 in
# mono8, so a frame whose mean is 16 is black, not dim. Brightness is reported
# above the black level for that reason.
#
# Getting the files to the VM (it sits behind the Mac's NAT, the Pi cannot
# reach it directly), from the Mac:
#   scp -3 viopi:<prefix>.y16 viopi:<prefix>.pts viopi:<prefix>.meta.json luke@192.168.64.3:cal/
set -euo pipefail

SECS=${1:-90}
SHUTTER=${2:-2000}
GAIN=${3:-4}
OUT=${4:-$HOME/cal/cam0_$(date +%Y%m%d_%H%M%S)}
FPS=${FPS:-5}
W=${W:-1280}; H=${H:-800}

mkdir -p "$(dirname "$OUT")"
mono() { python3 -c 'import time; print(time.monotonic_ns())'; }

echo "capturing ${SECS}s of ${W}x${H} at ${FPS} fps, ${SHUTTER} us, gain ${GAIN} -> ${OUT}.{y16,pts}"
start_ns=$(mono)
rpicam-raw -n --mode "$W:$H:8" --width "$W" --height "$H" \
    -t "$((SECS * 1000))" --framerate "$FPS" \
    --shutter "$SHUTTER" --gain "$GAIN" \
    -o "$OUT.y16" --save-pts "$OUT.pts" \
    --metadata "$OUT.meta.json" --metadata-format json 2> "$OUT.log"
end_ns=$(mono)

# Checked in python rather than awk: the metadata is JSON, and the Pi has
# numpy for a light measurement on a sample of frames.
python3 - "$OUT" "$start_ns" "$end_ns" "$W" "$H" <<'PY'
import json, os, sys
import numpy as np
out, start_ns, end_ns, W, H = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
def fail(m):
    sys.exit(f"FAIL: {m}")
per = W * H * 2
size = os.path.getsize(out + ".y16")
if size == 0 or size % per:
    fail(f"{size} bytes is not a whole number of {W}x{H} 16-bit frames -- wrong mode?")
n = size // per
pts = [float(l) for l in open(out + ".pts") if l.strip() and not l.startswith("#")]
try:
    recs = json.load(open(out + ".meta.json"))
except Exception as e:
    fail(f"metadata is not valid JSON ({e}) -- did the capture end cleanly?")
if not (n == len(pts) == len(recs)):
    fail(f"{n} frames, {len(pts)} pts, {len(recs)} metadata records -- they must pair one to one")
st = [int(r["SensorTimestamp"]) for r in recs]
worst = max(abs((s - st[0]) / 1e6 - p) for s, p in zip(st, pts)) * 1000
# 5 us, not 1: the pts file carries whole microseconds, and a real outdoor
# capture differed from SensorTimestamp by exactly 1.000 us on some of 432
# frames -- rounding, which a 1.0 us limit tripped on through float noise. A
# misaligned record is off by a whole frame period, ~208 ms at 5 fps.
if worst > 5.0:
    fail(f"metadata and frames disagree by up to {worst:.1f} us -- the metadata queue is not aligned with the frames")
lag = (st[0] - start_ns) / 1e6
if not (0 <= lag < 5000):
    fail(f"first SensorTimestamp is {lag:.0f} ms from capture start -- not absolute CLOCK_MONOTONIC")
if st[-1] > end_ns:
    fail("last SensorTimestamp lies after capture ended -- clock mismatch")
exp = sorted({r.get("ExposureTime") for r in recs})
gain = sorted({round(r.get("AnalogueGain", 0), 2) for r in recs})
black = recs[0].get("SensorBlackLevels", [0])[0] / 256.0
lux = float(np.median([r.get("Lux", float("nan")) for r in recs]))
mm = np.memmap(out + ".y16", dtype="<u2", mode="r").reshape(n, H, W)
idx = np.linspace(0, n - 1, min(n, 8)).astype(int)
m8 = (np.asarray(mm[idx]) >> 8).astype(np.float64)
signal = float(np.median(m8)) - black
hi = float((m8 >= 255).mean())
span = (st[-1] - st[0]) / 1e9
fps = (n - 1) / span if n > 1 and span > 0 else float("nan")
print(f"  frames      {n}, {fps:.2f} fps over {span:.1f} s")
print(f"  timestamps  absolute CLOCK_MONOTONIC, first frame {lag:.0f} ms after start; metadata pairs with frames to {worst:.3f} us")
print(f"  exposure    delivered {exp} us, gain {gain}; scene ~{lux:.0f} lux")
print(f"  light       median {signal:.0f}/255 above the black level of {black:.0f}; {100 * hi:.2f} % clipped at 255")
if signal < 25:
    print("  WARNING: under 25/255 above black -- too dark for reliable corners. More light first, then gain; not longer exposure.")
if hi > 0.02:
    print("  WARNING: more than 2 % clipped at 255 -- corners in clipped regions are lost.")
print(f"  next        copy {out}.y16, .pts and .meta.json to the VM, then kalibr_bag_from_raw.py --width {W} --height {H}")
PY
