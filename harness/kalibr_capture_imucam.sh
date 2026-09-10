#!/usr/bin/env bash
# kalibr_capture_imucam.sh -- record camera and IMU together for Kalibr's
# camera-IMU calibration (step 6). Runs on the Pi, with a terminal (sudo):
#
#   ssh -t viopi 'bash ~/harness/kalibr_capture_imucam.sh [seconds] [out_prefix]'
#
# Also records the step 7 VIO walks (camera at flight rate):
#   ssh -t viopi 'FPS=20 bash ~/harness/kalibr_capture_imucam.sh 120 ~/vio/walk1'
# FPS is passed through to kalibr_capture.sh; MAX_SHUTTER (us, default 1000) caps exposure.
#
# 1. Asks for the sudo password up front -- the IIO device is root-only -- so
#    nothing stops to prompt once you are holding the rig.
# 2. Lets auto-exposure look at the room for 2.5 s, then fixes exposure with
#    the shutter capped at 1 ms: the rig will be rotating, and at ~1116 px
#    focal length 100 deg/s is ~2 px of blur per ms. Refuses outright if the
#    room needs more than gain 16 -- the last dark capture was ~9/255 above
#    black and Kalibr found no corners in it at all.
# 3. Starts the IMU logger, waits 2 s so it is streaming first, records the
#    camera, then lets the IMU run 4 s past the end.
#
# Both sensors stamp on CLOCK_MONOTONIC: the camera's SensorTimestamp natively,
# the IMU through the udev rule and imu_log.py. No offset is applied anywhere
# here; Kalibr estimates the residual.
set -euo pipefail
SECS=${1:-60}
OUT=${2:-$HOME/cal/imucam_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$(dirname "$OUT")"

sudo -v || { echo "FAIL: sudo needed for the IMU"; exit 1; }

echo "=== exposure: 2.5 s of auto-exposure, point the camera at the target ==="
read -r SH G < <(python3 - <<'PY'
import json, os, subprocess, sys
import numpy as np
subprocess.run(["rpicam-raw", "-n", "--mode", "1280:800:8", "--width", "1280", "--height", "800",
                "-t", "2500", "--framerate", "5", "-o", "/tmp/ae.y16",
                "--metadata", "/tmp/ae.json", "--metadata-format", "json"], capture_output=True)
last = json.load(open("/tmp/ae.json"))[-1]
os.remove("/tmp/ae.y16"); os.remove("/tmp/ae.json")
P = last["ExposureTime"] * last["AnalogueGain"]
sh = int(min(P, int(os.environ.get('MAX_SHUTTER', 1000)))); g = max(1.0, P / sh)
print(f"  auto-exposure: {last['ExposureTime']} us x gain {last['AnalogueGain']:.2f} at ~{last.get('Lux', 0):.0f} lux"
      f" -> fixed {sh} us, gain {g:.2f}", file=sys.stderr)
if g > 16:
    print(f"  FAIL: needs gain {g:.0f} at {sh} us -- too dark. Add light and rerun.", file=sys.stderr)
    sys.exit(1)
print(f"{sh} {g:.2f}")
PY
) || exit 1

IMU_MIN=$(python3 -c "print(($SECS + 6) / 60)")
echo "=== IMU logging starts; camera in 2 s -- start moving the rig ==="
sudo python3 "$HOME/harness/imu_log.py" --minutes "$IMU_MIN" --odr 416 \
     --accel-range 16 --gyro-range 2000 --out "$OUT.imu" > "$OUT.imu.log" 2>&1 &
IMU_PID=$!
sleep 2
bash "$HOME/harness/kalibr_capture.sh" "$SECS" "$SH" "$G" "$OUT" || { sudo kill "$IMU_PID" 2>/dev/null; exit 1; }
echo "=== camera done -- keep still, IMU finishing ==="
wait "$IMU_PID" || { echo "FAIL: IMU logger exited non-zero:"; tail -5 "$OUT.imu.log"; exit 1; }
tail -2 "$OUT.imu.log" | sed 's/^/  imu: /'
echo "=== done: $OUT.{y16,pts,meta.json} + $OUT.imu.{json,_accel.bin,_gyro.bin} ==="
