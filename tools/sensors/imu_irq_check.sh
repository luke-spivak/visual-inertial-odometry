#!/usr/bin/env bash
# imu_irq_check.sh -- does the IMU stream survive at FIFO watermark 64 and 8?
#
#   ssh -t viopi 'sudo bash ~/tools/sensors/imu_irq_check.sh'
#
# vio_live's bench runs (2026-09-10) lost the IMU at watermark 8: run 1 after
# 15 s, run 2 after a single interrupt that delivered nothing. INT1 is wired
# rising-edge; if the handler ever leaves the FIFO above the watermark, INT1
# stays high, no new edge arrives, and the stream is dead until reset. Every
# earlier driver run that worked used watermark 64.
#
# 15 s of the proven logger at each watermark, nothing else running. Reports
# samples, interrupts, and INT1's level sampled mid-run: "hi" with the sample
# count stalled is the stuck-edge signature.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
irq() { awk '/lsm6dsx/ {s=0; for (i=2; i<=NF; i++) { if ($i ~ /^[0-9]+$/) s+=$i; else break }; print s}' /proc/interrupts; }

# One-shot reads first. After bench run 1 these returned ~20 g at rest with
# the two bytes of every word equal -- a bad SPI link or register state, which
# no watermark will fix.
python3 - "$HERE/../../src" <<'PY'
import sys; sys.path.insert(0, sys.argv[1]); import imu_log
d = imu_log.find_devices()["accel"]; s = float(imu_log.rd(d + "/in_accel_scale"))
v = [int(imu_log.rd(f"{d}/in_accel_{ax}_raw")) for ax in "xyz"]
n = sum((x * s) ** 2 for x in v) ** 0.5
bad = all(((x & 0xFFFF) >> 8) == (x & 0xFF) for x in v)
print(f"one-shot accel: raw {v} -> |a| {n:.2f} m/s^2 (9.6-9.8 at rest)"
      + ("  <-- every word has equal bytes: SPI link or register state is bad" if bad else ""))
PY

for wm in 64 8; do
    i0=$(irq)
    # pinctrl prints "25: ip    pd | lo // GPIO25 = input": the level is the
    # word after the bar, not the last word.
    ( sleep 10; pinctrl get 25 | sed -E 's/.*\| *(hi|lo).*/\1/' > /tmp/irqchk_$wm.int1 ) &
    timeout 60 python3 "$HERE/../../src/imu_log.py" --minutes 0.25 --watermark "$wm" --odr 416 \
        --accel-range 16 --gyro-range 2000 --out /tmp/irqchk_$wm > /tmp/irqchk_$wm.log 2>&1
    wait
    i1=$(irq)
    counts=$(grep -E "samples \(" /tmp/irqchk_$wm.log | tail -1)
    echo "watermark $wm: ${counts:-no count line, see /tmp/irqchk_$wm.log} | interrupts $((i1 - i0)) | INT1 at 10 s: $(cat /tmp/irqchk_$wm.int1)"
done
echo "(15 s at 440 Hz is ~6600 samples per sensor)"
