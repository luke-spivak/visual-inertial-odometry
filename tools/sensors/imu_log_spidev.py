#!/usr/bin/env python3
"""
imu_log_spidev.py -- Allan-variance capture straight off the ISM330DHCX's
hardware FIFO over spidev, with no kernel IIO driver in the picture.

    ./imu_log_spidev.py --minutes 1 --out ~/imu/check     # short sanity run
    ./imu_log_spidev.py --hours 3   --out ~/imu/run1      # the real thing

Writes the same files imu_log.py does -- <out>_accel.bin, <out>_gyro.bin and
<out>.json -- so tools/calibration/allan.py reads either without knowing the difference.

Why this exists. Pi OS ships no st_lsm6dsx: the running kernel's config says
"# CONFIG_IIO_ST_LSM6DSX is not set", and there is no module on disk, so the
overlay creates a correct spi0.0 node with compatible "st,ism330dhcx" and
nothing ever binds to it. Getting the driver means an out-of-tree build.

The Allan run does not need it. Allan variance needs a long, stationary,
uniformly sampled series, and the ISM330DHCX stamps and buffers its own samples
in hardware -- which is the property the part was chosen for. Draining that FIFO
from userspace gives exactly the same samples the kernel driver would have
handed over. The driver matters later, when IMU samples have to share a clock
with camera frames; it does not matter here.

Two wrinkles this works around, both consequences of the overlay:

  * The overlay disables spidev0.0, and nothing binds spi0.0, so /dev/spidev0.0
    is gone. Transfers therefore go out on /dev/spidev0.1, whose own CE1 (GPIO7,
    header pin 26) toggles into thin air -- nothing is wired to it -- while the
    three data lines are shared.

  * Our CS (GPIO8) is still held by the SPI core for the unbound spi0.0, so
    gpiod cannot claim it. pinctrl writes the pad register directly and can.
    Verified: 10/10 correct WHO_AM_I this way, and 0/5 with the CS assert
    removed, so it is the assert doing the work and not luck.

Neither needs root: this user is in the gpio and spi groups.
"""

import argparse
import json
import os
import signal
import struct
import subprocess
import sys
import time

import spidev

# --- registers -------------------------------------------------------------
CTRL1_XL, CTRL2_G, CTRL3_C = 0x10, 0x11, 0x12
FIFO_CTRL3, FIFO_CTRL4     = 0x09, 0x0A
FIFO_STATUS1               = 0x3A
FIFO_DATA_OUT_TAG          = 0x78
WHO_AM_I, EXPECTED         = 0x0F, 0x6B

ODR_CODE = {12.5: 0x1, 26: 0x2, 52: 0x3, 104: 0x4, 208: 0x5,
            416: 0x6, 833: 0x7, 1666: 0x8, 3332: 0x9, 6664: 0xA}
FS_XL = {2: 0b00, 16: 0b01, 4: 0b10, 8: 0b11}      # ST's ordering, not sorted
FS_G  = {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}
SENS_XL = {2: 0.061e-3, 4: 0.122e-3, 8: 0.244e-3, 16: 0.488e-3}   # g/LSB
SENS_G  = {250: 8.75e-3, 500: 17.50e-3, 1000: 35.0e-3, 2000: 70.0e-3}  # dps/LSB

TAG_GYRO, TAG_ACCEL = 0x01, 0x02
WORD = 7                       # 1 tag byte + 6 data bytes
MAX_WORDS = 500                # spidev's python buffer caps a transfer at 4096 B

_stop = False


def _on_signal(signum, frame):
    global _stop
    _stop = True


class Imu:
    """spidev0.1 for the data lines, pinctrl for our chip select."""

    def __init__(self, hz=10_000_000, cs_gpio=8):
        self.cs_gpio = str(cs_gpio)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 1)
        self.spi.mode = 0
        self.spi.max_speed_hz = hz

    def _cs(self, high):
        subprocess.run(["pinctrl", "set", self.cs_gpio, "op", "dh" if high else "dl"],
                       check=True, capture_output=True)

    def read(self, reg, n=1):
        self._cs(False)
        try:
            return self.spi.xfer2([reg | 0x80] + [0] * n)[1:]
        finally:
            self._cs(True)

    def write(self, reg, val):
        self._cs(False)
        try:
            self.spi.xfer2([reg & 0x7F, val & 0xFF])
        finally:
            self._cs(True)

    def close(self):
        self.spi.close()


def configure(imu, odr, accel_g, gyro_dps):
    who = imu.read(WHO_AM_I)[0]
    if who != EXPECTED:
        sys.exit(f"WHO_AM_I 0x{who:02X}, expected 0x{EXPECTED:02X} -- "
                 f"run tools/sensors/imu_probe.py")

    code = ODR_CODE[odr]
    imu.write(CTRL3_C, 0x44)                                  # BDU | IF_INC
    imu.write(CTRL1_XL, (code << 4) | (FS_XL[accel_g] << 2))
    imu.write(CTRL2_G,  (code << 4) | (FS_G[gyro_dps] << 2))
    imu.write(FIFO_CTRL3, (code << 4) | code)                 # batch both at ODR
    imu.write(FIFO_CTRL4, 0x00)                               # bypass: flush
    time.sleep(0.05)
    imu.write(FIFO_CTRL4, 0x06)                               # continuous
    time.sleep(0.1)


def fifo_status(imu):
    s = imu.read(FIFO_STATUS1, 2)
    return ((s[1] & 0x03) << 8) | s[0], bool(s[1] & 0x40)     # depth, overrun


def drain(imu, depth):
    n = min(depth, MAX_WORDS)
    return imu.read(FIFO_DATA_OUT_TAG, n * WORD) if n else []


def decode(buf, acc_out, gyr_out):
    """Split a tagged FIFO batch into the two sensors, keeping raw LSBs."""
    n_a = n_g = 0
    for i in range(0, len(buf) - WORD + 1, WORD):
        tag = buf[i] >> 3
        if tag == TAG_ACCEL:
            acc_out.append(bytes(buf[i + 1:i + 7])); n_a += 1
        elif tag == TAG_GYRO:
            gyr_out.append(bytes(buf[i + 1:i + 7])); n_g += 1
        # every other tag (timestamp, temperature, config-change) is ignored
    return n_a, n_g


def channels(prefix):
    ch = [{"name": f"{prefix}{a}", "index": i, "endian": "le", "signed": True,
           "bits": 16, "storage": 2, "shift": 0, "offset": i * 2}
          for i, a in enumerate("xyz")]
    ch.append({"name": "in_timestamp", "index": 3, "endian": "le", "signed": True,
               "bits": 64, "storage": 8, "shift": 0, "offset": 8})
    return ch


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="imu_run")
    ap.add_argument("--hours", type=float, default=0.0)
    ap.add_argument("--minutes", type=float, default=0.0)
    ap.add_argument("--odr", type=float, default=416)
    # Narrow ranges on purpose: stationary, nothing can clip, and a wide range
    # costs resolution -- see the note in imu_log.py.
    ap.add_argument("--accel-range", type=int, default=4, choices=sorted(FS_XL))
    ap.add_argument("--gyro-range", type=int, default=500, choices=sorted(FS_G))
    ap.add_argument("--drain-ms", type=int, default=250)
    args = ap.parse_args()

    duration = args.hours * 3600 + args.minutes * 60
    if duration <= 0:
        sys.exit("give --hours or --minutes")

    out = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    imu = Imu()
    configure(imu, args.odr, args.accel_range, args.gyro_range)

    scale_a = SENS_XL[args.accel_range] * 9.80665        # m/s^2 per LSB
    scale_g = SENS_G[args.gyro_range] * 3.141592653589793 / 180.0   # rad/s per LSB
    dt_ns = int(round(1e9 / args.odr))

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    fa = open(f"{out}_accel.bin", "wb", buffering=1 << 20)
    fg = open(f"{out}_gyro.bin", "wb", buffering=1 << 20)

    n_a = n_g = 0
    overruns = 0
    t0 = time.monotonic()
    last_drain = t0
    last_report = t0
    anchor = t0                   # host time of the last sample already written
    max_depth = 0

    print(f"capturing {duration/3600:.2f} h at {args.odr} Hz, "
          f"+/-{args.accel_range} g, +/-{args.gyro_range} dps -> {out}_*.bin",
          flush=True)

    try:
        while not _stop and (time.monotonic() - t0) < duration:
            time.sleep(args.drain_ms / 1000.0)
            now = time.monotonic()

            depth, ovr = fifo_status(imu)
            max_depth = max(max_depth, depth)
            if ovr:
                # Samples were lost. Drop this batch and move the anchor to now,
                # which leaves a real hole in the timestamps: allan.py then
                # splits the record here instead of quietly treating a dropout
                # as something the sensor did.
                overruns += 1
                imu.write(FIFO_CTRL4, 0x00)
                imu.write(FIFO_CTRL4, 0x06)
                anchor = last_drain = now
                continue

            acc, gyr = [], []
            while depth > 0:
                buf = drain(imu, depth)
                if not buf:
                    break
                decode(buf, acc, gyr)
                depth -= len(buf) // WORD
                if len(buf) // WORD < MAX_WORDS:
                    break

            # Timestamp against the host clock rather than the nominal ODR.
            # The FIFO guarantees the samples inside a batch are evenly spaced,
            # but it does not guarantee they arrive at the rate written on the
            # tin: this part delivers ~440 Hz when asked for 416, and assuming
            # 416 would stretch every tau by 5.8% and inflate the noise density
            # that comes out of it. Interpolating between drain boundaries makes
            # the true rate fall out of the data instead of being asserted.
            if acc or gyr:
                span = now - anchor
                for j, raw in enumerate(acc):
                    ts = anchor - t0 + span * (j + 1) / len(acc)
                    fa.write(raw + struct.pack("<xxq", int(ts * 1e9)))
                for j, raw in enumerate(gyr):
                    ts = anchor - t0 + span * (j + 1) / len(gyr)
                    fg.write(raw + struct.pack("<xxq", int(ts * 1e9)))
                n_a += len(acc)
                n_g += len(gyr)
                anchor = now
            last_drain = now

            if now - last_report >= 60:
                el = now - t0
                print(f"[{el/60:7.1f} min {100*el/duration:5.1f}%] "
                      f"accel={n_a} gyro={n_g} rate={n_a/el:.1f} Hz "
                      f"peak_fifo={max_depth} overruns={overruns}", flush=True)
                last_report = now
    finally:
        try:
            imu.write(FIFO_CTRL4, 0x00)
            imu.write(CTRL1_XL, 0x00)
            imu.write(CTRL2_G, 0x00)
        except Exception:
            pass
        imu.close()
        fa.close()
        fg.close()

    elapsed = time.monotonic() - t0
    meta = {}
    for kind, n, scale, prefix in (("accel", n_a, scale_a, "in_accel_"),
                                   ("gyro", n_g, scale_g, "in_anglvel_")):
        meta[kind] = {
            "sysfs": "spidev0.1 + manual CS on GPIO8",
            "name": f"ism330dhcx_{kind}",
            "chardev": "/dev/spidev0.1",
            "odr_hz": args.odr,
            "scale": scale,
            "timestamp_clock": "sensor FIFO order, uniform at ODR, gaps injected at overruns",
            "record_bytes": 16,
            "channels": channels(prefix),
            "samples": n,
            "file": f"{out}_{kind}.bin",
        }
    with open(f"{out}.json", "w") as f:
        json.dump({"created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                   "elapsed_s": elapsed, "argv": sys.argv,
                   "overruns": overruns, "peak_fifo_depth": max_depth,
                   "nominal_odr_hz": args.odr,
                   "measured_odr_hz": (n_a / elapsed) if elapsed else None,
                   "accel_range_g": args.accel_range,
                   "gyro_range_dps": args.gyro_range,
                   "devices": meta}, f, indent=2)

    print(f"\n{elapsed/60:.1f} min: accel {n_a} ({n_a/elapsed:.2f} Hz), "
          f"gyro {n_g} ({n_g/elapsed:.2f} Hz), overruns {overruns}, "
          f"peak FIFO {max_depth}")
    print(f"sidecar -> {out}.json")


if __name__ == "__main__":
    main()
