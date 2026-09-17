#!/usr/bin/env python3
"""
imu_probe.py -- talk to the ISM330DHCX over raw spidev and prove the wiring,
before any device tree overlay exists.

    ./imu_probe.py                    # full check
    ./imu_probe.py --whoami-only      # read-only, touches no registers

Why this runs before the overlay. If you install the overlay first and the
driver does not probe, the kernel tells you "failed" and almost nothing else --
a dead MISO, a swapped clock and data pair, an unpowered part and a wrong SPI
mode all look identical from there. Reading WHO_AM_I over /dev/spidev0.0
separates them, needs no reboot, and settles the spi-cpol/spi-cpha question in
the .dts by experiment rather than by datasheet reading.

Checks, in the order they can fail:

  1. WHO_AM_I == 0x6B, swept across SPI clock and mode. Covers power, GND,
     SCLK, MOSI, MISO and CS -- six of the seven wires. It sweeps rather than
     assuming a speed because on this Pi 5 the RP1 controller only talks to
     this part at 10 MHz: 100 kHz through 9 MHz fail completely, reads and
     writes alike, while a GPIO bit-bang of the same pins works at any speed.
     A single assumed speed therefore reports a perfectly good part as dead.
  2. Gravity. A stationary part reads |a| = 9.81 m/s^2 and omega = 0. Catches a
     part that answers on the bus but is not actually sensing.
  3. INT1 on GPIO25 -- the seventh wire, and the one the driver needs for FIFO
     mode. Data-ready is left latched (not pulsed) and the line is sampled
     either side of a register read, so the test is a deterministic
     high-then-low rather than a race against an edge counter.

Registers are left powered down on exit so the driver gets a clean part.
"""

import argparse
import subprocess
import sys
import time

import spidev

WHO_AM_I        = 0x0F
CTRL1_XL        = 0x10
CTRL2_G         = 0x11
CTRL3_C         = 0x12
INT1_CTRL       = 0x0D
COUNTER_BDR_REG1 = 0x0B
OUT_TEMP_L      = 0x20
OUTX_L_G        = 0x22
OUTX_L_A        = 0x28
STATUS_REG      = 0x1E

EXPECTED_WHOAMI = 0x6B
GPIOCHIP, INT1_LINE = "gpiochip0", 25

# CTRL1_XL / CTRL2_G: ODR in the top nibble, full-scale in bits [3:2].
ODR_12_5, ODR_104 = 0x1, 0x4
FS_XL_4G   = 0b10          # note ST's ordering: 00=2g 01=16g 10=4g 11=8g
FS_G_500   = 0b01          # 00=250 01=500 10=1000 11=2000 dps
SENS_A = 0.122e-3 * 9.80665   # m/s^2 per LSB at +/-4 g
SENS_G = 17.50e-3             # dps per LSB at +/-500 dps


class Bus:
    def __init__(self, mode, hz=1_000_000):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.mode = mode
        self.spi.max_speed_hz = hz

    def read(self, reg, n=1):
        out = self.spi.xfer2([reg | 0x80] + [0x00] * n)
        return out[1:]

    def write(self, reg, val):
        self.spi.xfer2([reg & 0x7F, val & 0xFF])

    def close(self):
        self.spi.close()


def s16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def gpio_read(line=INT1_LINE):
    r = subprocess.run(["gpioget", "-c", GPIOCHIP, str(line)],
                       capture_output=True, text=True)
    if r.returncode != 0:                       # libgpiod v1 took a different shape
        r = subprocess.run(["gpioget", GPIOCHIP, str(line)],
                           capture_output=True, text=True)
    if r.returncode != 0:
        return None, r.stderr.strip()
    t = r.stdout.strip().lower()
    if "inactive" in t: return 0, t
    if "active" in t:   return 1, t
    tail = t.split("=")[-1].strip()
    return (int(tail) if tail in ("0", "1") else None), t


def diagnose(vals):
    """vals: {mode: whoami or None}. Turn the reading into a wiring verdict."""
    seen = [v for v in vals.values() if v is not None]
    if EXPECTED_WHOAMI in seen:
        return None
    if all(v == 0x00 for v in seen):
        return ("Every read returned 0x00. MISO is not getting back to the Pi: "
                "SDO not connected, or landed on the wrong pad. Also possible the "
                "part is unpowered -- check 3V3 on VIN first, it is one probe.")
    if all(v == 0xFF for v in seen):
        return ("Every read returned 0xFF -- the bus is floating high. CS is not "
                "reaching the part (so it never drives MISO), or GND is missing.")
    return (f"Bus responds but WHO_AM_I is {['0x%02X' % v for v in seen]}, not 0x6B. "
            "Either SCLK/MOSI are swapped, the clock is too fast for the wiring, "
            "or this is not an ISM330DHCX.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--whoami-only", action="store_true",
                    help="read-only; write no registers")
    ap.add_argument("--hz", type=int, default=0,
                    help="pin the SPI clock instead of sweeping for one that works")
    args = ap.parse_args()

    speeds = [args.hz] if args.hz else [100_000, 1_000_000, 5_000_000,
                                        8_000_000, 10_000_000, 12_000_000]

    print("=== 1. WHO_AM_I (expect 0x6B), swept over clock and mode ===")
    print(f"  {'speed':>10}  {'mode 0':>8}  {'mode 3':>8}   (n=8 each)")
    results, best = {}, None
    for hz in speeds:
        row = {}
        for mode in (0, 3):
            try:
                bus = Bus(mode, hz)
                vals = [bus.read(WHO_AM_I)[0] for _ in range(8)]
                bus.close()
            except Exception:
                row[mode] = None
                continue
            hits = sum(1 for v in vals if v == EXPECTED_WHOAMI)
            row[mode] = (hits, vals[0])
            results[(hz, mode)] = vals[0]
            # Prefer the combination that is right every single time, and among
            # those the slowest -- headroom against wiring, not against nothing.
            if hits == 8 and best is None:
                best = (hz, mode)

        def cell(r):
            if r is None: return "err"
            h, v = r
            return f"{h}/8" if h else f"0x{v:02X}"
        print(f"  {hz:>10}  {cell(row.get(0)):>8}  {cell(row.get(3)):>8}")

    if best is None:
        problem = diagnose(results)
        print(f"\nFAIL: {problem}")
        print("\nExpected wiring:  VIN->pin1 3V3   GND->pin6   SCL->pin23 SCLK")
        print("                  SDA->pin19 MOSI  SDO->pin21 MISO  CS->pin24 CE0")
        print("                  INT1->pin22 GPIO25")
        return 1

    hz_ok, working_mode = best
    print(f"\n  part identified. Using {hz_ok} Hz, SPI mode {working_mode}.")
    print(f"  ism330dhcx-spi0.dts must carry spi-max-frequency = <{hz_ok}>;")
    print(f"  and {'spi-cpol + spi-cpha (mode 3)' if working_mode == 3 else 'NO spi-cpol/spi-cpha (mode 0)'}.")
    args.hz = hz_ok

    if args.whoami_only:
        return 0

    bus = Bus(working_mode, args.hz)
    try:
        print("\n=== 2. Gravity and rest (expect |a| = 9.81, omega = 0) ===")
        bus.write(CTRL3_C, 0x01)                       # SW_RESET
        time.sleep(0.05)
        bus.write(CTRL3_C, 0x44)                       # BDU | IF_INC
        bus.write(CTRL1_XL, (ODR_104 << 4) | (FS_XL_4G << 2))
        bus.write(CTRL2_G,  (ODR_104 << 4) | (FS_G_500 << 2))
        time.sleep(0.2)

        acc, gyr = [], []
        for _ in range(100):
            d = bus.read(OUTX_L_G, 12)
            gyr.append([s16(d[i], d[i + 1]) * SENS_G for i in (0, 2, 4)])
            acc.append([s16(d[i], d[i + 1]) * SENS_A for i in (6, 8, 10)])
            time.sleep(0.01)

        t = bus.read(OUT_TEMP_L, 2)
        temp_c = s16(t[0], t[1]) / 256.0 + 25.0

        def col(rows, i):
            return [r[i] for r in rows]

        def stats(rows, i):
            c = col(rows, i)
            m = sum(c) / len(c)
            sd = (sum((x - m) ** 2 for x in c) / len(c)) ** 0.5
            return m, sd

        for name, rows, unit in (("accel", acc, "m/s^2"), ("gyro", gyr, "dps")):
            parts = []
            for i, ax in enumerate("xyz"):
                m, sd = stats(rows, i)
                parts.append(f"{ax}={m:+8.4f} (sd {sd:.4f})")
            print(f"  {name:5} {'  '.join(parts)}  [{unit}]")

        mag = sum(sum(col(acc, i)) / len(acc) for i in range(3) for _ in [0])
        gmag = (sum((sum(col(acc, i)) / len(acc)) ** 2 for i in range(3))) ** 0.5
        wmag = (sum((sum(col(gyr, i)) / len(gyr)) ** 2 for i in range(3))) ** 0.5
        print(f"  |a| = {gmag:.4f} m/s^2 (expect 9.81)   "
              f"|omega| = {wmag:.4f} dps (expect ~0)   temp = {temp_c:.1f} C")

        g_ok = abs(gmag - 9.80665) < 0.5
        w_ok = wmag < 5.0
        print(f"  gravity {'OK' if g_ok else 'FAIL -- sensor answers but is not sensing'}"
              f"   rest {'OK' if w_ok else 'FAIL (is it moving?)'}")

        print("\n=== 3. INT1 on GPIO25 ===")
        bus.write(COUNTER_BDR_REG1, 0x00)              # latched data-ready, not pulsed
        bus.write(CTRL1_XL, (ODR_12_5 << 4) | (FS_XL_4G << 2))   # slow, so no race
        bus.write(CTRL2_G, 0x00)
        bus.write(INT1_CTRL, 0x01)                     # INT1_DRDY_XL
        time.sleep(0.3)

        highs, lows = 0, 0
        for _ in range(5):
            time.sleep(0.16)                           # >1 sample period at 12.5 Hz
            before, raw_b = gpio_read()
            bus.read(OUTX_L_A, 6)                      # reading clears data-ready
            after, raw_a = gpio_read()
            if before is None:
                print(f"  gpioget failed: {raw_b}")
                break
            highs += (before == 1)
            lows += (after == 0)
            print(f"  data ready -> INT1 = {before}   after read -> INT1 = {after}")

        int_ok = highs >= 4 and lows >= 4
        if int_ok:
            print("  INT1 OK -- asserts on data-ready and clears on read.")
        else:
            print("  INT1 FAIL. The line never moved as expected.")
            print("  Without it st_lsm6dsx gives polled sysfs reads only and the FIFO")
            print("  is unusable. Check INT1 -> pin 22 (GPIO25).")

        print("\n=== verdict ===")
        allg = g_ok and w_ok and int_ok
        print("  all seven wires good, part sane" if allg
              else "  NOT ready -- see the failures above")
        return 0 if allg else 1
    finally:
        try:
            bus.write(CTRL1_XL, 0x00)
            bus.write(CTRL2_G, 0x00)
            bus.write(INT1_CTRL, 0x00)
        except Exception:
            pass
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
