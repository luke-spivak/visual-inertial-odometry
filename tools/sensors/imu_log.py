#!/usr/bin/env python3
"""Configure Linux IIO sensors and record raw IMU samples for calibration.

Records include sensor timestamps and a sidecar describing their binary layout.
"""

import argparse
import glob
import json
import os
import select
import signal
import struct
import sys
from pathlib import Path
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from imu_device import SYSFS, find_devices, rd, setup, teardown

_stop = False


def _on_signal(signum, frame):
    global _stop
    _stop = True


def last_ts_in(data, m):
    """Timestamp of the final complete record in a freshly read block."""
    ts = next(c for c in m["channels"] if "timestamp" in c["name"])
    rec, off = m["record_bytes"], ts["offset"]
    n = len(data) // rec
    if n == 0:
        return None
    fmt = ("<" if ts["endian"] == "le" else ">") + "q"
    return struct.unpack_from(fmt, data, (n - 1) * rec + off)[0]


def capture(meta, out_prefix, duration_s, sample_target=None, pairs=None):
    """Record raw samples; optionally pair their timestamps with host arrival times."""
    files, fds, counts = {}, {}, {}
    poller = select.poll()
    for kind, m in meta.items():
        fd = os.open(m["chardev"], os.O_RDONLY)
        fds[fd] = kind
        files[kind] = open(f"{out_prefix}_{kind}.bin", "wb", buffering=1 << 20)
        counts[kind] = 0
        poller.register(fd, select.POLLIN)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    t0 = time.monotonic()
    last_report = t0
    try:
        while not _stop:
            elapsed = time.monotonic() - t0
            if duration_s and elapsed >= duration_s:
                break
            if sample_target and min(counts.values()) >= sample_target:
                break

            for fd, _ev in poller.poll(1000):
                kind = fds[fd]
                rec = meta[kind]["record_bytes"]
                data = os.read(fd, rec * 512)
                if data:
                    if pairs is not None:
                        host = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                        t = last_ts_in(data, meta[kind])
                        if t:
                            pairs.append((kind, host, t))
                    # Preserve every raw sample for Allan analysis. Conversion or
                    # decimation here would change the data and delay FIFO draining.
                    files[kind].write(data)
                    counts[kind] += len(data) // rec

            now = time.monotonic()
            if now - last_report >= 30:
                pct = f" {100*elapsed/duration_s:.1f}%" if duration_s else ""
                print(f"[{elapsed/60:7.1f} min{pct}] "
                      + "  ".join(f"{k}={v}" for k, v in counts.items()),
                      flush=True)
                last_report = now
    finally:
        for f in files.values():
            f.close()
        for fd in fds:
            os.close(fd)

    return counts, time.monotonic() - t0


def unpack_timestamps(path, meta):
    """Pull just the timestamp column back out, for the --check histogram."""
    ts_chan = next(c for c in meta["channels"] if "timestamp" in c["name"])
    rec, off = meta["record_bytes"], ts_chan["offset"]
    fmt = ("<" if ts_chan["endian"] == "le" else ">") + "q"
    out = []
    with open(path, "rb") as f:
        blob = f.read()
    for i in range(0, len(blob) - rec + 1, rec):
        out.append(struct.unpack_from(fmt, blob, i + off)[0])
    return out


def histogram(deltas_us, nominal_us, bins=12):
    lo, hi = min(deltas_us), max(deltas_us)
    if hi == lo:
        hi = lo + 1
    width = (hi - lo) / bins
    counts = [0] * bins
    for d in deltas_us:
        counts[min(bins - 1, int((d - lo) / width))] += 1
    peak = max(counts) or 1
    lines = []
    for i, c in enumerate(counts):
        edge = lo + i * width
        bar = "#" * int(48 * c / peak)
        lines.append(f"  {edge:9.1f} us | {bar:<48} {c}")
    return "\n".join(lines)


def report_skew(pairs, meta):
    """Regress sample timestamps against CLOCK_MONOTONIC.

    This is the measurement the delta histogram cannot make. Slope 1.0 means
    the driver's reconstructed clock tracks the host; anything else is skew
    that accumulates for the whole flight."""
    print("--- clock skew vs CLOCK_MONOTONIC ---")
    for kind in meta:
        h = [p[1] for p in pairs if p[0] == kind]
        t = [p[2] for p in pairs if p[0] == kind]
        if len(h) < 20:
            print(f"{kind}: only {len(h)} reads, need a longer capture")
            continue
        span = (h[-1] - h[0]) / 1e9
        n = len(h)
        hb, tb = sum(h) / n, sum(t) / n
        num = sum((x - hb) * (y - tb) for x, y in zip(h, t))
        den = sum((x - hb) ** 2 for x in h)
        slope = num / den if den else float("nan")
        ppm = (slope - 1.0) * 1e6
        drift10 = (slope - 1.0) * 600.0
        print(f"{kind}: over {span:.1f} s, slope={slope:.7f}  "
              f"({ppm:+.0f} ppm)  -> {drift10*1e3:+.0f} ms adrift per 10 min")
    print("\nUnder ~100 ppm the camera-IMU offset stays inside what")
    print("calib_camimu_dt can hold. Much past that and it walks away.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="imu_run", help="output prefix")
    ap.add_argument("--hours", type=float, default=0.0)
    ap.add_argument("--minutes", type=float, default=0.0)
    ap.add_argument("--check", action="store_true",
                    help="short run: 10k samples, then timestamp delta stats")
    ap.add_argument("--samples", type=int, default=10000, help="--check sample count")
    ap.add_argument("--odr", type=float, default=416.0)
    # Stationary calibration uses narrower ranges to resolve sensor noise above
    # quantization. Flight needs wider ranges to avoid vibration-induced clipping.
    ap.add_argument("--accel-range", type=float, default=4.0, help="g (Allan run; flight is 16)")
    ap.add_argument("--gyro-range", type=float, default=500.0, help="dps (Allan run; flight is 2000)")
    ap.add_argument("--watermark", type=int, default=64)
    args = ap.parse_args()

    if os.geteuid() != 0:
        sys.exit("needs root (sysfs writes). re-run with sudo.")

    devs = find_devices()
    missing = {"accel", "gyro"} - set(devs)
    if missing:
        sys.exit(f"missing IIO device(s): {sorted(missing)}\n"
                 f"found: {[ (p, rd(p+'/name')) for p in sorted(glob.glob(SYSFS+'/iio:device*')) ]}")

    out = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    meta = {}
    for kind, path in devs.items():
        meta[kind] = setup(path, args.odr, args.accel_range, args.gyro_range, args.watermark)
        m = meta[kind]
        print(f"{kind:5s} {m['name']:16s} odr={m['odr_hz']} scale={m['scale']:.9g} "
              f"clock={m['timestamp_clock']} record={m['record_bytes']}B")

    for kind, m in meta.items():
        if m["timestamp_clock"] != "monotonic":
            print(f"WARNING: {kind} timestamps are on {m['timestamp_clock']}, not monotonic")

    duration = args.hours * 3600 + args.minutes * 60
    target = args.samples if args.check else None
    if args.check and duration == 0:
        duration = max(60.0, 3 * args.samples / args.odr)

    print(f"\ncapturing -> {out}_{{accel,gyro}}.bin "
          f"({'%.0f samples' % target if target else '%.2f h' % (duration/3600)})\n", flush=True)

    pairs = []
    try:
        counts, elapsed = capture(meta, out, duration, target, pairs)
    finally:
        teardown(devs.values())

    for kind, m in meta.items():
        m["samples"] = counts[kind]
        m["file"] = f"{out}_{kind}.bin"
    sidecar = {
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "elapsed_s": elapsed,
        "argv": sys.argv,
        "devices": meta,
    }
    with open(f"{out}.json", "w") as f:
        json.dump(sidecar, f, indent=2)

    print(f"\n{elapsed/60:.1f} min, "
          + ", ".join(f"{k}: {v} samples ({v/elapsed:.1f} Hz)" for k, v in counts.items()))
    print(f"sidecar -> {out}.json")

    if args.check:
        print("\n--- timestamp deltas ---")
        print("Ragged deltas here mean software timestamping, and everything")
        print("downstream is built on sand. Fix before the long run.\n")
        for kind, m in meta.items():
            ts = unpack_timestamps(m["file"], m)
            if len(ts) < 3:
                print(f"{kind}: too few samples"); continue
            d = [(ts[i + 1] - ts[i]) / 1000.0 for i in range(len(ts) - 1)]
            nominal = 1e6 / m["odr_hz"]
            mean = sum(d) / len(d)
            var = sum((x - mean) ** 2 for x in d) / len(d)
            sd = var ** 0.5
            back = sum(1 for x in d if x <= 0)
            print(f"{kind}: n={len(d)}  nominal={nominal:.1f} us  mean={mean:.2f} us  "
                  f"stdev={sd:.2f} us  min={min(d):.1f}  max={max(d):.1f}  non-advancing={back}")
            print(histogram(d, nominal))
            print()

        # The delta histogram above says the stream is evenly spaced. It cannot
        # say whether those timestamps track real time, because a driver that
        # subdivides a FIFO batch produces even deltas by construction. This
        # does: how long after a sample was stamped did userspace see it, and
        # how much does that vary.
        print("--- delivery latency (sample stamp -> userspace read) ---")
        print("Set by the FIFO watermark. Costs control-loop latency; does not")
        print("corrupt timestamps, and is not what calib_camimu_dt absorbs.\n")
        for kind in meta:
            v = [(h - t) / 1e6 for k, h, t in pairs if k == kind]
            if len(v) < 3:
                print(f"{kind}: too few reads")
                continue
            mean = sum(v) / len(v)
            sd = (sum((x - mean) ** 2 for x in v) / len(v)) ** 0.5
            print(f"{kind}: n={len(v)}  mean={mean:.2f} ms  stdev={sd:.2f} ms  "
                  f"spread={max(v)-min(v):.2f} ms")
        print()
        report_skew(pairs, meta)


if __name__ == "__main__":
    main()
