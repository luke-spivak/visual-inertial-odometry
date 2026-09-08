#!/usr/bin/env python3
"""
Measure camera frame-timestamp quality on the Pi. Phase 4's timestamp gate.

PROJECT.md calls camera timestamping the highest-risk item in this build, and
the camera derivation (requirement 2) says why: the estimator integrates IMU
samples between frame times, so a frame stamped at t but exposed at t+d
attributes that motion to the wrong interval. At 5 m/s a 20 ms error injects
10 cm of position error per frame.

A CONSTANT offset d is fine -- OpenVINS estimates it online as
`calib_camimu_dt`, and Phase 3 step 9 demonstrated recovery of an injected
30 ms offset in sim. A VARIABLE d is unmodelable and corrupts everything. So
the quantity that matters is not the offset, it is the VARIANCE of the offset.
That is what this measures.

Three things it reports, in descending order of importance:

  1. WHICH CLOCK SensorTimestamp is on -- determined by comparison against all
     three candidates rather than assumed. It has to be the same timebase the
     IMU stamps on or nothing downstream means anything. See the IIO note.

  2. FRAME-TO-FRAME JITTER of SensorTimestamp. The sensor's own cadence. This
     is the number Phase 4 gates on, and it should be tight -- microseconds,
     not milliseconds.

  3. DELIVERY LATENCY -- SensorTimestamp against the monotonic clock read the
     instant the frame arrives in Python. This does NOT feed the estimator. It
     exists to show the jitter you WOULD have inherited by stamping frames on
     arrival, which is what UVC forces and is precisely why PROJECT.md chose
     CSI over USB. Expect it to be far looser than (2); that gap is the whole
     argument for the interface choice, made visible.

**IIO CROSS-REFERENCE -- read this before trusting the IMU.** libcamera stamps
on the monotonic timebase (confirmed by this tool). The Linux IIO subsystem
defaults its timestamps to CLOCK_REALTIME, which is NTP-disciplined and can
STEP -- backwards, mid-flight, without warning. Two sensors on two timebases
is the exact failure this exercise exists to prevent. Before the IMU is
trusted:

    cat /sys/bus/iio/devices/iio:device0/current_timestamp_clock
    echo monotonic | sudo tee /sys/bus/iio/devices/iio:device0/current_timestamp_clock

Verify rather than assume, and re-check after every reboot until it is made
persistent.

    python3 cam_timing.py                       # 300 frames at 30 fps
    python3 cam_timing.py --frames 1000 --fps 60
    python3 cam_timing.py --save stamps.csv     # for cross-checking against the IMU
"""
import argparse
import statistics
import sys
import time


def clock_probe(sensor_ts_ns):
    """Identify SensorTimestamp's timebase by comparison.

    REALTIME is seconds-since-1970 and so is ~1.8e18 ns -- unmistakable.
    MONOTONIC and BOOTTIME are identical on a machine that has never
    suspended, which a flight Pi never does, so they are not distinguishable
    here and do not need to be: both are steady and neither is NTP-stepped."""
    now = {
        "CLOCK_MONOTONIC": time.monotonic_ns(),
        "CLOCK_BOOTTIME": time.clock_gettime_ns(time.CLOCK_BOOTTIME),
        "CLOCK_REALTIME": time.clock_gettime_ns(time.CLOCK_REALTIME),
    }
    print("Clock identification")
    for name, v in now.items():
        d = (v - sensor_ts_ns) / 1e6
        verdict = "plausible" if -1000 < d < 5000 else "NO"
        print(f"  {name:<16} now-sensor = {d:>16.1f} ms   {verdict}")
    if abs(now["CLOCK_REALTIME"] - sensor_ts_ns) < 1e12:
        print("  -> SensorTimestamp appears to be on CLOCK_REALTIME. This is "
              "NTP-disciplined and can step. Do not fly on it.")
    else:
        print("  -> SensorTimestamp is on the monotonic timebase "
              "(MONOTONIC/BOOTTIME are the same here). Set IIO to `monotonic` "
              "to match -- see the module docstring.")
    print()


def histogram(vals, nbins=25, width=52, unit="us"):
    lo, hi = min(vals), max(vals)
    if hi - lo < 1e-9:
        print(f"  all {len(vals)} values identical at {lo:.1f} {unit}")
        return
    step = (hi - lo) / nbins
    counts = [0] * nbins
    for v in vals:
        k = min(nbins - 1, int((v - lo) / step))
        counts[k] += 1
    peak = max(counts)
    for k, c in enumerate(counts):
        if not c:
            continue
        edge = lo + k * step
        bar = "#" * max(1, round(width * c / peak))
        print(f"  {edge:9.1f} {unit} | {bar} {c}")


def summarise(name, vals, unit="us"):
    v = sorted(vals)
    n = len(v)
    print(f"{name}  (n={n})")
    print(f"  mean   {statistics.fmean(v):10.2f} {unit}")
    print(f"  stdev  {statistics.pstdev(v):10.2f} {unit}   <- the number that matters")
    print(f"  min    {v[0]:10.2f} {unit}")
    print(f"  p50    {v[n // 2]:10.2f} {unit}")
    print(f"  p99    {v[min(n - 1, int(0.99 * n))]:10.2f} {unit}")
    print(f"  max    {v[-1]:10.2f} {unit}")
    print(f"  spread {v[-1] - v[0]:10.2f} {unit}")
    print()


def main():
    ap = argparse.ArgumentParser(
        description="Camera frame-timestamp jitter measurement.")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--fps", type=float, default=30.0,
                    help="pinned frame rate (default 30)")
    ap.add_argument("--shutter", type=int, default=5000, metavar="US",
                    help="fixed exposure, us (default 5000). Pinned so that a "
                         "moving AGC cannot change FrameDuration underneath "
                         "the measurement")
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--save", metavar="CSV",
                    help="write per-frame timestamps for cross-checking "
                         "against the IMU")
    ap.add_argument("--warmup", type=int, default=20,
                    help="frames to discard before measuring (default 20)")
    a = ap.parse_args()

    try:
        from picamera2 import Picamera2
    except ImportError:
        sys.exit("needs picamera2:  sudo apt install -y python3-picamera2")

    period_us = 1e6 / a.fps
    dur = int(round(period_us))

    p = Picamera2()
    # The raw stream is the one that carries real pixels on this camera; the
    # processed path returns zeros (see PROJECT.md, Camera bringup). `main` is
    # mandatory in picamera2, so it is kept small to spend as little ISP time
    # as possible on an output nothing reads.
    cfg = p.create_video_configuration(
        main={"size": (64, 64)},
        raw={"size": (a.width, a.height), "format": "R8"},
        buffer_count=8,
        controls={
            "AeEnable": False,
            "ExposureTime": a.shutter,
            "AnalogueGain": a.gain,
            # Pinning both ends holds the cadence fixed. A range would let the
            # pipeline vary the period and the jitter measurement would then
            # be measuring policy, not the sensor.
            "FrameDurationLimits": (dur, dur),
        },
    )
    p.configure(cfg)
    p.start()

    rows = []
    try:
        for i in range(a.frames + a.warmup):
            req = p.capture_request()
            arrival = time.monotonic_ns()
            md = req.get_metadata()
            req.release()
            if i >= a.warmup:
                rows.append((
                    md["SensorTimestamp"], arrival, md.get("ExposureTime"),
                    md.get("AnalogueGain"), md.get("FrameDuration"),
                ))
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        p.stop()

    if len(rows) < 3:
        sys.exit("too few frames captured to say anything")

    print()
    print(f"{len(rows)} frames, requested {a.fps:g} fps "
          f"(period {period_us:.1f} us), exposure {a.shutter} us, gain {a.gain}")
    print()

    clock_probe(rows[-1][0])

    # Exposure must not have moved, or the cadence measurement is confounded.
    exps = {r[2] for r in rows}
    gains = {round(r[3], 3) for r in rows if r[3] is not None}
    if len(exps) > 1 or len(gains) > 1:
        print(f"WARNING: exposure/gain moved during capture "
              f"(exp {sorted(exps)}, gain {sorted(gains)}). "
              f"Jitter below is not attributable to the sensor alone.\n")

    sensor_d = [(rows[i][0] - rows[i - 1][0]) / 1e3 for i in range(1, len(rows))]
    arrival_d = [(rows[i][1] - rows[i - 1][1]) / 1e3 for i in range(1, len(rows))]
    latency = [(r[1] - r[0]) / 1e3 for r in rows]

    # A dropped frame shows up as a delta near an integer multiple of the
    # period. Counting them matters because a drop is not jitter -- averaging
    # it into the jitter statistics would hide both.
    drops = sum(round(d / period_us) - 1 for d in sensor_d
                if round(d / period_us) >= 2)
    kept = [d for d in sensor_d if round(d / period_us) < 2]

    summarise("SensorTimestamp frame-to-frame delta", kept)
    print(f"  dropped frames: {drops}"
          f"{'  <- investigate' if drops else ''}")
    print()
    summarise("Arrival-time frame-to-frame delta (userspace)", arrival_d)
    summarise("Delivery latency (arrival - SensorTimestamp)", latency)

    sj = statistics.pstdev(kept)
    aj = statistics.pstdev(arrival_d)
    print(f"Jitter ratio: userspace arrival is {aj / sj:.1f}x noisier than "
          f"SensorTimestamp.")
    print("  That ratio is the CSI-over-UVC argument, measured. Stamp on "
          "arrival and you inherit the larger number.")
    print()
    print("SensorTimestamp delta histogram")
    histogram(kept)

    if a.save:
        with open(a.save, "w") as fh:
            fh.write("sensor_ts_ns,arrival_ns,exposure_us,gain,frame_dur_us\n")
            for r in rows:
                fh.write("%d,%d,%s,%s,%s\n" % r)
        print(f"\nwrote {a.save}")


if __name__ == "__main__":
    main()
