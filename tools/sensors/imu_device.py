"""Discover, configure, and disable Linux IIO IMU devices."""
import glob
import os
import re

SYSFS = "/sys/bus/iio/devices"
TYPE_RE = re.compile(r"(?P<end>be|le):(?P<sign>[su])(?P<bits>\d+)/(?P<storage>\d+)>>(?P<shift>\d+)")


def rd(path):
    with open(path) as f:
        return f.read().strip()


def wr(path, value):
    with open(path, "w") as f:
        f.write(str(value))


def find_devices():
    """Map 'accel'/'gyro' -> sysfs path. st_lsm6dsx registers one iio device per
    sensor, so there are two buffers to drain, not one."""
    found = {}
    for path in sorted(glob.glob(f"{SYSFS}/iio:device*")):
        try:
            name = rd(f"{path}/name")
        except OSError:
            continue
        if "accel" in name:
            found["accel"] = path
        elif "gyro" in name or "anglvel" in name:
            found["gyro"] = path
    return found


def scan_dir(dev):
    """Scan-element *_en files live in scan_elements/ on older kernels and in
    buffer0/ on newer ones."""
    for d in (f"{dev}/scan_elements", f"{dev}/buffer0", f"{dev}/buffer"):
        if glob.glob(f"{d}/*_en"):
            return d
    raise RuntimeError(f"{dev}: no scan elements -- is the driver bound with an IRQ?")


def buffer_dir(dev):
    for d in (f"{dev}/buffer0", f"{dev}/buffer"):
        if os.path.isdir(d):
            return d
    raise RuntimeError(f"{dev}: no buffer directory")


def snap(dev, attr, target):
    """Write the available value closest to target. IIO exposes ranges as scale
    factors, so ranges are selected by picking a scale, not by naming a range."""
    avail = [float(v) for v in rd(f"{dev}/{attr}_available").split()]
    best = min(avail, key=lambda v: abs(v - target))
    wr(f"{dev}/{attr}", f"{best:.9f}".rstrip("0"))
    return float(rd(f"{dev}/{attr}"))


def layout(dev):
    """Build the record layout from the enabled scan elements.

    IIO packs enabled channels in scan-index order, each naturally aligned to
    its own storage size, and pads the record to the largest alignment. Parsing
    it rather than hardcoding 16 bytes means a driver change surfaces as a
    parse difference instead of silently shifted data."""
    sd = scan_dir(dev)
    chans = []
    for en in sorted(glob.glob(f"{sd}/*_en")):
        if rd(en) != "1":
            continue
        base = en[:-3]
        m = TYPE_RE.match(rd(base + "_type"))
        if not m:
            raise RuntimeError(f"unparsed type for {base}")
        storage = int(m.group("storage")) // 8
        chans.append({
            "name": os.path.basename(base),
            "index": int(rd(base + "_index")),
            "endian": m.group("end"),
            "signed": m.group("sign") == "s",
            "bits": int(m.group("bits")),
            "storage": storage,
            "shift": int(m.group("shift")),
        })
    chans.sort(key=lambda c: c["index"])

    offset = 0
    for c in chans:
        pad = (-offset) % c["storage"]
        offset += pad
        c["offset"] = offset
        offset += c["storage"]
    align = max(c["storage"] for c in chans)
    record = offset + ((-offset) % align)
    return chans, record


def setup(dev, odr, accel_range_g, gyro_range_dps, watermark):
    bd = buffer_dir(dev)
    sd = scan_dir(dev)

    wr(f"{bd}/enable", 0)

    # Match camera SensorTimestamp; CLOCK_REALTIME can step with wall-clock sync.
    clk = f"{dev}/current_timestamp_clock"
    if os.path.exists(clk):
        wr(clk, "monotonic")

    wr(f"{dev}/sampling_frequency", odr)

    scale = None
    if os.path.exists(f"{dev}/in_accel_scale_available"):
        # target scale = full-scale in m/s^2 spread over a signed 16-bit range
        scale = snap(dev, "in_accel_scale", accel_range_g * 9.80665 / 32768.0)
    elif os.path.exists(f"{dev}/in_anglvel_scale_available"):
        import math
        scale = snap(dev, "in_anglvel_scale", math.radians(gyro_range_dps) / 32768.0)

    for en in glob.glob(f"{sd}/*_en"):
        wr(en, 1)

    try:
        wr(f"{bd}/watermark", watermark)
    except OSError:
        pass
    wr(f"{bd}/length", max(4096, watermark * 4))

    chans, record = layout(dev)
    wr(f"{bd}/enable", 1)

    return {
        "sysfs": dev,
        "name": rd(f"{dev}/name"),
        "chardev": "/dev/" + os.path.basename(dev),
        "odr_hz": float(rd(f"{dev}/sampling_frequency")),
        "scale": scale,
        "timestamp_clock": rd(clk) if os.path.exists(clk) else "unknown",
        "record_bytes": record,
        "channels": chans,
    }


def teardown(devs):
    for dev in devs:
        try:
            wr(f"{buffer_dir(dev)}/enable", 0)
        except OSError:
            pass
