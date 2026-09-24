#!/usr/bin/env python3
"""
capture_session.py -- run OpenVINS live on viopi (Phase 4 step 7). With a terminal:

    ssh -t viopi 'sudo python3 ~/src/capture_session.py ~/vio/walk1'           # until Ctrl-C
    ssh -t viopi 'sudo python3 ~/src/capture_session.py ~/vio/walk1 --secs 180'

1. Select exposure using flight.json: sweep (default), auto, or fixed.
   Auto exposure caps shutter at max_shutter microseconds and makes up the
   rest with gain; refuses above gain 16. Values go to <out>.exposure.json.
2. The IMU is set up through imu_log.setup() -- the ODR, ranges and monotonic
   clock of the calibration run -- but at FIFO watermark 8, so samples reach
   the estimator ~18 ms after they are taken rather than the ~145 ms that the
   logger's watermark of 64 costs.
3. Starts ~/vio_live/vio_live, then rpicam-raw streaming into two FIFOs with
   --flush (measured: metadata arrives 10 ms after SensorTimestamp).
4. Records everything to <out>.{y16,meta.json,imu.json,imu_*.bin} unless
   --no-record, so the same run can be replayed on the VM through
   vio_bag_from_raw.py + replay_openvins.sh, and the estimate is <out>.est.txt.
5. vio_live low-passes the IMU at imu_lpf Hz from flight.json (default 50) before OpenVINS,
   against the motor vibration. The recording stays raw, so a replay is
   unfiltered unless the replay filters it too.
6. SIGTERM stops it the way Ctrl-C does, so flight_supervisor.py (started at boot by
   vio@.service) can stop it and keep the recording.
"""
from cli import parse_options
from flight_config import save_config
import json, os, pwd, shutil, signal, subprocess, sys, tempfile, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import imu_log  # noqa: E402

W, H = 1280, 800


def fail(msg):
    sys.exit(f"FAIL: {msg}")


def _sigterm(signum, frame):
    raise KeyboardInterrupt


def ae_probe(max_shutter, fixed_shutter=None, fixed_gain=None):
    """Same probe as kalibr_capture_imucam.sh: let AE settle, keep its exposure
    product, cap the shutter and move the rest into gain."""
    subprocess.run(["rpicam-raw", "-n", "--mode", f"{W}:{H}:8", "--width", str(W), "--height", str(H),
                    "-t", "2500", "--framerate", "5", "-o", "/tmp/ae.y16",
                    "--metadata", "/tmp/ae.json", "--metadata-format", "json"], capture_output=True)
    last = json.load(open("/tmp/ae.json"))[-1]
    os.remove("/tmp/ae.y16"); os.remove("/tmp/ae.json")
    P = last["ExposureTime"] * last["AnalogueGain"]
    if fixed_shutter is not None:
        sh = int(fixed_shutter)
        g = float(fixed_gain if fixed_gain is not None else 1.0)
    else:
        sh = int(min(P, max_shutter)); g = max(1.0, P / sh)
    print(f"  auto-exposure: {last['ExposureTime']} us x gain {last['AnalogueGain']:.2f} at ~{last.get('Lux', 0):.0f} lux"
          f" -> fixed {sh} us, gain {g:.2f}")
    if g > 16:
        fail(f"needs gain {g:.0f} at {sh} us -- too dark. Add light and rerun.")
    return sh, g


def exposure_sweep(candidates=(20, 30, 50, 75, 100, 200, 500, 1000, 2000, 4000)):
    """Choose the brightest fixed exposure below the clipping limit."""
    results = []
    for shutter in candidates:
        path = f"/tmp/exposure-sweep-{shutter}.y16"
        subprocess.run(["rpicam-raw", "-n", "--mode", f"{W}:{H}:8", "--width", str(W), "--height", str(H),
                        "--framerate", "20", "--shutter", str(shutter), "--gain", "1",
                        "--frames", "30", "-o", path], capture_output=True)
        try:
            data = open(path, "rb").read()
            pixels = data[1::2]
            if not pixels:
                continue
            clipped = sum(v >= 250 for v in pixels) / len(pixels)
            mean = sum(pixels) / len(pixels)
            results.append((shutter, clipped, mean))
        finally:
            try: os.remove(path)
            except OSError: pass
    if not results:
        fail("exposure sweep produced no frames")
    # Prefer the brightest candidate that remains below the clipping limit.
    # Choosing the shortest usable exposure made bright outdoor runs needlessly
    # dark: on run-20260915-173409, 20 us had mean 39 DN while 50 us had mean
    # 106 DN and still stayed just below the 1% clipping limit.
    usable = [r for r in results if r[1] <= 0.01 and r[2] >= 15]
    chosen = max(usable, key=lambda r: r[2]) if usable else min(results, key=lambda r: r[1])
    print("  exposure sweep: " + ", ".join(f"{s} us={clip*100:.2f}% clip, mean={mean:.1f}"
                                           for s, clip, mean in results))
    print(f"  exposure sweep selected {chosen[0]} us, gain 1.00 (brightest under clipping limit)")
    return chosen[0], 1.0


def imu_setup(watermark, out):
    devs = imu_log.find_devices()
    missing = {"accel", "gyro"} - set(devs)
    if missing:
        fail(f"missing IIO device(s) {sorted(missing)} -- is st_lsm6dsx loaded?")
    # One-shot reads before any buffer is enabled. After bench run 1
    # (2026-09-10) the sensor streamed nothing and single reads returned ~20 g
    # at rest with the two bytes of every word equal (0x6F6F, 0xAEAE...): its
    # SPI link or register state had gone bad while the rig was handled on
    # dupont clips. Refuse rather than spend a run finding that out.
    d = devs["accel"]
    scale = float(imu_log.rd(f"{d}/in_accel_scale"))
    raw = [int(imu_log.rd(f"{d}/in_accel_{ax}_raw")) for ax in "xyz"]
    norm = sum((v * scale) ** 2 for v in raw) ** 0.5
    doubled = all(((v & 0xFFFF) >> 8) == (v & 0xFF) for v in raw)
    print(f"  imu: one-shot |accel| {norm:.2f} m/s^2")
    if doubled or not 8.3 < norm < 11.3:
        fail(f"IMU reads garbage (raw {raw}, |accel| {norm:.1f} m/s^2, hold the rig still)"
             + (" -- every word has equal bytes: SPI link or register state is bad. Reseat the wires, "
                "power-cycle the Pi (a reboot keeps 3.3 V up), retest with imu_irq_check.sh" if doubled else ""))
    meta = {k: imu_log.setup(p, 416, 16, 2000, watermark) for k, p in devs.items()}
    lines = []
    for kind, m in meta.items():
        if m["timestamp_clock"] != "monotonic":
            fail(f"{kind} timestamps are on {m['timestamp_clock']}, not monotonic -- the udev rule?")
        ch = {c["name"]: c for c in m["channels"]}
        pre = "in_accel_" if kind == "accel" else "in_anglvel_"
        axes = [ch.get(pre + a) for a in "xyz"]
        ts = ch.get("in_timestamp")
        # vio_live reads le s16 axes and an le s64 timestamp at these offsets;
        # anything else is a driver change and must not be parsed blind.
        if any(c is None or (c["endian"], c["signed"], c["storage"], c["bits"], c["shift"]) != ("le", True, 2, 16, 0)
               for c in axes) or ts is None or (ts["endian"], ts["storage"]) != ("le", 8):
            fail(f"{kind}: unexpected IIO channel layout {m['channels']}")
        lines.append(f"{kind} {m['chardev']} {m['record_bytes']} {m['scale']!r} "
                     f"{ts['offset']} {axes[0]['offset']} {axes[1]['offset']} {axes[2]['offset']}")
        m["file"] = f"{out}.imu_{kind}.bin"
        print(f"  imu: {kind} {m['chardev']} odr {m['odr_hz']} scale {m['scale']:.6g} watermark {watermark}")
    with open(out + ".imu.cfg", "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(out + ".imu.json", "w") as f:   # imu_log.py's sidecar format; kalibr_imu_csv.load reads it
        json.dump({"created": time.strftime("%Y-%m-%dT%H:%M:%S"), "argv": sys.argv, "devices": meta}, f, indent=2)
    return devs


def main():
    user = pwd.getpwnam(os.environ.get("SUDO_USER") or pwd.getpwuid(os.getuid()).pw_name)
    a = parse_options("capture", user.pw_dir)
    if os.geteuid() != 0:
        fail("needs root for the IMU: run with sudo")
    for f in (a.estimator_binary, a.estimator_config):
        if not os.path.exists(f):
            fail(f"{f} not found -- run tools/deploy/build_vio.sh on the Mac")
    out = os.path.abspath(a.out)
    est = os.path.abspath(a.est) if a.est else out + ".est.txt"
    os.makedirs(os.path.dirname(out), exist_ok=True)
    save_config(a, out + ".flight.json")
    # Under systemd (flight_supervisor.py) a stop is SIGTERM to this process alone, with no
    # tty to hand SIGINT to rpicam-raw and vio_live as well. Take it as a Ctrl-C:
    # the finally block stops the camera, and vio_live ends when its FIFOs close.
    signal.signal(signal.SIGTERM, _sigterm)

    print(f"=== exposure: {a.exposure_mode}, point the camera at the scene ===")
    sh, g = (exposure_sweep() if a.exposure_mode == "sweep" else
             ae_probe(a.max_shutter, a.shutter if a.exposure_mode == "fixed" else None, a.gain))
    with open(out + ".exposure.json", "w") as f:
        json.dump({"shutter_us": sh, "gain": round(g, 2), "max_shutter_us": a.max_shutter}, f)

    devs = {}
    fdir = tempfile.mkdtemp(prefix="vio_live_")
    frames, meta = os.path.join(fdir, "frames"), os.path.join(fdir, "meta")
    os.mkfifo(frames); os.mkfifo(meta)
    vio = cam = None
    interrupted = False
    try:
        if not a.no_record:
            try:
                # Preserve calibration/config used to interpret feature coordinates.
                with open(out + ".recording.json", "w") as manifest:
                    json.dump({"version": 1, "width": W, "height": H,
                               "argv": sys.argv, "clock": "CLOCK_MONOTONIC",
                               "config_files": {name: open(os.path.join(os.path.dirname(a.estimator_config), name)).read()
                                                for name in os.listdir(os.path.dirname(a.estimator_config))
                                                if name.endswith(".yaml")}}, manifest, indent=2)
            except OSError as e:
                print(f"Could not save recording manifest: {e}")
        devs = imu_setup(a.watermark, out)
        vio = subprocess.Popen([a.estimator_binary, "--imu", out + ".imu.cfg", "--frames", frames, "--meta", meta,
                                "--config", a.estimator_config, "--out", est, "--verbosity", a.verbosity,
                                "--imu-lpf", str(a.imu_lpf)]
                               + ([] if a.no_record else ["--record", out]))
        time.sleep(1.5)
        if vio.poll() is not None:
            fail(f"vio_live exited {vio.returncode} during startup (config?)")
        print(f"=== camera: {W}x{H} at {a.fps:g} fps, {sh} us, gain {g:.2f}. "
              f"Keep the rig STILL until it says INITIALIZED. Ctrl-C to stop. ===", flush=True)
        cam = subprocess.Popen(["rpicam-raw", "-n", "--mode", f"{W}:{H}:8", "--width", str(W), "--height", str(H),
                                "-t", str(a.secs * 1000), "--framerate", str(a.fps),
                                "--shutter", str(sh), "--gain", f"{g:.2f}", "--flush",
                                "-o", frames, "--metadata", meta, "--metadata-format", "json"],
                               stdout=subprocess.DEVNULL, stderr=open(out + ".cam.log", "w"))
        while cam.poll() is None and vio.poll() is None:
            time.sleep(0.2)
    except KeyboardInterrupt:
        interrupted = True   # Ctrl-C (the tty sent SIGINT to rpicam-raw and vio_live too), or SIGTERM
    finally:
        # A second Ctrl-C used to land here and SIGKILL vio_live before it had
        # written the recording's metadata (walk2, 2026-09-10: 362 frames, a
        # stale 236-record meta.json, unreplayable). Shutdown takes seconds.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if interrupted:
            print("\n=== stopping: waiting for vio_live's summary (further Ctrl-C is ignored) ===", flush=True)
        for p, grace in ((cam, 10), (vio, 60)):
            if p is None:
                continue
            try:
                if p is cam and cam.poll() is None:
                    cam.send_signal(signal.SIGINT)    # vio_live ended first; stop the camera
                # Also when interrupted: a SIGTERM before the camera started reached
                # only this process, and vio_live would wait out its grace for FIFOs
                # nobody will open. After a tty's Ctrl-C a second signal is harmless.
                if p is vio and (cam is None or cam.returncode not in (0, None)):
                    vio.send_signal(signal.SIGTERM)   # camera never ran: nothing will close the FIFOs
                p.wait(timeout=grace)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                p.kill(); p.wait()
        if not a.no_record and vio is not None and vio.returncode == 0:
            try:
                with open(out + ".complete.json", "w") as marker:
                    json.dump({"version": 1, "vio_exit": vio.returncode,
                               "camera_exit": cam.returncode if cam else None}, marker)
            except OSError as e:
                print(f"Could not write completion marker: {e}")
        imu_log.teardown(devs.values())
        shutil.rmtree(fdir, ignore_errors=True)
        for f in os.listdir(os.path.dirname(out)):
            if f.startswith(os.path.basename(out) + "."):
                os.chown(os.path.join(os.path.dirname(out), f), user.pw_uid, user.pw_gid)
    if cam is not None and cam.returncode not in (0, None) and not interrupted:
        print(f"rpicam-raw exited {cam.returncode}:")
        print("".join(open(out + ".cam.log").readlines()[-5:]))
    print(f"\nfiles: {est}" + ("" if a.no_record else f", recording {out}.{{y16,meta.json,imu.json,imu_*.bin}}"))


if __name__ == "__main__":
    main()
