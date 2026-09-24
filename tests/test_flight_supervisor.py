#!/usr/bin/env python3
"""Tests for flight_supervisor.py's session logic, against a stand-in FC and a stand-in
capture_session.py. No pymavlink, camera or IMU needed. Run with python3 -m pytest from the repository root.

The one that matters is the restart: a new run's frame has an arbitrary yaw, so its
poses must not reach an armed aircraft until Viso Align has been accepted for it."""
import os
import json
import pwd
import signal
import sys
import tempfile
import threading
import time

import flight_supervisor as vf
from cli import parse_options

# Run 1 streams 30 poses and dies, as a crash would; later runs go until SIGTERM.
FAKE_VIO_LIVE = r'''
import os, signal, sys, time
prefix, est = sys.argv[1], sys.argv[sys.argv.index("--est") + 1]
first = not any(f.endswith(".args") for f in os.listdir(os.path.dirname(prefix)))
open(prefix + ".args", "w").write(" ".join(sys.argv[1:]))
stop = []
signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
print("  auto-exposure: 9000 us x gain 1.00 at ~300 lux -> fixed 4000 us, gain 2.25", flush=True)
print("\n  *** INITIALIZED at t=1.000 s -- poses streaming ***\n", flush=True)
with open(est, "w") as f:
    f.write("# timestamp(s) q(JPL xyzw) p v bg ba\n")
    i = 0
    while not stop and (not first or i < 30):
        f.write("%.9f 0 0 0 1 %.3f 0 0 0 0 0 0 0 0 0 0 0\n" % (time.monotonic(), 0.01 * i))
        f.flush()
        i += 1
        time.sleep(0.02)
print("=== vio_live summary ===", flush=True)
sys.exit(3 if first else 0)
'''


class Msg:
    def __init__(self, kind, **fields):
        self.kind = kind
        self.__dict__.update(fields)

    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


class FakeFC:
    """Stands in for the pymavlink connection: an ArduPilot heartbeat every 0.2 s
    while alive, an ack for every command, and a record of what was sent."""

    def __init__(self):
        self.mav = self
        self.armed, self.alive = False, True
        self.inbox, self.hb_t = [], 0.0
        self.poses, self.texts, self.aux = [], [], []

    def recv_match(self, blocking=False):
        if self.alive and time.monotonic() - self.hb_t > 0.2:
            self.hb_t = time.monotonic()
            self.inbox.append(Msg("HEARTBEAT", autopilot=3, type=2, base_mode=128 if self.armed else 0))
        return self.inbox.pop(0) if self.inbox else None

    def vision_position_estimate_send(self, usec, n, e, d, roll, pitch, yaw, cov, reset_counter):
        self.poses.append((reset_counter, self.armed))

    def vision_speed_estimate_send(self, *args):
        pass

    def statustext_send(self, severity, text):
        self.texts.append(text.decode())

    def command_long_send(self, target_system, target_component, command, confirmation, *params):
        self.aux.append((params[0], params[1], self.armed))
        self.inbox.append(Msg("COMMAND_ACK", command=command, result=0))


def wait_for(cond, what, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return
        time.sleep(0.02)
    raise AssertionError(f"timed out waiting for {what}")


def start(tmp, fc, min_free_gb=0.0):
    os.makedirs(os.path.join(tmp, "vio"))
    fake = os.path.join(tmp, "fake_capture_session.py")
    with open(fake, "w") as f:
        f.write(FAKE_VIO_LIVE)
    a = parse_options("flight", tmp, [])
    a.recording_dir, a.estimate_dir = os.path.join(tmp, "vio"), os.path.join(tmp, "run")
    a.device, a.min_free_gb = "fake", min_free_gb
    vf.STOP["sig"] = None
    t = threading.Thread(target=vf.fly, args=(a, vf.Link(fc), pwd.getpwuid(os.getuid()),
                                              [sys.executable, "-u", fake]))
    t.start()
    return a, t


def stop(t):
    vf.STOP["sig"] = signal.SIGTERM
    t.join(20)
    assert not t.is_alive()


def files(a, suffix):
    return sorted(f for f in os.listdir(a.recording_dir) if f.endswith(suffix))


def test_a_restarted_run_reaches_an_armed_aircraft_only_once_aligned():
    fc = FakeFC()
    with tempfile.TemporaryDirectory() as tmp:
        a, t = start(tmp, fc)
        try:
            # Run 1: streams while disarmed, is aligned, then dies.
            wait_for(lambda: any("VIO run 1 ended" in x for x in fc.texts), "run 1 to end")
            assert fc.aux == [(80, 2, False)]
            assert fc.poses and all(p == (0, False) for p in fc.poses)
            # Run 2 starts while armed: its poses are held, and no align is sent.
            fc.armed = True
            wait_for(lambda: any("VIO run 2 unaligned, FC armed" in x for x in fc.texts), "run 2 to hold")
            time.sleep(0.3)
            assert not [p for p in fc.poses if p[0] == 1] and len(fc.aux) == 1
            # Disarmed: run 2 streams, is aligned, and from then on flies armed.
            fc.armed = False
            wait_for(lambda: len(fc.aux) == 2, "run 2's Viso Align")
            assert fc.aux[1] == (80, 2, False)
            time.sleep(0.3)
            fc.armed = True
            wait_for(lambda: (1, True) in fc.poses, "run 2's poses while armed")
        finally:
            stop(t)
        # Each run left its estimate beside its recording, nothing on tmpfs.
        assert len(files(a, ".est.txt")) == 2 and len(files(a, ".log")) == 2
        assert os.listdir(a.estimate_dir) == ["reset_counter"]
        with open(os.path.join(a.estimate_dir, "reset_counter")) as f:
            assert f.read() == "1"
        with open(os.path.join(a.recording_dir, files(a, ".est.txt")[0])) as f:
            assert len(f.readlines()) == 31                 # header + 30 poses
        with open(os.path.join(a.recording_dir, files(a, ".args")[0])) as f:
            assert "--est" in f.read()
        for expected in ("VIO exposure fixed 4000 us, gain 2.25", "VIO initialized",
                         "VIO run 1 ended: exit 3, 30 poses"):
            assert expected in fc.texts, (expected, fc.texts)
        assert any(x.startswith("VIO run 1 recording") for x in fc.texts)


def test_waits_for_the_fc_and_skips_recording_when_short_of_space():
    fc = FakeFC()
    fc.alive = False
    with tempfile.TemporaryDirectory() as tmp:
        a, t = start(tmp, fc, min_free_gb=1e9)
        try:
            time.sleep(0.5)
            assert not files(a, ".args")                    # no FC, no run
            fc.alive = True
            wait_for(lambda: files(a, ".args"), "run 1 to start")
            with open(os.path.join(a.recording_dir, files(a, ".args")[0])) as f:
                args = f.read().split()
            assert "--no-record" in args
            config_path = args[args.index("--config") + 1]
            with open(config_path) as snapshot:
                config = json.load(snapshot)
            assert config["exposure_mode"] == "sweep"
            assert config["upside_down"] is True
            assert config["recording_dir"] == a.recording_dir
            assert any(x.startswith("VIO run 1 NOT recording") for x in fc.texts)
        finally:
            stop(t)


def test_status_lines_the_pilot_sees():
    assert vf.status_of("FAIL: needs gain 22 at 4000 us -- too dark. Add light and rerun.\n") == \
        (vf.SEV_ERROR, "VIO FAIL: needs gain 22 at 4000 us -- too dark. Add light and rerun.")
    assert vf.status_of("  auto-exposure: 14994 us x gain 1.44 at ~120 lux -> fixed 4000 us, gain 5.40\n") == \
        (vf.SEV_INFO, "VIO exposure fixed 4000 us, gain 5.40")
    assert vf.status_of("  --- moving: first full visual update at t=12.3 s ---") == \
        (vf.SEV_INFO, "VIO first visual update")
    assert vf.status_of("[  12.0 s] imu 440 Hz | cam 19.2 in 19.2 done fps, 0 dropped | ...") is None


def test_reset_counter_carries_over_a_restart_not_a_reboot():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "reset_counter")
        assert vf.load_counter(p) == 0                      # no file: the Pi (and FC) just booted
        for stored, first in (("7", 8), ("255", 0)):
            with open(p, "w") as f:
                f.write(stored)
            assert vf.load_counter(p) == first


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok ", name)
