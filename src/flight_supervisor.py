#!/usr/bin/env python3
"""Supervise onboard capture sessions and forward estimates to ArduPilot.

Waits for the flight controller, aligns new estimator frames, and restarts capture.
"""
from cli import parse_options
from flight_config import save_config
import os
import pwd
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mavlink_bridge  # noqa: E402

# MAVLink values, spelled out so this imports without pymavlink (the tests do).
SEV_CRITICAL, SEV_ERROR, SEV_WARNING, SEV_INFO = 2, 3, 4, 6
MAV_CMD_DO_AUX_FUNCTION, MAV_RESULT_ACCEPTED = 218, 0
VISODOM_ALIGN, SWITCH_HIGH = 80, 2                    # aux function 80 acts on high
MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_GCS, MAV_MODE_FLAG_SAFETY_ARMED = 3, 6, 128

STOP = {"sig": None}                                  # set by SIGTERM / SIGINT


def log(msg):
    print(f"[flight_supervisor] {msg}", flush=True)


def status_of(line):
    """(severity, STATUSTEXT) for a capture_session.py output line the pilot should see, else None."""
    if "FAIL:" in line:
        return SEV_ERROR, "VIO FAIL:" + line.split("FAIL:", 1)[1].rstrip()
    if "auto-exposure:" in line and "->" in line:
        return SEV_INFO, "VIO exposure " + line.rsplit("->", 1)[1].strip()
    if line.startswith("  fixed-exposure:"):
        return SEV_INFO, "VIO exposure " + line.split(":", 1)[1].strip()
    if line.startswith("  exposure sweep selected "):
        return SEV_INFO, "VIO exposure " + line.split("selected ", 1)[1].strip()
    if "*** INITIALIZED" in line:
        return SEV_INFO, "VIO initialized"
    if "--- moving:" in line:
        return SEV_INFO, "VIO first visual update"
    if line.startswith("rpicam-raw exited"):
        return SEV_ERROR, "VIO " + line.strip()
    if "NEVER INITIALIZED" in line:
        return SEV_WARNING, "VIO never initialized"
    return None


class Link:
    """The one MAVLink connection to the FC: poses and status out, heartbeats and acks in."""

    def __init__(self, conn):
        self.conn = conn
        self.target = None                            # (sysid, compid) of the autopilot
        self.hb_t = None                              # when its last heartbeat arrived
        self.armed = None
        self.acks = []

    def poll(self):
        while True:
            m = self.conn.recv_match(blocking=False)
            if m is None:
                return
            kind = m.get_type()
            if kind == "HEARTBEAT":
                if m.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA or m.type == MAV_TYPE_GCS:
                    continue
                if self.hb_t is None:
                    log(f"FC heartbeat, system {m.get_srcSystem()}")
                self.target, self.hb_t = (m.get_srcSystem(), m.get_srcComponent()), time.monotonic()
                armed = bool(m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                if armed != self.armed:
                    log("FC " + ("ARMED" if armed else "disarmed"))
                self.armed = armed
            elif kind == "COMMAND_ACK":
                self.acks.append(m)
            elif kind == "STATUSTEXT":
                text = m.text if isinstance(m.text, str) else m.text.decode(errors="replace")
                log(f"{m.get_srcSystem()}/{m.get_srcComponent()} says: {text.strip()}")

    def fresh(self):
        return self.hb_t is not None and time.monotonic() - self.hb_t < 3.0

    def disarmed(self):
        return self.fresh() and self.armed is False

    def say(self, severity, text):
        log(f"-> FC: {text}")
        self.conn.mav.statustext_send(severity, text.encode("ascii", "replace")[:50])

    def aux(self, function, position):
        self.conn.mav.command_long_send(self.target[0], self.target[1], MAV_CMD_DO_AUX_FUNCTION, 0,
                                        function, position, 0, 0, 0, 0, 0)

    def take_acks(self, command):
        mine = [a for a in self.acks if a.command == command]
        self.acks = [a for a in self.acks if a.command != command]
        return mine


def load_counter(path):
    """The first reset counter to use. The file is on tmpfs: it outlives a restart of
    this program, not a reboot -- which restarts the FC too, and it starts at 0."""
    try:
        with open(path) as f:
            return (int(f.read()) + 1) % 256
    except (OSError, ValueError):
        return 0


def new_prefix(d):
    base = os.path.join(d, time.strftime("run-%Y%m%d-%H%M%S"))
    prefix, k = base, 1
    while any(f.startswith(os.path.basename(prefix) + ".") for f in os.listdir(d)):
        k += 1
        prefix = f"{base}-{k}"
    return prefix


def keep(tmp, dst, user):
    """Move a run's estimate off tmpfs, next to its recording, owned like the rest."""
    if not os.path.exists(tmp):
        return
    try:
        shutil.copyfile(tmp, dst)
        os.chown(dst, user.pw_uid, user.pw_gid)
        os.remove(tmp)
        log(f"estimate: {dst}")
    except OSError as e:
        log(f"could not copy {tmp} to {dst} ({e}); it stays on tmpfs until reboot")


def run(a, link, sender, user, vio_cmd, n):
    """One capture_session.py run, until it ends. Returns how many poses reached the FC."""
    prefix = new_prefix(a.recording_dir)
    # Keep live poses on tmpfs so recording capacity does not gate pose delivery.
    est = os.path.join(a.estimate_dir, os.path.basename(prefix) + ".est.txt")
    free_gb = shutil.disk_usage(a.recording_dir).free / 1e9
    record = free_gb >= a.min_free_gb
    log(f"run {n}: {prefix}, reset counter {sender.reset_counter}")
    link.say(SEV_INFO if record else SEV_WARNING,
             f"VIO run {n} " + (f"recording, {free_gb:.0f} GB free" if record
                                else f"NOT recording: {free_gb:.0f} GB free"))
    # Pin each child to the settings loaded by this supervisor, even if the
    # source config is edited before a restart. The snapshot also supports replay.
    config_path = prefix + ".flight.json"
    save_config(a, config_path)
    with open(prefix + ".log", "w") as out:
        child = subprocess.Popen(vio_cmd + [prefix, "--config", config_path, "--est", est] + ([] if record else ["--no-record"]),
                                 stdout=out, stderr=subprocess.STDOUT)
    output, poses = mavlink_bridge.Tail(prefix + ".log"), mavlink_bridge.Tail(est)
    got = sent = held = 0
    aligned, align_t, stop_t = False, None, None
    warned = set()

    def echo(lines):
        for line in lines:
            print(line.rstrip("\n"), flush=True)
            st = status_of(line)
            if st:
                link.say(*st)

    while child.poll() is None:
        link.poll()
        echo(output.poll())
        for line in poses.poll():
            got += 1
            # Every restart creates an arbitrary yaw frame. Withhold its poses
            # from an armed FC until disarmed alignment has been acknowledged.
            if aligned or link.disarmed():
                sent += sender.line(line)
            else:
                held += 1
                why = "FC armed" if link.fresh() else "no FC heartbeat"
                if why not in warned:
                    warned.add(why)
                    link.say(SEV_CRITICAL, f"VIO run {n} unaligned, {why}: poses held")
        if not aligned and sent and link.disarmed() and (align_t is None or time.monotonic() - align_t > 2):
            link.aux(VISODOM_ALIGN, SWITCH_HIGH)
            align_t = time.monotonic()
            log("Viso Align requested")
        for ack in link.take_acks(MAV_CMD_DO_AUX_FUNCTION):
            if ack.result != MAV_RESULT_ACCEPTED:
                log(f"Viso Align refused: MAV_RESULT {ack.result}")
            elif align_t is not None and not aligned:
                aligned = True
                log("Viso Align accepted: this run's poses now go out armed or not")
        report = sender.report()
        if report and got:
            print(report + (f" | {held} held" if held else ""), flush=True)
        if STOP["sig"] is not None and stop_t is None:
            stop_t = time.monotonic()
            if STOP["sig"] == signal.SIGTERM:         # a tty's Ctrl-C reached capture_session.py already
                child.send_signal(signal.SIGTERM)
            log("stopping: waiting for capture_session.py to close its recording")
        if stop_t is not None and time.monotonic() - stop_t > 80 and child.poll() is None:
            log("capture_session.py overran its stop; killing it")
            child.kill()
        time.sleep(0.005)
    echo(output.poll())
    keep(est, prefix + ".est.txt", user)
    link.say(SEV_INFO if STOP["sig"] else SEV_WARNING,
             f"VIO run {n} ended: exit {child.returncode}, {sent} poses")
    return sent


def fly(a, link, user, vio_cmd):
    """One capture_session.py run after another, each once the FC is there, until STOP."""
    os.makedirs(a.estimate_dir, exist_ok=True)
    counter = os.path.join(a.estimate_dir, "reset_counter")
    sender = mavlink_bridge.Sender(link.conn, a.tilt_deg, a.upside_down, a.send_velocity, a.max_lag,
                                    reset_counter=load_counter(counter))
    n, backoff, waiting = 0, 5.0, False
    while STOP["sig"] is None:
        link.poll()
        if not link.fresh():
            if not waiting:
                log(f"waiting for the FC's heartbeat on {a.device}")
                waiting = True
            time.sleep(0.1)
            continue
        waiting = False
        n += 1
        if n > 1:
            # Tell the FC the estimator frame reset, rather than presenting a jump.
            sender.new_run()
        with open(counter, "w") as f:
            f.write(str(sender.reset_counter))
        sent = run(a, link, sender, user, vio_cmd, n)
        # A run that never streamed failed at startup (too dark, IMU, camera):
        # retry, but back off rather than fill ~/vio with attempts.
        wait = 2.0 if sent else backoff
        backoff = 5.0 if sent else min(60.0, 2 * backoff)
        t_end = time.monotonic() + wait
        while STOP["sig"] is None and time.monotonic() < t_end:
            link.poll()
            time.sleep(0.1)
    log("stopped")


def main():
    user = pwd.getpwnam(os.environ.get("SUDO_USER") or pwd.getpwuid(os.getuid()).pw_name)
    a = parse_options("flight", user.pw_dir)
    if os.geteuid() != 0:
        sys.exit("FAIL: capture_session.py needs root for the IMU: run with sudo (vio@.service does)")
    if not os.path.isdir(a.recording_dir):
        os.makedirs(a.recording_dir)
        os.chown(a.recording_dir, user.pw_uid, user.pw_gid)
    # vio@.service runs this as root, and pymavlink is usually a --user install of
    # the account that ran mavlink_bridge.py by hand: look there too.
    site = os.path.join(user.pw_dir, ".local", "lib", "python%d.%d" % sys.version_info[:2], "site-packages")
    if os.path.isdir(site):
        sys.path.append(site)
    os.environ.setdefault("MAVLINK20", "1")          # reset_counter is a MAVLink 2 extension
    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit(f"FAIL: no pymavlink for root or {user.pw_name}: pip install pymavlink as {user.pw_name}")
    conn = mavutil.mavlink_connection(a.device, baud=a.baud, source_system=1, source_component=197)
    log(f"MAVLink {a.device} @ {a.baud}; runs in {a.recording_dir}; camera {a.tilt_deg:g} deg nose-down, "
        f"{'upside-down' if a.upside_down else 'upright'}")
    signal.signal(signal.SIGTERM, lambda s, f: STOP.update(sig=s))
    signal.signal(signal.SIGINT, lambda s, f: STOP.update(sig=s))
    vio_cmd = [sys.executable, "-u", os.path.join(HERE, "capture_session.py")]
    fly(a, Link(conn), user, vio_cmd)


if __name__ == "__main__":
    main()
