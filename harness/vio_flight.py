#!/usr/bin/env python3
"""
vio_flight.py -- VIO on viopi from power-on to shutdown, nobody logged in. Started
at boot by vio@.service; replaces ssh, tmux, and vio_live.py + vio_mavlink.py by hand.

    sudo cp ~/harness/vio@.service /etc/systemd/system/     # once
    sudo systemctl daemon-reload
    sudo systemctl enable --now vio@$USER
    journalctl -u vio@$USER -f                               # watch it
    sudo systemctl stop vio@$USER                            # bench work: frees camera and IMU

1. Waits for the FC's heartbeat, so a Pi on the bench with the FC unpowered
   leaves the camera and IMU alone.
2. Runs vio_live.py into ~/vio/run-<date>-<time> as by hand: exposure probe, IMU,
   estimator, recording. Recording is ~2.5 GB/min (41 MB/s of Y16) and now happens
   at every power-up, so below --min-free-gb a run goes --no-record. The estimate
   file stays on tmpfs until the run ends: poses reach the FC through it, and a
   full SD card must cost the recording, not the flight.
3. Sends each pose with vio_mavlink.py's Sender, this aircraft's mount baked in
   (camera 15 deg nose-down, image upside down).
4. Passes on what matters in vio_live.py's output -- exposure, INITIALIZED, first
   visual update, FAIL -- as STATUSTEXT, which QGC shows when connected. The
   journal has all of it.
5. Viso Align at the first pose of every run, while disarmed. The FC aligns by
   itself only at the first pose after its own boot (_align_yaw starts true in
   AP_VisualOdom_IntelT265), and a restarted run's yaw is arbitrary. Until a run
   is aligned its poses are held back from an armed aircraft. (The FC's pre-arm
   check also refuses more than 10 deg between VIO and AHRS yaw.)
6. When vio_live.py ends, starts a new run with the reset counter bumped, so the
   EKF resets to the new frame instead of rejecting it.
7. SIGTERM -- shutdown, the Pi's power button, systemctl stop -- goes on to
   vio_live.py, which stops the camera and lets vio_live close the recording.
"""
import argparse
import os
import pwd
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vio_mavlink  # noqa: E402

# MAVLink values, spelled out so this imports without pymavlink (the tests do).
SEV_CRITICAL, SEV_ERROR, SEV_WARNING, SEV_INFO = 2, 3, 4, 6
MAV_CMD_DO_AUX_FUNCTION, MAV_RESULT_ACCEPTED = 218, 0
VISODOM_ALIGN, SWITCH_HIGH = 80, 2                    # aux function 80 acts on high
MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_GCS, MAV_MODE_FLAG_SAFETY_ARMED = 3, 6, 128

STOP = {"sig": None}                                  # set by SIGTERM / SIGINT


def log(msg):
    print(f"[vio_flight] {msg}", flush=True)


def status_of(line):
    """(severity, STATUSTEXT) for a vio_live.py output line the pilot should see, else None."""
    if "FAIL:" in line:
        return SEV_ERROR, "VIO FAIL:" + line.split("FAIL:", 1)[1].rstrip()
    if "auto-exposure:" in line and "->" in line:
        return SEV_INFO, "VIO exposure " + line.rsplit("->", 1)[1].strip()
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
    """One vio_live.py run, until it ends. Returns how many poses reached the FC."""
    prefix = new_prefix(a.dir)
    est = os.path.join(a.est_dir, os.path.basename(prefix) + ".est.txt")
    free_gb = shutil.disk_usage(a.dir).free / 1e9
    record = free_gb >= a.min_free_gb
    log(f"run {n}: {prefix}, reset counter {sender.reset_counter}")
    link.say(SEV_INFO if record else SEV_WARNING,
             f"VIO run {n} " + (f"recording, {free_gb:.0f} GB free" if record
                                else f"NOT recording: {free_gb:.0f} GB free"))
    with open(prefix + ".log", "w") as out:
        child = subprocess.Popen(vio_cmd + [prefix, "--est", est] + ([] if record else ["--no-record"]),
                                 stdout=out, stderr=subprocess.STDOUT)
    output, poses = vio_mavlink.Tail(prefix + ".log"), vio_mavlink.Tail(est)
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
            if STOP["sig"] == signal.SIGTERM:         # a tty's Ctrl-C reached vio_live.py already
                child.send_signal(signal.SIGTERM)
            log("stopping: waiting for vio_live.py to close its recording")
        if stop_t is not None and time.monotonic() - stop_t > 80 and child.poll() is None:
            log("vio_live.py overran its stop; killing it")
            child.kill()
        time.sleep(0.005)
    echo(output.poll())
    keep(est, prefix + ".est.txt", user)
    link.say(SEV_INFO if STOP["sig"] else SEV_WARNING,
             f"VIO run {n} ended: exit {child.returncode}, {sent} poses")
    return sent


def fly(a, link, user, vio_cmd):
    """One vio_live.py run after another, each once the FC is there, until STOP."""
    os.makedirs(a.est_dir, exist_ok=True)
    counter = os.path.join(a.est_dir, "reset_counter")
    sender = vio_mavlink.Sender(link.conn, a.tilt_deg, not a.upright, reset_counter=load_counter(counter))
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
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=os.path.join(user.pw_dir, "vio"), help="where runs are recorded")
    ap.add_argument("--est-dir", default="/run/vio", help="tmpfs for the live estimate files")
    ap.add_argument("--device", default="/dev/ttyAMA0", help="serial device, or any pymavlink URL")
    ap.add_argument("--baud", type=int, default=230400, help="must match SERIAL3_BAUD on the FC")
    ap.add_argument("--tilt-deg", type=float, default=15.0, help="camera pitch below horizontal, degrees")
    ap.add_argument("--upright", action="store_true",
                    help="camera image upright on the airframe; this one's is upside down (--expect-accel)")
    ap.add_argument("--min-free-gb", type=float, default=20.0, help="record only with this much free space")
    ap.add_argument("--shutter", type=int, help="override camera exposure time in microseconds")
    ap.add_argument("--gain", type=float, help="analogue gain used with --shutter; default 1")
    ap.add_argument("--exposure-sweep", action="store_true",
                    help="select the brightest usable exposure at each VIO run")
    a = ap.parse_args()
    if a.gain is not None and a.shutter is None:
        ap.error("--gain requires --shutter")
    if os.geteuid() != 0:
        sys.exit("FAIL: vio_live.py needs root for the IMU: run with sudo (vio@.service does)")
    if not os.path.isdir(a.dir):
        os.makedirs(a.dir)
        os.chown(a.dir, user.pw_uid, user.pw_gid)
    # vio@.service runs this as root, and pymavlink is usually a --user install of
    # the account that ran vio_mavlink.py by hand: look there too.
    site = os.path.join(user.pw_dir, ".local", "lib", "python%d.%d" % sys.version_info[:2], "site-packages")
    if os.path.isdir(site):
        sys.path.append(site)
    os.environ.setdefault("MAVLINK20", "1")          # reset_counter is a MAVLink 2 extension
    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit(f"FAIL: no pymavlink for root or {user.pw_name}: pip install pymavlink as {user.pw_name}")
    conn = mavutil.mavlink_connection(a.device, baud=a.baud, source_system=1, source_component=197)
    log(f"MAVLink {a.device} @ {a.baud}; runs in {a.dir}; camera {a.tilt_deg:g} deg nose-down, "
        f"{'upright' if a.upright else 'upside-down'}")
    signal.signal(signal.SIGTERM, lambda s, f: STOP.update(sig=s))
    signal.signal(signal.SIGINT, lambda s, f: STOP.update(sig=s))
    vio_cmd = [sys.executable, "-u", os.path.join(HERE, "vio_live.py")]
    if a.shutter is not None:
        vio_cmd += ["--shutter", str(a.shutter), "--gain", str(a.gain if a.gain is not None else 1.0)]
    elif a.exposure_sweep:
        vio_cmd += ["--exposure-sweep"]
    fly(a, Link(conn), user, vio_cmd)


if __name__ == "__main__":
    main()
