#!/usr/bin/env python3
"""
mavlink_bridge.py -- stream the Pi's live VIO pose to ArduPilot as
VISION_POSITION_ESTIMATE over the Pi-FC UART. The Pi runs VIO without ROS, so
this replaces sim/ros2/vio_bridge on hardware; same idea, extended for a camera
that is not aligned with the airframe.

    sudo python3 capture_session.py ~/vio/f1 --no-record     # writes ~/vio/f1.est.txt
    python3 mavlink_bridge.py ~/vio/f1.est.txt           # -> /dev/ttyAMA0 @ 230400
    python3 mavlink_bridge.py --expect-accel             # the pre-flight IMU check

On the aircraft flight_supervisor.py runs both from boot, using the Sender below;
these are for running by hand.

It follows the estimate file vio_live writes, one line per processed frame
("timestamp q(JPL xyzw) p v bg ba"). vio_live must flush that file per line
(sensor_runner.cpp does from 2026-09-11); otherwise poses arrive in 4 KB bursts,
over a second late, and are dropped here as stale.

FRAMES -- the part most likely to be silently wrong.
  G  OpenVINS global: gravity-aligned, z up, yaw arbitrary. Treated as ENU;
     the arbitrary yaw is ArduPilot's job (Viso Align, which only works under
     VISO_TYPE = 2 -- see fly_sim_mission.py).
  q  OpenVINS stores JPL q_GtoI, numerically the Hamilton quaternion of R_G_I
     (IMU vectors -> G). The sim bridge relied on the same identity.
  I  The ISM330DHCX's reported axes: the camera's optical axes (x right,
     y image-down, z out of the lens) turned 180 deg about y -- x and z
     reversed -- plus 1.4 deg. Kalibr 2026-09-14, on the 15-deg sensor plate.
  B  Airframe FRD. The camera looks forward, pitched TILT nose-down, image top up.
Sent: position of the IMU in NED, attitude of B in NED, and the IMU's velocity
in NED as VISION_SPEED_ESTIMATE (for EK3_SRC2/3_VELZ 6; flight.json send_velocity controls it).
VISO_POS_X/Y/Z tells ArduPilot where the IMU sits, so no lever arm is applied here.

The maths is unit-tested (test_mavlink_bridge.py); the mounting is not, so check
it on the aircraft before trusting it:
  1. Level and still, the Pi IMU must read what --expect-accel prints.
  2. By hand: nose down -> printed pitch goes negative; right side down -> roll
     goes positive; yaw clockwise seen from above -> yaw grows.
"""
from cli import parse_options
import math
import os
import pwd
import time

# Kalibr T_cam_imu rotation (IMU vectors -> camera), default IMU model,
# results/kalibr_imucam_2026-09-14-camchain-imucam.yaml: 180 deg about the
# camera's y axis + 1.4 deg, on the 15-deg sensor plate. Valid only while the
# camera and IMU stay mounted as calibrated; every re-mount so far (2026-09-10,
# -12, -14) has needed a new run.
R_CAM_IMU = (
    (-0.99991866, -0.01269620, 0.00121422),
    (-0.01266913, 0.99971620, 0.02017451),
    (-0.00147001, 0.02015749, -0.99979574),
)
GRAVITY = 9.80665
# ENU <-> NED: swap x and y, negate z. Symmetric, so it is its own inverse.
R_NED_ENU = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def matvec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def quat_to_R(qx, qy, qz, qw):
    """Hamilton quaternion (x, y, z, w) -> rotation matrix."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def R_body_cam(tilt_deg, upside_down=False):
    """Camera optical frame -> body FRD for a camera pitched tilt_deg nose-down.
    Columns: optical x (image right), y (image down), z (lens axis). Upright means
    the image top is up; upside_down is the camera rolled 180 deg about its lens."""
    t = math.radians(tilt_deg)
    c, s = math.cos(t), math.sin(t)
    if upside_down:
        cols = ((0.0, -1.0, 0.0), (s, 0.0, -c), (c, 0.0, s))
    else:
        cols = ((0.0, 1.0, 0.0), (-s, 0.0, c), (c, 0.0, s))
    return tuple(tuple(col[i] for col in cols) for i in range(3))


def R_body_imu(tilt_deg, upside_down=False):
    return matmul(R_body_cam(tilt_deg, upside_down), R_CAM_IMU)


def euler_zyx(r):
    """Aerospace roll, pitch, yaw (rad) of r = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = -math.asin(max(-1.0, min(1.0, r[2][0])))
    return math.atan2(r[2][1], r[2][2]), pitch, math.atan2(r[1][0], r[0][0])


def pose_to_ned(q_jpl, p_g, tilt_deg, upside_down=False):
    """One OpenVINS pose -> ((north, east, down), (roll, pitch, yaw)) of the airframe."""
    r_g_i = quat_to_R(*q_jpl)
    r_ned_b = matmul(matmul(R_NED_ENU, r_g_i), transpose(R_body_imu(tilt_deg, upside_down)))
    return (p_g[1], p_g[0], -p_g[2]), euler_zyx(r_ned_b)


def vel_to_ned(v_g):
    """OpenVINS global-frame velocity (ENU-like, z up) -> NED, the same axis swap as
    the position. Frame-independent of the mount: it is the IMU's velocity in G."""
    return v_g[1], v_g[0], -v_g[2]


def expected_accel(tilt_deg, upside_down=False):
    """The Pi IMU's reading, m/s^2, with the airframe level and still."""
    return matvec(transpose(R_body_imu(tilt_deg, upside_down)), (0.0, 0.0, -GRAVITY))


class Tail:
    """The complete lines appended to a file since the last poll(). Survives the
    file being recreated; returns nothing until it exists."""

    def __init__(self, path):
        self.path = os.path.expanduser(path)
        self.f, self.partial = None, ""

    def _read(self):
        lines = []
        while True:
            chunk = self.f.readline()
            if not chunk:
                return lines
            self.partial += chunk
            if self.partial.endswith("\n"):
                lines.append(self.partial)
                self.partial = ""

    def poll(self):
        if self.f is None:
            if not os.path.exists(self.path):
                return []
            self.f = open(self.path, "r")
        lines = self._read()
        try:
            if os.path.getsize(self.path) < self.f.tell():   # vio_live restarted: "w" truncated it
                self.f.close()
                self.f, self.partial = open(self.path, "r"), ""
                lines += self._read()
        except FileNotFoundError:
            pass
        return lines


def follow(path, poll_s=0.005):
    """Yield complete lines as they are appended; survive the file being recreated."""
    tail = Tail(path)
    if not os.path.exists(tail.path):
        print(f"[mavlink_bridge] waiting for {tail.path} to appear -- capture_session.py writes <prefix>.est.txt, "
              "so start it with the matching prefix", flush=True)
    while not os.path.exists(tail.path):
        time.sleep(0.2)
    print(f"[mavlink_bridge] following {tail.path}", flush=True)
    last_data, warned = time.monotonic(), False
    while True:
        lines = tail.poll()
        if lines:
            last_data, warned = time.monotonic(), False
            yield from lines
            continue
        if not warned and time.monotonic() - last_data > 5.0:
            print("[mavlink_bridge] no new poses for 5 s -- vio_live writes them from INITIALIZED on "
                  "(rest it still until then); if it has said so, check it is still running", flush=True)
            warned = True
        time.sleep(poll_s)


NO_COV = [math.nan] + [0.0] * 20                     # "unknown", per the message spec


class Sender:
    """Estimate lines -> VISION_POSITION_ESTIMATE, plus the IMU's velocity as
    VISION_SPEED_ESTIMATE unless velocity is False; mav None sends nothing. Keeps
    the rate and lag figures for the two-second report. flight_supervisor.py uses it too."""

    def __init__(self, mav, tilt_deg, upside_down=False, velocity=True, max_lag=0.5, reset_counter=0):
        self.mav, self.tilt_deg, self.upside_down = mav, tilt_deg, upside_down
        self.velocity, self.max_lag, self.reset_counter = velocity, max_lag, reset_counter
        self.last_t, self.shown, self.announced = None, None, False
        self.sent = self.dropped = 0
        self.lags = []
        self.t_report = time.monotonic()

    def new_run(self):
        """What follows comes from a new vio_live run, in a frame unrelated to the
        last one's: a new reset counter makes the EKF reset rather than reject it."""
        self.reset_counter = (self.reset_counter + 1) % 256
        self.last_t, self.announced = None, False

    def line(self, line):
        """Send one estimate line. True if a pose went out."""
        if line.startswith("#"):
            return False
        f = line.split()
        if len(f) < 8:
            return False
        t = float(f[0])
        if self.last_t is not None:
            if t < self.last_t:                      # a new vio_live run into the same file
                self.reset_counter = (self.reset_counter + 1) % 256
                print(f"[mavlink_bridge] VIO restarted; reset_counter {self.reset_counter}", flush=True)
            elif t == self.last_t:
                return False
        self.last_t = t
        lag = time.monotonic() - t                   # both CLOCK_MONOTONIC on the Pi
        if self.max_lag > 0 and lag > self.max_lag:
            self.dropped += 1
            return False
        (n, e, d), (roll, pitch, yaw) = pose_to_ned(
            tuple(map(float, f[1:5])), tuple(map(float, f[5:8])), self.tilt_deg, self.upside_down)
        if self.mav:
            self.mav.mav.vision_position_estimate_send(int(t * 1e6), n, e, d, roll, pitch, yaw,
                                                       NO_COV, self.reset_counter)
            if self.velocity and len(f) >= 11:
                # The IMU's velocity in G, so no mount rotation. The FC uses what
                # the EK3_SRCn_VEL* params ask for: VELZ only, with VELXY at 0.
                vn, ve, vd = vel_to_ned(tuple(map(float, f[8:11])))
                self.mav.mav.vision_speed_estimate_send(int(t * 1e6), vn, ve, vd,
                                                        [math.nan] + [0.0] * 8, self.reset_counter)
        self.sent += 1
        self.lags.append(lag)
        self.shown = (n, e, d, roll, pitch, yaw)
        if not self.announced:
            print(f"[mavlink_bridge] first pose: lag {1000 * lag:.0f} ms | NED {n:+.2f} {e:+.2f} {d:+.2f} m"
                  f" | roll {math.degrees(roll):+.1f} pitch {math.degrees(pitch):+.1f}"
                  f" yaw {math.degrees(yaw):+.1f} deg", flush=True)
            self.announced = True
        return True

    def report(self):
        """The two-second status line, or None until it is due."""
        now = time.monotonic()
        if now - self.t_report < 2.0:
            return None
        msg = f"[mavlink_bridge] {self.sent / (now - self.t_report):4.1f} Hz sent, {self.dropped} stale"
        if self.lags:
            self.lags.sort()
            msg += f" | lag {1000 * self.lags[len(self.lags) // 2]:4.0f} ms"
        elif self.dropped:
            msg += " | ALL STALE -- is vio_live flushing its estimate file?"
        if self.shown:
            n, e, d, roll, pitch, yaw = self.shown
            msg += (f" | NED {n:+6.2f} {e:+6.2f} {d:+6.2f} m"
                    f" | roll {math.degrees(roll):+5.1f} pitch {math.degrees(pitch):+5.1f}"
                    f" yaw {math.degrees(yaw):+6.1f} deg")
        self.sent = self.dropped = 0
        self.lags = []
        self.t_report = now
        return msg


def main():
    user = pwd.getpwnam(os.environ.get("SUDO_USER") or pwd.getpwuid(os.getuid()).pw_name)
    a = parse_options("bridge", user.pw_dir)

    if a.expect_accel:
        print(f"Airframe level and still, camera {a.tilt_deg:g} deg nose-down -- the Pi IMU reads:")
        ax, ay, az = expected_accel(a.tilt_deg, a.upside_down)
        print(f"  {'upside-down' if a.upside_down else 'upright'}  "
              f"x {ax:+.2f}  y {ay:+.2f}  z {az:+.2f} m/s^2")
        return 0

    mav = None
    if not a.dry_run:
        os.environ.setdefault("MAVLINK20", "1")      # reset_counter is a MAVLink 2 extension
        from pymavlink import mavutil
        mav = mavutil.mavlink_connection(a.device, baud=a.baud,
                                         source_system=1, source_component=197)
        print(f"[mavlink_bridge] MAVLink out: {a.device} @ {a.baud}", flush=True)
    print(f"[mavlink_bridge] camera {a.tilt_deg:g} deg nose-down, "
          f"{'upside-down' if a.upside_down else 'upright'}", flush=True)

    sender = Sender(mav, a.tilt_deg, a.upside_down, a.send_velocity, a.max_lag)
    for line in follow(a.est):
        sender.line(line)
        msg = sender.report()
        if msg:
            print(msg, flush=True)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
