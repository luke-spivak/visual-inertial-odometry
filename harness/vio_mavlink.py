#!/usr/bin/env python3
"""
vio_mavlink.py -- stream the Pi's live VIO pose to ArduPilot as
VISION_POSITION_ESTIMATE over the Pi-FC UART. The Pi runs VIO without ROS, so
this replaces ros2/vio_bridge on hardware; same idea, extended for a camera
that is not aligned with the airframe.

    sudo python3 vio_live.py ~/vio/f1 --no-record     # writes ~/vio/f1.est.txt
    python3 vio_mavlink.py ~/vio/f1.est.txt           # -> /dev/ttyAMA0 @ 230400
    python3 vio_mavlink.py --expect-accel             # the pre-flight IMU check

It follows the estimate file vio_live writes, one line per processed frame
("timestamp q(JPL xyzw) p v bg ba"). vio_live must flush that file per line
(vio_live.cpp does from 2026-09-11); otherwise poses arrive in 4 KB bursts,
over a second late, and are dropped here as stale.

FRAMES -- the part most likely to be silently wrong.
  G  OpenVINS global: gravity-aligned, z up, yaw arbitrary. Treated as ENU;
     the arbitrary yaw is ArduPilot's job (Viso Align, which only works under
     VISO_TYPE = 2 -- see fly_sim_mission.py).
  q  OpenVINS stores JPL q_GtoI, numerically the Hamilton quaternion of R_G_I
     (IMU vectors -> G). The sim bridge relied on the same identity.
  I  The ISM330DHCX's reported axes: the camera's optical axes (x right,
     y image-down, z out of the lens) turned 180 deg about y -- x and z
     reversed -- plus 4.4 deg. Kalibr 2026-09-12, after the IMU was re-oriented.
  B  Airframe FRD. The camera looks forward, pitched TILT nose-down, image top up.
Sent: position of the IMU in NED, attitude of B in NED. VISO_POS_X/Y/Z tells
ArduPilot where the IMU sits, so no lever arm is applied here.

The maths is unit-tested (test_vio_mavlink.py); the mounting is not, so check
it on the aircraft before trusting it:
  1. Level and still, the Pi IMU must read what --expect-accel prints.
  2. By hand: nose down -> printed pitch goes negative; right side down -> roll
     goes positive; yaw clockwise seen from above -> yaw grows.
"""
import argparse
import math
import os
import time

# Kalibr T_cam_imu rotation (IMU vectors -> camera), default IMU model,
# results/kalibr_imucam_2026-09-12-camchain-imucam.yaml: 180 deg about the
# camera's y axis + 4.4 deg. Valid only while the camera and IMU stay mounted as
# calibrated; the 2026-09-10 value (identity + 2.2 deg) died with the re-orientation.
R_CAM_IMU = (
    (-0.99983446, -0.01507096, -0.01019452),
    (-0.01578649, 0.99711912, 0.07419056),
    (0.00904703, 0.07433921, -0.99719197),
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


def expected_accel(tilt_deg, upside_down=False):
    """The Pi IMU's reading, m/s^2, with the airframe level and still."""
    return matvec(transpose(R_body_imu(tilt_deg, upside_down)), (0.0, 0.0, -GRAVITY))


def follow(path, poll_s=0.005):
    """Yield complete lines as they are appended; survive the file being recreated."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        print(f"[vio_mavlink] waiting for {path} to appear -- vio_live.py writes <prefix>.est.txt, "
              "so start it with the matching prefix", flush=True)
    while not os.path.exists(path):
        time.sleep(0.2)
    print(f"[vio_mavlink] following {path}", flush=True)
    f = open(path, "r")
    partial = ""
    last_data, warned = time.monotonic(), False
    while True:
        chunk = f.readline()
        if chunk:
            last_data, warned = time.monotonic(), False
            partial += chunk
            if partial.endswith("\n"):
                yield partial
                partial = ""
            continue
        try:
            if os.path.getsize(path) < f.tell():   # vio_live restarted: "w" truncated it
                f.close()
                f = open(path, "r")
                partial = ""
        except FileNotFoundError:
            pass
        if not warned and time.monotonic() - last_data > 5.0:
            print("[vio_mavlink] no new poses for 5 s -- vio_live writes none until its first full "
                  "update: after it says INITIALIZED, pick the aircraft up and move it a little", flush=True)
            warned = True
        time.sleep(poll_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("est", nargs="?", help="vio_live estimate file, e.g. ~/vio/f1.est.txt")
    ap.add_argument("--device", default="/dev/ttyAMA0",
                    help="serial device, or any pymavlink URL (e.g. udpout:HOST:14550)")
    ap.add_argument("--baud", type=int, default=230400,
                    help="must match SERIAL3_BAUD on the FC (230 = 230400)")
    ap.add_argument("--tilt-deg", type=float, default=30.0,
                    help="camera pitch below horizontal on the airframe, degrees")
    ap.add_argument("--upside-down", action="store_true",
                    help="camera image is upside down on the airframe (--expect-accel tells you)")
    ap.add_argument("--max-lag", type=float, default=0.5,
                    help="drop poses older than this many seconds; 0 disables (file replay)")
    ap.add_argument("--dry-run", action="store_true", help="print only; open no link")
    ap.add_argument("--expect-accel", action="store_true",
                    help="print what the Pi IMU should read level and still, then exit")
    a = ap.parse_args()

    if a.expect_accel:
        print(f"Airframe level and still, camera {a.tilt_deg:g} deg nose-down -- the Pi IMU reads:")
        for ud in (False, True):
            ax, ay, az = expected_accel(a.tilt_deg, ud)
            print(f"  {'upside-down' if ud else 'upright    '}  x {ax:+.2f}  y {ay:+.2f}  z {az:+.2f} m/s^2"
                  + ("   -> run with --upside-down" if ud else "   -> run as is"))
        print("  matching neither: the tilt is not --tilt-deg; it is atan2(|z|, |y|) degrees")
        return 0
    if not a.est:
        ap.error("give the estimate file, or --expect-accel")

    mav = None
    if not a.dry_run:
        os.environ.setdefault("MAVLINK20", "1")      # reset_counter is a MAVLink 2 extension
        from pymavlink import mavutil
        mav = mavutil.mavlink_connection(a.device, baud=a.baud,
                                         source_system=1, source_component=197)
        print(f"[vio_mavlink] MAVLink out: {a.device} @ {a.baud}", flush=True)
    print(f"[vio_mavlink] camera {a.tilt_deg:g} deg nose-down, "
          f"{'upside-down' if a.upside_down else 'upright'}", flush=True)

    no_cov = [math.nan] + [0.0] * 20                 # "unknown", per the message spec
    last_t, reset_counter, shown, announced = None, 0, None, False
    sent = dropped = 0
    lags = []
    t_report = time.monotonic()
    for line in follow(a.est):
        if line.startswith("#"):
            continue
        f = line.split()
        if len(f) < 8:
            continue
        t = float(f[0])
        if last_t is not None:
            if t < last_t:                           # a new vio_live run into the same file
                reset_counter = (reset_counter + 1) % 256
                print(f"[vio_mavlink] VIO restarted; reset_counter {reset_counter}", flush=True)
            elif t == last_t:
                continue
        last_t = t
        now = time.monotonic()
        lag = now - t                                # both CLOCK_MONOTONIC on the Pi
        if a.max_lag > 0 and lag > a.max_lag:
            dropped += 1
        else:
            (n, e, d), (roll, pitch, yaw) = pose_to_ned(
                tuple(map(float, f[1:5])), tuple(map(float, f[5:8])), a.tilt_deg, a.upside_down)
            if mav:
                mav.mav.vision_position_estimate_send(int(t * 1e6), n, e, d, roll, pitch, yaw,
                                                      no_cov, reset_counter)
            sent += 1
            lags.append(lag)
            shown = (n, e, d, roll, pitch, yaw)
            if not announced:
                print(f"[vio_mavlink] first pose: lag {1000 * lag:.0f} ms | NED {n:+.2f} {e:+.2f} {d:+.2f} m"
                      f" | roll {math.degrees(roll):+.1f} pitch {math.degrees(pitch):+.1f}"
                      f" yaw {math.degrees(yaw):+.1f} deg", flush=True)
                announced = True
        if now - t_report >= 2.0:
            msg = f"[vio_mavlink] {sent / (now - t_report):4.1f} Hz sent, {dropped} stale"
            if lags:
                lags.sort()
                msg += f" | lag {1000 * lags[len(lags) // 2]:4.0f} ms"
            elif dropped:
                msg += " | ALL STALE -- is vio_live flushing its estimate file?"
            if shown:
                n, e, d, roll, pitch, yaw = shown
                msg += (f" | NED {n:+6.2f} {e:+6.2f} {d:+6.2f} m"
                        f" | roll {math.degrees(roll):+5.1f} pitch {math.degrees(pitch):+5.1f}"
                        f" yaw {math.degrees(yaw):+6.1f} deg")
            print(msg, flush=True)
            sent = dropped = 0
            lags = []
            t_report = now


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
