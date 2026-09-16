#!/usr/bin/env python3
"""Tests for vio_mavlink: the frame maths, and the Sender and Tail that
vio_flight.py reuses. Runs with plain python3 or pytest.

The round trip builds the quaternion OpenVINS would report for a known airframe
attitude, feeds it through pose_to_ned, and requires that attitude back. That
proves the algebra is self-consistent; the physical mounting is checked on the
aircraft with --expect-accel and by hand."""
import math
import os
import tempfile
import time

import vio_mavlink as vm


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0, s), (0, 1, 0), (-s, 0, c))


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def R_to_quat(r):
    """Rotation matrix -> Hamilton (x, y, z, w)."""
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s, 0.25 * s)
    if r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        return (0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s,
                (r[2][1] - r[1][2]) / s)
    if r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        return ((r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s,
                (r[0][2] - r[2][0]) / s)
    s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
    return ((r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s,
            (r[1][0] - r[0][1]) / s)


def openvins_quat(roll, pitch, yaw, tilt_deg, upside_down=False):
    """The quaternion OpenVINS would report for this airframe attitude (NED/FRD)."""
    r_ned_b = vm.matmul(rz(yaw), vm.matmul(ry(pitch), rx(roll)))
    r_g_i = vm.matmul(vm.matmul(vm.R_NED_ENU, r_ned_b), vm.R_body_imu(tilt_deg, upside_down))
    return R_to_quat(r_g_i)


def ang_close(a, b, tol=1e-6):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b))) < tol


def test_attitude_round_trip():
    for tilt, ud in [(0.0, False), (15.0, False), (30.0, False), (30.0, True), (15.0, True)]:
        for r, p, y in [(0, 0, 0), (10, 0, 0), (0, -20, 0), (0, 0, 90),
                        (5, -10, -135), (-15, 25, 170)]:
            q = openvins_quat(math.radians(r), math.radians(p), math.radians(y), tilt, ud)
            _, (r2, p2, y2) = vm.pose_to_ned(q, (0.0, 0.0, 0.0), tilt, ud)
            assert (ang_close(r2, math.radians(r)) and ang_close(p2, math.radians(p))
                    and ang_close(y2, math.radians(y))), \
                (tilt, (r, p, y), [round(math.degrees(v), 4) for v in (r2, p2, y2)])


def test_position_enu_to_ned():
    (n, e, d), _ = vm.pose_to_ned((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0), 30.0)
    assert (n, e, d) == (2.0, 1.0, -3.0)


def test_velocity_uses_the_position_axis_swap():
    # Integrating the sent velocity must move the sent position the same way.
    p0, _ = vm.pose_to_ned((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0), 15.0)
    p1, _ = vm.pose_to_ned((0.0, 0.0, 0.0, 1.0), (1.5, 1.0, 2.0), 15.0)
    v = vm.vel_to_ned((0.5, -1.0, -1.0))              # the G displacement over 1 s
    assert all(abs(p1[i] - p0[i] - v[i]) < 1e-12 for i in range(3)), (p0, p1, v)


def test_level_accel_at_30_deg():
    ax, ay, az = vm.expected_accel(30.0)
    # cos30 g = 8.49 on -y, sin30 g = 4.90 on +z: the IMU's z is the camera's reversed
    # (Kalibr 2026-09-14), and its ~1.2 deg about the camera x axis moves each by ~0.2
    assert abs(ax) < 0.3 and abs(ay + 8.49) < 0.5 and abs(az - 4.90) < 0.8, (ax, ay, az)
    assert abs(math.sqrt(ax * ax + ay * ay + az * az) - vm.GRAVITY) < 1e-6


def _elevation_deg(a):
    """Apparent camera tilt from a level-and-still reading: gravity's angle in the y-z plane."""
    return math.degrees(math.atan2(abs(a[2]), abs(a[1])))


def _angle_deg(a, b):
    na, nb = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
    c = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def test_level_accel_upside_down_flips_y_and_splits_the_kalibr_residual():
    up, down = vm.expected_accel(15.0), vm.expected_accel(15.0, True)
    assert up[1] < -9.0 and down[1] > 9.0, (up, down)
    # Kalibr's IMU-camera residual (~1.2 deg about the camera x axis, 2026-09-14) does not
    # flip with the camera, so the apparent tilt is 15 - 1.2 upright and 15 + 1.2 upside-down.
    e_up, e_down = _elevation_deg(up), _elevation_deg(down)
    assert abs((e_up + e_down) / 2 - 15.0) < 0.3 and abs((e_down - e_up) - 2.3) < 0.5, (e_up, e_down)


def test_live_pi_reading_2026_09_14_is_upside_down_on_the_15_deg_plate():
    # On the airframe, level and still, 2026-09-14, on the new sensor plate.
    reading = (0.22, 9.20, 2.59)
    assert _angle_deg(vm.expected_accel(15.0, True), reading) < 3.0
    assert _angle_deg(vm.expected_accel(15.0), reading) > 90.0


# The 2026-09-10 rotation, identity + 2.2 deg: the IMU's mounting until it was
# re-oriented on 2026-09-12. Readings taken before then are judged against it.
R_CAM_IMU_2026_09_10 = (
    (0.9999759080976852, -0.00353080751033559, 0.005976338555862256),
    (0.0037548205026584876, 0.99927460605142, -0.03789674159834457),
    (-0.005838197256186488, 0.03791826867228832, 0.9992637891736563),
)


def test_live_pi_reading_is_an_upside_down_level_camera():
    # 2026-09-11, rig at rest: x +0.60, y +9.52, z -0.43 m/s^2. Upright cannot produce a
    # positive y at any tilt; upside-down with the lens within a few degrees of level fits.
    # Taken with the IMU in its 2026-09-10 orientation, so judged against that rotation.
    reading = (0.60, 9.52, -0.43)
    current, vm.R_CAM_IMU = vm.R_CAM_IMU, R_CAM_IMU_2026_09_10
    try:
        assert all(vm.expected_accel(t)[1] < 0 for t in range(-45, 46, 5))
        best = min(range(-20, 21), key=lambda t: _angle_deg(vm.expected_accel(t, True), reading))
        assert abs(best) <= 5 and _angle_deg(vm.expected_accel(best, True), reading) < 5.0, best
    finally:
        vm.R_CAM_IMU = current


def test_body_cam_is_a_proper_rotation():
    for tilt, ud in [(0.0, False), (30.0, False), (30.0, True)]:
        r = vm.R_body_cam(tilt, ud)
        rrt = vm.matmul(r, vm.transpose(r))
        assert all(abs(rrt[i][j] - (i == j)) < 1e-12 for i in range(3) for j in range(3))
        det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
               - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
               + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
        assert abs(det - 1) < 1e-12


def test_level_camera_at_zero_tilt_looks_forward():
    # tilt 0: the lens axis (optical z) must be body +x, image-down (optical y) body +z
    r = vm.R_body_cam(0.0)
    assert [r[i][2] for i in range(3)] == [1.0, 0.0, 0.0]
    assert [r[i][1] for i in range(3)] == [-0.0, 0.0, 1.0]


class FakeMav:
    """Records what a pymavlink connection would have sent."""

    def __init__(self):
        self.mav, self.vpe, self.vse = self, [], []

    def vision_position_estimate_send(self, usec, n, e, d, roll, pitch, yaw, cov, reset_counter):
        self.vpe.append((usec, n, e, d, reset_counter))

    def vision_speed_estimate_send(self, usec, vn, ve, vd, cov, reset_counter):
        self.vse.append((usec, vn, ve, vd, reset_counter))


def _est_line(t, p=(1.0, 2.0, 3.0), v=(0.5, -1.0, -1.0)):
    """One vio_live estimate line: t, q (JPL xyzw), p, v, bg, ba."""
    return " ".join(map(str, (t, 0.0, 0.0, 0.0, 1.0, *p, *v, 0, 0, 0, 0, 0, 0))) + "\n"


def test_sender_sends_each_pose_once_in_ned():
    mav = FakeMav()
    s = vm.Sender(mav, 15.0, True)
    now = time.monotonic()
    assert not s.line("# timestamp(s) q(JPL xyzw) p v bg ba\n")
    assert s.line(_est_line(now))
    assert not s.line(_est_line(now))                        # the same stamp again
    assert mav.vpe == [(int(now * 1e6), 2.0, 1.0, -3.0, 0)]  # ENU -> NED
    assert mav.vse == [(int(now * 1e6), -1.0, 0.5, 1.0, 0)]


def test_sender_drops_stale_poses_and_can_skip_velocity():
    mav = FakeMav()
    s = vm.Sender(mav, 15.0, velocity=False)
    assert not s.line(_est_line(time.monotonic() - 1.0))    # older than max_lag 0.5 s
    assert s.dropped == 1 and not mav.vpe
    assert s.line(_est_line(time.monotonic()))
    assert len(mav.vpe) == 1 and not mav.vse


def test_sender_marks_each_new_run_with_a_new_reset_counter():
    mav = FakeMav()
    s = vm.Sender(mav, 15.0, True)
    now = time.monotonic()
    s.line(_est_line(now))
    s.line(_est_line(now - 0.1))    # vio_live restarted into the same file: time went back
    s.new_run()                     # vio_flight.py's next run, in a new file
    s.line(_est_line(now - 0.2))
    assert [v[-1] for v in mav.vpe] == [0, 1, 2]


def test_tail_returns_whole_lines_and_survives_the_file_being_recreated():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "est.txt")
        tail = vm.Tail(p)
        assert tail.poll() == []                            # not there yet
        with open(p, "w") as f:
            f.write("a\nb")
        assert tail.poll() == ["a\n"]                       # "b" is not a line yet
        with open(p, "a") as f:
            f.write("c\n")
        assert tail.poll() == ["bc\n"]
        with open(p, "w") as f:                             # recreated, shorter
            f.write("x\n")
        assert tail.poll() == ["x\n"]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok ", name)
