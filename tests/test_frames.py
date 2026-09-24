"""
Assertions on the ENU/FLU -> NED/FRD conversion.

Every case is a physical attitude described twice: once in ROS convention
(the input), once in aerospace convention (the expected output). A wrong
composition order, a missing conjugate, or a swapped quaternion convention
fails at least one of these.

Run from the repository root: python3 -m pytest tests/test_frames.py -v
"""
import math
import sys

from scipy.spatial.transform import Rotation

from vio_bridge.frames import (
    orientation_enu_flu_to_ned_frd,
    position_enu_to_ned,
)

TOL = 1e-9
DEG = math.pi / 180.0


def _quat_from_enu_flu_euler(roll_deg, pitch_deg, yaw_deg):
    """Build a ROS-convention quaternion [x,y,z,w] from ENU/FLU Euler angles."""
    r = Rotation.from_euler("xyz", [roll_deg * DEG, pitch_deg * DEG, yaw_deg * DEG])
    return r.as_quat()


def _assert_rpy(actual_rad, expected_deg, case):
    for name, a, e in zip(("roll", "pitch", "yaw"), actual_rad, expected_deg):
        assert abs(a - e * DEG) < 1e-9, (
            f"{case}: {name} expected {e} deg, got {a / DEG:.6f} deg"
        )


# ---------------------------------------------------------------- position

def test_position_east():
    """1 m East in ENU is 1 m East in NED, which is the Y axis there."""
    assert position_enu_to_ned(1.0, 0.0, 0.0) == (0.0, 1.0, 0.0)


def test_position_north():
    assert position_enu_to_ned(0.0, 1.0, 0.0) == (1.0, 0.0, 0.0)


def test_position_up():
    """Up in ENU is negative Down in NED. The sign flip is the whole point."""
    assert position_enu_to_ned(0.0, 0.0, 1.0) == (0.0, 0.0, -1.0)


def test_position_combined():
    # 3 m east, 4 m north, 2 m up  ->  4 north, 3 east, -2 down
    assert position_enu_to_ned(3.0, 4.0, 2.0) == (4.0, 3.0, -2.0)


# ------------------------------------------------------------- orientation

def test_identity_enu_flu_is_east_facing():
    """
    Identity in ENU/FLU means the nose points along ENU X, i.e. East.
    In NED that is a 90 degree yaw from North.
    """
    rpy = orientation_enu_flu_to_ned_frd(0.0, 0.0, 0.0, 1.0)
    _assert_rpy(rpy, (0.0, 0.0, 90.0), "identity/east")


def test_nose_north_level():
    """Nose north, wings level: yaw 90 in ENU becomes yaw 0 in NED."""
    q = _quat_from_enu_flu_euler(0.0, 0.0, 90.0)
    rpy = orientation_enu_flu_to_ned_frd(*q)
    _assert_rpy(rpy, (0.0, 0.0, 0.0), "nose north, level")


def test_nose_north_pitched_up():
    """
    Nose north, 20 deg nose-up.

    In FLU, +Y is LEFT, so a nose-up attitude is NEGATIVE pitch.
    In FRD, +Y is RIGHT, so the same attitude is POSITIVE pitch.
    This case fails loudly if the body-frame rotation is missing.
    """
    q = _quat_from_enu_flu_euler(0.0, -20.0, 90.0)
    rpy = orientation_enu_flu_to_ned_frd(*q)
    _assert_rpy(rpy, (0.0, 20.0, 0.0), "nose north, pitched up 20")


def test_nose_north_rolled_right():
    """
    Nose north, banked 30 deg right wing down.

    Roll is PRESERVED through the body flip: conjugating a rotation about X
    by a 180 deg rotation about X leaves it unchanged. Only yaw shifts.
    """
    q = _quat_from_enu_flu_euler(30.0, 0.0, 90.0)
    rpy = orientation_enu_flu_to_ned_frd(*q)
    _assert_rpy(rpy, (30.0, 0.0, 0.0), "nose north, rolled right 30")


def test_nose_east_level():
    """Nose east: yaw 0 in ENU becomes yaw 90 in NED."""
    q = _quat_from_enu_flu_euler(0.0, 0.0, 0.0)
    rpy = orientation_enu_flu_to_ned_frd(*q)
    _assert_rpy(rpy, (0.0, 0.0, 90.0), "nose east, level")


def test_gravity_direction_is_preserved():
    """
    Sanity check independent of Euler conventions: the world-frame direction
    of the body's 'down' axis must agree between the two representations.
    """
    q = _quat_from_enu_flu_euler(15.0, -10.0, 40.0)
    r_enu_flu = Rotation.from_quat(q)
    # body Z is UP in FLU; its ENU direction, converted to NED
    up_enu = r_enu_flu.apply([0.0, 0.0, 1.0])
    up_as_ned = position_enu_to_ned(*up_enu)

    roll, pitch, yaw = orientation_enu_flu_to_ned_frd(*q)
    r_ned_frd = Rotation.from_euler("xyz", [roll, pitch, yaw])
    # body Z is DOWN in FRD, so 'up' is -Z
    up_from_ned = r_ned_frd.apply([0.0, 0.0, -1.0])

    for a, b in zip(up_as_ned, up_from_ned):
        assert abs(a - b) < 1e-9, f"up vector mismatch: {up_as_ned} vs {up_from_ned}"


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
