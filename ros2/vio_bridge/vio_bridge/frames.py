"""
Frame conversions: ROS (ENU world, FLU body) -> MAVLink (NED world, FRD body).

Kept free of ROS and MAVLink imports so it can be tested standalone. This is
the part of the bridge most likely to be silently wrong, so it is isolated
deliberately -- see test/test_frames.py.

ROS   REP-103:  world ENU (X East,  Y North, Z Up)
                body  FLU (X Fwd,   Y Left,  Z Up)
MAVLink/AP:     world NED (X North, Y East,  Z Down)
                body  FRD (X Fwd,   Y Right, Z Down)

Both the world frame and the body frame differ, so the measured rotation is
sandwiched between two 180-degree rotations.
"""
import math

from scipy.spatial.transform import Rotation

# scipy quaternions are [x, y, z, w].
# ENU->NED: 180 deg about (1,1,0)/sqrt(2)  =>  [[0,1,0],[1,0,0],[0,0,-1]]
_ENU_TO_NED = Rotation.from_quat([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0])
# FLU->FRD: 180 deg about X                =>  diag(1,-1,-1)
_FLU_TO_FRD = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])


def position_enu_to_ned(x_enu, y_enu, z_enu):
    """(East, North, Up) -> (North, East, Down)."""
    return (y_enu, x_enu, -z_enu)


def orientation_enu_flu_to_ned_frd(qx, qy, qz, qw):
    """
    Quaternion of an FLU body in an ENU world -> (roll, pitch, yaw) radians
    of an FRD body in an NED world, i.e. aerospace convention.

    Extrinsic 'xyz' is equivalent to intrinsic ZYX, which is the
    yaw-pitch-roll decomposition MAVLink expects.
    """
    r_ned_frd = _ENU_TO_NED * Rotation.from_quat([qx, qy, qz, qw]) * _FLU_TO_FRD
    roll, pitch, yaw = r_ned_frd.as_euler("xyz")
    return (roll, pitch, yaw)
