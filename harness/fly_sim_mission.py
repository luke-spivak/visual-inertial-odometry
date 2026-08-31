#!/usr/bin/env python3
"""
Fly a fixed mission in ArduPilot SITL via pymavlink, for VIO evaluation runs.

Deterministic and scriptable, unlike driving MAVProxy by hand -- the same
trajectory every run, which matters because run-to-run ATE varies enough
(see PROJECT.md "Validated baseline") that comparing different flights is
meaningless.

Usage: fly_sim_mission.py [--conn tcp:127.0.0.1:5760] [--alt 10] [--side 20]
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil


def wait_param(m, name, value, tries=8):
    for _ in range(tries):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        if msg and msg.param_id.strip("\x00") == name:
            return True
        time.sleep(0.5)
    return False


def set_mode(m, mode):
    m.set_mode(m.mode_mapping()[mode])
    for _ in range(20):
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg and mavutil.mode_string_v10(msg) == mode:
            return True
    return False


def arm(m, timeout=60):
    deadline = time.time() + timeout
    while time.time() < deadline:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                and ack.result == 0:
            return True
        time.sleep(2)
    return False


def goto_local(m, north, east, down, settle=12.0):
    """SET_POSITION_TARGET_LOCAL_NED, position only."""
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,          # position only
        north, east, down, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(settle)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="tcp:127.0.0.1:5760")
    ap.add_argument("--alt", type=float, default=10.0)
    ap.add_argument("--side", type=float, default=20.0)
    args = ap.parse_args()

    print(f"[mission] connecting to {args.conn}", flush=True)
    m = mavutil.mavlink_connection(args.conn)
    m.wait_heartbeat()
    print(f"[mission] heartbeat from sys {m.target_system}", flush=True)

    # FRAME_CLASS/TYPE must be set or pre-arm fails with
    # "Motors: Check frame class and type". Quad / X.
    wait_param(m, "FRAME_CLASS", 1)
    wait_param(m, "FRAME_TYPE", 1)

    print("[mission] waiting for EKF / GPS", flush=True)
    time.sleep(25)

    if not set_mode(m, "GUIDED"):
        print("[mission] FAILED to enter GUIDED", file=sys.stderr)
        return 1
    print("[mission] GUIDED", flush=True)

    if not arm(m):
        print("[mission] FAILED to arm", file=sys.stderr)
        return 1
    print("[mission] armed", flush=True)

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, args.alt)
    print(f"[mission] takeoff to {args.alt} m", flush=True)
    time.sleep(20)

    # Square, then a diagonal. Translation in several directions gives the
    # estimator observability it will not get from a pure hover.
    d = -args.alt
    s = args.side
    for n, e in [(s, 0), (s, s), (0, s), (0, 0), (s, s), (0, 0)]:
        print(f"[mission] goto N={n} E={e}", flush=True)
        goto_local(m, n, e, d)

    print("[mission] landing", flush=True)
    set_mode(m, "LAND")
    time.sleep(25)
    print("[mission] done", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
