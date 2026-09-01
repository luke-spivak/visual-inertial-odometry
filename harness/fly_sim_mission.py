#!/usr/bin/env python3
"""
Fly a fixed mission in ArduPilot SITL via pymavlink, for VIO evaluation runs.

Deterministic and scriptable, unlike driving MAVProxy by hand -- the same
trajectory every run, which matters because run-to-run ATE varies enough
(see PROJECT.md "Validated baseline") that comparing different flights is
meaningless.

Every wait here is on VEHICLE STATE, never on a wall-clock sleep. Gazebo runs
at RTF ~0.2 under software rendering, so `time.sleep(12)` buys 2.4 s of flight
and the next waypoint gets commanded while the vehicle is still most of the way
from the last one -- it chases a moving target and flies nothing like a square.
Wall-clock values below are timeouts (failure backstops), not pacing.

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


def boot_s(m):
    """The VEHICLE's clock, in seconds. This is sim time: under software
    rendering the sim runs at RTF ~0.2, so it advances five times slower than
    the wall. Budgets below are expressed in it, because 180 s of wall clock is
    31 s of flight and the EKF has not converged in 31 s."""
    for t in ("LOCAL_POSITION_NED", "ATTITUDE", "SYSTEM_TIME"):
        msg = m.messages.get(t)
        ms = getattr(msg, "time_boot_ms", None) if msg else None
        if ms:
            return ms / 1000.0
    return None


def local_pos(m, timeout=5.0):
    """Latest LOCAL_POSITION_NED as (n, e, d), or None."""
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
    return (msg.x, msg.y, msg.z) if msg else None


def wait_for(m, pred, sim_budget, what, wall_budget=None):
    """Poll vehicle state until pred(n, e, d) holds.

    Two budgets. `sim_budget` is the real one and is spent in vehicle time.
    `wall_budget` only exists so a frozen sim cannot hang the run forever --
    the ArduPilot plugin does block Gazebo's loop under some conditions, and
    then the vehicle clock stops advancing and no sim budget would ever
    expire."""
    wall_budget = wall_budget or max(900.0, sim_budget * 30)
    wall_end = time.time() + wall_budget
    sim_start, last = None, None
    while time.time() < wall_end:
        p = local_pos(m)
        if p is None:
            continue
        last = p
        if pred(*p):
            return True
        now = boot_s(m)
        if now is not None:
            if sim_start is None:
                sim_start = now
            elif now - sim_start > sim_budget:
                print(f"[mission] TIMEOUT waiting for {what} after "
                      f"{now - sim_start:.0f} s sim (last n={p[0]:.1f} "
                      f"e={p[1]:.1f} d={p[2]:.1f})", file=sys.stderr, flush=True)
                return False
    print(f"[mission] TIMEOUT waiting for {what}: wall backstop of "
          f"{wall_budget:.0f} s fired -- is the sim advancing? "
          + (f"(last n={last[0]:.1f} e={last[1]:.1f} d={last[2]:.1f})" if last
             else "(no position received at all)"),
          file=sys.stderr, flush=True)
    return False


def goto_local(m, north, east, down, tol=1.5, sim_budget=120.0, settle_s=3.0):
    """SET_POSITION_TARGET_LOCAL_NED, position only, held until the vehicle is
    actually within `tol` of the target."""
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,          # position only
        north, east, down, 0, 0, 0, 0, 0, 0, 0, 0)

    ok = wait_for(m, lambda n, e, d: math.dist((n, e, d), (north, east, down)) < tol,
                  sim_budget, f"arrival at N={north} E={east}")
    # A brief settle once there, so the estimator sees a stationary segment
    # between legs rather than a continuous slew. Sim time, via the vehicle's
    # own clock, not wall time.
    if ok:
        t0 = time.time()
        while time.time() - t0 < settle_s:
            m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    return ok


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

    # Ask for the streams rather than assuming: what ArduPilot sends by default
    # over TCP is not guaranteed to include EKF_STATUS_REPORT.
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # Accept either signal. EKF_STATUS_REPORT's position flags are the direct
    # answer; a 3D GPS fix plus a local position solution is the same condition
    # observed through messages that are definitely streamed. Whichever arrives
    # first, arming below is the real gate -- ArduPilot refuses until it is
    # genuinely happy, so this wait only avoids burning the arm retries.
    print("[mission] waiting for EKF position estimate", flush=True)
    need = (mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_PRED_POS_HORIZ_ABS)
    wall_end = time.time() + 1200
    sim_start = None
    ready = False
    while time.time() < wall_end and not ready:
        m.recv_match(blocking=True, timeout=5)
        ekf = m.messages.get("EKF_STATUS_REPORT")
        gps = m.messages.get("GPS_RAW_INT")
        ready = bool(ekf and (ekf.flags & need) == need) or bool(
            gps and gps.fix_type >= 3 and m.messages.get("LOCAL_POSITION_NED"))
        now = boot_s(m)
        if now is not None:
            if sim_start is None:
                sim_start = now
            elif now - sim_start > 180:
                break
    if not ready:
        flags = m.messages.get("EKF_STATUS_REPORT")
        print(f"[mission] no EKF position estimate after "
              f"{(boot_s(m) or 0) - (sim_start or 0):.0f} s sim "
              f"(ekf_flags={hex(flags.flags) if flags else 'never received'})",
              file=sys.stderr)
        return 1
    print(f"[mission] EKF ready at {boot_s(m):.0f} s sim", flush=True)

    # Which EKF answered matters more than how fast it answered. When lockstep
    # is off, ArduPilot quietly defaults AHRS_EKF_TYPE to 10 -- the SITL fake
    # EKF, which reads state straight from the simulator. Milestones 4 and 5
    # test what EKF3 does with vision; against type 10 they pass and prove
    # nothing. Fail loudly rather than collect a meaningless result.
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  b"AHRS_EKF_TYPE", -1)
    ekf_type = None
    for _ in range(20):
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
        if msg and msg.param_id.strip("\x00") == "AHRS_EKF_TYPE":
            ekf_type = int(msg.param_value)
            break
    if ekf_type == 10:
        print("[mission] AHRS_EKF_TYPE=10 (SITL fake EKF). Lockstep is off, so "
              "ArduPilot defaulted to it. Check <no_time_sync>0 in the model "
              "SDF and SIM_SPEEDUP=1.", file=sys.stderr)
        return 1
    print(f"[mission] AHRS_EKF_TYPE={ekf_type}", flush=True)

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
    # NED: down is negative, so "at altitude" is d <= -0.95*alt.
    if not wait_for(m, lambda n, e, d: d <= -0.95 * args.alt, 90,
                    f"climb to {args.alt} m"):
        return 1

    # Square, then a diagonal. Translation in several directions gives the
    # estimator observability it will not get from a pure hover.
    d = -args.alt
    s = args.side
    for n, e in [(s, 0), (s, s), (0, s), (0, 0), (s, s), (0, 0)]:
        print(f"[mission] goto N={n} E={e}", flush=True)
        goto_local(m, n, e, d)

    print("[mission] landing", flush=True)
    set_mode(m, "LAND")
    wait_for(m, lambda n, e, d: d > -0.4, 120, "touchdown")
    print("[mission] done", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
