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
import os
import sys
import time

from pymavlink import mavutil


def wait_param(m, name, value, tries=8, tol=1e-3, required=True):
    """Set a parameter and CONFIRM the autopilot took the value.

    The previous version returned success as soon as a PARAM_VALUE with the
    right name came back, without looking at the value. That hides two failures
    that look identical from here and are not:

      * the parameter does not exist under that name, so nothing is set
      * it exists but the value was clamped or rejected

    Both cost a whole flight. Measured: this script set `WPNAV_SPEED`, which
    ArduCopter 4.8-dev renamed to `WP_SPD`, and reported success. The vehicle
    kept flying at its 10 m/s default while the run was recorded and analysed as
    a slow flight -- the speed profiles came out identical, p90 6.30 m/s in
    both, and only a dataflash parameter dump showed why.

    A parameter that does not read back is a hard failure, because every later
    conclusion is attributed to a change that never happened.
    """
    for _ in range(tries):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.param_id.strip("\x00") != name:
                continue
            if abs(msg.param_value - float(value)) <= tol * max(1.0, abs(float(value))):
                return True
            print(f"[mission] {name}: asked {value}, autopilot reports "
                  f"{msg.param_value}", file=sys.stderr)
        time.sleep(0.5)
    msg = (f"[mission] PARAMETER NOT SET: {name}={value} never read back. "
           "Wrong name for this firmware, or rejected.")
    if required:
        raise SystemExit(msg)
    print(msg, file=sys.stderr)
    return False


def set_mode(m, mode):
    m.set_mode(m.mode_mapping()[mode])
    for _ in range(20):
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg and mavutil.mode_string_v10(msg) == mode:
            return True
    return False


def arm(m, sim_budget=60.0, wall_cap=600.0):
    """Arm, and say why not if it fails.

    ArduPilot explains every refusal in a STATUSTEXT ("PreArm: ..."), and
    without surfacing it an arm failure is a dead end that costs a dataflash
    dump to diagnose -- which is exactly what happened once here, for a
    VISO_TYPE left set by the previous run.

    The budget is SIM seconds, not wall seconds. Under software rendering this
    sim runs at RTF ~0.2, so the old wall-clock 60 s was ~12 s of vehicle time
    and could expire before pre-arm checks that legitimately take longer. The
    wall cap is only a backstop for a sim whose clock has stopped."""
    t0_sim, t0 = boot_s(m), time.time()
    seen = set()
    while time.time() - t0 < wall_cap:
        now = boot_s(m)
        if t0_sim is not None and now is not None and now - t0_sim > sim_budget:
            break
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"],
                               blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                txt = msg.text.strip() if isinstance(msg.text, str) \
                    else msg.text.decode(errors="replace").strip()
                if txt and txt not in seen:
                    seen.add(txt)
                    print(f"[mission] autopilot: {txt}", flush=True)
                continue
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                    and msg.result == 0:
                return True
    if seen:
        print("[mission] arm refused; last reasons: "
              + " | ".join(sorted(seen)[-3:]), file=sys.stderr)
    return False


def deny_gps(m):
    """Milestone 5: take GPS away mid-flight and leave the EKF on vision.

    Two steps, and the ORDER matters. Switching the EKF to source set 2 first
    means it already has a position source when GPS disappears; doing it the
    other way round leaves EKF3 with no horizontal position for a moment, which
    triggers a failsafe and tests nothing.

    Aux function 90 is EKF_POS_SOURCE: switch-low selects source set 1,
    switch-middle set 2, switch-high set 3. Set 2 was configured as ExternalNav
    during the --extnav setup and verified against GPS through milestone 4.

    Then SIM_GPS1_ENABLE=0 removes the GPS entirely. That second step is what
    makes this a real test rather than a preference: with the receiver still
    running, a source-set switch alone leaves ArduPilot able to fall back, and a
    silent fallback would look exactly like success."""
    print("[mission] switching EKF to source set 2 (ExternalNav)", flush=True)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, 0,
        90,   # EKF_POS_SOURCE
        1,    # switch middle = source set 2
        0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION:
        print(f"[mission] source-set ack result={ack.result}", flush=True)
    else:
        print("[mission] no ack for EKF_POS_SOURCE", file=sys.stderr)
    event(m, "src2_extnav")

    # Let the EKF settle on vision before removing its fallback.
    wait_for(m, lambda n, e, d: False, 4, "source switch settle")

    wait_param(m, "SIM_GPS1_ENABLE", 0)
    print("[mission] simulated GPS DISABLED -- vision only from here", flush=True)
    event(m, "gps_off")


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


EKF_LOG = None
EVENT_LOG = None


def event(m, name):
    """Timestamp a mission event on the VEHICLE's clock.

    The analysis has to know exactly when GPS went away, and it has to know it
    in the same time base as the recorded ground truth. Wall clock is useless
    here (RTF ~0.2) and "roughly after the first leg" is not a measurement."""
    t = boot_s(m)
    print(f"[mission] EVENT {name} at t_boot={t}", flush=True)
    if EVENT_LOG is not None:
        EVENT_LOG.write(f"{t if t is not None else -1:.3f},{name}\n")
        EVENT_LOG.flush()


def local_pos(m, timeout=5.0):
    """Latest LOCAL_POSITION_NED as (n, e, d), or None."""
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=timeout)
    if msg is None:
        return None
    if EKF_LOG is not None:
        EKF_LOG.write(f"{msg.time_boot_ms/1000.0:.3f},{msg.x:.4f},"
                      f"{msg.y:.4f},{msg.z:.4f}\n")
    return (msg.x, msg.y, msg.z)


class LinkDead(Exception):
    """The MAVLink connection has gone away.

    Worth its own exception because the failure is silent and expensive: when
    the peer closes a TCP link, pymavlink's recv_match returns immediately and
    prints "EOF on TCP socket" every time. A polling loop then spins as fast as
    the CPU allows. One run wrote 7.4 GB of that line and filled the disk,
    which took out the flight, the bag and the dataflash log with it."""


def wait_for(m, pred, sim_budget, what, wall_budget=None):
    """Poll vehicle state until pred(n, e, d) holds.

    Two budgets. `sim_budget` is the real one and is spent in vehicle time.
    `wall_budget` only exists so a frozen sim cannot hang the run forever --
    the ArduPilot plugin does block Gazebo's loop under some conditions, and
    then the vehicle clock stops advancing and no sim budget would ever
    expire."""
    wall_budget = wall_budget or max(900.0, sim_budget * 30)
    wall_end = time.time() + wall_budget
    sim_start, last, empty = None, None, 0
    while time.time() < wall_end:
        p = local_pos(m)
        if p is None:
            # recv_match already waited its timeout, so a rapid string of
            # empties means the link is gone rather than merely quiet.
            empty += 1
            if empty > 40:
                raise LinkDead(f"no MAVLink data while waiting for {what}")
            continue
        empty = 0
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
    ap.add_argument("--extnav", action="store_true",
                    help="Phase 3 milestone 4: configure ExternalNav as the "
                         "SECONDARY EKF source, keep GPS primary, and align the "
                         "vision frame to AHRS once airborne")
    ap.add_argument("--ready-file", metavar="PATH",
                    help="touch this once the EKF is ready, then wait for "
                         "--wait-file before arming. Lets the harness start the "
                         "estimator just before takeoff instead of leaving it "
                         "running through the whole EKF wait.")
    ap.add_argument("--wait-file", metavar="PATH",
                    help="wait for this to appear before arming")
    ap.add_argument("--ekf-log", metavar="PATH",
                    help="write LOCAL_POSITION_NED to a CSV for comparison")
    ap.add_argument("--speed", type=float, default=0.0,
                    help="WPNAV_SPEED in m/s (0 = leave at the firmware "
                         "default). Speed is an estimator variable, not just a "
                         "mission one: inter-frame image motion is "
                         "proportional to it, and KLT error grows with "
                         "displacement.")
    ap.add_argument("--gps-denied", action="store_true",
                    help="milestone 5: after the first leg, switch EKF3 to the "
                         "ExternalNav source set and disable the simulated GPS, "
                         "then fly the rest of the mission on vision alone. "
                         "Implies --extnav.")
    ap.add_argument("--events", metavar="PATH",
                    help="CSV of timestamped mission events (t_boot,name)")
    ap.add_argument("--fixed-yaw", action="store_true",
                    help="WP_YAW_BEHAVIOUR=0: hold heading through the whole "
                         "mission instead of turning to face each leg")
    args = ap.parse_args()

    # GPS denial needs the vision source configured, which is the --extnav path.
    if args.gps_denied:
        args.extnav = True

    global EKF_LOG, EVENT_LOG
    if args.events:
        EVENT_LOG = open(args.events, "w")
        EVENT_LOG.write("t_boot,event\n")
    if args.ekf_log:
        EKF_LOG = open(args.ekf_log, "w")
        EKF_LOG.write("t_boot,n,e,d\n")

    print(f"[mission] connecting to {args.conn}", flush=True)
    # pymavlink writes "EOF on TCP socket" to stdout, uncapped, on every read of
    # a closed link. Cap the damage regardless of where the loops are.
    sys.stdout.reconfigure(line_buffering=True)
    m = mavutil.mavlink_connection(args.conn)
    m.wait_heartbeat()
    print(f"[mission] heartbeat from sys {m.target_system}", flush=True)

    # FRAME_CLASS/TYPE must be set or pre-arm fails with
    # "Motors: Check frame class and type". Quad / X.
    wait_param(m, "FRAME_CLASS", 1)
    wait_param(m, "FRAME_TYPE", 1)

    # Yaw behaviour is an experiment variable, and it was an unexamined default
    # until now. ArduPilot ships WP_YAW_BEHAVIOUR=2 ("face the next waypoint"),
    # so a position target with the yaw bits masked off still makes the vehicle
    # turn ~90 deg at every corner of the square. The camera points DOWN, so
    # that yaw is a rotation of the whole image about its centre, and it happens
    # while the vehicle is stopped at the waypoint -- rotation with no
    # translation, hence no parallax, so features seen only across the turn
    # cannot be triangulated. --fixed-yaw removes that from the mission so the
    # corner can be tested as a pure change of travel direction.
    if args.speed > 0:
        # WP_SPD, not WPNAV_SPEED, and METRES per second, not centimetres.
        # ArduCopter 4.8-dev renamed the waypoint-nav parameters and converted
        # them to SI: WP_SPD / WP_ACC / WP_RADIUS_M, alongside LOIT_SPEED_MS and
        # RTL_SPEED_MS. WPNAV_SPEED does not exist at all -- confirmed against a
        # full 1380-entry dataflash parameter dump.
        #
        # The default WP_SPD is 10 m/s, and the vehicle was measured peaking at
        # 8.15 m/s. PROJECT.md's inter-frame motion figures were derived from an
        # assumed 5.8 m/s and are therefore ~1.4x optimistic.
        wait_param(m, "WP_SPD", args.speed)
        wait_param(m, "WP_ACC", max(1.0, args.speed * 0.5))
        print(f"[mission] WP_SPD={args.speed} m/s (confirmed read-back)",
              flush=True)

    wait_param(m, "WP_YAW_BEHAVIOR", 0 if args.fixed_yaw else 2)
    print(f"[mission] WP_YAW_BEHAVIOR="
          f"{0 if args.fixed_yaw else 2}"
          f"{' (heading held)' if args.fixed_yaw else ' (faces next waypoint)'}",
          flush=True)

    # Ask for the streams rather than assuming: what ArduPilot sends by default
    # over TCP is not guaranteed to include EKF_STATUS_REPORT.
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # Accept either signal. EKF_STATUS_REPORT's position flags are the direct
    # answer; a 3D GPS fix plus a local position solution is the same condition
    # observed through messages that are definitely streamed. Whichever arrives
    # first, arming below is the real gate -- ArduPilot refuses until it is
    # genuinely happy, so this wait only avoids burning the arm retries.
    if not args.extnav:
        # SITL parameters PERSIST across runs in its eeprom.bin, so a run
        # inherits whatever the previous one set. A Milestone-4 flight leaves
        # VISO_TYPE=2 behind, and ArduPilot then refuses to arm any later run
        # with "PreArm: VisOdom: not healthy" -- correctly, since with the
        # bridge off nothing is sending VISION_POSITION_ESTIMATE. Observed as a
        # whole flight lost to an unexplained arm failure.
        #
        # Every parameter this script depends on is therefore set explicitly in
        # both directions, never left at "whatever it was".
        # VISO_TYPE alone is not enough. AP_NavEKF_Source::pre_arm_check()
        # validates EVERY configured source set, not just the active one, so a
        # leftover EK3_SRC2_POSXY=6 (EXTNAV) with visual odometry now disabled
        # fails with "AHRS: EK3 sources require VisualOdom". Both halves of the
        # Milestone-4 configuration have to be undone together.
        #
        # 0 = None is the firmware default for the SRC2 set. The harness also
        # wipes SITL's eeprom.bin per run; this is the second line of defence,
        # and it documents which parameters this mission actually depends on.
        wait_param(m, "VISO_TYPE", 0)
        for name in ("EK3_SRC2_POSXY", "EK3_SRC2_POSZ", "EK3_SRC2_VELXY",
                     "EK3_SRC2_VELZ", "EK3_SRC2_YAW"):
            wait_param(m, name, 0)
        print("[mission] VISO_TYPE=0, EK3_SRC2_* cleared "
              "(no vision into ArduPilot this run)", flush=True)

    if args.extnav:
        # GPS stays PRIMARY. The whole point of this milestone is that a frame
        # error shows up as disagreement between two sources rather than as the
        # aircraft leaving; SRC2 is configured but not selected.
        print("[mission] configuring ExternalNav as secondary source", flush=True)
        # 2 = IntelT265, NOT 1 = MAVLink, and the reason is yaw alignment.
        # Both backends consume the same VISION_POSITION_ESTIMATE via the same
        # handle_pose_estimate() signature, but request_align_yaw_to_ahrs() is
        # overridden ONLY in AP_VisualOdom_IntelT265; in the base class, which
        # the MAVLink backend uses, it is an empty virtual. So under VISO_TYPE=1
        # a VISODOM_ALIGN silently aligns POSITION ONLY.
        #
        # Measured, with type 1: vision and EKF agreed to 0.14-0.29 m through
        # the entire climb, then diverged the moment the vehicle translated --
        # vision read -12.91 East where truth was +12.14 North. Same magnitude,
        # frame rotated 90 degrees, because OpenVINS' heading is arbitrary and
        # nothing had corrected it.
        #
        # The T265 backend applies no T265-specific transform to our data:
        # VISO_ORIENT defaults to ROTATION_NONE and VISO_SCALE to 1.0, and the
        # VOXL reset-jump handling is inert while the reset counter stays 0.
        wait_param(m, "VISO_TYPE", 2)          # 2 = IntelT265 (yaw align works)
        wait_param(m, "VISO_DELAY_MS", 50)
        wait_param(m, "VISO_POS_M_NSE", 0.3)
        wait_param(m, "EK3_SRC2_POSXY", 6)     # 6 = EXTNAV
        wait_param(m, "EK3_SRC2_POSZ", 6)
        wait_param(m, "EK3_SRC2_VELXY", 0)     # position only to start with
        # Yaw stays on the compass. Vision yaw is the least trustworthy part of
        # a monocular estimate and letting it drive heading confuses a frame
        # error with an estimator error, which is the opposite of the point.
        wait_param(m, "EK3_SRC2_YAW", 1)

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

    # Hand off to the harness: it starts the estimator now, and we do not arm
    # until it says it is up. A fixed sleep cannot work here -- the two sides
    # measure time differently, the harness in wall clock and this script in sim
    # time, and their ratio is whatever the renderer manages that run.
    #
    # This matters more than it looks. The estimator has no parallax while the
    # vehicle sits still, so MSCKF features never triangulate and it propagates
    # on IMU alone. Running it through the full 45 s EKF wait cost 623 m of ATE
    # against 10 m when it started shortly before takeoff.
    if args.ready_file:
        open(args.ready_file, "w").close()
        print(f"[mission] signalled EKF ready -> {args.ready_file}", flush=True)
    if args.wait_file:
        print("[mission] waiting for the estimator", flush=True)
        deadline = time.time() + 300
        while time.time() < deadline and not os.path.exists(args.wait_file):
            m.recv_match(blocking=True, timeout=1)
        if not os.path.exists(args.wait_file):
            print("[mission] estimator never signalled ready", file=sys.stderr)
            return 1
        print("[mission] estimator up", flush=True)

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

    if args.extnav:
        # VISODOM_ALIGN (aux function 80) sets the vision frame's yaw and
        # position offset from AHRS. It must happen while vision is NOT the
        # position source -- AP_VisualOdom says so in as many words -- so here,
        # hovering on GPS, is the right moment. Without it the vision stream is
        # rotated by OpenVINS' arbitrary initial heading: its frame is gravity-
        # aligned, but nothing makes its x-axis point north.
        print("[mission] VISODOM_ALIGN", flush=True)
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, 0,
            80,   # VISODOM_ALIGN
            2,    # switch position high
            0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION:
            print(f"[mission] align ack result={ack.result}", flush=True)
        else:
            print("[mission] no ack for VISODOM_ALIGN", file=sys.stderr)
        # Let the alignment settle before translating.
        wait_for(m, lambda n, e, d: False, 5, "align settle")

    # Square, then a diagonal. Translation in several directions gives the
    # estimator observability it will not get from a pure hover.
    d = -args.alt
    s = args.side
    event(m, "mission_start")
    legs = [(s, 0), (s, s), (0, s), (0, 0), (s, s), (0, 0)]
    for i, (n, e) in enumerate(legs):
        # Deny GPS after ONE leg on GPS. That first leg is the calibration
        # segment for the analysis: with GPS on, the EKF tracks truth, so it
        # fixes the rigid transform between ArduPilot's NED frame and the
        # simulator's, and everything after is measured through it. Denying
        # from the very start would leave nothing to align against.
        #
        # It also means the vehicle is TRANSLATING when GPS goes, not hovering.
        # A GPS-denied hover is nearly free -- drift is a fraction of distance
        # travelled and a hovering vehicle travels none -- so a hold test would
        # pass while proving very little.
        if args.gps_denied and i == 1:
            deny_gps(m)
        print(f"[mission] goto N={n} E={e}", flush=True)
        goto_local(m, n, e, d)
    event(m, "mission_end")

    print("[mission] landing", flush=True)
    set_mode(m, "LAND")
    wait_for(m, lambda n, e, d: d > -0.4, 120, "touchdown")
    event(m, "landed")
    if EKF_LOG is not None:
        EKF_LOG.close()
    if EVENT_LOG is not None:
        EVENT_LOG.close()
    print("[mission] done", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LinkDead as e:
        print(f"[mission] {e} -- did SITL exit? check sitl.log", file=sys.stderr)
        sys.exit(2)
