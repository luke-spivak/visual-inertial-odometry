#!/usr/bin/env python3
"""
Per-leg breakdown of an estimated trajectory against ground truth.

Why this exists. The headline drift number says the estimate is bad; it does not
say WHERE it goes bad, and on this vehicle that distinction is the whole
investigation. The measured failure is not accumulating noise -- the first leg
comes out at EuRoC quality (0.3 m over 19.5 m) and then, at the first corner,
the estimate keeps travelling in its original direction while the vehicle turns.

Two errors with the same ATE need completely different fixes:

    direction error   the estimate translates the right distance the wrong way
                      -> orientation / gravity / extrinsic / yaw observability
    magnitude error   right way, wrong distance
                      -> scale, which with an IMU in the loop means bias

So this splits the flight into legs at the mission's own stop-and-turn points,
and reports each leg's displacement as a VECTOR compared to truth. A leg that
tracks well followed by a leg that points 90 deg off localises the failure to a
single corner, which no aggregate can do.

The bias columns come from OpenVINS' save_total_state file. A constant-velocity
runaway is the signature of a bad state rather than a noisy one, and an
accelerometer bias that steps at a corner is a cause; ATE is only a symptom.

    turn_diagnostic.py --gt RUN/gt.tum --est RUN/est.tum \
        [--state RUN/replay_1.0/ov_sim_estimate.txt]
"""
import argparse
import math


def load_tum(path):
    """TUM: timestamp tx ty tz qx qy qz qw"""
    out = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = [float(x) for x in line.split()]
        out.append((v[0], v[1:4], v[4:8]))
    return out


def load_state(path):
    """OpenVINS save_total_state: t q_GtoI(4) p_IinG(3) v_IinG(3) bg(3) ba(3) ..."""
    out = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = [float(x) for x in line.split()]
        if len(v) < 17:
            continue
        out.append({"t": v[0], "p": v[5:8], "v": v[8:11],
                    "bg": v[11:14], "ba": v[14:17]})
    return out


def at(series, t, key=lambda s: s[0]):
    """Nearest sample, and how far off it was."""
    best = min(series, key=lambda s: abs(key(s) - t))
    return best, abs(key(best) - t)


def speed_series(poses, win=0.25):
    """Central-difference speed, robust to the 100 Hz truth rate."""
    out = []
    for i, (t, p, _) in enumerate(poses):
        j = i
        while j + 1 < len(poses) and poses[j][0] - t < win:
            j += 1
        k = i
        while k > 0 and t - poses[k][0] < win:
            k -= 1
        dt = poses[j][0] - poses[k][0]
        if dt <= 0:
            out.append((t, 0.0, [0.0, 0.0, 0.0]))
            continue
        vel = [(poses[j][1][d] - poses[k][1][d]) / dt for d in range(3)]
        out.append((t, math.hypot(math.hypot(vel[0], vel[1]), vel[2]), vel))
    return out


def find_legs(poses, move_thresh=0.8, min_leg=1.5):
    """Split into moving segments. The mission stops and settles at every
    waypoint, so the stationary gaps are the corners."""
    sp = speed_series(poses)
    legs, start = [], None
    for t, s, _ in sp:
        if s > move_thresh and start is None:
            start = t
        elif s <= move_thresh and start is not None:
            if t - start >= min_leg:
                legs.append((start, t))
            start = None
    if start is not None and sp[-1][0] - start >= min_leg:
        legs.append((start, sp[-1][0]))
    return legs


def disp(poses, t0, t1):
    a, _ = at(poses, t0)
    b, _ = at(poses, t1)
    return [b[1][d] - a[1][d] for d in range(3)]


def yaw_of(q):
    """Yaw about world Z from a TUM quaternion (qx, qy, qz, qw), degrees."""
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def norm(v):
    return math.sqrt(sum(c * c for c in v))


def angle_between(a, b):
    na, nb = norm(a), norm(b)
    if na < 1e-6 or nb < 1e-6:
        return float("nan")
    c = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--state", help="OpenVINS save_total_state file (biases)")
    ap.add_argument("--move-thresh", type=float, default=0.8,
                    help="m/s above which the vehicle counts as flying a leg")
    a = ap.parse_args()

    gt = load_tum(a.gt)
    est = load_tum(a.est)
    if not gt or not est:
        raise SystemExit("empty trajectory")

    # The two clocks must already agree; gt_to_tum.py takes both through the
    # same reader precisely so this is a real check and not a formality.
    t0 = max(gt[0][0], est[0][0])
    t1 = min(gt[-1][0], est[-1][0])
    print(f"ground truth {gt[-1][0] - gt[0][0]:.1f} s | "
          f"estimate {est[-1][0] - est[0][0]:.1f} s | "
          f"overlap {t1 - t0:.1f} s "
          f"({100 * (t1 - t0) / (gt[-1][0] - gt[0][0]):.0f} % of the flight)")
    if t1 <= t0:
        raise SystemExit("no temporal overlap -- rebase the bag first")

    legs = [(s, e) for s, e in find_legs(gt, a.move_thresh)
            if e > t0 and s < t1]
    print(f"\n{len(legs)} moving legs (split at the mission's own stop points)\n")

    hdr = (f"{'leg':>3} {'t0':>7} {'t1':>7} "
           f"{'true disp (m)':>22} {'est disp (m)':>22} "
           f"{'|true|':>7} {'|est|':>7} {'scale':>6} {'dir err':>8}")
    print(hdr)
    print("-" * len(hdr))
    for i, (s, e) in enumerate(legs):
        s, e = max(s, t0), min(e, t1)
        dt_ = disp(gt, s, e)
        de = disp(est, s, e)
        nt, ne = norm(dt_), norm(de)
        scale = ne / nt if nt > 1e-6 else float("nan")
        ang = angle_between(dt_, de)
        print(f"{i:>3} {s:7.1f} {e:7.1f} "
              f"[{dt_[0]:6.1f}{dt_[1]:7.1f}{dt_[2]:6.1f}] "
              f"[{de[0]:6.1f}{de[1]:7.1f}{de[2]:6.1f}] "
              f"{nt:7.1f} {ne:7.1f} {scale:6.2f} {ang:7.1f}d")

    print("\n  scale   = |estimated displacement| / |true displacement| (1.00 is perfect)")
    print("  dir err = angle between them. A leg that keeps the scale but loses")
    print("            the direction is an orientation failure, not drift.")

    # The mission never sets WP_YAW_BEHAVIOUR, so ArduPilot's default applies
    # and the vehicle turns to face each new leg. With a DOWNWARD camera that
    # rotates the whole image about its centre, and it happens while the
    # vehicle is stopped at the waypoint -- rotation without translation, which
    # yields no parallax, so features seen only during the turn cannot be
    # triangulated and the filter has nothing but the IMU across the corner.
    # If the estimate fails to follow a yaw the gyro measured directly, the
    # failure is in the filter, not the scene.
    print("\ncorners: rotation between legs (the camera is downward-facing, so"
          "\n         vehicle yaw is image roll)")
    h3 = f"{'gap':>5} {'t0':>7} {'t1':>7} {'dwell':>6} {'true dyaw':>10} {'est dyaw':>9} {'err':>7}"
    print(h3)
    print("-" * len(h3))
    for i in range(len(legs) - 1):
        s_, e_ = legs[i][1], legs[i + 1][0]
        (ga, _), (gb, _) = at(gt, s_), at(gt, e_)
        (ea, da), (eb, db) = at(est, s_), at(est, e_)
        dy_t = wrap180(yaw_of(gb[2]) - yaw_of(ga[2]))
        if max(da, db) > 0.5:
            print(f"{i:>5} {s_:7.1f} {e_:7.1f} {e_ - s_:6.1f} {dy_t:9.1f}d"
                  f"{'  (no est)':>10}")
            continue
        dy_e = wrap180(yaw_of(eb[2]) - yaw_of(ea[2]))
        print(f"{i:>5} {s_:7.1f} {e_:7.1f} {e_ - s_:6.1f} "
              f"{dy_t:9.1f}d {dy_e:8.1f}d {wrap180(dy_e - dy_t):6.1f}d")
    print("\n  A large true dyaw with a small est dyaw means the estimate did not"
          "\n  turn with the vehicle -- which is the reported failure exactly.")

    if a.state:
        st = load_state(a.state)
        if not st:
            print("\n(state file had no parsable rows)")
            return
        print(f"\nfilter state at each leg boundary  ({len(st)} rows)")
        h2 = (f"{'leg':>3} {'t':>7} {'|v| est':>8} {'|v| true':>9} "
              f"{'accel bias (m/s^2)':>26} {'gyro bias (rad/s)':>26}")
        print(h2)
        print("-" * len(h2))
        sp = speed_series(gt)
        for i, (s, e) in enumerate(legs):
            for label, t in (("in", s), ("out", e)):
                row, dtv = at(st, t, key=lambda r: r["t"])
                if dtv > 0.5:
                    continue
                tv, _ = at(sp, t, key=lambda r: r[0])
                print(f"{str(i) + label:>3} {t:7.1f} {norm(row['v']):8.2f} "
                      f"{tv[1]:9.2f} "
                      f"[{row['ba'][0]:8.3f}{row['ba'][1]:9.3f}{row['ba'][2]:8.3f}] "
                      f"[{row['bg'][0]:8.4f}{row['bg'][1]:9.4f}{row['bg'][2]:8.4f}]")
        print("\n  An accel bias that STEPS at a corner and then holds is a bad state,")
        print("  not accumulated noise: it integrates into a constant-velocity")
        print("  runaway, which is exactly the observed failure shape. Compare")
        print("  against the declared random walk (3.0e-3 m/s^3) -- a step far")
        print("  larger than the walk can explain is the filter absorbing")
        print("  something else into the bias.")


if __name__ == "__main__":
    main()
