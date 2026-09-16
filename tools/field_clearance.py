#!/usr/bin/env python3
"""
Will the vehicle hit anything, and how much depth will the camera actually see?

The obstacle field carries collision geometry, which makes mission altitude and
scene generation coupled in a way that is easy to miss until a 25-minute flight
ends in a crash. This answers both questions from the generated SDF, before
anything is flown:

  clearance   the tallest object near the flight path, against the altitude
  depth       the range of scene depth that lands inside the camera's frustum,
              which is the quantity the monocular planar degeneracy is about

Run it after make_feature_field.py, every time. It is cheap and the failure it
prevents is not.

    field_clearance.py sim/gazebo/models/vio_feature_field/model.sdf 10
"""
import argparse
import math
import re
import sys

# The SIM camera, which is not the flight camera. camera_info reports
# f = 381.347 at 640x400, i.e. 80 deg horizontal -- the real OV9281 is 118 deg.
# Altitude arithmetic done with the hardware number is wrong here by a lot.
FX, W, H = 381.347, 640, 400
HALF_H = math.atan(W / 2 / FX)
HALF_DIAG = math.atan(math.hypot(W / 2, H / 2) / FX)

# fly_sim_mission.py --side 20: square, then both diagonals.
DEFAULT_PATH = [(0, 0), (20, 0), (20, 20), (0, 20), (0, 0), (20, 20), (0, 0)]


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def load(path):
    """Each collision as (x, y, top_z, plan_radius). The radius is the box
    DIAGONAL half-length, not half a side: an elongated box rotated toward the
    path reaches further than its nominal width suggests, and understating that
    is how a 'capped' corridor ends up with something tall in it."""
    s = open(path).read()
    objs = []
    for pose, geom in re.findall(
            r'<collision name="c\d+">\s*<pose>([^<]+)</pose>\s*'
            r'<geometry>(.*?)</geometry>', s, re.S):
        x, y, z = [float(v) for v in pose.split()[:3]]
        b = re.search(r"<box><size>([^<]+)</size>", geom)
        c = re.search(r"<cylinder><radius>([\d.eE+-]+)</radius>"
                      r"<length>([\d.eE+-]+)</length>", geom)
        sp = re.search(r"<sphere><radius>([\d.eE+-]+)</radius>", geom)
        if b:
            sx, sy, sz = [float(v) for v in b.group(1).split()]
            top, rad = z + sz / 2, math.hypot(sx, sy) / 2
        elif c:
            r, L = float(c.group(1)), float(c.group(2))
            top, rad = z + L / 2, r
        elif sp:
            r = float(sp.group(1))
            top, rad = z + r, r
        else:
            continue
        objs.append((x, y, top, rad))
    return objs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf")
    ap.add_argument("altitudes", nargs="+", type=float)
    ap.add_argument("--min-clearance", type=float, default=1.5,
                    help="metres of vertical margin required over anything the "
                         "vehicle passes within --corridor of")
    ap.add_argument("--corridor", type=float, default=2.0,
                    help="lateral distance from the path that counts as 'under "
                         "the vehicle', allowing for position error")
    a = ap.parse_args()

    objs = load(a.sdf)
    if not objs:
        sys.exit(f"no collision geometry parsed from {a.sdf}")

    def edge(o):
        return max(min(seg_dist(o[0], o[1], *DEFAULT_PATH[i], *DEFAULT_PATH[i + 1])
                       for i in range(len(DEFAULT_PATH) - 1)) - o[3], 0.0)

    near = [o for o in objs if edge(o) < a.corridor]
    tall = max([o[2] for o in near], default=0.0)
    print(f"{len(objs)} objects, tallest {max(o[2] for o in objs):.2f} m")
    print(f"{len(near)} within {a.corridor} m of the flight path, "
          f"tallest {tall:.2f} m\n")

    bad = False
    for alt in a.altitudes:
        clear = alt - tall
        dmax = alt / math.cos(HALF_DIAG)
        shallow = dmax
        seen = 0
        for (x, y, top, rad) in objs:
            d = edge((x, y, top, rad))
            # Highest point of this object still inside the frustum.
            hvis = min(top, alt - d / math.tan(HALF_H))
            if hvis <= 0:
                continue
            seen += 1
            shallow = min(shallow, alt - hvis)
        ok = clear >= a.min_clearance
        bad = bad or not ok
        ratio = dmax / shallow if shallow > 1e-6 else float("inf")
        print(f"alt {alt:>5.1f} m  clearance {clear:+6.2f} m  "
              f"{'OK' if ok else '*** COLLISION RISK ***':<24} "
              f"depth {shallow:5.2f}..{dmax:5.2f} m = {ratio:4.1f}:1  "
              f"({seen} objects reach frame)")

    print(f"\n  depth ratio is the spread of scene distance inside the frame.")
    print("  Near 1:1 the scene is a plane and monocular depth is ill-conditioned;")
    print("  bigger is better, and it is bounded by how tall an object the")
    print("  vehicle can safely fly over -- which is why flying LOWER over a")
    print("  collidable field flattens the scene rather than enriching it.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
