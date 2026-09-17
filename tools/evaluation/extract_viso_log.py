#!/usr/bin/env python3
"""
Pull the vision/EKF comparison out of an ArduPilot dataflash log.

This is the authoritative check for Phase 3 milestone 4. Everything else shows
what we SENT; VISP shows what ArduPilot received, whether it accepted it, and
what its own EKF thought at the same moment.

    VISP  TimeUS,RTimeUS,CTimeMS,PX,PY,PZ,R,P,Y,PErr,AErr,Rst,Ign,Q
    XKF1  TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH

The field that matters most is VISP.Ign -- ArduPilot's count of ignored vision
messages. A stream that arrives and is silently ignored looks identical to a
stream that is working, right up until you switch the EKF onto it.

    python3 extract_viso_log.py --log ~/ardupilot/logs/00000013.BIN
"""
import argparse
import math
import sys

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--out", help="write a CSV of the paired comparison")
    a = ap.parse_args()

    mlog = mavutil.mavlink_connection(a.log)
    visp, xkf1 = [], []
    counts = {}
    while True:
        m = mlog.recv_match(type=["VISP", "XKF1"])
        if m is None:
            break
        t = m.get_type()
        counts[t] = counts.get(t, 0) + 1
        if t == "VISP":
            visp.append(m)
        elif t == "XKF1" and getattr(m, "C", 0) == 0:   # primary EKF core only
            xkf1.append(m)

    print(f"VISP messages: {counts.get('VISP', 0)}")
    print(f"XKF1 messages: {counts.get('XKF1', 0)} (core 0: {len(xkf1)})")
    if not visp:
        print("\nNo VISP records. ArduPilot never received a "
              "VISION_POSITION_ESTIMATE: check that the bridge is running, that "
              "VISO_TYPE=1, and that the stream reaches the autopilot's MAVLink "
              "endpoint rather than a GCS-only port.", file=sys.stderr)
        return 1

    ign = sum(1 for m in visp if getattr(m, "Ign", 0))
    print(f"ignored by ArduPilot: {ign} of {len(visp)}"
          f"  ({100.0*ign/len(visp):.1f}%)")
    rst = {getattr(m, "Rst", 0) for m in visp}
    print(f"reset counters seen: {sorted(rst)}")

    if not xkf1:
        print("no XKF1 core-0 records to compare against", file=sys.stderr)
        return 1

    # Pair each VISP with the nearest EKF state in time.
    xs = [m.TimeUS for m in xkf1]
    rows, errs = [], []
    for m in visp:
        j = min(range(len(xs)), key=lambda k: abs(xs[k] - m.TimeUS))
        e = xkf1[j]
        d = math.dist((m.PX, m.PY, m.PZ), (e.PN, e.PE, e.PD))
        rows.append((m.TimeUS / 1e6, m.PX, m.PY, m.PZ, e.PN, e.PE, e.PD, d,
                     getattr(m, "Ign", 0)))
        errs.append(d)

    errs.sort()
    print(f"\nvision vs EKF position, {len(errs)} pairs (metres)")
    print(f"  median {errs[len(errs)//2]:.2f}   p90 {errs[int(len(errs)*0.9)]:.2f}"
          f"   max {errs[-1]:.2f}")
    print("\nA milestone-4 pass is these agreeing to roughly a metre with 0% "
          "ignored, AFTER a VISODOM_ALIGN. Before aligning, expect a large "
          "constant offset and yaw error: OpenVINS' global frame is gravity-"
          "aligned but its heading is arbitrary.")

    if a.out:
        with open(a.out, "w") as f:
            f.write("t,vis_n,vis_e,vis_d,ekf_n,ekf_e,ekf_d,err,ignored\n")
            for r in rows:
                f.write(",".join(f"{v:.6f}" for v in r) + "\n")
        print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
