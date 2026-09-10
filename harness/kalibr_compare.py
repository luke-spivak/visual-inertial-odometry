#!/usr/bin/env python3
"""
Compare a Kalibr calibration against a reference, and say whether it agrees.

This exists to run BEFORE Kalibr is ever pointed at the flight bracket. The
project's established method is to prove a tool on data with a known answer
first -- OpenVINS was validated on EuRoC V1_01_easy (ATE 0.115 m, RPE 0.72 %)
before it was trusted on the sim, and that is the only reason the sim's 97.8 %
drift was legible as a bug rather than a property of the estimator. Kalibr
deserves the same treatment, and more urgently: its output is baked into a
bracket that Phase 4 forbids disassembling, so a misconfigured Kalibr is
discovered late and expensively.

Run it on EuRoC's calibration sequences and compare against the intrinsics and
T_BS that ship in EuRoC's own `sensor.yaml`. Agreement means your Kalibr is
configured correctly. Disagreement means find out why NOW.

What "agreement" means here: the reference values are themselves the output of
a calibration, not ground truth, so exact equality is not the bar and would be
suspicious if seen. Focal lengths within ~1 % and the principal point within
~1 % of image width is a match. Distortion coefficients are reported but not
gated -- they trade off against each other and against focal length, so
individual coefficients can differ while the models agree closely in practice.

**PRINTED TARGET SIZE IS NOT REQUESTED TARGET SIZE.** When you get to your own
calibration: a target printed at 99 % scale puts a 1 % scale error directly
into the focal length, and it will not look like an error -- reprojection
error stays low because the model is internally consistent, just wrong. Print
the target, measure a tag with calipers across as many tags as you can span,
divide, and put THAT number in the target YAML.

    python3 kalibr_compare.py camchain-euroc.yaml --reference sensor.yaml
    python3 kalibr_compare.py camchain-a.yaml --reference camchain-b.yaml
"""
import argparse
import sys

try:
    import numpy as np
    import yaml
except ImportError as e:
    sys.exit(f"needs pyyaml and numpy: {e}")


def _strip_ros_yaml(path):
    """Kalibr and EuRoC both emit YAML that PyYAML will not load as-is:
    Kalibr prefixes a `%YAML:1.0` directive with an unsupported version, and
    OpenCV-flavoured files carry `!!opencv-matrix` tags. Both are cosmetic
    here -- drop them rather than pulling in an OpenCV YAML reader."""
    out = []
    for line in open(path):
        s = line.strip()
        if s.startswith("%YAML") or s == "---":
            continue
        out.append(line.replace("!!opencv-matrix", ""))
    return yaml.safe_load("".join(out))


def _first_cam(doc):
    """Accept either a Kalibr camchain (cam0:, cam1:, ...) or a EuRoC
    sensor.yaml (the camera fields sit at the top level)."""
    if "intrinsics" in doc:
        return "sensor.yaml", doc
    for k in sorted(doc):
        if isinstance(doc[k], dict) and "intrinsics" in doc[k]:
            return k, doc[k]
    sys.exit("no camera block with `intrinsics` found")


def _dist(cam):
    # Kalibr writes distortion_coeffs; EuRoC writes distortion_coefficients.
    for k in ("distortion_coeffs", "distortion_coefficients"):
        if k in cam:
            return list(cam[k])
    return []


def _T(cam):
    """Camera-IMU extrinsics as a 4x4, plus the key it came from. Kalibr gives
    T_cam_imu / T_imu_cam as nested lists; EuRoC gives T_BS as a flat `data`
    array with rows/cols. T_cn_cnm1 is deliberately not looked up: it is a
    camera-to-camera transform, a different quantity, and comparing it with a
    camera-IMU transform is a category error."""
    for k in ("T_cam_imu", "T_imu_cam", "T_BS"):
        if k in cam:
            v = cam[k]
            if isinstance(v, dict) and "data" in v:
                return np.array(v["data"], float).reshape(v.get("rows", 4),
                                                          v.get("cols", 4)), k
            return np.array(v, float), k
    return None, None


def _imu_cam(T, key):
    """Normalise to T_imu_cam: maps a point from the camera frame into the IMU
    frame.

      T_imu_cam  Kalibr, already this direction.
      T_BS       EuRoC, p_B = T_BS p_S (body <- sensor). EuRoC's body frame is
                 its IMU frame -- imu0's own T_BS is the identity -- so this is
                 T_imu_cam too.
      T_cam_imu  Kalibr's camchain-imucam output, IMU -> camera: the inverse.

    This used to compare whichever two matrices it found and print a note that
    "a ~180 deg answer means convention, not error". Tested against a perfect
    result -- T_cam_imu set to exactly inv(T_BS) -- it reported 178.3 deg and
    99 mm of error. The note's heuristic only held by accident: comparing a
    rotation with its own inverse yields twice its angle, so 178 deg is EuRoC's
    ~89 deg camera mount doubled. A sensor mounted at 45 deg would show 90 deg,
    which nobody would read as a convention problem."""
    if key == "T_cam_imu":
        return np.linalg.inv(T), "inv(T_cam_imu)"
    return T, key


def rot_angle_deg(Ra, Rb):
    """Single-axis angle of the residual rotation Ra^T Rb -- the honest
    scalar for 'how far apart are these two orientations', rather than
    comparing nine matrix entries."""
    R = Ra.T @ Rb
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def main():
    ap = argparse.ArgumentParser(
        description="Compare a Kalibr result against a reference calibration.")
    ap.add_argument("result", help="Kalibr output, e.g. camchain-....yaml")
    ap.add_argument("--reference", required=True,
                    help="reference YAML: EuRoC sensor.yaml, or another camchain")
    ap.add_argument("--focal-tol", type=float, default=1.0,
                    help="focal length tolerance, %% (default 1.0)")
    ap.add_argument("--pp-tol", type=float, default=1.0,
                    help="principal point tolerance, %% of image width (default 1.0)")
    ap.add_argument("--rot-tol", type=float, default=1.0,
                    help="camera-IMU rotation tolerance, degrees (default 1.0)")
    ap.add_argument("--trans-tol", type=float, default=10.0,
                    help="camera-IMU translation tolerance, mm (default 10)")
    a = ap.parse_args()

    rn, rc = _first_cam(_strip_ros_yaml(a.result))
    fn, fc = _first_cam(_strip_ros_yaml(a.reference))
    print(f"result    {a.result}  [{rn}]")
    print(f"reference {a.reference}  [{fn}]\n")

    for label, c in (("result", rc), ("reference", fc)):
        print(f"  {label:<10} model={c.get('camera_model')} "
              f"dist={c.get('distortion_model')} res={c.get('resolution')}")
    if rc.get("camera_model") != fc.get("camera_model"):
        print("\n  NOTE: different camera models -- intrinsics are not directly "
              "comparable.")
    print()

    ri, fi = list(rc["intrinsics"]), list(fc["intrinsics"])
    width = (rc.get("resolution") or [640])[0]
    names = ["fu", "fv", "cu", "cv"]
    ok = True

    print(f"{'':6} {'result':>12} {'reference':>12} {'diff':>10} {'':>9}")
    for i, n in enumerate(names[:min(len(ri), len(fi))]):
        d = ri[i] - fi[i]
        if n in ("fu", "fv"):
            pct = 100.0 * d / fi[i] if fi[i] else float("inf")
            good = abs(pct) <= a.focal_tol
            note = f"{pct:+.2f}%"
        else:
            pct = 100.0 * d / width
            good = abs(pct) <= a.pp_tol
            note = f"{pct:+.2f}% of W"
        ok &= good
        print(f"{n:6} {ri[i]:12.3f} {fi[i]:12.3f} {d:+10.3f} {note:>12} "
              f"{'ok' if good else 'OUT'}")

    rd, fd = _dist(rc), _dist(fc)
    if rd and fd:
        print(f"\n  distortion (reported, not gated -- coefficients trade off "
              f"against each other)")
        for i in range(min(len(rd), len(fd))):
            print(f"  k{i + 1:<5} {rd[i]:12.6f} {fd[i]:12.6f} "
                  f"{rd[i] - fd[i]:+10.6f}")

    Tr, kr = _T(rc)
    Tf, kf = _T(fc)
    compared_extrinsics = Tr is not None and Tf is not None
    if compared_extrinsics:
        Tr, lr = _imu_cam(Tr, kr)
        Tf, lf = _imu_cam(Tf, kf)
        ang = rot_angle_deg(Tr[:3, :3], Tf[:3, :3])
        dt_mm = 1000.0 * float(np.linalg.norm(Tr[:3, 3] - Tf[:3, 3]))
        rot_ok, tr_ok = ang <= a.rot_tol, dt_mm <= a.trans_tol
        ok &= rot_ok and tr_ok
        print(f"\n  extrinsics, both as T_imu_cam (camera -> IMU): {lr} vs {lf}")
        print(f"  rotation difference     {ang:8.3f} deg  {'ok' if rot_ok else 'OUT'}")
        print(f"  translation difference  {dt_mm:8.2f} mm   {'ok' if tr_ok else 'OUT'}")

    what = "intrinsics and extrinsics" if compared_extrinsics else "intrinsics"
    print()
    if ok:
        print(f"VERDICT: {what} agree within tolerance. Kalibr is "
              "configured correctly; proceed to the real target.")
    else:
        print(f"VERDICT: {what} DISAGREE. Do not calibrate the bracket "
              "until this is understood -- the usual causes are the wrong "
              "target YAML (tag size / spacing), the wrong camera model, "
              "a bag whose topics were remapped, or, for extrinsics, too "
              "little rotational excitation or a wrong IMU noise model.")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
