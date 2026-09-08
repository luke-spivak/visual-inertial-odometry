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
    """Extrinsics, as a 4x4. Kalibr gives T_cam_imu as nested lists; EuRoC
    gives T_BS as a flat `data` array with rows/cols. Different conventions
    (T_cam_imu vs T_BS is body->sensor) -- this returns whatever is there and
    the caller is responsible for knowing which direction it points."""
    for k in ("T_cam_imu", "T_imu_cam", "T_BS", "T_cn_cnm1"):
        if k in cam:
            v = cam[k]
            if isinstance(v, dict) and "data" in v:
                return np.array(v["data"], float).reshape(v.get("rows", 4),
                                                          v.get("cols", 4)), k
            return np.array(v, float), k
    return None, None


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
    if Tr is not None and Tf is not None:
        print(f"\n  extrinsics: {kr} vs {kf}")
        if kr != kf:
            print("  NOTE: different keys, so possibly inverse conventions. "
                  "A ~180 deg or sign-flipped answer here means convention, "
                  "not error -- check before believing it.")
        ang = rot_angle_deg(Tr[:3, :3], Tf[:3, :3])
        dt = np.linalg.norm(Tr[:3, 3] - Tf[:3, 3])
        print(f"  rotation difference     {ang:8.3f} deg")
        print(f"  translation difference  {dt * 1000:8.2f} mm")

    print()
    if ok:
        print("VERDICT: intrinsics agree within tolerance. Kalibr is "
              "configured correctly; proceed to the real target.")
    else:
        print("VERDICT: intrinsics DISAGREE. Do not calibrate the bracket "
              "until this is understood -- the usual causes are the wrong "
              "target YAML (tag size / spacing), the wrong camera model, or "
              "a bag whose topics were remapped.")
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
