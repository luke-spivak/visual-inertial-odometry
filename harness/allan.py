#!/usr/bin/env python3
"""
allan.py -- overlapping Allan deviation from an imu_log.py capture, reduced to
the four numbers the estimator config actually wants.

    ./allan.py ~/imu/run1.json
    ./allan.py ~/imu/run1.json --plot run1.png

Reads the json sidecar, so it runs equally well on the Pi or on the Mac after
scp'ing the three files back.

What comes out:
  noise density N   = sigma at tau = 1 s, off a slope -1/2 fit
                      -> kalibr {accelerometer,gyroscope}_noise_density
  bias instability B = min(sigma) / 0.664          -- reported, not consumed by kalibr
  random walk K     = sigma at tau = 3 s, off a slope +1/2 fit
                      -> kalibr {accelerometer,gyroscope}_random_walk

Three checks that matter more than the numbers:

  * Gaps. A stationary run is only Allan-analysable where it is contiguous. A
    loose connector or a scheduling stall splits the record, and a gap treated
    as continuous shows up as bias instability that is really a wiring fault.
    This uses the longest contiguous segment and says how much it threw away.

  * Short-tau slope. White noise is -1/2. If the measured slope is nearer -1
    the run is quantisation-limited, meaning the full-scale range was set too
    wide for the noise floor, and N is an artefact of the ADC step.

  * Trusted tau ceiling. Confidence collapses as the sample count per tau
    falls, so nothing beyond T/10 is used, and B is disbelieved if the minimum
    sits at the edge of that.

Worth keeping in proportion: PROJECT.md:634 records correcting the sim IMU
densities moving ATE from 399.5 m to 404.6 m, i.e. nothing. These are inputs to
fill in, not a knob to tune. It is also standard to inflate them 5-10x before
flight, because a bench run contains no vibration and no thermal transient.
"""

import argparse
import json
import os
import sys

import numpy as np

AXES = ("x", "y", "z")


def load(sidecar_path, kind):
    with open(sidecar_path) as f:
        side = json.load(f)
    meta = side["devices"][kind]

    path = meta["file"]
    if not os.path.exists(path):  # sidecar moved between machines
        path = os.path.join(os.path.dirname(os.path.abspath(sidecar_path)),
                            os.path.basename(path))
    if not os.path.exists(path):
        sys.exit(f"cannot find {meta['file']} (or its basename next to the sidecar)")

    names, formats, offsets = [], [], []
    for c in meta["channels"]:
        if c["bits"] != c["storage"] * 8 or c["shift"]:
            sys.exit(f"{kind}: channel {c['name']} is {c['bits']}b in {c['storage']*8}b "
                     f"shifted {c['shift']} -- this reader assumes packed full-width")
        names.append(c["name"])
        formats.append(("<" if c["endian"] == "le" else ">")
                       + ("i" if c["signed"] else "u") + str(c["storage"]))
        offsets.append(c["offset"])

    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                   "itemsize": meta["record_bytes"]})
    return meta, np.fromfile(path, dtype=dt)


def longest_contiguous(ts_ns, nominal_dt_s, tolerance=3.0):
    """Index slice of the longest run with no sample-interval blowout."""
    d = np.diff(ts_ns.astype(np.int64)) * 1e-9
    breaks = np.flatnonzero((d > nominal_dt_s * tolerance) | (d <= 0))
    edges = np.concatenate(([0], breaks + 1, [len(ts_ns)]))
    lengths = np.diff(edges)
    i = int(np.argmax(lengths))
    return int(edges[i]), int(edges[i + 1]), len(breaks), (float(d.max()) if len(d) else 0.0)


def adev(x, dt, n_taus=120):
    """Overlapping Allan deviation of a rate signal."""
    n = len(x)
    theta = np.concatenate(([0.0], np.cumsum(x, dtype=np.float64) * dt))
    m_max = n // 10                       # confidence ceiling: tau <= T/10
    ms = np.unique(np.geomspace(1, max(m_max, 2), n_taus).astype(np.int64))
    ms = ms[ms <= m_max]

    taus, sigmas = [], []
    for m in ms:
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        if len(d) < 2:
            continue
        tau = m * dt
        taus.append(tau)
        sigmas.append(np.sqrt(np.sum(d * d) / (2.0 * tau * tau * len(d))))
    return np.array(taus), np.array(sigmas)


def fixed_slope_fit(taus, sigmas, slope, at_tau, tols=(0.15, 0.25, 0.35)):
    """Fit log(sigma) = slope*log(tau) + c over the widest window whose local
    slope stays near `slope`, then evaluate at `at_tau`.

    The tolerance widens until a window of at least three points exists, and the
    tolerance that succeeded is returned with the window: a fit that needed 0.35
    is a weaker claim than one found at 0.15, and that should be visible rather
    than buried. Returns None only when no window is found at any tolerance --
    an honest 'the run does not show this process'."""
    lt, ls = np.log10(taus), np.log10(sigmas)
    local = np.gradient(ls, lt)

    for tol in tols:
        ok = np.abs(local - slope) < tol
        best, run_start = (0, 0, 0), None
        for i, good in enumerate(np.append(ok, False)):
            if good and run_start is None:
                run_start = i
            elif not good and run_start is not None:
                if i - run_start > best[0]:
                    best = (i - run_start, run_start, i)
                run_start = None
        if best[0] >= 3:
            _, a, b = best
            c = np.mean(ls[a:b] - slope * lt[a:b])
            value = 10.0 ** (slope * np.log10(at_tau) + c)
            return value, (float(taus[a]), float(taus[b - 1]), tol)
    return None, None


def clip(arr, start_s, end_s):
    """Keep only samples in [start, end) seconds from the first sample.

    Exists because a bench run can be disturbed at a known moment -- someone
    walks past -- and that is not a timestamp gap, so nothing else in here would
    catch it. Trimming is a claim about the data, so it is a flag you pass and a
    line the report prints, never something applied quietly."""
    if start_s is None and end_s is None:
        return arr
    t = (arr["in_timestamp"].astype(np.int64) - int(arr["in_timestamp"][0])) * 1e-9
    keep = np.ones(len(arr), dtype=bool)
    if start_s is not None:
        keep &= t >= start_s
    if end_s is not None:
        keep &= t < end_s
    return arr[keep]


def analyse(kind, meta, arr, unit):
    prefix = "in_accel_" if kind == "accel" else "in_anglvel_"
    ts = arr["in_timestamp"]
    odr = meta["odr_hz"]
    dt_nom = 1.0 / odr

    a, b, n_breaks, max_gap = longest_contiguous(ts, dt_nom)
    seg = arr[a:b]
    ts_seg = seg["in_timestamp"].astype(np.int64)
    span = (ts_seg[-1] - ts_seg[0]) * 1e-9
    dt = span / (len(seg) - 1)

    print(f"\n=== {kind}  ({meta['name']}) ===")
    print(f"  samples          {len(arr)} total, {len(seg)} in longest contiguous segment "
          f"({100*len(seg)/max(len(arr),1):.2f}%)")
    print(f"  duration         {span/3600:.3f} h   measured rate {1/dt:.3f} Hz "
          f"(nominal {odr})")
    print(f"  gaps > 3x dt     {n_breaks}   largest interval {max_gap*1e3:.2f} ms")
    print(f"  trusted tau      up to {span/10:.1f} s")
    if n_breaks:
        print(f"  NOTE: {n_breaks} gap(s). Check the connector before trusting bias instability.")

    # A channel that never moves is not a quiet sensor, it is a disconnected one.
    # Catch it here rather than letting log10(0) turn into a nan in the fits.
    dead = []
    for ax in AXES:
        raw = seg[prefix + ax]
        if len(np.unique(raw)) < 3:
            dead.append(f"{ax} (raw stuck at {np.unique(raw)[:3].tolist()})")
    if dead:
        print(f"\n  DEAD CHANNEL(S): {', '.join(dead)}")
        print("  Fewer than 3 distinct LSB values across the whole run. Either the")
        print("  full-scale range is so wide the noise floor is under one LSB, or")
        print("  the axis is not actually being read -- check MISO/CS wiring first.")
        return {ax: dict(N=None, B=None, K=None, tau_B=0.0, short_slope=float("nan"),
                         taus=np.array([]), sigmas=np.array([]),
                         N_win=None, K_win=None) for ax in AXES}

    results = {}
    print(f"\n  {'axis':4}  {'N (1s)':>12}  {'B = min/0.664':>14}  {'tau_B':>8}  "
          f"{'K (3s)':>12}  {'slope@short':>11}")
    for ax in AXES:
        x = seg[prefix + ax].astype(np.float64) * meta["scale"]
        x -= x.mean()                    # remove static bias (and gravity, on accel z)
        taus, sig = adev(x, dt)
        if not np.all(sig > 0):
            keep = sig > 0
            taus, sig = taus[keep], sig[keep]
        if len(taus) < 4:
            results[ax] = dict(N=None, B=None, K=None, tau_B=0.0,
                               short_slope=float("nan"), taus=taus, sigmas=sig,
                               N_win=None, K_win=None)
            print(f"  {ax:4}  {'--':>12}  {'--':>14}  {'--':>8}  {'--':>12}  {'--':>11}"
                  f"   (degenerate: sigma collapsed to zero)")
            continue

        N, N_win = fixed_slope_fit(taus, sig, -0.5, 1.0)
        K, K_win = fixed_slope_fit(taus, sig, +0.5, 3.0)
        i_min = int(np.argmin(sig))
        B, tau_B = sig[i_min] / 0.664, taus[i_min]

        lt, ls = np.log10(taus), np.log10(sig)
        short = float(np.mean(np.gradient(ls, lt)[:max(3, len(taus)//10)]))

        results[ax] = dict(N=N, B=B, K=K, tau_B=tau_B, short_slope=short,
                           taus=taus, sigmas=sig, N_win=N_win, K_win=K_win)
        print(f"  {ax:4}  {('%.4e' % N) if N else '   --':>12}  {B:>14.4e}  "
              f"{tau_B:>8.1f}  {('%.4e' % K) if K else '   --':>12}  {short:>11.2f}")

    print(f"  units: N [{unit}/sqrt(Hz)]   B [{unit}]   K [{unit}*sqrt(Hz)]   "
          f"tau_B [s]")

    def win(w):
        return f"{w[0]:.2f}-{w[1]:.0f}s @tol {w[2]:.2f}" if w else "none"
    for ax in AXES:
        print(f"  {ax} fit windows:  -1/2 over {win(results[ax]['N_win']):24}"
              f"  +1/2 over {win(results[ax]['K_win'])}")

    # Resolution check, measured rather than inferred from the slope.
    #
    # The tempting test is "short-tau slope trends to -1, so it is quantisation-
    # limited". That is true for quantised *angle*, and false here: these are
    # quantised rate samples, and noise below one LSB is deadbanded away by the
    # rounding rather than added to. Verified on synthetic data with a known
    # answer -- 0.15 LSB of noise came back as N four times too SMALL, with the
    # short-tau slope sitting at a healthy -0.50 and nothing to see.
    #
    # Too low is the dangerous direction: the estimator is handed a sensor it
    # believes is quieter than it is. So compare the raw spread against the LSB
    # directly. Above ~2 LSB, quantisation adds under 2% of variance.
    lsb = [float(np.std(seg[prefix + ax].astype(np.float64))) for ax in AXES]
    worst_lsb = min(lsb)
    print(f"\n  resolution       {worst_lsb:.1f} LSB rms on the noisiest-limited axis "
          f"(scale {meta['scale']:.4g} {unit}/LSB)")
    if worst_lsb < 2.0:
        print(f"  WARNING: under 2 LSB of spread. The full-scale range is too wide for")
        print(f"  this noise floor, sub-LSB motion is being rounded away, and N reads")
        print(f"  LOW -- not high. Re-run at a narrower range; the part is stationary,")
        print(f"  so there is nothing to clip. N above is not usable as it stands.")
    if any(r["taus"].size and abs(r["tau_B"] - r["taus"][-1]) < 1e-9
           for r in results.values()):
        print("  WARNING: bias-instability minimum sits at the trusted-tau ceiling. "
              "The run is too short to see it; B is a lower bound.")
    return results


def emit_yaml(acc, gyr):
    def worst(res, key):
        vals = [r[key] for r in res.values() if r[key] is not None]
        return max(vals) if vals else None

    print("\n--- kalibr_imu_chain.yaml (worst axis of three) ---")
    for label, res, unit in (("accelerometer", acc, "m/s^2"), ("gyroscope", gyr, "rad/s")):
        n, k = worst(res, "N"), worst(res, "K")
        print(f"  {label}_noise_density: {n:.6e}" if n else
              f"  {label}_noise_density: <no -1/2 region found>")
        print(f"  {label}_random_walk:  {k:.6e}" if k else
              f"  {label}_random_walk:  <no +1/2 region; run longer>")
    print("\nInflate 5-10x before flight: a bench run has no vibration and no")
    print("thermal transient. Move kalibr_imu_chain.yaml and the SDF together")
    print("if these ever feed the sim (PROJECT.md:613).")


def plot(acc, gyr, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        # The numbers are the deliverable and they have already been printed.
        # Losing them to a traceback over a missing plotting library, after a
        # three-hour capture, would be absurd.
        print(f"\nno matplotlib here, skipping {path} "
              f"(the numbers above are unaffected)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax_plot, res, title, unit in ((axes[0], acc, "accelerometer", "m/s^2"),
                                      (axes[1], gyr, "gyroscope", "rad/s")):
        for ax in AXES:
            r = res[ax]
            if not r["taus"].size:
                continue
            ax_plot.loglog(r["taus"], r["sigmas"], label=ax)
        ax_plot.set_title(f"{title} Allan deviation")
        ax_plot.set_xlabel("tau [s]")
        ax_plot.set_ylabel(f"sigma [{unit}]")
        ax_plot.grid(True, which="both", alpha=0.3)
        ax_plot.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"\nplot -> {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sidecar", help="the .json written by imu_log.py")
    ap.add_argument("--plot", help="write a log-log Allan plot here")
    ap.add_argument("--start", type=float, default=None,
                    help="drop everything before this many seconds in")
    ap.add_argument("--end", type=float, default=None,
                    help="drop everything from this many seconds in (e.g. a bumped bench)")
    args = ap.parse_args()

    acc_meta, acc_arr = load(args.sidecar, "accel")
    gyr_meta, gyr_arr = load(args.sidecar, "gyro")

    if args.start is not None or args.end is not None:
        n0 = len(acc_arr)
        acc_arr = clip(acc_arr, args.start, args.end)
        gyr_arr = clip(gyr_arr, args.start, args.end)
        print(f"TRIMMED to [{args.start or 0:.0f}, "
              f"{args.end if args.end is not None else float('inf'):.0f}) s: "
              f"{len(acc_arr)}/{n0} samples kept "
              f"({100*len(acc_arr)/n0:.1f}%)")

    acc = analyse("accel", acc_meta, acc_arr, "m/s^2")
    gyr = analyse("gyro", gyr_meta, gyr_arr, "rad/s")

    emit_yaml(acc, gyr)
    if args.plot:
        plot(acc, gyr, args.plot)


if __name__ == "__main__":
    main()
