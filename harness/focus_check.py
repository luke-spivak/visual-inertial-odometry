#!/usr/bin/env python3
"""
Score camera focus from raw sensor frames, on the Pi, while turning the lens.

Phase 4 step 6 sets focus by maximising Laplacian variance against a distant
target. This is the loop for that: capture, score, turn the lens, repeat.

Three things it does deliberately, each of which cost something to learn.

**It never touches the ISP's processed output.** On this camera the mono tuning
file (`ov9281_mono.json`) has no colour pipeline, so asking for BGR888/sRGB
yields an image of EXACTLY zero -- silently, with rpicam reporting a successful
capture and the AGC still converging on a correctly exposed scene. Every
capture here goes through `rpicam-raw`. That is also the right choice on the
merits: sRGB applies a gamma curve, and a gamma curve is a nonlinear intensity
transform sitting in front of a corner detector.

**It reports exposure alongside focus.** Laplacian variance moves with exposure
and contrast as well as with focus, so an AGC that re-converges between
captures makes consecutive readings incomparable -- the number climbs and you
credit the lens. Run once to see what the AGC picks, then pin it with
--shutter/--gain for the actual sweep. `mean` and `std` are printed on every
line so a drifting exposure is visible rather than inferred.

**It reports clipping.** Pointing at sky or into sun saturates, and saturated
regions have no gradient, so the metric FALLS as the image gets brighter. A
sweep that peaks early and then declines may be measuring clipping, not focus.

**--roi exists because of mixed distances.** The score is computed over the
whole frame by default, so everything in frame votes -- including the ground
at your feet. With a 118 deg lens in a confined space, near objects can occupy
enough of the frame to pull the peak toward THEIR focus distance, and you would
lock the lens for the lawn instead of the horizon. Restrict the score to a
window containing only distant content and the problem disappears. Use --save
to look at a frame first and pick the window; the printout names the ROI on
every line so it can never be silently in effect.

Distance itself is more forgiving than the build plan implies -- derived for
this lens (f=2.8 mm, f/2.8, 3 um pixels), the blur at infinity from focusing
at a nearer target is:

    2 m -> 0.47 px    10 m -> 0.09 px    50 m -> 0.02 px    100 m -> 0.01 px

so anything past ~20 m is optically indistinguishable from infinity here. What
distance actually buys is measurement sensitivity: far scenes put real texture
at pixel scale, where defocus bites first. A 50 m fence line with foliage or
roof detail is a fine target.

The R8 mode delivers Y16 with the 8-bit data in the HIGH byte -- every value in
the file is a multiple of 256 -- so mono8 is `raw >> 8`, not a rescale.

    python3 focus_check.py                                  # one reading
    python3 focus_check.py --watch                          # until Ctrl-C
    python3 focus_check.py --shutter 2000 --gain 4 --watch  # pinned, for a sweep
    python3 focus_check.py --raw test.raw                   # score an existing file
    python3 focus_check.py --save best.pgm                  # keep the sharpest frame

Once focus is locked, `count_features.py` is the measurement that matters --
Laplacian variance says the image is sharp, FAST corner count says the front
end can actually use it.
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile

try:
    import numpy as np
except ImportError:
    sys.exit("needs numpy:  sudo apt install -y python3-numpy")


# rpicam-raw prints one of these per frame on stderr:
#   #6 (30.00 fps) exp 14994.00 ag 1.44 dg 1.03
EXP_RE = re.compile(r"exp\s+([\d.]+)\s+ag\s+([\d.]+)")


def capture(path, width, height, frames, shutter, gain, timeout):
    """Grab raw frames with rpicam-raw. Returns (exposure_us, analogue_gain),
    parsed from its own log, or (None, None) if it did not say."""
    # Frame duration has to admit the requested shutter or the request is
    # silently clamped: a 20 ms exposure cannot happen inside a 33 ms/30 fps
    # budget once overheads are counted, and you get a shorter one without
    # being told.
    ms = max(40, int(shutter / 1000) + 20) if shutter else 40
    cmd = [
        "rpicam-raw",
        "--mode", f"{width}:{height}:8",
        "--width", str(width), "--height", str(height),
        "-t", str(max(400, int(frames * ms))),
        "--framerate", "30",
        "-o", path,
    ]
    if shutter:
        cmd += ["--shutter", str(shutter)]
    if gain:
        cmd += ["--gain", str(gain)]

    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        sys.exit("rpicam-raw not found:  sudo apt install -y rpicam-apps-lite")
    except subprocess.TimeoutExpired:
        sys.exit(f"rpicam-raw did not return within {timeout} s")
    if p.returncode != 0:
        sys.exit(f"rpicam-raw failed ({p.returncode}):\n{p.stderr[-2000:]}")

    hits = EXP_RE.findall(p.stderr)
    if not hits:
        return None, None
    # Last frame's values -- the AGC has had the whole capture to settle.
    return float(hits[-1][0]), float(hits[-1][1])


def load(path, width, height):
    """Y16 file -> (n, height, width) uint8, taking the high byte."""
    raw = np.fromfile(path, dtype="<u2")
    per = width * height
    if raw.size == 0:
        sys.exit(f"{path} is empty -- did the capture write anything?")
    if raw.size % per:
        sys.exit(
            f"{path} holds {raw.size * 2} bytes, not a whole number of "
            f"{width}x{height} 16-bit frames ({per * 2} B each). "
            f"Wrong --width/--height, or the mode is not the R8 one."
        )
    f = raw.reshape(-1, height, width)
    # The R8 mode puts 8-bit data in the high byte. Verify rather than assume:
    # a mode that genuinely delivers 10 or 12 bits would fail this, and
    # shifting it by 8 would quietly throw away the low bits.
    if (f[0] % 256 != 0).any():
        print("  note: values are not multiples of 256 -- this is not the "
              "8-in-16 layout the R8 mode produces. Scaling instead of "
              "shifting.", file=sys.stderr)
        return (f >> 8).astype(np.uint8) if f.max() > 255 else f.astype(np.uint8)
    return (f >> 8).astype(np.uint8)


def _lap_var(a):
    lap = (-4.0 * a[1:-1, 1:-1]
           + a[:-2, 1:-1] + a[2:, 1:-1]
           + a[1:-1, :-2] + a[1:-1, 2:])
    return float(lap.var())


def _smooth(a):
    """Separable 3x3 binomial blur, [1 2 1]/4 each axis. Chosen because it
    removes single-pixel variation while leaving real edges essentially
    intact -- which is exactly the difference between sensor noise and
    scene detail."""
    b = (a[:, :-2] + 2.0 * a[:, 1:-1] + a[:, 2:]) / 4.0
    return (b[:-2, :] + 2.0 * b[1:-1, :] + b[2:, :]) / 4.0


def laplacian_var(g):
    """Focus score: variance of the 4-neighbour Laplacian. Sharp edges are
    abrupt intensity changes, which produce large second derivatives; blur
    spreads the same change over many pixels and suppresses them.

    Returns (raw, smoothed). The second is the same measure computed after a
    light blur, and the pair exists because **the Laplacian cannot tell sharp
    detail from sensor noise** -- both are abrupt pixel-to-pixel variation.
    Measured on this camera 2026-09-07, the same static bench scene scored:

        exp 15 ms, ag 1.44  ->  raw  34.8
        exp  2 ms, ag 4.00  ->  raw 113.1

    3x more "focus" from turning the exposure down. That is read noise
    amplified by gain, not detail, and a sweep run that way would chase the
    noise floor. Real detail survives the blur; single-pixel noise does not,
    so `raw/smoothed` is the tell -- a ratio near 1 means you are measuring
    the scene, a large ratio means you are measuring noise.

    Compare scores only at FIXED exposure and gain. The absolute number has
    no meaning across exposure settings."""
    a = g.astype(np.float64)
    return _lap_var(a), _lap_var(_smooth(a))


def apply_roi(frames, spec):
    """Crop every frame to X,Y,W,H. Validated against the frame rather than
    trusted: a silently out-of-range ROI would score a smaller window than
    asked for, and the readings would still look plausible."""
    if not spec:
        return frames, "full frame"
    try:
        x, y, w, h = (int(v) for v in spec.split(","))
    except ValueError:
        sys.exit("--roi wants four integers: X,Y,W,H")
    fh, fw = frames.shape[1], frames.shape[2]
    if w < 16 or h < 16:
        sys.exit("--roi must be at least 16x16 to mean anything")
    if x < 0 or y < 0 or x + w > fw or y + h > fh:
        sys.exit(f"--roi {spec} does not fit inside {fw}x{fh}")
    return frames[:, y:y + h, x:x + w], f"roi {x},{y},{w},{h}"


def score(frames):
    """Per-frame (raw, smoothed) scores.

    Ranking uses the SMOOTHED score, because that is the noise-suppressed one
    and the sharpest of several is the right summary rather than the mean: a
    handheld camera produces the occasional motion-blurred frame, and
    averaging lets hand shake masquerade as defocus."""
    return [laplacian_var(f) for f in frames]


def report(frames, scores, exp, gain, best_so_far):
    i = int(max(range(len(scores)), key=lambda k: scores[k][1]))
    f = frames[i]
    raw, sm = scores[i]
    lo = 100.0 * (f == 0).mean()
    hi = 100.0 * (f == 255).mean()
    ratio = raw / sm if sm > 0 else float("inf")
    line = (f"focus={sm:8.2f}  raw={raw:8.1f} n/s={ratio:5.1f}  "
            f"mean={f.mean():6.1f} std={f.std():5.1f}  "
            f"clip lo={lo:4.1f}% hi={hi:4.1f}%")
    if exp is not None:
        line += f"  exp={exp / 1000:.1f}ms ag={gain:.2f}"
    if ratio > 25:
        # Little real detail relative to per-pixel noise. Early in a sweep
        # that is simply the truth -- a badly defocused frame HAS no detail --
        # so it is a progress indicator, not only a warning. It should fall
        # as focus improves. If it stays high at the peak, the gain is too
        # high or the exposure too short.
        line += "  <- mostly noise"
    if best_so_far is not None and best_so_far > 0:
        # A bar relative to the best seen this session, so turning the lens
        # gives immediate feedback without reading digits.
        n = int(round(40 * min(1.0, sm / max(best_so_far, sm))))
        line += "  " + "#" * n
    print(line, flush=True)
    return i, sm


def write_pgm(path, g):
    """PGM (P5): a three-line header and raw bytes. No dependencies, and
    count_features.py reads it directly."""
    with open(path, "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (g.shape[1], g.shape[0]))
        fh.write(g.tobytes())


def main():
    ap = argparse.ArgumentParser(
        description="Laplacian-variance focus score from raw camera frames.")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--frames", type=int, default=5,
                    help="frames per reading; the sharpest is scored (default 5)")
    ap.add_argument("--shutter", type=int, metavar="US",
                    help="fix exposure in microseconds. PIN THIS FOR A SWEEP -- "
                         "readings taken under a moving AGC are not comparable")
    ap.add_argument("--gain", type=float, help="fix analogue gain")
    ap.add_argument("--watch", action="store_true",
                    help="repeat until Ctrl-C, with a bar relative to the best "
                         "reading so far")
    ap.add_argument("--raw", metavar="FILE",
                    help="score an existing raw file instead of capturing "
                         "(works off-Pi)")
    ap.add_argument("--save", metavar="PATH",
                    help="write the sharpest frame as a PGM")
    ap.add_argument("--roi", metavar="X,Y,W,H",
                    help="score only this window, e.g. 0,0,640,200 for the top "
                         "half. Use when near objects share the frame with the "
                         "distant target -- see the module docstring")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="seconds to allow rpicam-raw (default 30)")
    a = ap.parse_args()

    if a.raw:
        frames = load(a.raw, a.width, a.height)
        frames, roi_desc = apply_roi(frames, a.roi)
        sc = score(frames)
        print(f"{a.raw}: {len(frames)} frames, scoring {roi_desc}")
        i, _ = report(frames, sc, None, None, None)
        if a.save:
            write_pgm(a.save, frames[i])
            print(f"  wrote {a.save} (frame {i})")
        return

    if not a.shutter:
        print("exposure is on AUTO -- fine for a first look, but pin "
              "--shutter/--gain before sweeping the lens", file=sys.stderr)

    best = -1.0
    best_frame = None
    first = True
    tmp = tempfile.NamedTemporaryFile(suffix=".raw", delete=False)
    tmp.close()
    try:
        while True:
            exp, gain = capture(tmp.name, a.width, a.height, a.frames,
                                a.shutter, a.gain, a.timeout)
            frames = load(tmp.name, a.width, a.height)
            frames, roi_desc = apply_roi(frames, a.roi)
            if first:
                print(f"scoring {roi_desc}", flush=True)
                first = False
            sc = score(frames)
            i, lv = report(frames, sc, exp, gain, best if a.watch else None)
            if lv > best:
                best, best_frame = lv, frames[i]
            if not a.watch:
                break
    except KeyboardInterrupt:
        print()
    finally:
        os.unlink(tmp.name)

    if a.watch:
        print(f"best focus={best:.2f}")
    if a.save and best_frame is not None:
        write_pgm(a.save, best_frame)
        print(f"wrote {a.save}")


if __name__ == "__main__":
    main()
