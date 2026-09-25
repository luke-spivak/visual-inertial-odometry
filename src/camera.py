"""Build camera commands and select fixed, automatic, or swept exposure."""
import json
from pathlib import Path
import subprocess
import tempfile

from flight_config import FlightConfig

W, H = 1280, 800


def camera_command(output, *, fps, shutter=None, gain=None, duration_ms=0,
                   frames=None, metadata=None, flush=False):
    command = ["rpicam-raw", "-n", "--mode", f"{W}:{H}:8", "--width", str(W),
               "--height", str(H), "--framerate", str(fps), "-o", str(output)]
    command += ["--frames", str(frames)] if frames is not None else ["-t", str(duration_ms)]
    if shutter is not None:
        command += ["--shutter", str(shutter), "--gain", str(gain)]
    if metadata is not None:
        command += ["--metadata", str(metadata), "--metadata-format", "json"]
    if flush:
        command.append("--flush")
    return command


def _probe(command):
    # Bound a hung probe and surface camera failures before reading its output.
    subprocess.run(command, check=True, capture_output=True, timeout=15)


def auto_exposure(max_shutter):
    with tempfile.TemporaryDirectory(prefix="vio-exposure-") as directory:
        metadata = Path(directory) / "metadata.json"
        _probe(camera_command(Path(directory) / "probe.y16", fps=5,
                              duration_ms=2500, metadata=metadata))
        rows = json.loads(metadata.read_text())
        if not rows:
            raise ValueError("auto-exposure probe returned no metadata")
        last = rows[-1]
        product = last["ExposureTime"] * last["AnalogueGain"]
        shutter = int(min(product, max_shutter))
        if shutter <= 0:
            raise ValueError("auto-exposure probe returned a nonpositive exposure")
        gain = max(1.0, product / shutter)
        if gain > 16:
            raise ValueError(f"needs gain {gain:.0f} at {shutter} us -- too dark")
        print(f"  auto-exposure: {last['ExposureTime']} us x gain {last['AnalogueGain']:.2f}"
              f" -> fixed {shutter} us, gain {gain:.2f}")
        return shutter, gain


def exposure_sweep(candidates=(20, 30, 50, 75, 100, 200, 500, 1000, 2000, 4000)):
    results = []
    with tempfile.TemporaryDirectory(prefix="vio-exposure-") as directory:
        path = Path(directory) / "probe.y16"
        for shutter in candidates:
            # Remove each result before reuse so a missing output cannot look successful.
            path.unlink(missing_ok=True)
            _probe(camera_command(path, fps=20, shutter=shutter, gain=1, frames=30))
            data = path.read_bytes()
            if not data or len(data) % (W * H * 2):
                raise ValueError("exposure sweep returned empty or incomplete frames")
            # R8 is stored in the high byte of each little-endian 16-bit sample.
            pixels = data[1::2]
            clipped = sum(v >= 250 for v in pixels) / len(pixels)
            mean = sum(pixels) / len(pixels)
            results.append((shutter, clipped, mean))
    if not results:
        raise ValueError("exposure sweep has no candidates")
    usable = [r for r in results if r[1] <= 0.01 and r[2] >= 15]
    chosen = max(usable, key=lambda r: r[2]) if usable else min(results, key=lambda r: r[1])
    print("  exposure sweep: " + ", ".join(f"{s} us={clip*100:.2f}% clip, mean={mean:.1f}"
                                           for s, clip, mean in results))
    print(f"  exposure sweep selected {chosen[0]} us, gain 1.00")
    return chosen[0], 1.0


def select_exposure(config: FlightConfig):
    if config.exposure_mode == "fixed":
        print(f"  fixed-exposure: {config.shutter} us, gain {config.gain:.2f}")
        return config.shutter, config.gain
    if config.exposure_mode == "auto":
        return auto_exposure(config.max_shutter)
    if config.exposure_mode == "sweep":
        return exposure_sweep()
    raise ValueError(f"unknown exposure mode: {config.exposure_mode}")
