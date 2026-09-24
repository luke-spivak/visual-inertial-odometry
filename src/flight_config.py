"""Load and validate aircraft settings shared by flight, capture, and diagnostics."""
from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "flight.json"


@dataclass(frozen=True)
class FlightConfig:
    recording_dir: str
    estimate_dir: str
    min_free_gb: float
    device: str
    baud: int
    tilt_deg: float
    upside_down: bool
    send_velocity: bool
    max_lag: float
    fps: float
    max_shutter: int
    exposure_mode: str
    shutter: int
    gain: float
    watermark: int
    imu_lpf: float
    estimator_binary: str
    estimator_config: str
    verbosity: str

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            valid = (type(value) in (int, float) and math.isfinite(value)
                     if field.type is float else type(value) is field.type)
            if not valid:
                raise ValueError(f"{field.name}: expected {field.type.__name__}")
            if field.type is str and not value.strip():
                raise ValueError(f"{field.name}: must not be empty")
        for name in ("baud", "fps", "max_shutter", "shutter", "watermark", "gain"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name}: must be positive")
        for name in ("min_free_gb", "max_lag", "imu_lpf"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name}: must not be negative")
        if self.gain > 16:
            raise ValueError("gain: must not exceed 16")
        if self.imu_lpf >= 220:
            raise ValueError("imu_lpf: must be below the 440 Hz IMU's Nyquist frequency (220 Hz)")
        if self.exposure_mode not in ("sweep", "auto", "fixed"):
            raise ValueError("exposure_mode: choose sweep, auto, or fixed")
        if self.verbosity not in ("ALL", "DEBUG", "INFO", "WARNING", "ERROR", "SILENT"):
            raise ValueError("verbosity: unknown OpenVINS print level")


def resolve_path(value, user_home, base):
    # Under sudo, '~' must refer to the operating user, not root.
    if value == "~" or value.startswith("~/"):
        path = Path(user_home) / value[2:]
    elif value.startswith("~"):
        raise ValueError("use ~/ for the operating user's home, not ~username")
    else:
        path = Path(value)
    return str((base / path).resolve())


def load_config(path, user_home):
    path = Path(resolve_path(str(path), user_home, Path.cwd()))
    with path.open() as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("flight configuration must be a JSON object")
    expected = {f.name for f in fields(FlightConfig)}
    if set(data) != expected:
        raise ValueError(f"configuration keys: missing {sorted(expected - set(data))}, "
                         f"unknown {sorted(set(data) - expected)}")
    config = FlightConfig(**data)
    resolved = asdict(config)
    for name in ("recording_dir", "estimate_dir", "estimator_binary", "estimator_config"):
        resolved[name] = resolve_path(resolved[name], user_home, path.parent)
    return FlightConfig(**resolved), str(path)


def save_config(options, path):
    """Record resolved aircraft settings without transient command-line options."""
    config = FlightConfig(**{f.name: getattr(options, f.name) for f in fields(FlightConfig)})
    with open(path, "w") as stream:
        json.dump(asdict(config), stream, indent=2)
        stream.write("\n")
