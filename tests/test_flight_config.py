"""Configuration contracts across flight, bench capture, and diagnostics."""
from dataclasses import FrozenInstanceError
import json
from pathlib import Path

import pytest

from cli import parse_options
from flight_config import DEFAULT_CONFIG, FlightConfig, load_config, save_config


def test_all_commands_share_aircraft_settings():
    flight, flight_options = parse_options("flight", "/home/pilot", [])
    capture, capture_options = parse_options("capture", "/home/pilot", ["bench", "--secs", "15", "--no-record"])
    bridge, bridge_options = parse_options("bridge", "/home/pilot", ["--expect-accel"])
    config, _ = load_config(DEFAULT_CONFIG, "/home/pilot")
    for command in (flight, capture, bridge):
        assert isinstance(command, FlightConfig)
        assert command == config
    assert vars(flight_options) == {"config": str(DEFAULT_CONFIG)}
    for options in (flight_options, capture_options, bridge_options):
        assert not hasattr(options, "baud")
    with pytest.raises(FrozenInstanceError):
        flight.baud = 9600
    assert flight.upside_down is True
    assert flight.exposure_mode == "sweep"
    assert flight.estimator_binary == str(Path("/home/pilot/vio_live/vio_live").resolve())
    assert capture_options.secs == 15 and capture_options.no_record


def test_relative_paths_and_snapshot_are_reusable(tmp_path):
    data = json.loads(DEFAULT_CONFIG.read_text())
    data.update(recording_dir="recordings", estimator_config="estimator.yaml", exposure_mode="fixed",
                shutter=500, tilt_deg=25, upside_down=False)
    path = tmp_path / "custom.json"
    path.write_text(json.dumps(data))
    a, _ = parse_options("flight", "/home/pilot", ["--config", str(path)])
    assert a.recording_dir == str(tmp_path / "recordings")
    assert a.estimator_config == str(tmp_path / "estimator.yaml")
    snapshot = tmp_path / "snapshot.json"
    save_config(a, snapshot)
    # Editing the original file must not change a child's pinned settings.
    path.write_text("{}")
    child, options = parse_options("capture", "/root", ["bench", "--config", str(snapshot)])
    assert options.out == "bench"
    assert not hasattr(child, "out")
    assert child.shutter == 500 and child.tilt_deg == 25 and not child.upside_down
    assert child.estimator_binary == str(Path("/home/pilot/vio_live/vio_live").resolve())


@pytest.mark.parametrize("change", [
    {"baud": True}, {"fps": 0}, {"fps": float("nan")}, {"min_free_gb": -1},
    {"upside_down": "false"}, {"exposure_mode": "swep"}, {"gain": 17},
    {"watermark": 0}, {"imu_lpf": 220}, {"unexpected": 1}, {"device": ""},
])
def test_invalid_settings_fail_before_runtime(tmp_path, change):
    data = json.loads(DEFAULT_CONFIG.read_text())
    data.update(change)
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(data))
    with pytest.raises(SystemExit) as error:
        parse_options("flight", "/home/pilot", ["--config", str(path)])
    assert error.value.code == 2


def test_missing_configuration_does_not_fall_back(tmp_path):
    for contents in (None, "{}", "{invalid"):
        path = tmp_path / "bad.json"
        if contents is not None:
            path.write_text(contents)
        with pytest.raises(SystemExit) as error:
            parse_options("capture", "/home/pilot", ["bench", "--config", str(path)])
        assert error.value.code == 2


@pytest.mark.parametrize("command,args", [
    ("flight", ["--shutter", "200"]), ("bridge", ["--upside-down"]),
    ("capture", ["bench", "--secs", "-1"]), ("bridge", []),
])
def test_invalid_or_retired_flags_are_not_silently_accepted(command, args):
    with pytest.raises(SystemExit) as error:
        parse_options(command, "/home/pilot", args)
    assert error.value.code == 2
