"""Exposure selection and camera failure handling without camera hardware."""
import json
from pathlib import Path
import subprocess
from dataclasses import replace
from flight_config import DEFAULT_CONFIG, load_config

import pytest

import camera
from flight_supervisor import status_of, SEV_INFO


def test_fixed_exposure_never_starts_a_probe(monkeypatch):
    def unexpected(*args, **kwargs):
        pytest.fail("fixed exposure must not start the camera")
    monkeypatch.setattr(camera.subprocess, "run", unexpected)
    config, _ = load_config(DEFAULT_CONFIG, "/home/pilot")
    assert camera.select_exposure(replace(config, exposure_mode="fixed", shutter=500, gain=2)) == (500, 2)


def test_auto_exposure_caps_shutter_and_cleans_up(monkeypatch):
    paths = []
    def run(command, **kwargs):
        assert kwargs == dict(check=True, capture_output=True, timeout=15)
        assert command[command.index("-t")+1] == "2500"
        path = Path(command[command.index("--metadata")+1])
        paths.append(path)
        path.write_text(json.dumps([{"ExposureTime": 8000, "AnalogueGain": 1.5}]))
    monkeypatch.setattr(camera.subprocess, "run", run)
    assert camera.auto_exposure(4000) == (4000, 3)
    assert not paths[0].parent.exists()


def test_sweep_chooses_brightest_unclipped_candidate(monkeypatch):
    monkeypatch.setattr(camera, "W", 2)
    monkeypatch.setattr(camera, "H", 2)
    paths = []
    def run(command, **kwargs):
        shutter = int(command[command.index("--shutter")+1])
        path = Path(command[command.index("-o")+1])
        assert not path.exists()
        paths.append(path)
        path.write_bytes(bytes([0, {20: 40, 50: 100, 100: 255}[shutter]]) * 4)
    monkeypatch.setattr(camera.subprocess, "run", run)
    assert camera.exposure_sweep((20, 50, 100)) == (50, 1)
    assert all(not path.parent.exists() for path in paths)


@pytest.mark.parametrize("failure", ["process", "timeout", "empty", "truncated", "missing"])
def test_failed_sweep_cleans_up_and_does_not_return_an_exposure(monkeypatch, failure):
    paths = []
    def run(command, **kwargs):
        path = Path(command[command.index("-o")+1])
        paths.append(path)
        if failure == "process":
            raise subprocess.CalledProcessError(1, command)
        if failure == "timeout":
            raise subprocess.TimeoutExpired(command, 15)
        if failure != "missing":
            path.write_bytes(b"" if failure == "empty" else b"\x00")
    monkeypatch.setattr(camera.subprocess, "run", run)
    with pytest.raises((OSError, ValueError, subprocess.SubprocessError)):
        camera.exposure_sweep((20,))
    assert not paths[0].parent.exists()


def test_live_command_keeps_timestamps_and_fixed_exposure():
    cmd = camera.camera_command("frames", fps=20, shutter=200, gain=1, metadata="meta", flush=True)
    assert "--flush" in cmd and cmd[cmd.index("--metadata")+1] == "meta"
    assert cmd[cmd.index("--mode")+1] == "1280:800:8"
    assert cmd[cmd.index("--shutter")+1] == "200"
    assert cmd[cmd.index("-t")+1] == "0"


def test_all_exposure_modes_report_to_pilot():
    for line in ["  fixed-exposure: 200 us, gain 1.00", "  exposure sweep selected 200 us, gain 1.00"]:
        assert status_of(line) == (SEV_INFO, "VIO exposure 200 us, gain 1.00")
