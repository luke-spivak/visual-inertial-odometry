# Onboard runtime

Start with `flight_supervisor.py` for unattended flight, or `capture_session.py`
for a single bench capture. Each command supports `--help`. Tests live in
[`../tests/`](../tests/); run `python3 -m pytest -q` from the repository root.

| File | Responsibility |
|---|---|
| `flight_supervisor.py` | Own the FC connection, wait for heartbeats, start/restart capture sessions, align each new estimator frame, and report status to the pilot. |
| `capture_session.py` | Configure exposure and IMU, launch the camera and C++ runner, and close recordings on shutdown. Runs independently for bench work. |
| `mavlink_bridge.py` | Convert estimates into ArduPilot frames, reject stale poses, and send MAVLink. Imported by the supervisor; also runnable for diagnostics. |
| `openvins_runner/sensor_runner.cpp` | Read IIO IMU samples and camera FIFOs, schedule OpenVINS updates, and write estimates and recordings. |
| `cli.py` | Expose the small flight, bench, and diagnostic command interfaces. |
| `flight_config.py` | Load, validate, and snapshot aircraft settings. |
| `camera.py` | Shared camera command construction and exposure selection. |
| `imu_device.py` | Shared IIO discovery, setup, layout parsing, and teardown. |
| `config/` | Flight settings (`flight.json`), OpenVINS settings, and camera/IMU calibration. |

## Process and data flow

```text
systemd → flight_supervisor.py (imports mavlink_bridge)
             │                         ↑ estimates via tmpfs
             └→ capture_session.py      │
                    ├→ vio_live (built from sensor_runner.cpp)
                    │     ↑ IMU samples from Linux IIO
                    └→ rpicam-raw → image + timestamp FIFOs → vio_live

flight_supervisor → MAVLink UART → ArduPilot
```

The supervisor outlives individual capture sessions. The bridge is a library
inside that process during flight, not another required Python process. These
boundaries let bench capture run without a flight controller and let frame
conversion and restart/alignment behavior be tested without sensors.

## Why a native OpenVINS runner?

The current Pi deployment uses a ROS-free OpenVINS library, direct IIO reads,
and `rpicam-raw` FIFOs. The runner adapts those inputs to OpenVINS; it does not
implement the estimator. Simulation and offline replay use ROS 2.

This reduces onboard middleware dependencies but makes this project responsible
for transport, queueing, recording, and lifecycle code. It is not evidence that
ROS would be too slow. A ROS migration should evaluate sensor timestamp fidelity,
message transport, recording, and restart/alignment handling together.

## Names and compatibility

- `vio_flight.py` is now `flight_supervisor.py`.
- `vio_live.py` is now `capture_session.py`.
- `vio_mavlink.py` is now `mavlink_bridge.py`.
- `openvins_runner/vio_live.cpp` is now `openvins_runner/sensor_runner.cpp`.

The compiled executable and bundle directory remain `~/vio_live/vio_live` and
`~/vio_live/`, preserving build/deployment paths. Python entry points use the new
names; update manual commands and reinstall the service unit when deploying.
See [setup](../docs/setup.md) for migration steps. Historical logs keep old names.

## Flight configuration and bench commands

`config/flight.json` is the single source for aircraft runtime settings. It
preserves the deployed service's 20 fps capture, exposure sweep, 50 Hz IMU
filter, 230400 baud link, 15-degree upside-down mount, 0.5-second pose-age limit,
and 20 GB minimum recording space. Estimator calibration remains in OpenVINS
YAML; `estimator_config` points to it rather than duplicating it.

The service loads this file once. Each capture session receives a resolved
`<prefix>.flight.json` snapshot; restart the supervisor to apply settings changes.
Bench capture writes the same snapshot, even without raw recording. Missing,
unknown, mistyped, and invalid settings fail before hardware access. Runtime code
keeps a frozen `FlightConfig` object; CLI options such as duration and output
prefix stay separate and are never included in aircraft snapshots.

```sh
sudo python3 src/flight_supervisor.py
sudo python3 src/capture_session.py ~/vio/bench --secs 30 --no-record
python3 src/mavlink_bridge.py --expect-accel
python3 src/mavlink_bridge.py ~/vio/bench.est.txt --dry-run
```

All commands accept `--config PATH` for a complete alternative flight JSON.
For an experiment, copy the file and change its settings; there are no separate
CLI overrides for exposure, mounting, serial settings, or estimator paths.
The supervisor privately supplies the capture estimate path on tmpfs.

`exposure_mode` is `sweep`, `auto`, or `fixed`. `max_shutter` caps auto exposure;
`shutter` (microseconds) and `gain` apply in fixed mode, which skips probing.
Auto/sweep probes use temporary directories and stop on camera errors, timeouts,
or invalid output rather than reusing a previous result. `imu_lpf` is in Hz,
`tilt_deg` in degrees below horizontal, and `max_lag` in seconds (zero disables
stale-pose rejection for offline diagnostics). `min_free_gb` uses decimal GB.
`send_velocity` controls velocity messages alongside position. `verbosity`
selects the OpenVINS print level.

Relative filesystem paths resolve beside the selected JSON file. `~/` resolves
to the operating user's home, including under sudo; serial URLs are unchanged.

This replaces the earlier large CLIs. `--config` now selects flight JSON, not
OpenVINS YAML. Manual capture now uses the flight exposure sweep by default,
and bridge diagnostics use the flight's upside-down mount instead of an upright
default. Update the JSON for a different setup. The standalone IMU calibration
logger and internal C++ runner interfaces are unchanged.

## Hardware checks and recordings

Before flight, compare the stationary IMU reading with
`mavlink_bridge.py --expect-accel`. A software transform test cannot verify the physical mount.
By hand, nose-down should produce negative pitch, right-side-down positive roll,
and clockwise yaw viewed from above increasing yaw. ArduPilot must align the
estimator's arbitrary heading; this integration uses `VISO_TYPE=2`.

For standalone IMU timing checks or stationary calibration on the Pi:

```sh
sudo python3 tools/sensors/imu_log.py --check
sudo python3 tools/sensors/imu_log.py --hours 3 --out ~/imu/run1
```

Capture sessions record raw camera/IMU data plus timestamps and configuration
sidecars for offline replay. The live estimator filters IMU samples, while the
recording remains unfiltered; replay must apply the same filtering for comparison.
Historical measurements and debugging investigations remain in the
[development log](../docs/development-log.md).

## C++ migration

The [migration plan](../docs/cpp-migration.md) tracks the staged replacement of
the Python flight runtime. `estimate.*` and `frames.*` are the tested native
foundation; `src/CMakeLists.txt` builds them independently. They are not yet wired
into the active onboard service.
