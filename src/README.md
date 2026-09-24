# Onboard runtime

Start with `flight_supervisor.py` for unattended flight, or `capture_session.py`
for a single bench capture. Each command supports `--help`.

| File | Responsibility |
|---|---|
| `flight_supervisor.py` | Own the FC connection, wait for heartbeats, start/restart capture sessions, align each new estimator frame, and report status to the pilot. |
| `capture_session.py` | Configure exposure and IMU, launch the camera and C++ runner, and close recordings on shutdown. Runs independently for bench work. |
| `mavlink_bridge.py` | Convert estimates into ArduPilot frames, reject stale poses, and send MAVLink. Imported by the supervisor; also runnable for diagnostics. |
| `openvins_runner/sensor_runner.cpp` | Read IIO IMU samples and camera FIFOs, schedule OpenVINS updates, and write estimates and recordings. |
| `cli.py` | Define command-line interfaces, shared option groups, and exposure validation. |
| `imu_log.py` | Shared IIO discovery/setup/teardown plus a standalone IMU recorder. |
| `config/` | OpenVINS settings and camera/IMU calibration. |

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

CLI flags and defaults are preserved, including the supervisor's upside-down
mount default and the diagnostic bridge's upright default. Both capture and
supervision reject `--gain` without `--shutter`, and now both reject combining
`--shutter` with `--exposure-sweep` before accessing hardware.
