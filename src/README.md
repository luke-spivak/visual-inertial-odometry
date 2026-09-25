# Onboard C++ application

`vio_flight config/flight.json` runs the onboard pipeline in one process. Linux
IIO supplies IMU samples, libcamera supplies timestamped images, OpenVINS estimates
motion, and MAVLink carries validated measurements to ArduPilot over the UART.
ArduPilot owns motor control and flight failsafes.

| Files | Responsibility |
|---|---|
| `flight_main.cpp` | Load one configuration, install signal handlers, start the application. |
| `flight_application.*` | Coordinate capture, estimator workers, controller communication, and recovery. |
| `imu_device.*` | Configure and own IIO sensor buffers; restore them on shutdown. |
| `libcamera_source.cpp`, `camera_capture.*` | Own camera buffers, timestamps, and exposure selection. |
| `openvins_runner/sensor_runner.cpp` | Feed timestamped camera/IMU data into OpenVINS. |
| `estimate.*`, `frames.*` | Define estimator/interface measurements and coordinate conversions. |
| `mavlink_link.*`, `serial_port.*` | Encode MAVLink and service the nonblocking UART. |
| `flight_session.*`, `session_store.*` | Enforce alignment/publication policy and persist reset counters. |
| `recording.*` | Write recordings through a bounded asynchronous queue. |
| `flight_config.*`, `config/` | Validate aircraft settings and retain sensor calibration. |

```text
IIO IMU ──────┐
             ├─→ OpenVINS → latest estimate → frame conversion → MAVLink → ArduPilot
libcamera ───┘                    │
          sensor data + estimates └─→ bounded recording queue → disk
```

The estimator worker owns mutable OpenVINS state. The main thread owns the
controller link and reads copied estimates. Camera, IMU, and recording work use
bounded queues; recording failure does not block navigation. Estimate files are
for analysis, not communication between processes.

## Configuration and execution

`config/flight.json` selects the UART, mount, exposure policy, IMU filter,
recording location, and OpenVINS calibration. The executable takes this single
JSON path; use a copy for a bench experiment. The legacy `estimator_binary`
configuration field is accepted for old configuration compatibility but unused.

```sh
sudo env SUDO_USER="$USER" ./build/native/vio_flight src/config/flight.json
```

Stop with SIGINT or SIGTERM. Do not run alongside another controller publisher.
The application waits for a disarmed controller before capture. Calibration
YAML paths must point to the actual calibrated assembly.

## Tests and validation

```sh
cmake -S src -B build/native-core
cmake --build build/native-core
ctest --test-dir build/native-core --output-on-failure
```

The full application additionally needs OpenVINS and libcamera; see
[setup](../docs/setup.md). Python remains in `tools/`, simulation, and tests for
offline work. It is not part of the flight process. The old runtime and its tests
remain accessible in Git history; the Pi's rollback deployment is preserved.
The optional `vio_live` C++ compatibility executable supports older FIFO tooling.

[Pi bench validation](../results/native-validation-2026-09-25/README.md) covers
capture, movement, MAVLink, recordings, and shutdown. This native revision has not
been flight validated. [Migration notes](../docs/cpp-migration.md) track remaining
controller recovery and deployment checks.
