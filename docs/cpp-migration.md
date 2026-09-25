# Native flight application migration

Target: one C++ process owns Linux IIO acquisition, camera capture, OpenVINS,
MAVLink communication, flight-session policy, and asynchronous recording.
Python remains for calibration, analysis, and tests. ArduPilot still owns vehicle
control and failsafes; systemd owns application startup and process restart.

## Progress and gates

1. **Implemented: estimate contract and frame conversion.** `src/estimate.*`
   defines monotonic timestamps, estimator-session identity, initialized state,
   and numeric/freshness validation. `src/frames.*` converts immutable-by-contract
   estimate snapshots to navigation measurements and computes stationary IMU
   acceleration. Calibration is an input, not a compiled aircraft constant.
   This library is tested independently and is not yet connected to live flight.
2. **Next: MAVLink transport and flight-session state machine.** Use generated
   MAVLink headers, a single serial owner, bounded/nonblocking transmission,
   explicit controller identity and alignment acknowledgment handling. Session
   control owns reset counters and publication permission. Test using a fake FC
   and recorded inputs before enabling a physical transmitter. Keep the current
   camera path during this step; never run two publishers against the FC.
3. Move IIO setup and capture supervision into the C++ application. Verify startup,
   partial-startup cleanup, disarmed alignment, restart, and signal handling. Keep
   `rpicam-raw` as a temporary camera adapter.
4. Replace `rpicam-raw` with libcamera. Verify actual Pi pixel layout/stride, sensor
   timestamps and timebase, exposure settling, calibration, and buffer ownership.
   This is a separate hardware gate, not assumed equivalent from compilation.
5. Isolate recording behind bounded queues. Inject slow/full storage and interrupted
   shutdown; record gaps explicitly and keep live estimate delivery independent.
6. Switch the service and remove replaced Python runtime paths only after replay,
   simulated-FC, and hardware gates pass. Retain a rollback deployment.

Each stage is a separate reviewable commit or set of commits. This document tracks
actual progress; the planned application is not yet a replacement for the service.

## Data and ownership contracts

- Sensor timestamps are integer nanoseconds on the host monotonic clock. No
  wall-clock time or receive-time substitution. MAVLink microseconds are converted
  only at the serialization boundary.
- `Estimate::world_from_imu` uses Eigen's Hamilton rotation semantics. OpenVINS JPL
  xyzw coefficients represent the same numeric rotation here, but the Eigen
  constructor takes w first. The adapter must make that ordering explicit.
- World position/velocity are gravity-aligned, z up, with arbitrary heading. The
  frame transform swaps x/y and negates z; ArduPilot alignment establishes heading.
- Position is the IMU origin. ArduPilot applies the configured lever arm.
- The session generation is independent of MAVLink's wrapping 8-bit reset count.
  Validity checks do not grant publication permission or infer session restarts.
- The estimator worker owns OpenVINS access and publishes value snapshots. A
  communication/control loop owns UART and session policy; it does not access
  mutable estimator state. Disk writing never holds up either path.
- Sensor/estimate buffers are bounded. Camera backlog can discard old frames;
  IMU overflow is an explicit integrity fault. No stale-pose retransmission as
  if it were a fresh measurement.

## Calibration direction

`FrameTransform` takes camera-from-IMU rotation, matching Kalibr `T_cam_imu`.
The checked-in OpenVINS chain uses `T_imu_cam`, its inverse. The future config
adapter must invert that rotation exactly once. The tests include rounded
historical calibration only as a compatibility fixture; production code has no
hardcoded aircraft calibration.

## Build and test the foundation

Requires CMake, a C++17 compiler, and Eigen. This target does not require OpenVINS,
libcamera, the Pi, or a flight controller:

```sh
cmake -S src -B build/native-core -DCMAKE_BUILD_TYPE=Release
cmake --build build/native-core
ctest --test-dir build/native-core --output-on-failure
```

The existing onboard build remains `src/openvins_runner/CMakeLists.txt` and the
current deployment scripts still use it. `src/CMakeLists.txt` currently builds
only the native core and its tests, not a flight executable.

Validation to date: native macOS Release build and CTest pass. Pi/Linux builds and
hardware verification remain outstanding; the local Docker daemon was unavailable
when this first step was implemented.
