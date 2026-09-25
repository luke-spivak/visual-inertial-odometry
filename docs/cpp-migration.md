# Native flight application migration

Target: one C++ process owns Linux IIO acquisition, camera capture, OpenVINS,
MAVLink communication, flight-session policy, and asynchronous recording.
Python remains for calibration, analysis, and tests. ArduPilot still owns vehicle
control and failsafes; systemd owns application startup and process restart.

## Progress and gates

1. **Implemented: estimate contract and frame conversion.** `src/estimate.*`
   defines monotonic timestamps, estimator-session identity, initialized state,
   and numeric/freshness validation. `src/frames.*` converts immutable-by-contract
   estimate snapshots to ArduPilot measurements and computes stationary IMU
   acceleration. Calibration is an input, not a compiled aircraft constant.
   This library is tested independently and is not yet connected to live flight.
2. **Implemented as a tested library: MAVLink transport and flight-session policy.**
   `serial_port.*` owns an exclusive nonblocking POSIX UART. `mavlink_link.*` uses
   pinned generated MAVLink headers, typed snapshots, and a fixed transmit buffer.
   `flight_session.*` owns controller freshness, alignment, generations, and reset
   counters. Fake-FC, pseudoterminal, and recorded-input tests pass. This code is
   not wired into the running estimator or service; physical transmission remains
   gated on integration and hardware checks. Never run two publishers against the FC.
3. **Next: application integration.** Move IIO setup and capture supervision into the C++ application. Verify startup,
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
- `EstimatorEstimate::world_from_imu` uses Eigen's Hamilton rotation semantics. OpenVINS JPL
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

## MAVLink link and session contract

The communication loop owns `MavlinkLink`, calls `start_session(generation)` once
per new estimator frame, then calls `poll(now)` and `publish(snapshot, now)`.
`publish` returns `Queued`, `Held`, `InvalidEstimate`, `OutOfOrder`, or `Busy`.
The estimator remains independent: it hands over value snapshots, never a file
path or access to its mutable state. The next stage must supply that handoff.

- Controller system/component IDs are configured explicitly (default 1/1), and
  heartbeats must identify ArduPilot rather than a GCS. The source defaults to
  system 1, component 197, matching the existing bridge. These are addresses,
  not authentication; MAVLink signing is not implemented in this stage.
- States are `WaitingForController`, `Priming`, `AwaitingAlignment`, `Aligned`,
  and `Failed`. Before alignment, only a fresh, disarmed controller may receive
  priming measurements. At least one complete measurement pair must be written
  before requesting auxiliary function 80 at switch position 2.
- Only an ACK from the configured FC, for `MAV_CMD_DO_AUX_FUNCTION`, and addressed
  to this source (or with omitted/zero target extensions) can resolve an outstanding
  request. Accepted permits publication while armed. In-progress does not;
  temporary rejection backs off; other rejection fails closed. Default limits are
  three attempts, two seconds per attempt, and ten seconds for in-progress.
- Heartbeat loss revokes alignment. Arming during a pending alignment fails closed.
  Heartbeats alone cannot detect an FC reboot shorter than the freshness timeout;
  boot detection/recovery remains an integration gate before flight use.
  Restarting the estimator while an alignment request is unresolved is rejected,
  including between retries. Generations must increase; only explicit generation
  changes increment the MAVLink reset byte. Timestamp regression never implies reset.
- COMMAND_ACK has no transaction ID or echoed auxiliary-function parameters.
  Source/target/command checks cannot distinguish every delayed duplicate ACK from
  an earlier identical request. `Aligned` means command acceptance, not independently
  measured heading convergence. Before live integration, verify the deployed FC's
  behavior and define disarmed link recovery/receive draining; reconnect alone does
  not eliminate this protocol ambiguity. Serialize auxiliary-function commands on
  this source identity.
- Each poll reads at most 4096 bytes and attempts one write. The fixed buffer holds
  one pose/velocity pair or one control packet. Busy callers retain only their newest
  snapshot. Unsent measurements expire; a partial packet that expires or loses
  permission closes the link rather than sending its remainder or splicing frames.
- Writes completing means the OS accepted the bytes, not that the FC received them.
  UART driver buffering and physical latency still require Pi testing at the
  configured baud. There is no blocking drain call. Failure releases the stream and
  exposes `failure_reason()`; automatic reconnect is deliberately not implemented.
- Pose and optional velocity use MAVLink 2, integer monotonic microseconds, unknown
  covariance markers, and the same reset byte. Nonfinite values and doubles outside
  float range are rejected before serialization. Per-link parser and sequence state
  avoid shared global MAVLink channels.
- The application's next stage must persist/reserve the initial reset byte across
  process restarts, load calibration/configuration, copy estimates under the correct
  ownership, and arrange shutdown and recovery. The current library accepts the
  initial byte as an input; it does not silently invent cross-process persistence.

Protocol references: [MAVLink command protocol](https://mavlink.io/en/services/command.html)
and [ArduPilot auxiliary functions](https://ardupilot.org/copter/docs/common-auxiliary-functions.html).

## Build and test the native libraries

Requires CMake, a C++17 compiler, Eigen, and POSIX serial APIs. CMake fetches the
ArduPilotMega generated MAVLink C headers at revision
`c1fd65eb702106097a4253c259aba34a18235848`, with a pinned archive SHA-256. For an
offline build, extract that revision and pass
`-DVIO_MAVLINK_INCLUDE_DIR=/absolute/path/to/c_library_v2`.
Neither library requires OpenVINS, libcamera, the Pi, or a flight controller:

```sh
cmake -S src -B build/native-core -DCMAKE_BUILD_TYPE=Release
cmake --build build/native-core
ctest --test-dir build/native-core --output-on-failure
```

The existing onboard build remains `src/openvins_runner/CMakeLists.txt` and the
current deployment scripts still use it. `src/CMakeLists.txt` currently builds
the native core, flight-link library, and tests, not a flight executable.

Validation to date: native macOS Release and AddressSanitizer/UndefinedBehaviorSanitizer
builds pass all three CTest targets. Tests cover frame conversion, raw pseudoterminal
I/O and hangup, fragmented/corrupt packets, controller/ACK filtering, timeout/retry
policy, arming, resets, stale/invalid estimates, bounded input/output, and partial-write
failures. The fake FC also decodes all 7,623 estimates from the checked-in
`results/flight-review-2026-09-15/pi/run-20260915-121608.est.txt` recording. Replay
uses recorded timestamps as a virtual clock; file parsing is test-only.
Pi/Linux builds and physical-FC verification remain outstanding.
