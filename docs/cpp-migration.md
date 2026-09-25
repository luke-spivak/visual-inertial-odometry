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
   This library is used by the new native application and tested independently.
2. **Implemented as a tested library: MAVLink transport and flight-session policy.**
   `serial_port.*` owns an exclusive nonblocking POSIX UART. `mavlink_link.*` uses
   pinned generated MAVLink headers, typed snapshots, and a fixed transmit buffer.
   `flight_session.*` owns controller freshness, alignment, generations, and reset
   counters. Fake-FC, pseudoterminal, and recorded-input tests pass. This code is
   wired into the native executable; the deployed service still uses Python.
   Physical flight use remains gated on hardware checks. Never run two publishers against the FC.
3. **Implemented, hardware validation pending: native application integration.**
   `flight_main.cpp` / `flight_application.*` own startup, capture, and recovery.
   `imu_device.*` discovers/configures IIO and disables owned buffers on failure.
   `session_store.*` owns the application lock and durable reset-counter reservations. OpenVINS runs in-process through `estimator_runner.h`,
   handing copied snapshots to the MAVLink loop. Linux ARM64 builds and simulated
   lifecycle/signal tests pass. `run_flight()` orchestrates attempts; recording
   preparation and capture supervision are separate functions.
4. **Implemented, Pi validation pending: in-process libcamera capture.**
   `libcamera_source.cpp` owns the camera, requests, DMA mappings, and shutdown.
   The native path has no camera subprocess, FIFO, or JSON metadata parser.
   `camera_source.h` carries owned mono8 frames with matching timestamps;
   `camera_capture.*` validates layouts and performs fixed/auto/sweep selection.
   Verify the actual Pi mode, stride, timestamps, exposure, calibration, and
   buffer reuse under load before deployment; compilation does not establish equivalence.
5. **Implemented, Pi throughput validation pending: recording isolation.**
   `recording.*` owns a bounded recording queue and one disk worker shared by raw
   IMU, camera/metadata pairs, estimates, and feature snapshots. Slow/full storage
   and interrupted shutdown tests pass. Overflow disables recording for the rest
   of that capture, retaining an explicitly incomplete prefix while navigation continues.
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
  mutable estimator state. The recording worker owns all streaming disk writes; producers
  only copy/serialize and enqueue data.
- Sensor/estimate buffers are bounded. Camera backlog can discard old frames;
  IMU overflow is an explicit integrity fault. No stale-pose retransmission as
  if it were a fresh measurement.

## Calibration direction

`FrameTransform` takes camera-from-IMU rotation, matching Kalibr `T_cam_imu`.
The checked-in OpenVINS chain uses `T_imu_cam`, its inverse. OpenVINS parsing
inverts it once; the native adapter reads that parsed camera-from-IMU rotation. The tests include rounded
historical calibration only as a compatibility fixture; production code has no
hardcoded aircraft calibration.

## MAVLink link and session contract

The communication loop owns `MavlinkLink`, calls `start_session(generation)` once
per new estimator frame, then calls `poll(now)` and `publish(snapshot, now)`.
`publish` returns `Queued`, `Held`, `InvalidEstimate`, `OutOfOrder`, or `Busy`.
The estimator remains independent: it hands over value snapshots, never a file
path or access to its mutable state. The native application's communication loop now supplies that handoff.

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
- `SessionStore` now reserves the reset byte with file/directory fsync and atomic
  rename before each native session can publish. Its process lock excludes a second
  native owner. A malformed counter is an error, not a silent reset to zero.
- The native application calls `end_session()` before cleanup, withdrawing queued
  measurements while keeping controller monitoring available. Due alignment and
  heartbeat packets get priority over newly offered estimates.

Protocol references: [MAVLink command protocol](https://mavlink.io/en/services/command.html)
and [ArduPilot auxiliary functions](https://ardupilot.org/copter/docs/common-auxiliary-functions.html).

## Build and test the native libraries

Requires CMake, a C++17 compiler, Eigen, nlohmann-json 3.11+, and POSIX serial APIs. CMake fetches the
ArduPilotMega generated MAVLink C headers at revision
`c1fd65eb702106097a4253c259aba34a18235848`, with a pinned archive SHA-256. For an
offline build, extract that revision and pass
`-DVIO_MAVLINK_INCLUDE_DIR=/absolute/path/to/c_library_v2`.
The default library/test build requires no OpenVINS, libcamera, Pi, or flight controller:

```sh
cmake -S src -B build/native-core -DCMAKE_BUILD_TYPE=Release
cmake --build build/native-core
ctest --test-dir build/native-core --output-on-failure
```

To build the native application inside the Linux build environment, install
`libcamera-dev` (libcamera >= 0.4) and `nlohmann-json3-dev`, provide the existing
OpenVINS source/library, and enable the executable:

```sh
cmake -S src -B build/native-linux -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DVIO_BUILD_FLIGHT_APP=ON \
  -DOV_SRC=/work/build/vio_live/open_vins \
  -DOV_LIB=/work/build/vio_live/ov/libov_msckf_lib.so
cmake --build build/native-linux -j2
ctest --test-dir build/native-linux --output-on-failure
```

The resulting executable takes one argument: the path to `flight.json`. It retains
the same configuration schema, including the legacy `estimator_binary` field for
Python compatibility; native capture does not launch that binary. It requires a
positive freshness limit and a filter cutoff below 45% of the configured 416 Hz
IIO rate. The old standalone `src/openvins_runner` build still produces `vio_live`
through a compatibility entry point. Deployment scripts and `vio@.service` have
not been switched to the native application.

## Native lifecycle and remaining gates

The application loads flight settings once, acquires its runtime-directory lock,
loads the estimator calibration, and opens its UART. Each capture waits for a
fresh disarmed heartbeat, selects exposure, configures IIO, and starts the camera
and estimator workers. Fixed, automatic, and swept exposure retain the Python
selection policy. No Python process participates in this native path.

One estimator worker feeds IMU samples and camera updates to OpenVINS, including
initialization; its initializer is joined rather than detached. Acquisition workers
use bounded queues (1024 IMU samples, 10 estimator camera frames, and 4 camera
callback frames). The legacy FIFO adapter alone retains its 128-timestamp queue. IMU overflow
fails the capture instead of silently dropping samples. The communication loop
reads a one-slot snapshot mailbox every 5 ms; estimate files are recordings only.
Camera calibration comes from OpenVINS's parsed IMU-to-camera rotation, with a test
against the checked-in calibration to catch a second inversion.

SIGINT/SIGTERM request ordinary cleanup. libcamera stops and finishes callbacks;
acquisition workers observe the shared stop flag and are joined before camera
buffers, recordings, or IMU buffers are released. Capture failures retry with
bounded exponential backoff and require a fresh disarmed controller. An unresolved
alignment or link failure requires process-level recovery rather than silently
starting another session. The counter is reserved before each attempt, including
attempts that fail during startup.

The native path saves flight/exposure/IMU metadata and the existing recording file
formats. Completion markers require the capture to return without errors after
requested shutdown (the compatibility `camera_exit: 0` field now means native
camera shutdown succeeded).
Streaming recording uses one worker with at most 32 MiB/1024 pending records,
plus one in-flight record and producers' temporary copies. Its mutex is never held
across disk I/O. The first overflow or write failure stops further recording for
that capture; navigation continues. Camera pixels and their metadata are enqueued
as one item. A partial disk write still makes the entire recording incomplete.
`*.recording-status.json` starts as unfinished and, when storage permits, reports
per-stream submitted/written/rejected counts and the failure reason. Counts refer
to records (an IMU record here is one read batch). Recording stops at the first gap,
so it does not silently concatenate later IMU data across a missing interval.
`writer_drained` describes the writer only; the application's `.complete.json`
marker additionally requires error-free capture shutdown and successful writer
close. Missing, unfinished, or failed status must never be treated as a full recording.

Shutdown allows two seconds for the recorder to drain. A writer stuck in a kernel
file operation may outlive the capture, but owns only its sink, queued data, and
shared status. It cannot reference the estimator or sensor handles. A process-wide
single-writer slot prevents retries from accumulating blocked threads; further
recording stays disabled until that writer exits. A timeout permanently disqualifies
that capture's completion marker, even if its writer later finishes. This is not
cancellation of the underlying filesystem operation. systemd's process-level stop
timeout remains necessary for deployment and for blocked OpenVINS/library calls.
Startup configuration metadata and the final completion marker are still written
outside the streaming path. Abrupt power-loss durability is not guaranteed by file
close; no claim of fsync-level recording durability is made. Service-account file
ownership and sustained recording throughput still require Pi validation.

Validation: macOS Release and ASan/UBSan pass the six library/lifecycle CTest
programs. Linux ARM64 builds the actual OpenVINS adapter, `vio_flight`, and legacy
`vio_live`; its tests additionally exercise calibration direction, worker-failure
unwinding, real SIGINT/SIGTERM, and duplicate application exclusion using a virtual
UART. Lifecycle tests use a fake FC, estimator, camera source, and sysfs tree:
normal shutdown, armed startup, disarmed restart, camera start/read failure, failed
IIO setup, exposure cancellation/selection, and persisted reset bytes are covered.
Frame tests cover padded R8/R16 extraction, malformed frames, clock conversion,
regressing/stale timestamps, bounded queue drops, errors, and waking blocked readers.
The real OpenVINS runner test feeds a native frame and injects a camera failure,
checking worker cleanup and recording pixels/timestamps/sequence together.
Recording tests hold the sink blocked while producers fill the bounded queue,
verify overflow accounting, inject storage failure (including Linux `/dev/full`),
and destroy a timed-out recorder before releasing its writer. Lifecycle tests
verify that failed recording suppresses the completion marker without stopping
pose publication. Feature serialization is covered by the same CTest target.
The existing 7,623-estimate flight replay remains part of the MAVLink tests.

Pi hardware gates remain: sensor timing and filter behavior at the read-back IIO
rate, CPU/queue performance with serialized OpenVINS ownership, real libcamera
startup/shutdown, UART buffering, FC boot detection and alignment semantics, and
signal/error cleanup under load. The Python service remains the deployment default
until those hardware checks and deployment cutover are complete.


## Native camera contract

- Exactly one OV9281 is accepted. Request 1280×800, 8-bit sensor readout, raw
  output, and no image rotation; refuse changes to the calibrated size/bit depth.
  Only uncompressed R8 or R16 is accepted. R16 must contain zero low bytes on
  every pixel; row stride is honored. Compressed, Bayer, cropped-size, and
  processed RGB outputs are rejected instead of guessed.
- libcamera DMA buffers remain camera-owned. Each completed frame is mapped with
  its plane offset, synchronized for CPU reads, copied to owned mono8 pixels,
  and synchronized back before request reuse. Neither OpenVINS nor recording
  retains a view into a recycled buffer. Callbacks only validate/copy/enqueue;
  they perform no disk writes or estimator updates.
- The four-frame callback queue drops the oldest image under load and reports
  its drop count on stop. Recording metadata includes the hardware sequence
  number, exposing capture/queue gaps. Backend faults reach the reader as
  exceptions; camera stalls also stop the estimator workers.
- `SensorTimestamp` is documented by libcamera as exposure-start nanoseconds on
  CLOCK_BOOTTIME. A bracketed clock sample converts it to CLOCK_MONOTONIC for
  IIO, OpenVINS, and recordings. Non-increasing, future, and stale timestamps
  fail capture; an offset change over 1 ms (such as suspend/resume) also fails.
  Check this contract against the deployed Pi's libcamera version and a moving
  camera/IMU recording; do not infer alignment from matching units alone.
- Exposure probes use the same source as flight capture. Discard startup frames
  and require manual exposure/gain readback to settle; check frame-duration
  readback too. Sweep examines 30 frames per candidate after settling and honors
  `max_shutter`. Auto uses reported shutter×gain, then freezes the selected
  manual setting. This preserves the selection policy, not byte-identical probe
  behavior; compare it with the old path on the Pi.
- Build deployment binaries against the Pi's installed libcamera headers and
  libraries. The Debian container verifies the API/build, not Pi pipeline or ABI
  compatibility. `vio_live` and the Python service remain the rollback path.

Reference contracts: [libcamera sensor timestamp definition](https://github.com/raspberrypi/libcamera/blob/main/src/libcamera/control_ids_core.yaml),
[Pi raw pipeline](https://github.com/raspberrypi/libcamera/blob/main/src/libcamera/pipeline/rpi/pisp/pisp.cpp).
