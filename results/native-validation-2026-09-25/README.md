# Pi native validation — 2026-09-25

Hardware: Pi 5 (4 GB), OV9281, ISM330DHCX, ArduPilot over
`/dev/ttyAMA0` at 230400 baud. The user confirmed stationary bench placement
with propellers removed. The existing Python service was stopped before testing;
its service unit and deployment were preserved. The IMU driver was not changed.

## Build

The native application built on the Pi against libcamera 0.7.2 and the existing
OpenVINS library/calibration. All eight CTest tests passed (7.67 seconds).
Artifacts are staged at `/home/luke/vio-native-staged` on the Pi.

Stale APT indexes initially caused unixodbc package 404s. Refreshing the indexes
resolved installation. The running Python service then filled the SD card with
raw images. With explicit user permission, only
`/home/luke/vio/run-20260925-110252.y16` (40,718,336,000 bytes) was deleted,
recovering 36 GiB available space. Its remaining files are not a complete replay.

## First stationary run

- Exposure sweep and native camera startup succeeded: 1280×800 R16, 2560-byte
  stride, 8-bit sensor data in the high byte. All 1,206 frames were processed,
  with zero sequence gaps or dropped frames. Measured camera rate: 19.20 Hz.
- IMU rate: 439.56 Hz, despite the configured/read-back 416 Hz. Both timestamp
  streams were strictly increasing; maximum sample interval was 2.302 ms.
  This is an observed pre-existing discrepancy, not a resolved timing issue.
- 1,177 pose/velocity pairs were observed. The FC accepted the alignment
  command addressed to component 197 and reported a visual-odometry yaw shift.
  This establishes command acceptance, not dynamic heading accuracy.
- Pose age p95: 49.30 ms; maximum: 626.15 ms. Three observations exceeded
  500 ms. The first diagnostic forwarded UART traffic and synchronously wrote
  its log, so storage delays could affect this measurement. A repeat run uses
  bounded memory for wire evidence and writes it after capture stops.
- SIGTERM produced exit code 0, complete recording accounting with no rejected
  records, and a completion marker. Both IIO buffers read disabled afterward.

The first `summary.json` contains `passed: true` because the initial diagnostic
checked p95 only. That is not a deployment approval: the diagnostic now requires
the maximum observed age to meet the limit. Preserve this original evidence.

## Repeat and guided movement

The second 90-second run, with disk writes removed from the diagnostic's forwarding
loop, passed: 1,160 poses, age p95 49.22 ms, maximum 72.92 ms. Its reset byte
advanced to 2 and alignment was accepted. All 1,190 camera frames were recorded
without sequence gaps; all recording queues drained without rejected records.

The third run lasted 150 seconds. After initialization, the user performed a
guided hand translation/rotation and confirmed setting the aircraft down. It
used the previous sweep's exposure (4000 us, gain 1). Results:

- 2,816 poses; age p95 61.96 ms, maximum 88.57 ms; reset byte 3.
- 2,846 camera frames, no sequence gaps, no reported dropped frames. The last
  received frame was recorded but not processed before requested shutdown.
- Estimated position spanned 0.352/0.163/0.094 m along its three axes. During the
  last ten seconds, estimated speed stayed below 0.0046 m/s. The stationary
  zero-velocity update contributes to this stability; it is not ground truth.
- Both IMU streams remained monotonic at 439.565 Hz; maximum interval 2.302 ms.
- Recording accounting completed without rejected records; exit code 0.
- FC alignment was accepted, but the controller intermittently reported
  `GPS Glitch or Compass error` followed by `Glitch cleared`. This needs diagnosis
  and comparison with the preserved application before cutover.

## Deployment gates

Cutover is not yet complete. Remaining checks include the controller glitch
messages, dynamic camera/IMU timing validation, and the documented controller
reboot/recovery behavior. The native link does not yet reliably revoke old
alignment on a controller reboot shorter than its heartbeat timeout. The IMU
driver is intentionally unchanged. No flight was attempted.
