# Live feature tracking videos

The flight recorder writes actual OpenVINS tracker observations. The desktop
renders those observations over the recorded camera frames; it does not rerun
the estimator. Dots indicate tracked image features, **not features accepted by
an EKF update**, and the overlay is not a ground-truth accuracy measurement.

## Setup and deployment

On the Mac, create a Python environment and install the rendering dependencies.
```sh
python3 -m venv .venv-video
source .venv-video/bin/activate
python -m pip install numpy opencv-python imageio-ffmpeg
```

The renderer uses the FFmpeg executable provided by imageio-ffmpeg. The fetch
command also requires `ssh` and `rsync` with access to `viopi`.

Build/deploy the updated live harness using the existing workflow, while the
vehicle is disarmed and the VIO service is stopped:

```sh
tools/build_vio.sh
```

This command **deploys to the Pi**; compilation during development does not.
Restart the service using the existing project procedure after deployment.

Feature logging is enabled whenever `--record` is enabled. The flight service's
low-space `--no-record` behavior also disables feature logging. No FC parameter,
mission, calibration value, or MAVLink pose format is changed by this feature.

## After a recording ends

Stop recording using the normal shutdown/service procedure. A run can span
multiple flights: disarming alone does not end the existing recording service.
Then run on the desktop (from your Python environment):

```sh
python tools/fetch_tracking.py datasets/flights
```

This fetches runs with a `.complete.json` marker from `viopi:~/vio`, then renders
each locally. It never launches an encoder on the Pi. Re-running skips MP4s
whose inputs, options, and output size are unchanged. An interrupted download
can be retried. Do not reuse or modify completed run prefixes on the Pi.

Options: `--host viopi --remote-dir ~/vio --fps 20` (quote `~/vio` when passing
it explicitly so your local shell does not expand it).

For an already-downloaded run, including an older or interrupted recording:

```sh
python tools/render_tracking.py datasets/flights/run-YYYYMMDD-HHMMSS
```

Older recordings with no feature log produce plain camera video, explicitly
labeled as having no live feature observations. Default dimensions for these
are 1280 x 800; override with `--width` and `--height` if necessary.

## Files

- `.features.jsonl`: versioned header; one record per processed camera frame;
  final record reporting dropped logger snapshots on orderly shutdown.
- `.recording.json`: image dimensions, clock domain, invocation and YAML snapshots.
- `.complete.json`: written after the camera and live process exit, if the live
  process exited successfully. It means recording finished, not flight succeeded.
- `.tracking.mp4`: H.264 video with dots, short trails, tracking count,
  initialization state, elapsed time, and capture timestamp.
- `.tracking-summary.json`: coverage, duration, timing policy and truncation warnings.

Each observation contains the original integer `SensorTimestamp`, zero-based
raw frame index, camera ID, dimensions, and `[feature_id, x, y]` in raw image
pixels. Downsampled tracker coordinates are mapped back to the original image.
The live adapter uses the pinned OpenVINS revision in `build.sh` without editing
upstream source. Feature IDs are scoped to a run.

The writer uses a bounded 64-frame queue and nonblocking producer locking.
Queue pressure drops visualization snapshots; file-open/write errors disable
feature output. Both are reported. Neither changes the pose-delivery path.
This is not a guarantee against OS-wide storage stalls; orderly shutdown drains
the writer, subject to the existing wrapper's process timeout.

## Timing and gaps

Images and observations join on exact capture timestamp **and** frame index.
The renderer refuses mismatches, unsupported schemas, invalid coordinates, and
large raw/metadata count discrepancies. Small incomplete tails are trimmed and
reported, following the raw-bag converter's eight-frame tolerance.

MP4 uses a fixed frame rate and resamples the capture timeline: timing error is
less than one output frame. Missing observations have no dots. Trails break at
observation gaps. Long camera gaps become explicitly labeled black frames,
rather than misleading frozen feature overlays. This is offline visualization
of capture-time observations, not a display of their live arrival latency.

External aircraft footage and FC logs have separate clocks. Synchronizing those
requires measured offsets/markers and is not implemented here.

## Validation

```sh
python -m unittest discover -s harness -p test_render_tracking.py -v
```

The standalone C++ test `src/vio_live/test_feature_log.cpp` runs in Linux
(the existing build container), including `/dev/full` write-failure handling.

Before a flight with the new binary, do a props-off recorded camera movement:
check dots against image features and compare logging-on/off frame drops,
update time and pose latency. Capture overhead is included in the live update
and lag metrics. Pi performance and physical pixel alignment require this
hardware check; desktop tests and compilation cannot establish them.
