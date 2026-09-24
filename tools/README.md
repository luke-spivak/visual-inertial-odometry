# Tools

Tools are grouped by purpose. Run the examples from the repository root;
script-local helpers and configuration resolve relative to the script location.
Python dependencies vary by tool; see its imports and usage comments and the
[setup guide](../docs/setup.md). Desktop test dependencies are in
[requirements-test.txt](requirements-test.txt).

| Directory | Tools and supporting files | Environment |
|---|---|---|
| [sensors/](sensors/) | Camera streaming, focus and timestamp checks; IMU probing, capture and IRQ checks; driver build, overlays and patch | Raspberry Pi |
| [calibration/](calibration/) | Kalibr setup, camera/IMU capture, bag and CSV conversion, calibration comparison, Allan deviation and capture profiling; target PDFs/YAMLs and IMU noise configuration | Capture on Pi; Kalibr on Linux/container; analysis on desktop |
| [evaluation/](evaluation/) | Bag conversion, replay, repeated runs, ablations, trajectory scoring, simulation and recorded-sensor diagnostics | ROS 2 for bags/replay; desktop for file-based analysis |
| [tracking/](tracking/) | Fetch recordings, render live or accepted feature tracks, renderer tests | Desktop; SSH/rsync for fetching |
| [deploy/](deploy/) | Build/deploy live VIO, deploy tracking, synchronize ROS packages with the VM | Desktop with Docker and/or SSH |
| [ardupilot/](ardupilot/) | Flight-controller firmware build and feature configurations | ArduPilot build environment |
| [buildenv/](buildenv/) | Container image and target ABI verification | Desktop with Docker; SSH to Pi for verification |

## Common commands

```sh
# Score a retained recording without ROS or hardware.
python tools/evaluation/vio_closure.py results/vio_walk3_2026-09-10-live-estimate.txt

# Analyze a recorded simulation run (ROS 2/OpenVINS/evo environment).
bash tools/evaluation/analyze_run.sh ~/vio_runs/simvio_example

# Fetch Pi recordings and render tracking footage locally.
python tools/tracking/fetch_tracking.py datasets/flights

# Render an already downloaded recording.
python tools/tracking/render_tracking.py datasets/flights/run-YYYYMMDD-HHMMSS

# Inspect camera focus on the Pi.
python3 tools/sensors/focus_check.py --help

# Compare a Kalibr result with a reference calibration.
python tools/calibration/kalibr_compare.py camchain.yaml --reference reference.yaml

# Run the standalone desktop test suite.
python -m pytest -q
```

Build and deployment commands are in the [setup guide](../docs/setup.md#raspberry-pi).
`deploy/build_vio.sh` builds **and deploys** to the Pi.

## Copying tools to another machine

Preserve the directory structure. On the Pi, copy `tools/` and `src/` as
siblings (or clone the repository); sensor and calibration scripts use the
shared IMU logger in `src/`. ROS replay also needs `sim/config/`, so use the
whole checkout. `evaluation/vio_bag_from_raw.py` loads its IMU reader from
`calibration/kalibr_imu_csv.py`; copy both directories together.

The former flat `tools/<script>` paths have moved to the directories above;
update any external commands or saved shortcuts. Filenames are unchanged.
Historical experiment records retain the commands used at the time.
