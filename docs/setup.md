# Setup and deployment

Run repository commands from its root. This project has three environments:
a desktop for analysis/builds, the Raspberry Pi for live VIO, and a Linux
ROS 2 environment for simulation/replay.

## Desktop tests and analysis

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r tools/requirements-test.txt
python -m pytest -q src tools/test_render_tracking.py sim/ros2/vio_bridge/test
```

The test dependencies cover coordinate transforms, session management, and
tracking-video rendering. Other analysis tools can additionally need
Matplotlib, PyYAML, pymavlink, or ROS; their imports and usage comments describe
their inputs. `tools/render_tracking.py` uses FFmpeg supplied by imageio-ffmpeg.

The feature logger also has a standalone C++17 test:

```sh
c++ -std=c++17 -pthread -I src/openvins_runner \
  src/openvins_runner/test_feature_log.cpp -o /tmp/vio-feature-log-test
/tmp/vio-feature-log-test /tmp/vio-feature-log-test.jsonl
```

## Raspberry Pi

The existing target is a Pi 5 running Debian 13-based Raspberry Pi OS, with
rpicam-raw, the OV9281 camera, and the ISM330DHCX on SPI. It requires the IIO
driver/overlay and monotonic timestamp setup described in the
[development log](development-log.md). Driver build scripts, device-tree
sources, and the timestamp patch live in `tools/`. The runtime also requires
NumPy and pymavlink in the Pi's system Python environment.

The heavy C++ build uses Docker and SSH access to the Pi:

```sh
bash tools/buildenv/buildenv.sh build
PI=viopi bash tools/buildenv/buildenv.sh verify
PI=viopi bash tools/build_vio.sh
```

`verify` runs a probe on the Pi. `build_vio.sh` builds **and deploys**: it pins
OpenVINS to `69488123ed9362dd44b6f28e7f4680abbff1442b`, builds for Cortex-A76,
copies the executable/libraries/configuration to `~/vio_live/`, and copies
the runtime Python files and service unit to `~/src/` on the Pi. Deploy with
the aircraft disarmed and the service stopped. The script does not install
or restart the systemd service.

### Migrating the existing Pi service

Earlier deployments ran from `~/harness/`. The new service runs from `~/src/`.
After deploying, install the new unit **on the Pi**:

```sh
sudo cp ~/src/vio@.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable vio@$USER
```

Start it when ready for a hardware check with `sudo systemctl start vio@$USER`;
inspect logs with `journalctl -u vio@$USER -f`.
Updating this repository alone does not migrate an already installed service.

For manual live capture on the configured Pi:

```sh
sudo python3 ~/src/vio_live.py ~/vio/bench
```

To use calibration/diagnostic tools on the Pi, clone this repository there
or copy `tools/` and `src/` as siblings. Scripts such as
`tools/kalibr_capture_imucam.sh` resolve the shared IMU helper from `../src/`.
The incremental `tools/deploy_tracking.sh` remains specific to the existing
`luke` account and expects the new `~/src/` deployment; use `build_vio.sh`
for the complete deployment.

## Simulation and ROS replay

The scripts target the existing Ubuntu 24.04 / ROS 2 Jazzy environment. They
expect Gazebo Harmonic, ArduPilot SITL at `~/ardupilot`, the built Gazebo
plugin at `~/ardupilot_gazebo`, a built OpenVINS workspace at `~/ws_ov`,
pymavlink, and evo. This repository does not install those upstream projects.
The [development log](development-log.md) records the tested setup and fixes.

Clone the **whole repository** into that Linux environment; the scripts now
resolve worlds, project configuration, and helper tools from the checkout,
rather than separately copied `~/vio_gazebo`, `~/vio_openvins`, and
`~/vio_harness` directories.

Build the local ROS bridge into the workspace expected by the launcher:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --base-paths sim/ros2 \
  --build-base "$HOME/ws_vio/build" \
  --install-base "$HOME/ws_vio/install" --symlink-install
WORLD=iris_field_vio.sdf RECORD_SENSORS=1 bash sim/run_sim_vio.sh example
bash tools/analyze_run.sh ~/vio_runs/simvio_example
```

`tools/sync_to_vm.sh` and `sync_from_vm.sh` synchronize only the ROS package;
they do not deploy the entire simulation. Pass the host/workspace explicitly
when using them outside the original VM setup.

Hardware recordings can be converted with `tools/vio_bag_from_raw.py` and
replayed with `tools/replay_openvins.sh` in the ROS environment. Supply the
recording's matching estimator config through `CONFIG`; the default is the
simulation configuration.
