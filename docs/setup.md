# Setup and deployment

The Pi runtime is one C++ application, `vio_flight`. Python tools are used for
calibration, offline analysis, simulation, and test orchestration. See the
[runtime guide](../src/README.md) for ownership and data flow.

## Build and test

Core tests need CMake, a C++17 compiler, Eigen, and nlohmann-json:

```sh
cmake -S src -B build/native-core
cmake --build build/native-core
ctest --test-dir build/native-core --output-on-failure
```

The complete executable additionally needs libcamera >= 0.4, OpenCV, Ceres,
Boost, and the ROS-free OpenVINS library built with `ENABLE_ARUCO_TAGS=OFF`:

```sh
cmake -S src -B build/native -G Ninja -DVIO_BUILD_FLIGHT_APP=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DOV_SRC=/absolute/path/to/open_vins \
  -DOV_LIB=/absolute/path/to/libov_msckf_lib.so
cmake --build build/native -j1
ctest --test-dir build/native --output-on-failure
```

Build against the Pi's installed libcamera headers and libraries. The pinned
OpenVINS revision is `69488123ed9362dd44b6f28e7f4680abbff1442b`.
`tools/deploy/build_vio.sh` builds its dependency bundle in Docker; it no longer
replaces files on the Pi. `prepare_native_build.sh` installs native build
packages on the existing Pi image. `build_native_on_pi.sh` expects a staged tree
containing `src`, `tests`, the replay fixture under `results`, OpenVINS source in
`deps/open_vins`, generated MAVLink headers in `deps/mavlink-headers`, and the
preserved OpenVINS library/dependencies in `candidate/lib`. It builds, runs all
eight tests, and places the executable in `candidate`.

## Raspberry Pi deployment

The target uses an OV9281 camera and ISM330DHCX IIO sensors. Driver setup and the
existing timestamp patch are documented in the [development log](development-log.md).
The native runtime does not require Python, pymavlink, rpicam subprocesses, or ROS.

The service template expects this release layout:

```text
~/vio-native/
  vio_flight
  lib/                       OpenVINS and bundled dependencies
  config/flight.json         aircraft runtime settings
  config/*.yaml              matching calibration and estimator settings
```

Set `estimator_config` in the release JSON to its matching YAML path. Preserve
the actual aircraft calibration instead of overwriting it with an example.
Before replacing an installed service, preserve its unit and deployment as a
rollback, stop it, verify the release's `ldd` output, and complete the hardware
checks. Never run two UART publishers. The service runs with root access for IIO;
`SUDO_USER` selects the recording user's home and `Group` permits group access.

After validation, install the unit on the Pi:

```sh
sudo install -m 644 src/vio@.service /etc/systemd/system/vio@.service
sudo systemctl daemon-reload
sudo systemctl enable vio@"$USER"
sudo systemctl start vio@"$USER"
journalctl -u vio@"$USER" -f
```

Updating this repository does not change the installed Pi service. At the latest
[bench validation](../results/native-validation-2026-09-25/README.md), the native
application passed stationary/movement checks but the old deployment was retained
and stopped. Controller reboot recovery and navigation warnings remain open;
boot-service cutover and outdoor flight validation are not yet complete.

## Offline tools

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r tools/requirements-test.txt
python -m pytest -q
sudo python3 tools/sensors/imu_log.py --check
```

The standalone IMU logger imports `tools/sensors/imu_device.py`; it does not
import flight runtime code. Python tests cover analysis and simulation transforms;
native runtime tests are in CTest. Other tools may need ROS, pymavlink, or plotting
packages as described in their imports. `validate_native.py` is a bounded bench
UART tap requiring pymavlink, not an onboard runtime dependency.

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
bash tools/evaluation/analyze_run.sh ~/vio_runs/simvio_example
```

`tools/deploy/sync_to_vm.sh` and `sync_from_vm.sh` synchronize only the ROS package;
they do not deploy the entire simulation. Pass the host/workspace explicitly
when using them outside the original VM setup.

Hardware recordings can be converted with `tools/evaluation/vio_bag_from_raw.py` and
replayed with `tools/evaluation/replay_openvins.sh` in the ROS environment. Supply the
recording's matching estimator config through `CONFIG`; the default is the
simulation configuration.
