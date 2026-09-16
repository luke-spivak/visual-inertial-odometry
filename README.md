# Flying a drone without GPS using onboard vision
This project uses visual-inertial odometry (VIO)—combining camera images with motion-sensor readings to estimate how the drone moves.

I built the hardware and software integration needed to run OpenVINS onboard a small quadcopter. The work included synchronizing the sensors, calibrating their alignment, running the estimator on a Raspberry Pi, and translating its output into movement estimates ArduPilot can use.
I also built tools to simulate flights, replay recordings, measure error, and diagnose problems during real flight tests.

## How it works
A camera captures the scene while a motion sensor measures acceleration and rotation. OpenVINS combines these readings on a Raspberry Pi to estimate the drone’s position, orientation, and velocity. My software sends those estimates to ArduPilot, which controls the motors to execute the flight.
All motion estimation runs onboard.

## Results
- Real flight: Completed an autonomous takeoff, position hold, and landing using VIO for both horizontal positioning and altitude.
- Simulation: Completed drone missions without GPS using vision-based navigation.

## Where things live

| Directory | Contents |
|---|---|
| [src/](src/) | Pi flight software, runtime configuration, missions, and tests |
| [sim/](sim/) | Gazebo worlds, ROS bridge, simulation configuration and mission scripts |
| [tools/](tools/) | Calibration, replay, analysis, diagnostics, and build scripts |
| [docs/](docs/) | Setup, hardware specifications, development history, and archived plans |
| [results/](results/) | Recorded measurements, plots, and experiment reports |

The live path is `src/vio_flight.py` → `src/vio_live.py` →
`src/openvins_runner/vio_live.cpp`, with `src/vio_mavlink.py` sending the resulting
poses to ArduPilot. `src/imu_log.py` also provides the sensor setup used at runtime.
Hardware runs without ROS; the simulation bridge lives in `sim/ros2/vio_bridge/`.

## Run and test

Run commands from the repository root. The hardware and simulation require
different environments; see [setup and deployment](docs/setup.md).

For the standalone Python tests on a desktop:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r tools/requirements-test.txt
python -m pytest -q src tools/test_render_tracking.py sim/ros2/vio_bridge/test
```

Common workflows, after setting up the corresponding environment:

```sh
# Build and deploy the live software to the Pi (uses SSH; changes the Pi).
PI=viopi bash tools/build_vio.sh

# Run a recorded simulation in the configured Linux/ROS environment.
WORLD=iris_field_vio.sdf RECORD_SENSORS=1 bash sim/run_sim_vio.sh example
bash tools/analyze_run.sh ~/vio_runs/simvio_example

# Score a retained handheld estimate without hardware or ROS.
python tools/vio_closure.py results/vio_walk3_2026-09-10-live-estimate.txt
```

Simulation configuration is in `sim/config/`; hardware configuration is in
`src/config/`. The latter is specific to the calibrated sensor assembly.
Mission plans are in `src/missions/`. Historical flight-controller parameters
are in `results/config-snapshots/`, not a current recommended configuration.

Large datasets and local environments stay outside the tracked source tree.
Historical experiment records may refer to paths used before the directory
reorganization; the commands above and the setup guide use the current layout.
