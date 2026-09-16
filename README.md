# Flying a drone without GPS using onboard vision

This project uses visual-inertial odometry (VIO)—combining camera images with motion-sensor readings to estimate how the drone moves.

I built the hardware and software integration needed to run OpenVINS onboard a small quadcopter. The work included synchronizing the sensors, calibrating their alignment, running the estimator on a Raspberry Pi, and translating its output into movement estimates ArduPilot can use.
I also built tools to simulate flights, replay recordings, measure error, and diagnose problems during real flight tests.

https://github.com/user-attachments/assets/e7e14323-1aef-48ec-923d-263cbb5268eb

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

The onboard software runs without ROS; simulation uses ROS 2 and Gazebo.

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

See the [setup and deployment guide](docs/setup.md) for Raspberry Pi deployment,
simulation, replay, and configuration details.
