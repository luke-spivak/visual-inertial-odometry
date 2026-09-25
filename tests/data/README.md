# Navigation reference cases

`navigation_cases.txt` freezes 30 results from the Python MAVLink bridge before
native migration. They cover five camera mounts and six aircraft attitudes.
Inputs were produced using `tests/test_mavlink_bridge.py::openvins_quat`; outputs
use the bridge's pose, velocity, and expected-acceleration conversions. Columns
are documented in the first line; angles in outputs are radians.

This is a compatibility oracle, not independent proof of the physics. The C++
test also checks physical axis mappings, a separately specified north-facing
camera attitude, inverted mounting, bad calibration, and invalid estimates.
The rounded September 14 camera/IMU rotation is test data, not a runtime default.
