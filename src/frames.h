#pragma once

#include "estimate.h"

namespace vio {

struct CameraMount {
    /// Camera optical-axis depression below aircraft forward; positive points down.
    double tilt_below_horizontal_rad;
    /// Camera is rolled 180 degrees about its optical axis relative to upright.
    bool upside_down;
};

/// Converts estimator coordinates to the ArduPilot interface using a fixed mount.
/// Rotation names use destination_from_source: multiplying a source-frame vector
/// by that matrix expresses the same vector in the destination frame.
class FrameTransform {
public:
    /**
     * Build the fixed IMU-to-aircraft-body rotation from calibration and mounting.
     * camera_from_imu is the rotation in Kalibr's T_cam_imu, not its inverse.
     * Body axes are forward/right/down; camera axes are right/down/forward.
     * Throws std::invalid_argument for a nonfinite tilt or an invalid rotation
     * (nonfinite, nonorthogonal, or determinant different from +1).
     */
    FrameTransform(const Eigen::Matrix3d& camera_from_imu, CameraMount mounting);

    /**
     * Convert a world-frame estimate to local NED position/velocity and body RPY.
     * Position remains at the IMU origin; ArduPilot applies the lever-arm offset.
     * Roll, pitch, and yaw are in radians. Timestamp and generation are preserved.
     * The caller owns heading alignment, freshness checks, and permission to send.
     * Throws std::invalid_argument for nonfinite pose/velocity, a nonfinite
     * quaternion norm, or a quaternion norm below 1e-12; otherwise normalizes it.
     */
    ArduPilotMeasurement transform(const EstimatorEstimate& estimate) const;

    /**
     * Predict accelerometer specific force in IMU axes for a level, stationary
     * aircraft, in m/s^2. Its body-frame value is (0, 0, -gravity): an accelerometer
     * at rest measures upward support force, not downward gravitational acceleration.
     * Throws std::invalid_argument if gravity_mps2 is nonfinite or nonpositive.
     */
    Eigen::Vector3d expected_stationary_acceleration(double gravity_mps2 = 9.80665) const;

private:
    Eigen::Matrix3d body_from_imu_;
};

} // namespace vio
