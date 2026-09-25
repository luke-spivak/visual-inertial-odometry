#include "frames.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vio {
namespace {

Eigen::Matrix3d ned_from_world() {
  Eigen::Matrix3d rotation;
  rotation << 0, 1, 0,
              1, 0, 0,
              0, 0, -1;
  return rotation;
}

Eigen::Matrix3d body_from_camera(CameraMount mount) {
  const double c = std::cos(mount.tilt_below_horizontal_rad);
  const double s = std::sin(mount.tilt_below_horizontal_rad);
  Eigen::Matrix3d rotation;
  if (mount.upside_down) {
    rotation << 0, s, c,
               -1, 0, 0,
                0, -c, s;
  } else {
    rotation << 0, -s, c,
                1, 0, 0,
                0, c, s;
  }
  return rotation;
}

}  // namespace

FrameTransform::FrameTransform(const Eigen::Matrix3d& camera_from_imu, CameraMount mounting) {
  if (!camera_from_imu.allFinite() || !std::isfinite(mounting.tilt_below_horizontal_rad) ||
      (camera_from_imu * camera_from_imu.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
      std::abs(camera_from_imu.determinant() - 1.0) > 1e-6) {
    throw std::invalid_argument("camera/IMU calibration must be a finite proper rotation");
  }
  body_from_imu_ = body_from_camera(mounting) * camera_from_imu;
}

NavigationEstimate FrameTransform::transform(const Estimate& estimate) const {
  // Status, age, and session eligibility are checked by the caller. Reject
  // malformed numeric input here too rather than manufacturing an orientation.
  const double norm = estimate.world_from_imu.coeffs().stableNorm();
  if (!estimate.position_world_m.allFinite() || !estimate.velocity_world_mps.allFinite() ||
      !estimate.world_from_imu.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-12) {
    throw std::invalid_argument("estimate contains invalid position, velocity, or quaternion");
  }
  Eigen::Quaterniond orientation = estimate.world_from_imu;
  orientation.coeffs() /= norm;
  // World is z-up with arbitrary yaw; ArduPilot alignment establishes heading.
  const Eigen::Matrix3d ned_from_body = ned_from_world() * orientation.toRotationMatrix() *
                                       body_from_imu_.transpose();
  Eigen::Vector3d rpy(std::atan2(ned_from_body(2, 1), ned_from_body(2, 2)),
                      -std::asin(std::clamp(ned_from_body(2, 0), -1.0, 1.0)),
                      std::atan2(ned_from_body(1, 0), ned_from_body(0, 0)));
  // Send IMU-origin position. ArduPilot's VISO_POS settings apply the lever arm.
  // Velocity is already world-frame, so the mounting rotation does not apply.
  return {estimate.timestamp, estimate.generation,
          ned_from_world() * estimate.position_world_m,
          ned_from_world() * estimate.velocity_world_mps, rpy};
}

Eigen::Vector3d FrameTransform::expected_stationary_acceleration(double gravity_mps2) const {
  if (!std::isfinite(gravity_mps2) || gravity_mps2 <= 0) {
    throw std::invalid_argument("gravity must be finite and positive");
  }
  return body_from_imu_.transpose() * Eigen::Vector3d(0, 0, -gravity_mps2);
}

}  // namespace vio
