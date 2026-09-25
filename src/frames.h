#pragma once

#include "estimate.h"

namespace vio {

struct CameraMount {
  double tilt_below_horizontal_rad;
  bool upside_down;
};

struct NavigationEstimate {
  MonotonicTime timestamp;
  SessionGeneration generation;
  Eigen::Vector3d position_ned_m;
  Eigen::Vector3d velocity_ned_mps;
  Eigen::Vector3d roll_pitch_yaw_rad;
};

class FrameTransform {
 public:
  // camera_from_imu is Kalibr's T_cam_imu rotation, supplied by configuration.
  FrameTransform(const Eigen::Matrix3d& camera_from_imu, CameraMount mounting);
  NavigationEstimate transform(const Estimate& estimate) const;
  Eigen::Vector3d expected_stationary_acceleration(double gravity_mps2 = 9.80665) const;

 private:
  Eigen::Matrix3d body_from_imu_;
};

}  // namespace vio
