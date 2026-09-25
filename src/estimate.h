#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>

namespace vio {

// Sensor and host timestamps must be mapped to CLOCK_MONOTONIC before use.
using MonotonicTime = std::chrono::nanoseconds;

// Identifies an estimator frame, independently of MAVLink's wrapping reset byte.
struct SessionGeneration {
  std::uint64_t value;
};

enum class EstimatorStatus { Initializing, Ready };

// Copy under the estimator's ownership; consumers never access OpenVINS state.
struct Estimate {
  MonotonicTime timestamp{0};
  SessionGeneration generation{0};
  EstimatorStatus status{EstimatorStatus::Initializing};
  Eigen::Vector3d position_world_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world_mps{Eigen::Vector3d::Zero()};
  // Hamilton IMU -> world rotation. OpenVINS JPL xyzw has the same numeric
  // coefficients, but Eigen's constructor takes w, x, y, z in that order.
  Eigen::Quaterniond world_from_imu{Eigen::Quaterniond::Identity()};
};

enum class EstimateRejection {
  None,
  NotInitialized,
  WrongSession,
  InvalidTimestamp,
  FutureTimestamp,
  Stale,
  NonFinite,
  InvalidOrientation,
};

// Stateless measurement checks only. Arming/alignment permission belongs to the
// flight session, and duplicate/out-of-order handling belongs to its publisher.
EstimateRejection validate_estimate(const Estimate& estimate, SessionGeneration session,
                                    MonotonicTime now, MonotonicTime max_age);

}  // namespace vio
