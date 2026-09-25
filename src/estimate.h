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

enum class EstimatorStatus {
    Initializing,
    Ready
};

// OpenVINS snapshot in its z-up, arbitrary-heading world frame.
// Copy under the estimator's ownership; consumers never access OpenVINS state.
struct EstimatorEstimate {
    MonotonicTime timestamp{0};
    SessionGeneration generation{0};
    EstimatorStatus status{EstimatorStatus::Initializing};
    Eigen::Vector3d position_world_m{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_world_mps{Eigen::Vector3d::Zero()};
    // Hamilton IMU -> world rotation. OpenVINS JPL xyzw has the same numeric
    // coefficients, but Eigen's constructor takes w, x, y, z in that order.
    Eigen::Quaterniond world_from_imu{Eigen::Quaterniond::Identity()};
};

// ArduPilot interface values: IMU-origin position/velocity in local NED axes
// and aircraft-body attitude. Heading alignment and publication eligibility
// remain the flight session's responsibility.
struct ArduPilotMeasurement {
    MonotonicTime timestamp;
    SessionGeneration generation;
    Eigen::Vector3d position_ned_m;
    Eigen::Vector3d velocity_ned_mps;
    Eigen::Vector3d roll_pitch_yaw_rad;
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
EstimateRejection validate_estimate(const EstimatorEstimate& estimate, SessionGeneration session,
                                    MonotonicTime now, MonotonicTime max_age);

} // namespace vio
