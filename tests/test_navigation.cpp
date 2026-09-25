#include "estimate.h"
#include "frames.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
constexpr double pi = 3.14159265358979323846;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
          const std::string& name, double tolerance = 1e-8) {
  require(actual.allFinite() && (actual - expected).norm() < tolerance, name);
}

template <typename Function>
void rejects(Function operation) {
  bool rejected = false;
  try { operation(); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "invalid input accepted");
}

Eigen::Matrix3d calibration() {
  // Frozen reference only: production geometry is passed to FrameTransform.
  Eigen::Matrix3d rotation;
  rotation << -0.99991866, -0.01269620, 0.00121422,
              -0.01266913, 0.99971620, 0.02017451,
              -0.00147001, 0.02015749, -0.99979574;
  return rotation;
}

void reference_cases(const std::string& path) {
  std::ifstream file(path);
  require(file.good(), "cannot open Python reference cases");
  std::string line;
  int count = 0;
  while (std::getline(file, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream row(line);
    double tilt, x, y, z, w;
    bool upside_down;
    vio::Estimate estimate;
    Eigen::Vector3d position, rpy, velocity, acceleration;
    row >> tilt >> upside_down >> x >> y >> z >> w;
    for (int i = 0; i < 3; ++i) row >> estimate.position_world_m[i];
    for (int i = 0; i < 3; ++i) row >> estimate.velocity_world_mps[i];
    for (int i = 0; i < 3; ++i) row >> position[i];
    for (int i = 0; i < 3; ++i) row >> rpy[i];
    for (int i = 0; i < 3; ++i) row >> velocity[i];
    for (int i = 0; i < 3; ++i) row >> acceleration[i];
    require(!row.fail(), "malformed reference case");
    estimate.world_from_imu = Eigen::Quaterniond(w, x, y, z);
    estimate.timestamp = vio::MonotonicTime{123456789};
    estimate.generation = {42};
    vio::FrameTransform transform(calibration(), {tilt * pi / 180, upside_down});
    const auto navigation = transform.transform(estimate);
    near(navigation.position_ned_m, position, "position parity");
    near(navigation.velocity_ned_mps, velocity, "velocity parity");
    for (int i = 0; i < 3; ++i) {
      const double error = navigation.roll_pitch_yaw_rad[i] - rpy[i];
      require(std::abs(std::atan2(std::sin(error), std::cos(error))) < 1e-8, "attitude parity");
    }
    near(transform.expected_stationary_acceleration(), acceleration, "gravity parity");
    require(navigation.timestamp == estimate.timestamp && navigation.generation.value == 42,
            "timestamp or session changed in conversion");
    // Opposite signs represent the same quaternion. Normalize finite scale too.
    estimate.world_from_imu.coeffs() *= -3;
    near(transform.transform(estimate).roll_pitch_yaw_rad,
         navigation.roll_pitch_yaw_rad, "quaternion sign and scale");
    ++count;
  }
  require(count == 30, "reference cases missing");
}

void physical_cases() {
  const vio::FrameTransform transform(Eigen::Matrix3d::Identity(), {0, false});
  vio::Estimate estimate;
  // Level, north-facing body with camera optical axes aligned to the IMU.
  // World x=east, y=north, z=up; optical x=right, y=down, z=forward.
  estimate.world_from_imu = Eigen::AngleAxisd(-pi / 2, Eigen::Vector3d::UnitX());
  near(transform.transform(estimate).roll_pitch_yaw_rad, Eigen::Vector3d::Zero(),
       "independent north-facing attitude");
  near(transform.expected_stationary_acceleration(), Eigen::Vector3d(0, -9.80665, 0),
       "upright optical gravity");
  const vio::FrameTransform inverted(Eigen::Matrix3d::Identity(), {0, true});
  near(inverted.expected_stationary_acceleration(), Eigen::Vector3d(0, 9.80665, 0),
       "inverted optical gravity");
  for (int axis = 0; axis < 3; ++axis) {
    estimate.position_world_m = Eigen::Vector3d::Unit(axis);
    estimate.velocity_world_mps = estimate.position_world_m;
    const auto result = transform.transform(estimate);
    const Eigen::Vector3d expected = axis == 0 ? Eigen::Vector3d(0, 1, 0) :
                                     axis == 1 ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(0, 0, -1);
    near(result.position_ned_m, expected, "physical position axis");
    near(result.velocity_ned_mps, expected, "physical velocity axis");
  }
  Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
  reflection(2, 2) = -1;
  rejects([&] { vio::FrameTransform invalid(reflection, {0, false}); });
  rejects([&] { vio::FrameTransform invalid(2 * Eigen::Matrix3d::Identity(), {0, false}); });
  rejects([&] { vio::FrameTransform invalid(Eigen::Matrix3d::Identity(), {NAN, false}); });
  estimate.world_from_imu.coeffs().setZero();
  rejects([&] { transform.transform(estimate); });
}

void validity_cases() {
  using vio::EstimateRejection;
  using vio::MonotonicTime;
  vio::Estimate estimate;
  estimate.timestamp = MonotonicTime{1000};
  estimate.generation = {3};
  auto check = [&](EstimateRejection expected, long age = 500) {
    require(vio::validate_estimate(estimate, {3}, MonotonicTime{1500}, MonotonicTime{age}) == expected,
            "wrong validation result");
  };
  check(EstimateRejection::NotInitialized);
  estimate.status = vio::EstimatorStatus::Ready;
  check(EstimateRejection::None);  // Exactly at age limit is permitted.
  estimate.timestamp = MonotonicTime{999};
  check(EstimateRejection::Stale);
  check(EstimateRejection::None, 0);  // Disabled age gate for offline use.
  estimate.timestamp = MonotonicTime{1501};
  check(EstimateRejection::FutureTimestamp);
  estimate.timestamp = MonotonicTime{-1};
  check(EstimateRejection::InvalidTimestamp);
  estimate.timestamp = MonotonicTime{1000};
  estimate.generation = {4};
  check(EstimateRejection::WrongSession);
  estimate.generation = {3};
  estimate.position_world_m.x() = NAN;
  check(EstimateRejection::NonFinite);
  estimate.position_world_m.setZero();
  estimate.velocity_world_mps.y() = std::numeric_limits<double>::infinity();
  check(EstimateRejection::NonFinite);
  estimate.velocity_world_mps.setZero();
  estimate.world_from_imu.coeffs().setZero();
  check(EstimateRejection::InvalidOrientation);
  estimate.world_from_imu.coeffs()[0] = NAN;
  check(EstimateRejection::NonFinite);
  rejects([&] { vio::validate_estimate(estimate, {3}, MonotonicTime{0}, MonotonicTime{-1}); });
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 2, "provide reference fixture path");
    reference_cases(argv[1]);
    physical_cases();
    validity_cases();
    std::cout << "navigation: reference, physical-frame, and validation cases passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
