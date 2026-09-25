#include "estimate.h"

#include <cmath>
#include <stdexcept>

namespace vio {

EstimateRejection validate_estimate(const Estimate& estimate, SessionGeneration session,
                                    MonotonicTime now, MonotonicTime max_age) {
  if (now.count() < 0 || max_age.count() < 0) {
    throw std::invalid_argument("now and max_age must be nonnegative");
  }
  if (estimate.status != EstimatorStatus::Ready) return EstimateRejection::NotInitialized;
  if (estimate.generation.value != session.value) return EstimateRejection::WrongSession;
  if (estimate.timestamp.count() < 0) return EstimateRejection::InvalidTimestamp;
  if (estimate.timestamp > now) return EstimateRejection::FutureTimestamp;
  // A zero limit is permitted for offline diagnostics, matching the Python CLI.
  if (max_age.count() != 0 && now - estimate.timestamp > max_age) return EstimateRejection::Stale;
  if (!estimate.position_world_m.allFinite() || !estimate.velocity_world_mps.allFinite() ||
      !estimate.world_from_imu.coeffs().allFinite()) return EstimateRejection::NonFinite;
  const double norm = estimate.world_from_imu.coeffs().stableNorm();
  if (!std::isfinite(norm) || norm < 1e-12) return EstimateRejection::InvalidOrientation;
  return EstimateRejection::None;
}

}  // namespace vio
