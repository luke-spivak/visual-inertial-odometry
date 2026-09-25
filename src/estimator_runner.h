#pragma once

#include "camera_source.h"
#include "estimate.h"
#include "imu_device.h"
#include "recording.h"
#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace vio {

/// One-slot handoff: producers replace old snapshots rather than building latency.
class LatestEstimate {
public:
    void store(EstimatorEstimate estimate) {
        std::lock_guard<std::mutex> guard(mutex_);
        value_ = std::move(estimate);
    }
    std::optional<EstimatorEstimate> load() const {
        std::lock_guard<std::mutex> guard(mutex_);
        return value_;
    }

private:
    mutable std::mutex mutex_;
    std::optional<EstimatorEstimate> value_;
};

struct RunnerOptions {
    std::vector<ImuDevice> devices;
    std::shared_ptr<RecordingStatus> recording_status;
    CameraSource* camera{nullptr}; // Owned by the capture session; null selects legacy FIFOs.
    std::filesystem::path frames, metadata, estimate_log, recording_prefix;
    double imu_lpf_hz{50}, imu_rate_hz{416}, camera_fps{20};
    std::size_t max_camera_queue{10};
    SessionGeneration generation{0};
};

/// Adapter boundary lets lifecycle tests replace sensors/OpenVINS without hardware.
class EstimatorRunner {
public:
    virtual ~EstimatorRunner() = default;
    virtual Eigen::Matrix3d camera_from_imu() const = 0;
    virtual void run(const RunnerOptions&, std::atomic<bool>& stop, LatestEstimate&) = 0;
};

std::unique_ptr<EstimatorRunner> make_openvins_runner(const std::filesystem::path& config,
                                                      const std::string& verbosity);
} // namespace vio
