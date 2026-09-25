#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>

namespace vio {
struct FlightConfig {
    std::filesystem::path recording_dir, runtime_dir, estimator_config;
    std::string device, exposure_mode, verbosity;
    unsigned baud, watermark, shutter, max_shutter;
    double min_free_gb, tilt_deg, max_lag, fps, gain, imu_lpf;
    bool upside_down, send_velocity;
    nlohmann::json snapshot;
};
/// Load the existing flight.json schema, expanding ~/ for the operating user.
FlightConfig load_flight_config(const std::filesystem::path& file,
                                const std::filesystem::path& home);
} // namespace vio
