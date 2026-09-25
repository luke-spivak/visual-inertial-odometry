#pragma once

#include "flight_config.h"
#include <filesystem>
#include <functional>
#include <vector>

namespace vio {
struct Exposure {
    unsigned shutter;
    double gain;
};
/// Build the temporary rpicam-raw adapter's command without invoking a shell.
std::vector<std::string> camera_command(const FlightConfig&, Exposure,
                                        const std::filesystem::path& frames,
                                        const std::filesystem::path& metadata);
/// Select exposure using the same fixed/auto/sweep policy as the Python capture.
/// pump() services the FC and returns false when capture must stop.
Exposure select_exposure(const FlightConfig&, const std::filesystem::path& directory,
                         const std::function<bool()>& pump);
} // namespace vio
