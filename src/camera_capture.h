#pragma once

#include "camera_source.h"
#include "flight_config.h"
#include <functional>

namespace vio {
/// Keep the fixed/auto/sweep exposure policy while servicing the FC between probe frames.
Exposure select_exposure(const FlightConfig&, CameraSource&, const std::function<bool()>& pump);

/// Strip row padding, checking the Pi's R8-in-16 low bytes before extracting mono8.
std::vector<std::uint8_t> copy_mono_frame(const std::uint8_t* data, std::size_t bytes,
                                          unsigned stride, bool high_byte);

/// Convert documented libcamera BOOTTIME to IIO MONOTONIC; reject stale/regressing timestamps.
std::int64_t camera_timestamp(std::int64_t sensor_ns, std::int64_t boot_minus_monotonic,
                              std::int64_t now_ns, std::int64_t previous_ns);
} // namespace vio
