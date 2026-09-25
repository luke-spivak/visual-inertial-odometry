#pragma once

#include "estimator_runner.h"
#include "flight_config.h"
#include "serial_port.h"
#include <functional>

namespace vio {
struct HardwarePaths {
    std::filesystem::path sysfs{"/sys/bus/iio/devices"}, devices{"/dev"};
};
/// Own capture/restart/shutdown while keeping all MAVLink I/O on the calling thread.
/// Sensor roots and the estimator factory are replaceable for hardware-free tests.
void run_flight(const FlightConfig&, const std::function<std::unique_ptr<ByteStream>()>&,
                const std::function<std::unique_ptr<EstimatorRunner>()>& make_estimator,
                const std::function<bool()>& stopping, HardwarePaths paths = {});
} // namespace vio
