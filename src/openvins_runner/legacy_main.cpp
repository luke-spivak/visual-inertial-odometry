// Compatibility entry point for the existing Python capture supervisor.
#include "estimator_runner.h"

#include <csignal>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
std::atomic<bool> stop{false};
static_assert(std::atomic<bool>::is_always_lock_free);
void on_signal(int) {
    stop.store(true);
}
} // namespace

int main(int argc, char** argv) {
    try {
        vio::RunnerOptions options;
        options.imu_lpf_hz = 0;
        options.imu_rate_hz =
            440; // Preserve the old command's default; native setup uses readback.
        std::string imu_config, config, verbosity = "WARNING";
        for (int i = 1; i < argc; ++i) {
            std::string key = argv[i];
            if (i + 1 == argc)
                throw std::runtime_error("missing value for " + key);
            std::string value = argv[++i];
            if (key == "--imu")
                imu_config = value;
            else if (key == "--frames")
                options.frames = value;
            else if (key == "--meta")
                options.metadata = value;
            else if (key == "--config")
                config = value;
            else if (key == "--out")
                options.estimate_log = value;
            else if (key == "--record")
                options.recording_prefix = value;
            else if (key == "--verbosity")
                verbosity = value;
            else if (key == "--imu-lpf")
                options.imu_lpf_hz = std::stod(value);
            else if (key == "--max-queue")
                options.max_camera_queue = std::stoul(value);
            else if (key == "--width" || key == "--height") {
                if (std::stoi(value) != (key == "--width" ? 1280 : 800))
                    throw std::runtime_error("camera dimensions must match 1280x800 calibration");
            } else if (key == "--init-verbosity") {
                if (value != "INFO")
                    throw std::runtime_error("initialization verbosity currently requires INFO");
            } else if (key == "--imu-rate")
                options.imu_rate_hz = std::stod(value);
            else
                throw std::runtime_error("unknown argument " + key);
        }
        if (imu_config.empty() || config.empty() || options.frames.empty() ||
            options.metadata.empty() || options.estimate_log.empty())
            throw std::runtime_error(
                "usage: vio_live --imu FILE --frames FIFO --meta FIFO --config YAML --out FILE "
                "[--record PREFIX] [--imu-lpf HZ] [--imu-rate HZ]");
        std::ifstream input(imu_config);
        std::string line;
        while (std::getline(input, line)) {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream row(line);
            vio::ImuDevice d;
            std::string path;
            if (!(row >> d.kind >> path >> d.record_bytes >> d.scale >> d.timestamp_offset >>
                  d.axis_offsets[0] >> d.axis_offsets[1] >> d.axis_offsets[2]))
                throw std::runtime_error("bad IMU configuration");
            d.chardev = path;
            options.devices.push_back(d);
        }
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);
        auto runner = vio::make_openvins_runner(config, verbosity);
        vio::LatestEstimate estimates;
        runner->run(options, stop, estimates);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "vio_live: " << error.what() << '\n';
        return 1;
    }
}
