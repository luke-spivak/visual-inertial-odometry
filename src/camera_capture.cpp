#include "camera_capture.h"
#include "process.h"

#include <array>
#include <cmath>
#include <csignal>
#include <fstream>
#include <stdexcept>
#include <thread>

namespace vio {
namespace {
/// Bound camera probes and keep servicing shutdown/controller input while waiting.
void probe(const std::vector<std::string>& command, const std::filesystem::path& directory,
           const std::function<bool()>& pump) {
    ChildProcess child(command, directory / "probe.log");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (!child.poll()) {
        if (!pump())
            throw std::runtime_error("camera probe cancelled");
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error("camera probe timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (*child.poll() != 0)
        throw std::runtime_error("camera exposure probe failed");
}
} // namespace

std::vector<std::string> camera_command(const FlightConfig& config, Exposure exposure,
                                        const std::filesystem::path& frames,
                                        const std::filesystem::path& metadata) {
    return {"rpicam-raw",
            "-n",
            "--mode",
            "1280:800:8",
            "--width",
            "1280",
            "--height",
            "800",
            "--framerate",
            std::to_string(config.fps),
            "-o",
            frames.string(),
            "-t",
            "0",
            "--shutter",
            std::to_string(exposure.shutter),
            "--gain",
            std::to_string(exposure.gain),
            "--metadata",
            metadata.string(),
            "--metadata-format",
            "json",
            "--flush"};
}

Exposure select_exposure(const FlightConfig& config, const std::filesystem::path& directory,
                         const std::function<bool()>& pump) {
    if (config.exposure_mode == "fixed")
        return {config.shutter, config.gain};
    const auto frames = directory / "probe.y16", metadata = directory / "probe.json";
    if (config.exposure_mode == "auto") {
        probe({"rpicam-raw", "-n", "--mode", "1280:800:8", "--width", "1280", "--height", "800",
               "--framerate", "5", "-o", frames.string(), "-t", "2500", "--metadata",
               metadata.string(), "--metadata-format", "json"},
              directory, pump);
        nlohmann::json rows;
        std::ifstream(metadata) >> rows;
        if (!rows.is_array() || rows.empty())
            throw std::runtime_error("camera probe has no metadata");
        const double product = rows.back().at("ExposureTime").get<double>() *
                               rows.back().at("AnalogueGain").get<double>();
        if (!std::isfinite(product) || product < 1 || product > config.max_shutter * 16.0)
            throw std::runtime_error("camera exposure out of range");
        unsigned shutter =
            static_cast<unsigned>(std::min(product, static_cast<double>(config.max_shutter)));
        return {shutter, std::max(1.0, product / shutter)};
    }
    struct Result {
        unsigned shutter;
        double clipped, mean;
    };
    std::vector<Result> results;
    for (unsigned shutter : {20, 30, 50, 75, 100, 200, 500, 1000, 2000, 4000}) {
        std::filesystem::remove(frames);
        probe({"rpicam-raw", "-n", "--mode", "1280:800:8", "--width", "1280", "--height", "800",
               "--framerate", "20", "-o", frames.string(), "--frames", "30", "--shutter",
               std::to_string(shutter), "--gain", "1"},
              directory, pump);
        const auto bytes = std::filesystem::file_size(frames);
        if (!bytes || bytes % (1280 * 800 * 2) || bytes > 30ULL * 1280 * 800 * 2)
            throw std::runtime_error("invalid exposure probe frames");
        std::ifstream input(frames, std::ios::binary);
        std::array<unsigned char, 8192> block;
        std::uint64_t sum = 0, clipped = 0, pixels = 0;
        while (input.read(reinterpret_cast<char*>(block.data()), block.size()) || input.gcount()) {
            for (std::streamsize i = 0; i < input.gcount(); i += 2) {
                if (block[i] != 0)
                    throw std::runtime_error("camera probe is not R8-in-16 format");
                sum += block[i + 1];
                clipped += block[i + 1] >= 250;
                ++pixels;
            }
            if (!pump())
                throw std::runtime_error("camera probe cancelled");
        }
        if (pixels * 2 != bytes)
            throw std::runtime_error("incomplete exposure probe read");
        results.push_back({shutter, double(clipped) / pixels, double(sum) / pixels});
    }
    auto chosen = results.front();
    bool usable = false;
    for (const auto& result : results) {
        bool candidate = result.clipped <= 0.01 && result.mean >= 15;
        if ((candidate && (!usable || result.mean > chosen.mean)) ||
            (!candidate && !usable && result.clipped < chosen.clipped))
            chosen = result;
        usable = usable || candidate;
    }
    return {chosen.shutter, 1};
}
} // namespace vio
