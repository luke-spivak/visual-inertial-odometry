#include "flight_config.h"

#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>

namespace vio {
/// Validate the shared configuration before opening devices or spawning children.
FlightConfig load_flight_config(const std::filesystem::path& file,
                                const std::filesystem::path& home) {
    using namespace std::filesystem;
    nlohmann::json json;
    std::ifstream input(file);
    if (!input)
        throw std::runtime_error("cannot open flight configuration " + file.string());
    input >> json;
    const std::set<std::string> keys{
        "recording_dir", "estimate_dir", "estimator_binary", "estimator_config", "min_free_gb",
        "device",        "baud",         "tilt_deg",         "upside_down",      "send_velocity",
        "max_lag",       "fps",          "max_shutter",      "exposure_mode",    "shutter",
        "gain",          "watermark",    "imu_lpf",          "verbosity"};
    if (!json.is_object() || json.size() != keys.size())
        throw std::runtime_error("invalid flight configuration keys");
    for (const auto& key : keys)
        if (!json.contains(key))
            throw std::runtime_error("missing flight setting " + key);
    auto number = [&](const char* key, double minimum, double maximum) {
        if (!json.at(key).is_number())
            throw std::runtime_error(std::string("expected number: ") + key);
        double value = json.at(key).get<double>();
        if (!std::isfinite(value) || value < minimum || value > maximum)
            throw std::runtime_error(std::string("invalid setting: ") + key);
        return value;
    };
    auto integer = [&](const char* key, unsigned maximum) {
        if (!json.at(key).is_number_integer())
            throw std::runtime_error(std::string("expected integer: ") + key);
        return static_cast<unsigned>(number(key, 1, maximum));
    };
    auto text = [&](const char* key) {
        auto value = json.at(key).get<std::string>();
        if (value.find_first_not_of(" \t\r\n") == std::string::npos)
            throw std::runtime_error(std::string("empty setting: ") + key);
        return value;
    };
    auto resolved = [&](const char* key) {
        auto value = text(key);
        path result;
        if (value == "~")
            result = home;
        else if (value.rfind("~/", 0) == 0)
            result = home / value.substr(2);
        else if (value.front() == '~')
            throw std::runtime_error("use ~/ rather than ~username");
        else
            result = value;
        if (result.is_relative())
            result = absolute(file).parent_path() / result;
        result = result.lexically_normal();
        json[key] = result.string();
        return result;
    };
    FlightConfig c;
    c.recording_dir = resolved("recording_dir");
    c.runtime_dir = resolved("estimate_dir");
    c.estimator_config = resolved("estimator_config");
    resolved("estimator_binary");
    c.device = text("device");
    c.exposure_mode = text("exposure_mode");
    c.verbosity = text("verbosity");
    c.baud = integer("baud", 921600);
    c.watermark = integer("watermark", 1024);
    c.shutter = integer("shutter", 1000000);
    c.max_shutter = integer("max_shutter", 1000000);
    c.min_free_gb = number("min_free_gb", 0, 1e9);
    c.tilt_deg = number("tilt_deg", -360, 360);
    c.max_lag = number("max_lag", 0.001, 10);
    c.fps = number("fps", 0.1, 120);
    c.gain = number("gain", 1, 16);
    c.imu_lpf = number("imu_lpf", 0, 187);
    c.upside_down = json.at("upside_down").get<bool>();
    c.send_velocity = json.at("send_velocity").get<bool>();
    if (c.exposure_mode != "fixed" && c.exposure_mode != "auto" && c.exposure_mode != "sweep")
        throw std::runtime_error("unknown exposure mode");
    if (!std::set<std::string>{"ALL", "DEBUG", "INFO", "WARNING", "ERROR", "SILENT"}.count(
            c.verbosity))
        throw std::runtime_error("unknown verbosity");
    c.snapshot = std::move(json);
    return c;
}
} // namespace vio
