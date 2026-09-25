#include "imu_device.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace vio {
namespace {
namespace fs = std::filesystem;

/// Read one sysfs attribute; a missing or unreadable setting is a startup error.
std::string read(const fs::path& path) {
    std::ifstream file(path);
    std::string value;
    std::getline(file, value);
    if (!file || value.empty())
        throw std::runtime_error("cannot read IIO attribute " + path.string());
    return value;
}

/// Write an attribute and check close-time errors, as sysfs can reject settings.
void write(const fs::path& path, const std::string& value) {
    if (!fs::exists(path))
        throw std::runtime_error("missing IIO attribute " + path.string());
    std::ofstream file(path);
    file << value;
    file.close();
    if (!file)
        throw std::runtime_error("cannot write IIO attribute " + path.string());
}

/// Locate the old or new kernel's buffer/scan directory without assuming its name.
fs::path directory(const fs::path& device, bool scan) {
    for (const auto* name : {"scan_elements", "buffer0", "buffer"}) {
        auto path = device / name;
        if (!fs::is_directory(path))
            continue;
        if (!scan && std::string(name) != "scan_elements")
            return path;
        if (scan)
            for (const auto& entry : fs::directory_iterator(path)) {
                auto filename = entry.path().filename().string();
                if (filename.size() > 3 && filename.substr(filename.size() - 3) == "_en")
                    return path;
            }
    }
    throw std::runtime_error("missing IIO buffer/scan directory: " + device.string());
}

/// Choose the driver's nearest supported scale and use its readback for decoding.
double select_scale(const fs::path& device, const std::string& attribute, double target) {
    std::istringstream values(read(device / (attribute + "_available")));
    double candidate, best = 0;
    while (values >> candidate) {
        if (!std::isfinite(candidate) || candidate <= 0)
            throw std::runtime_error("invalid IIO scale");
        if (best == 0 || std::abs(candidate - target) < std::abs(best - target))
            best = candidate;
    }
    if (best == 0)
        throw std::runtime_error("no available IIO scales");
    std::ostringstream value;
    value << std::setprecision(12) << best;
    write(device / attribute, value.str());
    best = std::stod(read(device / attribute));
    if (!std::isfinite(best) || best <= 0)
        throw std::runtime_error("invalid IIO scale readback");
    return best;
}

/// Validate the exact binary formats consumed by the current acquisition reader.
void load_layout(ImuDevice& device, const fs::path& scan) {
    struct Channel {
        int index, bytes;
        std::string name;
    };
    std::vector<Channel> channels;
    const std::string prefix = device.kind == "accel" ? "in_accel_" : "in_anglvel_";
    const std::regex type_pattern(R"((le|be):([su])(\d+)/(\d+)>>(\d+))");
    for (const auto& entry : fs::directory_iterator(scan)) {
        auto name = entry.path().filename().string();
        if (name.size() < 3 || name.substr(name.size() - 3) != "_en")
            continue;
        name.resize(name.size() - 3);
        const bool timestamp = name == "in_timestamp";
        const bool axis = name == prefix + "x" || name == prefix + "y" || name == prefix + "z";
        write(entry.path(), timestamp || axis ? "1" : "0");
        if (read(entry.path()) != (timestamp || axis ? "1" : "0"))
            throw std::runtime_error("IIO channel enable readback mismatch");
        if (!timestamp && !axis)
            continue;
        std::smatch match;
        const auto type = read(scan / (name + "_type"));
        if (!std::regex_match(type, match, type_pattern) || match[1] != "le" || match[2] != "s" ||
            std::stoi(match[3]) != (timestamp ? 64 : 16) ||
            std::stoi(match[4]) != (timestamp ? 64 : 16) || std::stoi(match[5]) != 0)
            throw std::runtime_error("unsupported IIO type: " + name + " " + type);
        channels.push_back({std::stoi(read(scan / (name + "_index"))), timestamp ? 8 : 2, name});
    }
    if (channels.size() != 4)
        throw std::runtime_error("IIO needs xyz and timestamp channels");
    std::sort(channels.begin(), channels.end(),
              [](const auto& a, const auto& b) { return a.index < b.index; });
    int offset = 0, previous = -1;
    for (const auto& channel : channels) {
        if (channel.index < 0 || channel.index == previous)
            throw std::runtime_error("invalid IIO scan index");
        previous = channel.index;
        offset = (offset + channel.bytes - 1) / channel.bytes * channel.bytes;
        if (channel.name == "in_timestamp")
            device.timestamp_offset = offset;
        else
            device.axis_offsets[channel.name.back() - 'x'] = offset;
        offset += channel.bytes;
    }
    device.record_bytes = (offset + 7) / 8 * 8;
}
} // namespace

/// Discover the accel/gyro pair, check gravity, then configure both buffers.
ImuDevices::ImuDevices(unsigned watermark, fs::path sysfs, fs::path nodes) {
    if (watermark == 0 || watermark > 1024)
        throw std::invalid_argument("IIO watermark must be 1..1024");
    try {
        for (const auto& entry : fs::directory_iterator(sysfs)) {
            if (entry.path().filename().string().find("iio:device") != 0)
                continue;
            const auto name = read(entry.path() / "name");
            std::string kind;
            if (name.find("accel") != std::string::npos)
                kind = "accel";
            else if (name.find("gyro") != std::string::npos ||
                     name.find("anglvel") != std::string::npos)
                kind = "gyro";
            else
                continue;
            if (std::any_of(devices_.begin(), devices_.end(),
                            [&](const auto& d) { return d.kind == kind; }))
                throw std::runtime_error("ambiguous IIO " + kind + " devices");
            ImuDevice d;
            d.kind = kind;
            d.sysfs = entry.path();
            d.chardev = nodes / entry.path().filename();
            devices_.push_back(d);
        }
        if (devices_.size() != 2)
            throw std::runtime_error("missing IIO accelerometer or gyroscope");
        // Refuse to take over enabled buffers belonging to another capture.
        for (const auto& d : devices_) {
            const auto buffer = directory(d.sysfs, false);
            if (read(buffer / "enable") != "0")
                throw std::runtime_error("IIO buffer already enabled");
        }
        const auto& accel = *std::find_if(devices_.begin(), devices_.end(),
                                          [](const auto& d) { return d.kind == "accel"; });
        const double scale = std::stod(read(accel.sysfs / "in_accel_scale"));
        double norm2 = 0;
        bool doubled = true;
        for (char axis : std::string("xyz")) {
            int raw = std::stoi(read(accel.sysfs / (std::string("in_accel_") + axis + "_raw")));
            norm2 += std::pow(raw * scale, 2);
            doubled = doubled && (((raw & 0xffff) >> 8) == (raw & 0xff));
        }
        if (doubled || !std::isfinite(norm2) || norm2 <= 8.3 * 8.3 || norm2 >= 11.3 * 11.3)
            throw std::runtime_error(
                "IMU gravity check failed; hold still and check sensor wiring");
        for (auto& d : devices_) {
            auto buffer = directory(d.sysfs, false);
            buffers_.push_back(buffer);
            write(buffer / "enable", "0");
            write(d.sysfs / "current_timestamp_clock", "monotonic");
            if (read(d.sysfs / "current_timestamp_clock") != "monotonic")
                throw std::runtime_error("IIO clock is not monotonic");
            write(d.sysfs / "sampling_frequency", "416");
            d.rate_hz = std::stod(read(d.sysfs / "sampling_frequency"));
            if (!std::isfinite(d.rate_hz) || std::abs(d.rate_hz - 416) > 0.1)
                throw std::runtime_error("unexpected IIO sample rate");
            const bool is_accel = d.kind == "accel";
            d.scale =
                select_scale(d.sysfs, is_accel ? "in_accel_scale" : "in_anglvel_scale",
                             is_accel ? 16 * 9.80665 / 32768 : 2000 * std::acos(-1) / 180 / 32768);
            load_layout(d, directory(d.sysfs, true));
            if (fs::exists(buffer / "watermark"))
                write(buffer / "watermark", std::to_string(watermark));
            write(buffer / "length", "4096");
        }
        // Validate both devices before enabling either; failed enable also unwinds both.
        for (const auto& buffer : buffers_) {
            write(buffer / "enable", "1");
            if (read(buffer / "enable") != "1")
                throw std::runtime_error("IIO enable readback failed");
        }
    } catch (...) {
        disable();
        throw;
    }
}

/// Leave no kernel buffer running after a normal or failed capture.
ImuDevices::~ImuDevices() {
    disable();
}

/// Cleanup is best effort and never hides the original startup/worker error.
void ImuDevices::disable() noexcept {
    for (const auto& buffer : buffers_) {
        try {
            write(buffer / "enable", "0");
        } catch (const std::exception& error) {
            std::cerr << "IIO cleanup: " << error.what() << '\n';
        }
    }
}

/// Save the existing replay sidecar schema; live acquisition uses devices() directly.
void ImuDevices::save_metadata(const fs::path& prefix) const {
    nlohmann::json metadata{{"devices", nlohmann::json::object()}};
    for (const auto& d : devices_) {
        nlohmann::json channels = nlohmann::json::array();
        const auto pre = d.kind == "accel" ? "in_accel_" : "in_anglvel_";
        for (int i = 0; i < 4; ++i) {
            const bool timestamp = i == 3;
            channels.push_back({{"name", timestamp ? "in_timestamp" : std::string(pre) + "xyz"[i]},
                                {"offset", timestamp ? d.timestamp_offset : d.axis_offsets[i]},
                                {"endian", "le"},
                                {"signed", true},
                                {"bits", timestamp ? 64 : 16},
                                {"storage", timestamp ? 8 : 2},
                                {"shift", 0}});
        }
        metadata["devices"][d.kind] = {{"sysfs", d.sysfs.string()},
                                       {"chardev", d.chardev.string()},
                                       {"name", read(d.sysfs / "name")},
                                       {"odr_hz", d.rate_hz},
                                       {"scale", d.scale},
                                       {"timestamp_clock", "monotonic"},
                                       {"record_bytes", d.record_bytes},
                                       {"channels", channels},
                                       {"file", prefix.string() + ".imu_" + d.kind + ".bin"}};
    }
    std::ofstream out(prefix.string() + ".imu.json");
    out << metadata.dump(2) << '\n';
    out.close();
    if (!out)
        throw std::runtime_error("cannot save IMU metadata");
}
} // namespace vio
