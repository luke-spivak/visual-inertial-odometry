#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace vio {

struct ImuDevice {
    std::string kind;
    std::filesystem::path sysfs, chardev;
    double scale{0}, rate_hz{0};
    int record_bytes{0}, timestamp_offset{0};
    std::array<int, 3> axis_offsets{};
};

/// Owns the enabled IIO buffers for one capture. Always disables both on destruction,
/// including if setup fails halfway through. Construct only while the rig is still.
class ImuDevices {
public:
    explicit ImuDevices(unsigned watermark, std::filesystem::path sysfs = "/sys/bus/iio/devices",
                        std::filesystem::path devices = "/dev");
    ~ImuDevices();
    ImuDevices(const ImuDevices&) = delete;
    ImuDevices& operator=(const ImuDevices&) = delete;
    const std::vector<ImuDevice>& devices() const {
        return devices_;
    }
    void save_metadata(const std::filesystem::path& prefix) const;

private:
    void disable() noexcept;
    std::vector<std::filesystem::path> buffers_;
    std::vector<ImuDevice> devices_;
};
} // namespace vio
