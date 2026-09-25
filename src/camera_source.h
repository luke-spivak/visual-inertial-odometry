#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace vio {
struct Exposure {
    unsigned shutter;
    double gain;
};

/// Owned, tightly packed mono8 image; its timestamp is exposure start on CLOCK_MONOTONIC.
struct CameraFrame {
    static constexpr unsigned width = 1280, height = 800;
    std::vector<std::uint8_t> pixels;
    std::int64_t timestamp_ns;
    std::uint64_t sequence;
    Exposure exposure;
};

/// A capture session owns the source. One reader may wait while the owner stops it.
class CameraSource {
public:
    virtual ~CameraSource() = default;
    /// Start raw capture; an omitted exposure enables automatic exposure.
    virtual void start(double fps, std::optional<Exposure>) = 0;
    /// Return an owned frame, or no frame on timeout/stop; acquisition faults throw.
    virtual std::optional<CameraFrame> read(std::chrono::milliseconds timeout) = 0;
    /// Stop callbacks before releasing buffers. Safe to call again during cleanup.
    virtual void stop() = 0;
};

std::unique_ptr<CameraSource> make_libcamera_source();
} // namespace vio
