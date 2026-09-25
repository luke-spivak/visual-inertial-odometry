// Exposure selection and raw-frame validation, independent of the camera driver.
#include "camera_capture.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vio {
std::vector<std::uint8_t> copy_mono_frame(const std::uint8_t* data, std::size_t bytes,
                                          unsigned stride, bool high_byte) {
    const unsigned step = high_byte ? 2 : 1;
    const auto row_bytes = CameraFrame::width * step;
    if (!data || stride < row_bytes ||
        bytes < std::size_t(stride) * (CameraFrame::height - 1) + row_bytes)
        throw std::runtime_error("incomplete camera frame or invalid stride");
    std::vector<std::uint8_t> pixels(CameraFrame::width * CameraFrame::height);
    for (unsigned y = 0; y < CameraFrame::height; ++y) {
        const auto* row = data + std::size_t(y) * stride;
        for (unsigned x = 0; x < CameraFrame::width; ++x) {
            if (high_byte && row[x * 2] != 0)
                throw std::runtime_error("camera is not the calibrated R8-in-16 layout");
            pixels[y * CameraFrame::width + x] = row[x * step + step - 1];
        }
    }
    return pixels;
}

std::int64_t camera_timestamp(std::int64_t sensor, std::int64_t offset, std::int64_t now,
                              std::int64_t previous) {
    if (offset < 0 || sensor <= offset)
        throw std::runtime_error("invalid camera clock offset/timestamp");
    const auto timestamp = sensor - offset;
    if (timestamp <= previous || timestamp > now + 10000000 || now - timestamp > 2000000000)
        throw std::runtime_error("camera timestamp is stale, future, or non-increasing");
    return timestamp;
}

namespace {
/// Stop the stream on cancellation, read failure, or successful completion of a probe.
template <typename Consume>
void probe(CameraSource& camera, double fps, std::optional<Exposure> exposure, unsigned count,
           const std::function<bool()>& pump, Consume consume) {
    try {
        camera.start(fps, exposure);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        unsigned received = 0;
        while (received < count) {
            if (!pump())
                throw std::runtime_error("camera probe cancelled");
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("camera probe timed out");
            if (auto frame = camera.read(std::chrono::milliseconds(5))) {
                // Allow the sensor/AE to settle before evaluating brightness.
                if (++received > 10)
                    consume(*frame);
            }
        }
        camera.stop();
    } catch (...) {
        camera.stop();
        throw;
    }
}
} // namespace

Exposure select_exposure(const FlightConfig& config, CameraSource& camera,
                         const std::function<bool()>& pump) {
    if (config.exposure_mode == "fixed")
        return {config.shutter, config.gain};
    if (config.exposure_mode == "auto") {
        Exposure measured{};
        probe(camera, 5, std::nullopt, 13, pump,
              [&](const CameraFrame& frame) { measured = frame.exposure; });
        const double product = measured.shutter * measured.gain;
        if (!std::isfinite(product) || product < 1 || product > config.max_shutter * 16.0)
            throw std::runtime_error("camera exposure out of range");
        const auto shutter = unsigned(std::min(product, double(config.max_shutter)));
        return {shutter, std::max(1.0, product / shutter)};
    }
    struct Result {
        unsigned shutter;
        double clipped, mean;
    };
    std::vector<Result> results;
    for (unsigned shutter : {20, 30, 50, 75, 100, 200, 500, 1000, 2000, 4000}) {
        if (shutter > config.max_shutter)
            continue;
        std::uint64_t sum = 0, clipped = 0, pixels = 0;
        probe(camera, 20, Exposure{shutter, 1}, 40, pump, [&](const CameraFrame& frame) {
            if (frame.pixels.size() != CameraFrame::width * CameraFrame::height)
                throw std::runtime_error("invalid exposure probe dimensions");
            for (auto pixel : frame.pixels) {
                sum += pixel;
                clipped += pixel >= 250;
            }
            pixels += frame.pixels.size();
        });
        results.push_back({shutter, double(clipped) / pixels, double(sum) / pixels});
    }
    if (results.empty())
        throw std::runtime_error("max_shutter is below the shortest exposure probe (20 us)");
    auto chosen = results.front();
    bool usable = false;
    for (const auto& result : results) {
        const bool candidate = result.clipped <= 0.01 && result.mean >= 15;
        if ((candidate && (!usable || result.mean > chosen.mean)) ||
            (!candidate && !usable && result.clipped < chosen.clipped))
            chosen = result;
        usable = usable || candidate;
    }
    return {chosen.shutter, 1};
}
} // namespace vio
