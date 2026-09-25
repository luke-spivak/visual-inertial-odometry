// Own the calibrated raw camera and copy completed buffers into a bounded frame queue.
#include "camera_capture.h"
#include "camera_queue.h"

#include <libcamera/libcamera.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <system_error>
#include <time.h>

namespace vio {
namespace {
using namespace libcamera;
void check(int result, const char* operation) {
    if (result < 0)
        throw std::system_error(-result, std::generic_category(), operation);
}
std::int64_t clock_ns(clockid_t clock) {
    timespec value{};
    if (clock_gettime(clock, &value))
        throw std::system_error(errno, std::generic_category(), "read camera clock");
    return std::int64_t(value.tv_sec) * 1000000000 + value.tv_nsec;
}

/// Sample both clocks closely; BOOTTIME includes suspend, MONOTONIC does not.
std::int64_t clock_offset() {
    for (unsigned attempt = 0; attempt < 10; ++attempt) {
        const auto before = clock_ns(CLOCK_MONOTONIC);
        const auto boot = clock_ns(CLOCK_BOOTTIME);
        const auto after = clock_ns(CLOCK_MONOTONIC);
        if (after - before < 100000)
            return std::max<std::int64_t>(0, boot - (before + (after - before) / 2));
    }
    throw std::runtime_error("could not sample camera clock offset within 100 us");
}

/// mmap offsets must be page aligned; a plane's offset need not be.
class MappedPlane {
public:
    explicit MappedPlane(const FrameBuffer::Plane& plane)
        : fd(plane.fd.get()), length(plane.length) {
        if (plane.offset == FrameBuffer::Plane::kInvalidOffset)
            throw std::runtime_error("camera buffer has no mappable offset");
        const auto page = ::sysconf(_SC_PAGESIZE);
        if (page <= 0)
            throw std::runtime_error("cannot determine mmap page size");
        const auto aligned = plane.offset / page * page;
        offset_ = plane.offset - aligned;
        mapped_size_ = offset_ + length;
        memory_ = ::mmap(nullptr, mapped_size_, PROT_READ, MAP_SHARED, fd, aligned);
        if (memory_ == MAP_FAILED)
            throw std::system_error(errno, std::generic_category(), "map camera plane");
    }
    ~MappedPlane() {
        ::munmap(memory_, mapped_size_);
    }
    const std::uint8_t* data() const {
        return static_cast<const std::uint8_t*>(memory_) + offset_;
    }
    void sync(std::uint64_t flags) const {
        dma_buf_sync value{flags | DMA_BUF_SYNC_READ};
        int result;
        do {
            result = ::ioctl(fd, DMA_BUF_IOCTL_SYNC, &value);
        } while (result < 0 && errno == EINTR);
        if (result < 0)
            throw std::system_error(errno, std::generic_category(),
                                    "synchronize camera DMA buffer");
    }
    const int fd;
    const std::size_t length;

private:
    void* memory_{MAP_FAILED};
    std::size_t offset_{0}, mapped_size_{0};
};

class LibcameraSource final : public CameraSource {
public:
    ~LibcameraSource() override {
        try {
            stop();
        } catch (const std::exception& error) {
            std::cerr << error.what() << '\n';
        }
        release();
    }
    void start(double fps, std::optional<Exposure> exposure) override;
    std::optional<CameraFrame> read(std::chrono::milliseconds timeout) override {
        return frames_.read(timeout);
    }
    void stop() override {
        stopping_ = true;
        // stop() waits for request callbacks; never hold the queue mutex across it.
        const bool was_running = running_;
        int result = running_ ? camera_->stop() : 0;
        running_ = false;
        frames_.close();
        if (was_running && frames_.dropped())
            std::cerr << "camera handoff dropped " << frames_.dropped() << " frames\n";
        check(result, "stop camera");
    }

private:
    void configure();
    void completed(Request* request) noexcept;
    void disconnected() noexcept {
        frames_.fail(std::make_exception_ptr(std::runtime_error("camera disconnected")));
    }
    void release() {
        // Requests must die before the buffers they refer to, including failed startup.
        if (camera_) {
            camera_->requestCompleted.disconnect(this);
            camera_->disconnected.disconnect(this);
        }
        requests_.clear();
        mappings_.clear();
        allocator_.reset();
        configuration_.reset();
        if (acquired_)
            camera_->release();
        acquired_ = false;
        camera_.reset();
        if (manager_)
            manager_->stop();
        manager_.reset();
    }
    std::unique_ptr<CameraManager> manager_;
    std::shared_ptr<Camera> camera_;
    std::unique_ptr<CameraConfiguration> configuration_;
    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::map<FrameBuffer*, std::unique_ptr<MappedPlane>> mappings_;
    std::vector<std::unique_ptr<Request>> requests_;
    CameraQueue frames_;
    std::atomic<bool> stopping_{true};
    bool acquired_{false}, running_{false}, high_byte_{false};
    unsigned stride_{0}, received_{0}, settled_{0};
    std::int64_t offset_{0}, previous_{0}, duration_us_{0};
    std::optional<Exposure> exposure_;
};

/// Acquire the calibrated sensor and allocate raw buffers without starting capture.
void LibcameraSource::configure() {
    manager_ = std::make_unique<CameraManager>();
    check(manager_->start(), "start camera manager");
    if (manager_->cameras().size() != 1)
        throw std::runtime_error("native capture requires exactly one calibrated camera");
    camera_ = manager_->cameras().front();
    if (camera_->properties().get(properties::Model).value_or("") != "ov9281")
        throw std::runtime_error("native capture requires the calibrated OV9281 sensor");
    check(camera_->acquire(), "acquire camera");
    acquired_ = true;
    configuration_ = camera_->generateConfiguration({StreamRole::Raw});
    if (!configuration_ || configuration_->size() != 1)
        throw std::runtime_error("camera cannot provide one raw stream");
    configuration_->orientation = Orientation::Rotate0;
    auto& stream = configuration_->at(0);
    stream.size = {CameraFrame::width, CameraFrame::height};
    stream.pixelFormat = formats::R8;
    stream.bufferCount = 6;
    configuration_->sensorConfig = SensorConfiguration{};
    configuration_->sensorConfig->bitDepth = 8;
    configuration_->sensorConfig->outputSize = stream.size;
    if (configuration_->validate() == CameraConfiguration::Invalid ||
        stream.size != Size(CameraFrame::width, CameraFrame::height) ||
        (stream.pixelFormat != formats::R8 && stream.pixelFormat != formats::R16) ||
        configuration_->orientation != Orientation::Rotate0 || !configuration_->sensorConfig ||
        configuration_->sensorConfig->bitDepth != 8 ||
        configuration_->sensorConfig->outputSize != stream.size)
        throw std::runtime_error("camera adjusted the calibrated raw mode; refusing capture");
    check(camera_->configure(configuration_.get()), "configure raw camera");
    high_byte_ = stream.pixelFormat == formats::R16;
    stride_ = stream.stride;
    std::cerr << "camera " << camera_->id() << ": " << stream.toString() << " stride=" << stride_
              << '\n';
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    check(allocator_->allocate(stream.stream()), "allocate camera buffers");
    for (const auto& buffer : allocator_->buffers(stream.stream())) {
        if (buffer->planes().size() != 1)
            throw std::runtime_error("raw camera must have one plane");
        mappings_.emplace(buffer.get(), std::make_unique<MappedPlane>(buffer->planes()[0]));
        auto request = camera_->createRequest();
        if (!request)
            throw std::runtime_error("cannot allocate camera request");
        check(request->addBuffer(stream.stream(), buffer.get()), "attach camera buffer");
        requests_.push_back(std::move(request));
    }
    if (requests_.empty())
        throw std::runtime_error("camera allocated no buffers");
    camera_->requestCompleted.connect(this, &LibcameraSource::completed);
    camera_->disconnected.connect(this, &LibcameraSource::disconnected);
}

/// Start a fresh stream so exposure probes cannot leak queued frames into flight capture.
void LibcameraSource::start(double fps, std::optional<Exposure> exposure) {
    stop();
    release();
    try {
        configure();
        if (!std::isfinite(fps) || fps <= 0 || fps > 120)
            throw std::runtime_error("invalid camera frame rate");
        const auto duration = std::int64_t(std::llround(1e6 / fps));
        ControlList controls(camera_->controls());
        // Reject unsupported controls instead of silently capturing with different settings.
        for (const ControlId* id :
             std::vector<const ControlId*>{&controls::AeEnable, &controls::ExposureTime,
                                           &controls::AnalogueGain, &controls::FrameDurationLimits})
            if (!camera_->controls().count(id))
                throw std::runtime_error("required camera control unavailable: " + id->name());
        controls.set(controls::AeEnable, !exposure);
        controls.set(controls::FrameDurationLimits, {duration, duration});
        if (exposure) {
            if (!exposure->shutter || exposure->shutter > duration ||
                !std::isfinite(exposure->gain) || exposure->gain < 1 || exposure->gain > 16)
                throw std::runtime_error("fixed exposure exceeds frame duration or gain limits");
            controls.set(controls::ExposureTime, std::int32_t(exposure->shutter));
            controls.set(controls::AnalogueGain, float(exposure->gain));
        }
        duration_us_ = duration;
        offset_ = clock_offset();
        previous_ = 0;
        received_ = settled_ = 0;
        exposure_ = exposure;
        frames_.reset();
        stopping_ = false;
        check(camera_->start(&controls), "start raw camera");
        running_ = true;
        for (auto& request : requests_)
            check(camera_->queueRequest(request.get()), "queue initial camera request");
    } catch (...) {
        stop();
        release();
        throw;
    }
}

/// Called on libcamera's thread; never let exceptions escape into the library.
void LibcameraSource::completed(Request* request) noexcept {
    if (stopping_ || request->status() == Request::RequestCancelled)
        return;
    try {
        auto* buffer = request->buffers().at(configuration_->at(0).stream());
        const auto& metadata = buffer->metadata();
        if (metadata.status != FrameMetadata::FrameSuccess || metadata.planes().size() != 1)
            throw std::runtime_error("camera delivered an incomplete frame");
        const auto timestamp = request->metadata().get(controls::SensorTimestamp);
        const auto shutter = request->metadata().get(controls::ExposureTime);
        const auto gain = request->metadata().get(controls::AnalogueGain);
        const auto duration = request->metadata().get(controls::FrameDuration);
        if (!timestamp || !shutter || !gain || *shutter <= 0 || !std::isfinite(*gain) || *gain <= 0)
            throw std::runtime_error("camera metadata missing timestamp/exposure");
        // A suspend changes the offset: restart rather than silently shift camera/IMU alignment.
        if (std::abs(clock_offset() - offset_) > 1000000)
            throw std::runtime_error("camera clock offset changed during capture");
        const auto mono =
            camera_timestamp(*timestamp, offset_, clock_ns(CLOCK_MONOTONIC), previous_);
        previous_ = mono;
        ++received_;
        bool matches = !exposure_ || (std::abs(double(*shutter) - exposure_->shutter) <=
                                          std::max(20.0, exposure_->shutter * .05) &&
                                      std::abs(*gain - exposure_->gain) <= exposure_->gain * .05);
        settled_ = matches ? settled_ + 1 : 0;
        if (received_ >= 60 && settled_ < 3)
            throw std::runtime_error("camera did not apply requested exposure");
        if (received_ > 10 && settled_ >= 3) {
            if (!duration ||
                std::abs(*duration - duration_us_) > std::max<std::int64_t>(100, duration_us_ / 20))
                throw std::runtime_error("camera did not apply requested frame duration");
            const auto& mapped = *mappings_.at(buffer);
            const auto bytes = metadata.planes()[0].bytesused;
            if (bytes > mapped.length)
                throw std::runtime_error("camera bytesused exceeds mapping");
            mapped.sync(DMA_BUF_SYNC_START);
            std::vector<std::uint8_t> pixels;
            try {
                pixels = copy_mono_frame(mapped.data(), bytes, stride_, high_byte_);
            } catch (...) {
                mapped.sync(DMA_BUF_SYNC_END);
                throw;
            }
            mapped.sync(DMA_BUF_SYNC_END);
            frames_.push({std::move(pixels), mono, metadata.sequence, {unsigned(*shutter), *gain}});
        }
        if (!stopping_) {
            request->reuse(Request::ReuseBuffers);
            check(camera_->queueRequest(request), "requeue camera request");
        }
    } catch (...) {
        if (!stopping_)
            frames_.fail(std::current_exception());
    }
}
} // namespace

std::unique_ptr<CameraSource> make_libcamera_source() {
    return std::make_unique<LibcameraSource>();
}
} // namespace vio
