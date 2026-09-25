#include "flight_application.h"
#include "camera_capture.h"
#include "mavlink_link.h"
#include "session_store.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>
#include <time.h>

namespace vio {
namespace {
/// Use the host clock shared by IIO and normalized camera timestamps.
MonotonicTime now() {
    timespec value{};
    if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0)
        throw std::runtime_error("monotonic clock failed");
    return std::chrono::seconds(value.tv_sec) + std::chrono::nanoseconds(value.tv_nsec);
}

/// Save capture metadata with checked errors before launching sensor workers.
void save(const std::filesystem::path& path, const nlohmann::json& value) {
    std::ofstream file(path);
    file << value.dump(2) << '\n';
    file.close();
    if (!file)
        throw std::runtime_error("cannot save " + path.string());
}

/// Join the estimator before its sensor handles, mailbox, or stop flag are destroyed.
class CaptureWorker {
public:
    std::atomic<bool> stop{false}, done{false};
    std::exception_ptr error;
    CaptureWorker(EstimatorRunner& runner, const RunnerOptions& options, LatestEstimate& estimates)
        : thread_([&, options] {
              try {
                  runner.run(options, stop, estimates);
              } catch (...) {
                  error = std::current_exception();
              }
              done = true;
          }) {}
    ~CaptureWorker() {
        stop = true;
        thread_.join();
    }

private:
    std::thread thread_;
};

struct Recording {
    std::filesystem::path prefix;
    bool enabled;
};

/// Save the configuration alongside this capture, even when disk space disables raw recording.
Recording prepare_recording(const FlightConfig& config, Exposure exposure) {
    std::filesystem::create_directories(config.recording_dir);
    const auto prefix = config.recording_dir / ("run-native-" + std::to_string(now().count()));
    const bool record =
        std::filesystem::space(config.recording_dir).available / 1e9 >= config.min_free_gb;
    save(prefix.string() + ".flight.json", config.snapshot);
    save(prefix.string() + ".exposure.json",
         {{"shutter_us", exposure.shutter}, {"gain", exposure.gain}});
    if (record) {
        nlohmann::json configs = nlohmann::json::object();
        for (const auto& file :
             std::filesystem::directory_iterator(config.estimator_config.parent_path())) {
            if (file.path().extension() != ".yaml")
                continue;
            std::ifstream input(file.path());
            configs[file.path().filename().string()] =
                std::string(std::istreambuf_iterator<char>(input), {});
        }
        save(prefix.string() + ".recording.json", {{"version", 1},
                                                   {"width", 1280},
                                                   {"height", 800},
                                                   {"clock", "CLOCK_MONOTONIC"},
                                                   {"config_files", configs}});
    }
    return {prefix, record};
}

/// Run acquisition while the main thread services the flight controller; unwind before returning.
void run_capture(const FlightConfig& config, EstimatorRunner& estimator, MavlinkLink& link,
                 CameraSource& camera, Exposure exposure, SessionGeneration generation,
                 HardwarePaths paths, const std::function<bool()>& pump,
                 const std::function<bool()>& stopping) {
    const auto recording = prepare_recording(config, exposure);
    const auto& prefix = recording.prefix;
    const bool record = recording.enabled;
    ImuDevices imu(config.watermark, paths.sysfs, paths.devices);
    imu.save_metadata(prefix);
    RunnerOptions options;
    options.devices = imu.devices();
    options.camera = &camera;
    options.camera_fps = config.fps;
    options.imu_lpf_hz = config.imu_lpf;
    options.imu_rate_hz = options.devices.front().rate_hz;
    options.generation = generation;
    if (record) {
        options.recording_status = std::make_shared<RecordingStatus>();
        options.recording_prefix = prefix;
        options.estimate_log = prefix.string() + ".est.txt";
    }
    LatestEstimate estimates;
    std::exception_ptr capture_error;
    bool requested_stop = false;
    {
        camera.start(config.fps, exposure);
        CaptureWorker worker(estimator, options, estimates);

        const auto started = now();
        try {
            while (pump() && !worker.done) {
                if (auto estimate = estimates.load()) {
                    link.publish(*estimate, now());
                    if (now() - estimate->timestamp > std::chrono::seconds(3))
                        throw std::runtime_error("estimator stopped producing fresh snapshots");
                } else if (now() - started > std::chrono::seconds(60)) {
                    throw std::runtime_error("estimator initialization timed out");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            requested_stop = stopping();
            if (!requested_stop)
                throw std::runtime_error("camera or estimator ended unexpectedly");
        } catch (...) {
            capture_error = std::current_exception();
        }
        // Stop publication before stopping capture; no stale poses during cleanup.
        link.end_session();
        worker.stop = true;
        try {
            camera.stop();
        } catch (...) {
            capture_error = std::current_exception();
        }
        while (!worker.done) {
            // Keep heartbeats and SIGTERM responsive while acquisition unwinds.
            link.poll(now());
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (worker.error)
            capture_error = worker.error;
    }
    if (capture_error)
        std::rethrow_exception(capture_error);
    if (record && requested_stop && options.recording_status->complete())
        save(prefix.string() + ".complete.json",
             {{"version", 1}, {"vio_exit", 0}, {"camera_exit", 0}});
}
} // namespace

void run_flight(const FlightConfig& config,
                const std::function<std::unique_ptr<ByteStream>()>& open_stream,
                const std::function<std::unique_ptr<EstimatorRunner>()>& make_estimator,
                const std::function<std::unique_ptr<CameraSource>()>& make_camera,
                const std::function<bool()>& stopping, HardwarePaths paths) {
    SessionStore store(config.runtime_dir);
    auto estimator = make_estimator();
    const auto calibration = estimator->camera_from_imu();
    MavlinkLinkConfig link_config;
    link_config.send_velocity = config.send_velocity;
    link_config.session.max_estimate_age =
        MonotonicTime{static_cast<int64_t>(config.max_lag * 1e9)};
    MavlinkLink link(
        open_stream(), link_config,
        FrameTransform(calibration, {config.tilt_deg * std::acos(-1) / 180, config.upside_down}),
        store.reserve_reset());
    std::uint64_t generation = 0;
    unsigned backoff = 5;
    auto pump = [&] {
        link.poll(now());
        if (link.state() == FlightSessionState::Failed)
            throw std::runtime_error(link.failure_reason());
        return !stopping();
    };
    while (!stopping()) {
        if (generation)
            store.reserve_reset();
        link.start_session({++generation});
        // Every estimator restart starts a new frame; wait for a fresh disarmed FC.
        while (pump() && !link.controller_disarmed(now()))
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (stopping())
            break;
        try {
            if (!estimator)
                estimator = make_estimator();
            if ((estimator->camera_from_imu() - calibration).norm() > 1e-10)
                throw std::runtime_error("calibration changed during flight application lifetime");
            auto camera = make_camera();
            auto exposure = select_exposure(
                config, *camera, [&] { return pump() && link.controller_disarmed(now()); });
            if (!pump() || !link.controller_disarmed(now()))
                continue;
            run_capture(config, *estimator, link, *camera, exposure, {generation}, paths, pump,
                        stopping);
            backoff = 5;
        } catch (const std::exception& error) {
            std::cerr << "capture " << generation << ": " << error.what() << '\n';
            if (stopping())
                break;
            if (link.state() == FlightSessionState::Failed ||
                link.state() == FlightSessionState::AwaitingAlignment)
                throw;
        }
        estimator.reset();
        const auto retry_at = now() + std::chrono::seconds(backoff);
        while (now() < retry_at && pump())
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        backoff = std::min(60U, backoff * 2);
    }
}
} // namespace vio
