#include "camera_capture.h"
#include "camera_queue.h"
#include "flight_application.h"
#include "iio_fixture.h"
#include "mavlink_link.h"
#include "session_store.h"

#include <atomic>
#include <csignal>
#include <deque>
#include <iostream>
#include <thread>
#include <time.h>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace vio;
namespace {
MonotonicTime clock_now() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return std::chrono::seconds(t.tv_sec) + std::chrono::nanoseconds(t.tv_nsec);
}

void session_checks() {
    Fixture f;
    bool failed = false;
    {
        SessionStore store(f.root);
        require(store.reserve_reset() == 0 && store.reserve_reset() == 1, "counter reservations");
        failed = false;
        try {
            SessionStore second(f.root);
        } catch (const std::system_error&) {
            failed = true;
        }
        require(failed, "single application owner");
    }
    {
        SessionStore store(f.root);
        require(store.reserve_reset() == 2, "counter survives process owner replacement");
        put(f.root / "reset_counter", "255");
        require(store.reserve_reset() == 0, "counter wraps");
        put(f.root / "reset_counter", "corrupt");
        failed = false;
        try {
            store.reserve_reset();
        } catch (const std::runtime_error&) {
            failed = true;
        }
        require(failed, "corrupt counter fails closed");
    }
}

struct CaptureState {
    std::atomic<int> runs{0}, exits{0};
    unsigned camera_starts{0}, camera_stops{0};
    std::atomic<bool> stop{false};
    bool fail_first{false}, armed{false};
    unsigned pose_count{0}, alignment_count{0};
    std::vector<unsigned char> counters;
};

class FakeEstimator final : public EstimatorRunner {
public:
    explicit FakeEstimator(CaptureState& state) : state_(state) {}
    Eigen::Matrix3d camera_from_imu() const override {
        return Eigen::Matrix3d::Identity();
    }
    void run(const RunnerOptions& options, std::atomic<bool>& stop,
             LatestEstimate& mailbox) override {
        int run = ++state_.runs;
        for (int i = 0; !stop; ++i) {
            try {
                options.camera->read(1ms);
            } catch (...) {
                ++state_.exits;
                throw;
            }
            EstimatorEstimate estimate;
            estimate.generation = options.generation;
            estimate.status = EstimatorStatus::Ready;
            estimate.timestamp = clock_now();
            mailbox.store(estimate);
            if (state_.fail_first && run == 1 && i == 10) {
                ++state_.exits;
                throw std::runtime_error("injected estimator failure");
            }
            std::this_thread::sleep_for(5ms);
        }
        ++state_.exits;
    }

private:
    CaptureState& state_;
};

class FakeController final : public ByteStream {
public:
    explicit FakeController(CaptureState& state) : state_(state) {}
    std::size_t read(std::uint8_t* buffer, std::size_t capacity) override {
        if (incoming_.empty()) {
            mavlink_message_t message{};
            mavlink_msg_heartbeat_pack_status(
                1, 1, &tx_, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                state_.armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, 0, MAV_STATE_ACTIVE);
            enqueue(message);
        }
        std::size_t count = 0;
        while (!incoming_.empty() && count < capacity) {
            buffer[count++] = incoming_.front();
            incoming_.pop_front();
        }
        return count;
    }
    std::size_t write(const std::uint8_t* buffer, std::size_t size) override {
        for (std::size_t i = 0; i < size; ++i) {
            mavlink_message_t message{};
            mavlink_status_t status{};
            if (mavlink_frame_char_buffer(&parser_, &rx_, buffer[i], &message, &status) !=
                MAVLINK_FRAMING_OK)
                continue;
            if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                ++state_.alignment_count;
                mavlink_message_t ack{};
                mavlink_msg_command_ack_pack_status(1, 1, &tx_, &ack, MAV_CMD_DO_AUX_FUNCTION,
                                                    MAV_RESULT_ACCEPTED, 0, 0, 1, 197);
                enqueue(ack);
            } else if (message.msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE) {
                mavlink_vision_position_estimate_t pose{};
                mavlink_msg_vision_position_estimate_decode(&message, &pose);
                ++state_.pose_count;
                if (state_.counters.empty() || state_.counters.back() != pose.reset_counter)
                    state_.counters.push_back(pose.reset_counter);
                if (state_.alignment_count >= (state_.fail_first ? 2U : 1U) &&
                    state_.pose_count >= (state_.fail_first ? 15U : 5U))
                    state_.stop = true;
            }
        }
        return size;
    }

private:
    void enqueue(const mavlink_message_t& message) {
        std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
        auto count = mavlink_msg_to_send_buffer(bytes, &message);
        incoming_.insert(incoming_.end(), bytes, bytes + count);
    }
    CaptureState& state_;
    std::deque<std::uint8_t> incoming_;
    mavlink_message_t parser_{};
    mavlink_status_t rx_{}, tx_{};
};

class FakeCamera final : public CameraSource {
public:
    explicit FakeCamera(CaptureState* state = nullptr) : state(state) {}
    ~FakeCamera() override {
        stop();
    }
    CaptureState* state;
    bool fail_start{false}, fail_read{false};
    std::atomic<bool> running{false};
    unsigned starts{0}, stops{0};
    std::optional<Exposure> exposure;
    void start(double, std::optional<Exposure> value) override {
        ++starts;
        if (fail_start)
            throw std::runtime_error("injected camera startup failure");
        exposure = value;
        if (state)
            ++state->camera_starts;
        running = true;
    }
    std::optional<CameraFrame> read(std::chrono::milliseconds) override {
        if (fail_read)
            throw std::runtime_error("injected camera read failure");
        if (!running)
            return std::nullopt;
        auto measured = exposure.value_or(Exposure{8000, 2});
        return CameraFrame{std::vector<std::uint8_t>(1280 * 800, std::min(measured.shutter, 255U)),
                           clock_now().count(), 0, measured};
    }
    void stop() override {
        if (running.exchange(false) && state)
            ++state->camera_stops;
        ++stops;
    }
};

void exposure_checks(const char* config_path) {
    Fixture f;
    nlohmann::json json;
    std::ifstream(config_path) >> json;
    put(f.root / "flight.json", json.dump());
    auto config = load_flight_config(f.root / "flight.json", f.root);
    FakeCamera camera;
    config.exposure_mode = "auto";
    auto exposure = select_exposure(config, camera, [] { return true; });
    require(exposure.shutter == 4000 && exposure.gain == 4, "auto preserves exposure product");
    config.exposure_mode = "sweep";
    exposure = select_exposure(config, camera, [] { return true; });
    require(exposure.shutter == 200 && exposure.gain == 1, "brightest unclipped sweep candidate");
    config.exposure_mode = "fixed";
    config.shutter = 123;
    config.gain = 2;
    exposure = select_exposure(config, camera, [] { return true; });
    require(exposure.shutter == 123 && exposure.gain == 2, "fixed exposure preserved");
    config.exposure_mode = "auto";
    bool failed = false;
    try {
        select_exposure(config, camera, [] { return false; });
    } catch (const std::runtime_error&) {
        failed = true;
    }
    require(failed && !camera.running, "cancelled probe stops camera");
    camera.fail_read = true;
    failed = false;
    try {
        select_exposure(config, camera, [] { return true; });
    } catch (const std::runtime_error&) {
        failed = true;
    }
    require(failed && !camera.running, "failed probe stops camera");
}

void frame_checks() {
    const unsigned stride = 1280 * 2 + 64;
    std::vector<std::uint8_t> padded(stride * 800, 0);
    for (unsigned y = 0; y < 800; ++y)
        for (unsigned x = 0; x < 1280; ++x)
            padded[y * stride + x * 2 + 1] = (x + y) % 256;
    auto pixels = copy_mono_frame(padded.data(), padded.size(), stride, true);
    require(pixels[1280 + 1] == 2 && pixels.back() == (1279 + 799) % 256, "row padding removed");
    require(copy_mono_frame(pixels.data(), pixels.size(), 1280, false) == pixels,
            "native R8 copied");
    auto rejects = [](auto operation) {
        bool failed = false;
        try {
            operation();
        } catch (const std::runtime_error&) {
            failed = true;
        }
        require(failed, "invalid camera input rejected");
    };
    rejects([&] { copy_mono_frame(padded.data(), 10, stride, true); });
    rejects([&] { copy_mono_frame(padded.data(), padded.size(), 100, true); });
    padded[0] = 1;
    rejects([&] { copy_mono_frame(padded.data(), padded.size(), stride, true); });
    require(camera_timestamp(5000000000, 2000000000, 3010000000, 2900000000) == 3000000000,
            "suspend offset removed from BOOTTIME");
    rejects([&] { camera_timestamp(100, 200, 300, 0); });
    rejects([&] { camera_timestamp(100, 0, 100, 100); });
    rejects([&] { camera_timestamp(1000000000, 0, 1, 0); });
    rejects([&] { camera_timestamp(1, 0, 4000000000, 0); });
    CameraQueue queue;
    queue.reset();
    for (unsigned i = 0; i < 6; ++i)
        queue.push({{std::uint8_t(i)}, 1, i, {1, 1}});
    auto frame = queue.read(1ms);
    require(frame && frame->sequence == 2 && queue.dropped() == 2, "bounded queue drops oldest");
    queue.fail(std::make_exception_ptr(std::runtime_error("callback failed")));
    rejects([&] { queue.read(1ms); });
    queue.reset();
    std::atomic<bool> woke{false};
    std::thread reader([&] {
        require(!queue.read(5s), "closed reader has no old frames");
        woke = true;
    });
    queue.close();
    reader.join();
    require(woke, "stop wakes blocked frame reader");
}

void lifecycle(const char* config_path, bool restart, bool armed, bool missing_camera,
               bool camera_error = false) {
    Fixture f;
    nlohmann::json json;
    std::ifstream(config_path) >> json;
    json["recording_dir"] = (f.root / "recordings").string();
    json["estimate_dir"] = (f.root / "runtime").string();
    json["exposure_mode"] = "fixed";
    json["min_free_gb"] = 1e9; // No recordings are expected from the fake estimator.
    put(f.root / "flight.json", json.dump());
    const auto config = load_flight_config(f.root / "flight.json", f.root);
    require(config.estimator_config.string().find(f.root.string()) == 0, "home expansion");
    CaptureState state;
    state.fail_first = restart;
    state.armed = armed;
    const auto deadline = std::chrono::steady_clock::now() +
                          ((armed || missing_camera || camera_error) ? 100ms : 12s);
    run_flight(
        config, [&] { return std::make_unique<FakeController>(state); },
        [&] { return std::make_unique<FakeEstimator>(state); },
        [&] {
            auto camera = std::make_unique<FakeCamera>(&state);
            camera->fail_start = missing_camera;
            camera->fail_read = camera_error;
            return camera;
        },
        [&] { return state.stop || std::chrono::steady_clock::now() >= deadline; },
        {f.root, f.root});
    f.disabled();
    require(state.camera_starts == state.camera_stops, "all started cameras stopped");
    require(state.runs == state.exits, "all estimator workers joined");
    for (const auto& entry : fs::directory_iterator(config.runtime_dir))
        require(entry.path().filename().string().find("camera-") != 0, "temporary FIFOs removed");
    if (armed)
        require(state.runs == 0 && state.pose_count == 0, "armed startup never starts capture");
    else if (missing_camera)
        require(state.runs == 0 && state.pose_count == 0,
                "camera failure prevents estimator startup");
    else if (camera_error)
        require(state.runs == 1 && state.pose_count == 0, "camera read fault ends capture");
    else {
        if (!state.stop)
            std::cerr << "runs=" << state.runs << " poses=" << state.pose_count
                      << " aligns=" << state.alignment_count
                      << " counters=" << state.counters.size() << '\n';
        require(state.stop, "lifecycle completed before watchdog");
        require(state.runs == (restart ? 2 : 1), "capture restart count");
        require(state.counters ==
                    (restart ? std::vector<unsigned char>{0, 1} : std::vector<unsigned char>{0}),
                "reserved session counters reach wire");
    }
}
} // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "flight config fixture path required");
        session_checks();
        exposure_checks(argv[1]);
        frame_checks();
        lifecycle(argv[1], false, false, false);
        lifecycle(argv[1], true, false, false);
        lifecycle(argv[1], false, true, false);
        lifecycle(argv[1], false, false, true);
        lifecycle(argv[1], false, false, false, true);
        std::cout << "Application startup, restart, and cleanup checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
