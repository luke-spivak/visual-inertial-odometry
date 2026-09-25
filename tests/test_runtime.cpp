#include "camera_capture.h"
#include "flight_application.h"
#include "iio_fixture.h"
#include "mavlink_link.h"
#include "process.h"

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
volatile std::sig_atomic_t child_stop = 0;
void stop_child(int) {
    child_stop = 1;
}
MonotonicTime clock_now() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return std::chrono::seconds(t.tv_sec) + std::chrono::nanoseconds(t.tv_nsec);
}

void process_checks(const std::string& self) {
    Fixture f;
    {
        CameraPipes pipes(f.root);
        require(fs::is_fifo(pipes.frames()) && fs::is_fifo(pipes.metadata()),
                "camera FIFOs created");
        auto directory = pipes.frames().parent_path();
        ChildProcess child({self, "--echo", "literal $(echo wrong)"}, directory / "child.log");
        while (!child.poll())
            std::this_thread::sleep_for(1ms);
        require(*child.poll() == 7, "child exit status preserved");
        std::ifstream log(directory / "child.log");
        std::string line;
        std::getline(log, line);
        require(line == "literal $(echo wrong)", "arguments do not pass through shell");
    }
    bool failed = false;
    try {
        ChildProcess child({"/nonexistent-vio-program"}, f.root / "error.log");
    } catch (const std::system_error&) {
        failed = true;
    }
    require(failed, "failed exec reported");
    {
        ChildProcess child({self, "--ignore-term"}, f.root / "ignore.log");
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (get(f.root / "ignore.log") != "ready" && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(1ms);
        require(get(f.root / "ignore.log") == "ready", "stubborn child ready");
        child.stop(SIGTERM, 10ms);
        require(child.poll() == 128 + SIGKILL, "stubborn child killed and reaped");
    }
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

void exposure_checks(const std::string& self, const char* config_path) {
    Fixture f;
    nlohmann::json json;
    std::ifstream(config_path) >> json;
    put(f.root / "flight.json", json.dump());
    auto config = load_flight_config(f.root / "flight.json", f.root);
    fs::create_directory(f.root / "bin");
    fs::create_symlink(self, f.root / "bin/rpicam-raw");
    const std::string old_path = std::getenv("PATH") ? std::getenv("PATH") : "";
    ::setenv("PATH", (f.root / "bin").c_str(), 1);
    try {
        config.exposure_mode = "auto";
        auto exposure = select_exposure(config, f.root, [] { return true; });
        require(exposure.shutter == 4000 && exposure.gain == 4,
                "automatic exposure preserves light product");
        config.exposure_mode = "sweep";
        exposure = select_exposure(config, f.root, [] { return true; });
        require(exposure.shutter == 200 && exposure.gain == 1,
                "sweep chooses brightest unclipped probe");
        config.exposure_mode = "fixed";
        config.shutter = 123;
        config.gain = 2;
        exposure = select_exposure(config, f.root, [] { return true; });
        require(exposure.shutter == 123 && exposure.gain == 2, "fixed exposure preserved");
    } catch (...) {
        ::setenv("PATH", old_path.c_str(), 1);
        throw;
    }
    ::setenv("PATH", old_path.c_str(), 1);
}

void lifecycle(const std::string& self, const char* config_path, bool restart, bool armed,
               bool missing_camera) {
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
    fs::create_directory(f.root / "bin");
    if (!missing_camera)
        fs::create_symlink(self, f.root / "bin/rpicam-raw");
    const std::string old_path = std::getenv("PATH") ? std::getenv("PATH") : "";
    ::setenv("PATH", (f.root / "bin").c_str(), 1);
    CaptureState state;
    state.fail_first = restart;
    state.armed = armed;
    const auto deadline =
        std::chrono::steady_clock::now() + ((armed || missing_camera) ? 100ms : 12s);
    try {
        run_flight(
            config, [&] { return std::make_unique<FakeController>(state); },
            [&] { return std::make_unique<FakeEstimator>(state); },
            [&] { return state.stop || std::chrono::steady_clock::now() >= deadline; },
            {f.root, f.root});
    } catch (...) {
        ::setenv("PATH", old_path.c_str(), 1);
        throw;
    }
    ::setenv("PATH", old_path.c_str(), 1);
    f.disabled();
    require(state.runs == state.exits, "all estimator workers joined");
    for (const auto& entry : fs::directory_iterator(config.runtime_dir))
        require(entry.path().filename().string().find("camera-") != 0, "temporary FIFOs removed");
    if (armed)
        require(state.runs == 0 && state.pose_count == 0, "armed startup never starts capture");
    else if (missing_camera)
        require(state.runs == 1 && state.pose_count == 0, "partial startup stops estimator");
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
    const auto name = fs::path(argv[0]).filename().string();
    if (name == "rpicam-raw") {
        auto value = [&](const std::string& key) -> std::string {
            for (int i = 1; i + 1 < argc; ++i)
                if (argv[i] == key)
                    return argv[i + 1];
            return "";
        };
        if (value("-t") == "2500") {
            put(value("--metadata"), "[{\"ExposureTime\":8000,\"AnalogueGain\":2}]");
            return 0;
        }
        if (!value("--frames").empty()) {
            auto shutter = std::stoi(value("--shutter"));
            std::vector<unsigned char> pixels(1280 * 800 * 2, 0);
            for (std::size_t i = 1; i < pixels.size(); i += 2)
                pixels[i] = std::min(shutter, 255);
            std::ofstream output(value("-o"), std::ios::binary);
            output.write(reinterpret_cast<const char*>(pixels.data()), pixels.size());
            return 0;
        }
        std::signal(SIGINT, stop_child);
        std::signal(SIGTERM, stop_child);
        while (!child_stop)
            std::this_thread::sleep_for(1ms);
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "--echo") {
        std::cerr << argv[2] << '\n';
        return 7;
    }
    if (argc > 1 && std::string(argv[1]) == "--ignore-term") {
        std::signal(SIGTERM, SIG_IGN);
        std::cerr << "ready\n";
        while (true)
            std::this_thread::sleep_for(1ms);
    }
    try {
        require(argc == 2, "flight config fixture path required");
        auto self = fs::absolute(argv[0]).string();
        process_checks(self);
        exposure_checks(self, argv[1]);
        lifecycle(self, argv[1], false, false, false);
        lifecycle(self, argv[1], true, false, false);
        lifecycle(self, argv[1], false, true, false);
        lifecycle(self, argv[1], false, false, true);
        std::cout << "Application startup, restart, and cleanup checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
