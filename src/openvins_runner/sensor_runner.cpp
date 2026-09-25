// Feed Linux IIO IMU samples and timestamped camera frames to OpenVINS.
// The application owns setup; this adapter owns sensor workers and estimator state.

#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "estimator_runner.h"
#include "feature_log.h"
#include "state/State.h"
#include "track/TrackBase.h"
#include "types/IMU.h"
#include "types/Vec.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"
#include <functional>
#include <stdexcept>

static int64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

[[noreturn]] static void die(const std::string& msg) {
    throw std::runtime_error(msg);
}

// Wait for readable data or stop. Returns bytes read, 0 on EOF, -1 on stop.
static ssize_t read_some(int fd, void* buf, size_t n, std::atomic<bool>& stop) {
    while (!stop) {
        pollfd p{fd, POLLIN, 0};
        int r = poll(&p, 1, 200);
        if (r < 0 && errno != EINTR)
            die(std::string("poll: ") + strerror(errno));
        if (r <= 0)
            continue;
        ssize_t got = read(fd, buf, n);
        if (got < 0 && (errno == EINTR || errno == EAGAIN))
            continue;
        if (got < 0)
            die(std::string("read: ") + strerror(errno));
        return got;
    }
    return -1;
}

static bool read_full(int fd, uint8_t* buf, size_t n, std::atomic<bool>& stop) {
    size_t have = 0;
    while (have < n) {
        ssize_t got = read_some(fd, buf + have, n - have, stop);
        if (got <= 0)
            return false;
        have += size_t(got);
    }
    return true;
}

struct ImuDev {
    std::string kind, chardev;
    int rec = 0, ts_off = 0, x_off = 0, y_off = 0, z_off = 0;
    double scale = 0;
    int fd = -1;
    FILE* rec_fp = nullptr;
};

// Suppress motor vibration above the useful motion band. Subtract the filter's
// DC group delay from IMU timestamps to keep filtered samples aligned with the
// camera; online time-offset calibration handles the residual frequency dependence.
struct ImuLowPass {
    bool on = false;
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    int64_t delay_ns = 0;

    void design(double fc, double fs) {
        const double K = std::tan(M_PI * fc / fs), n = 1.0 / (1.0 + M_SQRT2 * K + K * K);
        b0 = K * K * n;
        b1 = 2 * b0;
        b2 = b0;
        a1 = 2 * (K * K - 1) * n;
        a2 = (1 - M_SQRT2 * K + K * K) * n;
        const double samples = (b1 + 2 * b2) / (b0 + b1 + b2) - (a1 + 2 * a2) / (1 + a1 + a2);
        delay_ns = int64_t(samples / fs * 1e9);
        on = true;
    }
};

// One sensor's filter state, transposed direct form II, one lane per axis.
struct ImuLowPassState {
    Eigen::Vector3d s1 = Eigen::Vector3d::Zero(), s2 = Eigen::Vector3d::Zero();
    int64_t last_t = 0;

    Eigen::Vector3d step(const ImuLowPass& f, int64_t t, const Eigen::Vector3d& x) {
        if (!f.on)
            return x;
        // First sample, or a gap in the stream: restart at steady state on this
        // sample rather than ringing out of stale state.
        if (!last_t || t < last_t || t - last_t > 50000000LL) {
            s1 = x * (1 - f.b0);
            s2 = x * (f.b2 - f.a2);
        }
        last_t = t;
        const Eigen::Vector3d y = f.b0 * x + s1;
        s1 = f.b1 * x - f.a1 * y + s2;
        s2 = f.b2 * x - f.a2 * y;
        return y;
    }
};

// Camera updates are serialized by update_thread; copy both tracker arrays there.
class LoggedVioManager : public ov_msckf::VioManager {
public:
    using ov_msckf::VioManager::VioManager;
    std::vector<FeaturePoint> snapshot() {
        auto obs = trackFEATS->get_last_obs();
        auto ids = trackFEATS->get_last_ids();
        std::vector<FeaturePoint> points;
        const float scale = params.downsample_cameras ? 2.f : 1.f;
        const auto& uv = obs[0];
        const auto& id = ids[0];
        if (uv.size() != id.size())
            return points;
        points.reserve(id.size());
        for (size_t i = 0; i < id.size(); ++i)
            points.push_back({id[i], uv[i].pt.x * scale, uv[i].pt.y * scale});
        return points;
    }
};
struct CapturedFrame {
    ov_core::CameraData camera;
    int64_t timestamp_ns;
    size_t frame_index;
};
struct Shared {
    std::atomic<bool>* stop{nullptr};
    vio::LatestEstimate* estimates{nullptr};
    vio::SessionGeneration generation{0};
    std::deque<ov_core::ImuData> imu_q;
    std::shared_ptr<LoggedVioManager> sys;
    FeatureLog features;
    ImuLowPass lpf;
    int W = 1280, H = 800;

    std::mutex meta_mtx;
    std::condition_variable meta_cv;
    std::deque<int64_t> meta_ts;
    bool meta_done = false;

    std::mutex q_mtx;
    std::condition_variable q_cv;
    std::deque<CapturedFrame> cam_q;
    bool frames_done = false;

    std::atomic<int64_t> last_imu_ns{0};
    std::atomic<long> imu_n{0}, frames_in{0}, frames_done_n{0}, frames_dropped{0};
    std::atomic<double> upd_ms_sum{0}, upd_ms_max{0}, lag_ms_last{0};
    // Distinguish a stalled reader from a blocked estimator feed or kernel backlog.
    std::atomic<int64_t> imu_loop_ns{0};
    std::atomic<double> feed_ms_max{0}, read_gap_ms_max{0};

    // Initialization failures are only visible at INFO; reduce logging afterward.
    std::string run_verbosity = "WARNING";
    // initialized() also requires a full visual update. ZUPT can keep an already
    // initialized stationary estimator from reaching that update, so track both.
    std::atomic<bool> vio_init{false};

    std::mutex st_mtx; // last reported state, for the stats line and summary
    bool init = false;
    double init_t = 0;
    Eigen::Vector3d p_first = Eigen::Vector3d::Zero(), p_last = Eigen::Vector3d::Zero(),
                    v_last = Eigen::Vector3d::Zero();
    double path_m = 0;
};

// ---------------------------------------------------------------- IMU

static void imu_thread(Shared& S, std::vector<ImuDev>& devs) {
    ImuDev *acc = nullptr, *gyr = nullptr;
    for (auto& d : devs)
        (d.kind == "accel" ? acc : gyr) = &d;
    if (!acc || !gyr)
        die("--imu must list both accel and gyro");

    std::deque<std::pair<int64_t, Eigen::Vector3d>> aq, gq;
    ImuLowPassState af, gf;
    std::vector<uint8_t> buf(64 * 1024);
    pollfd p[2] = {{acc->fd, POLLIN, 0}, {gyr->fd, POLLIN, 0}};

    auto parse = [](const ImuDev& d, const uint8_t* r, int64_t& t, Eigen::Vector3d& v) {
        int16_t x, y, z;
        memcpy(&t, r + d.ts_off, 8);
        memcpy(&x, r + d.x_off, 2);
        memcpy(&y, r + d.y_off, 2);
        memcpy(&z, r + d.z_off, 2);
        v << x * d.scale, y * d.scale, z * d.scale;
    };

    int64_t last_data = now_ns();
    int64_t last_timestamp[2] = {0, 0};
    while (!*S.stop) {
        S.imu_loop_ns = now_ns();
        int r = poll(p, 2, 200);
        if (r < 0 && errno != EINTR)
            die(std::string("imu poll: ") + strerror(errno));
        if (r <= 0)
            continue;
        for (int i = 0; i < 2; i++) {
            if (p[i].revents & (POLLERR | POLLHUP | POLLNVAL))
                die("IMU device disconnected");
            if (!(p[i].revents & POLLIN))
                continue;
            ImuDev& d = (i == 0) ? *acc : *gyr;
            ssize_t n = read(d.fd, buf.data(), (buf.size() / d.rec) * d.rec);
            if (n < 0 && (errno == EINTR || errno == EAGAIN))
                continue;
            if (n < 0)
                die("imu read " + d.chardev + ": " + strerror(errno));
            if (n == 0)
                die("IMU stream ended");
            if (n % d.rec)
                die("IIO returned a partial record from " + d.chardev);
            if (n > 0) {
                int64_t t = now_ns();
                if ((t - last_data) * 1e-6 > S.read_gap_ms_max)
                    S.read_gap_ms_max = (t - last_data) * 1e-6;
                last_data = t;
            }
            // Preserve raw samples for replay before filtering or shifting timestamps.
            if (d.rec_fp)
                fwrite(buf.data(), 1, size_t(n), d.rec_fp);
            for (ssize_t o = 0; o < n; o += d.rec) {
                int64_t t;
                Eigen::Vector3d v;
                parse(d, buf.data() + o, t, v);
                if (t <= last_timestamp[i] || t > now_ns() + 10000000LL)
                    die("invalid or non-increasing IMU timestamp");
                last_timestamp[i] = t;
                v = (i == 0 ? af : gf).step(S.lpf, t, v);
                auto& queue = i == 0 ? aq : gq;
                if (queue.size() >= 1024)
                    die("unpaired IMU queue overflow");
                if (!queue.empty() && t - S.lpf.delay_ns <= queue.back().first)
                    die("non-increasing IMU timestamp");
                queue.emplace_back(t - S.lpf.delay_ns, v);
            }
        }
        // Lay rows on the gyro's timestamps with accel interpolated onto them --
        // the same pairing kalibr_imu_csv.py used for calibration.
        while (!gq.empty() && !aq.empty() && aq.back().first >= gq.front().first) {
            auto [tg, w] = gq.front();
            gq.pop_front();
            if (tg < aq.front().first)
                continue; // before the first accel sample
            while (aq.size() >= 2 && aq[1].first <= tg)
                aq.pop_front();
            Eigen::Vector3d a = aq[0].second;
            if (aq.size() >= 2 && aq[1].first > aq[0].first) {
                double f = double(tg - aq[0].first) / double(aq[1].first - aq[0].first);
                a = aq[0].second + f * (aq[1].second - aq[0].second);
            }
            ov_core::ImuData m;
            m.timestamp = tg * 1e-9;
            m.wm = w;
            m.am = a;
            {
                std::lock_guard<std::mutex> lock(S.q_mtx);
                if (S.imu_q.size() >= 1024)
                    die("IMU queue overflow; estimator cannot keep up");
                S.imu_q.push_back(m);
            }
            S.imu_n++;
        }
        S.q_cv.notify_one();
    }
}

// ---------------------------------------------------------------- camera

static void meta_thread(Shared& S, int fd) {
    static const std::string key = "\"SensorTimestamp\"";
    std::string buf;
    char tmp[8192];
    ssize_t n;
    while ((n = read_some(fd, tmp, sizeof tmp, *S.stop)) > 0) {
        buf.append(tmp, size_t(n));
        if (buf.size() > 65536)
            die("camera metadata record exceeds size limit");
        size_t k;
        while ((k = buf.find(key)) != std::string::npos) {
            size_t c = buf.find(':', k);
            size_t e = (c == std::string::npos) ? c : buf.find_first_of(",}\n", c);
            if (e == std::string::npos)
                break; // number not complete yet
            int64_t ts = std::stoll(buf.substr(c + 1, e - c - 1));
            buf.erase(0, e);
            std::lock_guard<std::mutex> lk(S.meta_mtx);
            if (S.meta_ts.size() >= 128)
                die("camera metadata backlog overflow");
            S.meta_ts.push_back(ts);
            S.meta_cv.notify_all();
        }
        if (buf.size() > 64 && buf.find(key) == std::string::npos)
            buf.erase(0, buf.size() - 32);
    }
    std::lock_guard<std::mutex> lk(S.meta_mtx);
    S.meta_done = true;
    S.meta_cv.notify_all();
}

static void frame_thread(Shared& S, int fd, FILE* rec_fp, FILE* meta_fp, size_t max_queue) {
    const size_t npx = size_t(S.W) * S.H;
    std::vector<uint8_t> raw(npx * 2);
    size_t idx = 0;
    while (read_full(fd, raw.data(), raw.size(), *S.stop)) {
        idx++;
        int64_t ts;
        {
            std::unique_lock<std::mutex> lk(S.meta_mtx);
            // Observe stop requests even when no metadata notification arrives.
            while (S.meta_ts.empty() && !S.meta_done && !*S.stop)
                S.meta_cv.wait_for(lk, std::chrono::milliseconds(200));
            if (S.meta_ts.empty())
                break;
            ts = S.meta_ts.front();
            S.meta_ts.pop_front();
        }
        if (rec_fp)
            fwrite(raw.data(), 1, raw.size(), rec_fp);
        // Metadata goes out with each frame, not at exit: a run killed before its
        // summary still leaves timestamps for every frame written (the converter
        // reads a file missing its closing bracket).
        if (meta_fp) {
            fprintf(meta_fp, "%s{\"SensorTimestamp\": %lld}", idx == 1 ? "" : ",\n", (long long)ts);
            fflush(meta_fp);
        }
        // R8 mode: 8-bit data in the high byte of a little-endian u16. The low
        // byte must be zero; if it is not, this is a different layout and >>8
        // would silently produce a plausible wrong image. Whole first frame, then
        // a sparse sample of every frame.
        size_t step = (idx == 1) ? 1 : 997;
        for (size_t i = 0; i < npx; i += step)
            if (raw[2 * i])
                die("frame low byte nonzero -- not the R8 8-bit-in-16 layout");
        cv::Mat img(S.H, S.W, CV_8UC1);
        uint8_t* dst = img.data;
        for (size_t i = 0; i < npx; i++)
            dst[i] = raw[2 * i + 1];

        ov_core::CameraData c;
        c.timestamp = ts * 1e-9;
        c.sensor_ids.push_back(0);
        c.images.push_back(img);
        c.masks.push_back(cv::Mat::zeros(S.H, S.W, CV_8UC1));
        S.frames_in++;
        {
            std::lock_guard<std::mutex> lk(S.q_mtx);
            // Bounded queue: if the estimator falls behind, drop the OLDEST frame
            // rather than lag without limit. Drops are counted and reported --
            // they are what broke live tracking in the sim.
            while (S.cam_q.size() >= max_queue) {
                S.cam_q.pop_front();
                S.frames_dropped++;
            }
            S.cam_q.push_back({std::move(c), ts, idx - 1});
        }
        S.q_cv.notify_one();
    }
    std::lock_guard<std::mutex> lk(S.q_mtx);
    S.frames_done = true;
    S.q_cv.notify_one();
}

// ---------------------------------------------------------------- estimator

static void update_thread(Shared& S, FILE* out) {
    double calib_dt = S.sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
    int64_t stall_since = 0;
    bool first_update = false;
    while (!*S.stop) {
        CapturedFrame captured;
        {
            std::unique_lock<std::mutex> lk(S.q_mtx);
            // Both streams use CLOCK_MONOTONIC. Wait for IMU coverage through the
            // camera time plus the estimated offset; camera updates run off the IMU thread.
            auto ready = [&] {
                return !S.cam_q.empty() &&
                       S.cam_q.front().camera.timestamp < S.last_imu_ns * 1e-9 - calib_dt;
            };
            S.q_cv.wait_for(lk, std::chrono::milliseconds(100), [&] {
                return ready() || !S.imu_q.empty() || S.frames_done || *S.stop;
            });
            auto samples = std::move(S.imu_q);
            S.imu_q.clear();
            lk.unlock();
            for (const auto& sample : samples) {
                S.sys->feed_measurement_imu(sample);
                S.last_imu_ns = static_cast<int64_t>(std::llround(sample.timestamp * 1e9));
            }
            lk.lock();
            if (*S.stop)
                break;
            if (!ready()) {
                if (!S.frames_done)
                    continue;
                if (S.cam_q.empty())
                    break;
                // Frames left that the IMU never passed: give the IMU a second, then stop.
                if (!stall_since)
                    stall_since = now_ns();
                if (now_ns() - stall_since > 1000000000LL)
                    break;
                continue;
            }
            captured = std::move(S.cam_q.front());
            S.cam_q.pop_front();
        }
        auto& c = captured.camera;
        int64_t t0 = now_ns();
        S.sys->feed_measurement_camera(c);
        if (S.features.active())
            S.features.push({captured.timestamp_ns, captured.frame_index, S.W, S.H,
                             S.sys->initialized_time() > 0, S.sys->snapshot()});

        int64_t t1 = now_ns();
        double ms = (t1 - t0) * 1e-6;
        S.upd_ms_sum = S.upd_ms_sum + ms;
        if (ms > S.upd_ms_max)
            S.upd_ms_max = ms;
        S.lag_ms_last = (t1 - int64_t(c.timestamp * 1e9)) * 1e-6;
        S.frames_done_n++;

        if (!S.vio_init && S.sys->initialized_time() > 0) {
            S.vio_init = true;
            ov_core::Printer::setPrintLevel(S.run_verbosity);
            printf("\n  *** INITIALIZED at t=%.3f s -- poses streaming; zero-velocity updates hold "
                   "it while it rests ***\n\n",
                   S.sys->initialized_time());
            fflush(stdout);
        }

        auto state = S.sys->get_state();
        calib_dt = state->_calib_dt_CAMtoIMU->value()(0);
        // Publish the initialized ZUPT state at rest. Waiting for a full visual
        // update would withhold the poses ArduPilot needs for its pre-arm check.
        if (!S.vio_init)
            continue;
        // This flag belongs to the capture, not to the process lifetime.
        if (!first_update && S.sys->initialized()) {
            first_update = true;
            printf("\n  --- moving: first full visual update at t=%.3f s ---\n\n",
                   state->_timestamp);
            fflush(stdout);
        }
        auto imu = state->_imu;
        Eigen::Vector4d q = imu->quat();
        Eigen::Vector3d p = imu->pos(), v = imu->vel(), bg = imu->bias_g(), ba = imu->bias_a();
        vio::EstimatorEstimate snapshot;
        snapshot.timestamp =
            vio::MonotonicTime{static_cast<int64_t>(std::llround(state->_timestamp * 1e9))};
        snapshot.generation = S.generation;
        snapshot.status = vio::EstimatorStatus::Ready;
        snapshot.position_world_m = p;
        snapshot.velocity_world_mps = v;
        snapshot.world_from_imu = Eigen::Quaterniond(q(3), q(0), q(1), q(2));
        S.estimates->store(snapshot);
        if (out) {
            fprintf(
                out,
                "%.9f %.9f %.9f %.9f %.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f "
                "%.6f\n",
                state->_timestamp, q(0), q(1), q(2), q(3), p(0), p(1), p(2), v(0), v(1), v(2),
                bg(0), bg(1), bg(2), ba(0), ba(1), ba(2));
            fflush(out);
        }
        std::lock_guard<std::mutex> lk(S.st_mtx);
        if (!S.init) {
            S.init = true;
            S.init_t = state->_timestamp;
            S.p_first = p;
            S.p_last = p;
        }
        S.path_m += (p - S.p_last).norm();
        S.p_last = p;
        S.v_last = v;
    }
}

namespace vio {
namespace {
class OpenVinsRunner final : public EstimatorRunner {
public:
    OpenVinsRunner(const std::filesystem::path& config, const std::string& verbosity) {
        auto parser = std::make_shared<ov_core::YamlParser>(config.string());
        ov_core::Printer::setPrintLevel("INFO");
        ov_msckf::VioManagerOptions params;
        params.print_and_load(parser);
        params.use_multi_threading_subs = false;
        if (!parser->successful())
            die("OpenVINS could not parse " + config.string());
        if (params.camera_intrinsics.at(0)->w() != 1280 ||
            params.camera_intrinsics.at(0)->h() != 800)
            die("config resolution does not match the camera stream");
        // OpenVINS has already inverted T_imu_cam when loading this calibration.
        camera_from_imu_ = ov_core::quat_2_Rot(params.camera_extrinsics.at(0).head<4>());
        state_.sys = std::make_shared<LoggedVioManager>(params);
        state_.run_verbosity = verbosity;
    }
    Eigen::Matrix3d camera_from_imu() const override {
        return camera_from_imu_;
    }
    void run(const RunnerOptions& options, std::atomic<bool>& stop,
             LatestEstimate& estimates) override;

private:
    Shared state_;
    Eigen::Matrix3d camera_from_imu_;
};

/// Close capture files/descriptors after the workers that use them have joined.
struct CaptureFiles {
    std::vector<int> descriptors;
    std::vector<FILE*> files;
    ~CaptureFiles() {
        for (auto* file : files)
            fclose(file);
        for (int fd : descriptors)
            close(fd);
    }
    FILE* output(const std::string& path, const char* mode) {
        auto* file = fopen(path.c_str(), mode);
        if (!file)
            die("cannot open " + path);
        files.push_back(file);
        return file;
    }
    int input(const std::string& path) {
        int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0)
            die("cannot open " + path + ": " + strerror(errno));
        descriptors.push_back(fd);
        return fd;
    }
};

/// Catch worker failures, request a shared stop, and join even on partial startup.
struct Workers {
    std::atomic<bool>& stop;
    std::mutex mutex;
    std::exception_ptr error;
    std::vector<std::thread> threads;
    explicit Workers(std::atomic<bool>& flag) : stop(flag) {}
    void launch(std::function<void()> work) {
        threads.emplace_back([this, work = std::move(work)] {
            try {
                work();
            } catch (...) {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (!error)
                        error = std::current_exception();
                }
                stop = true;
            }
        });
    }
    void join() {
        for (auto& thread : threads)
            if (thread.joinable())
                thread.join();
    }
    ~Workers() {
        stop = true;
        join();
    }
};
} // namespace

/// Run one capture inside this process; snapshots leave through the one-slot mailbox.
void OpenVinsRunner::run(const RunnerOptions& options, std::atomic<bool>& stop,
                         LatestEstimate& estimates) {
    Shared& S = state_;
    S.stop = &stop;
    S.estimates = &estimates;
    S.generation = options.generation;
    if (options.imu_lpf_hz > 0) {
        if (options.imu_lpf_hz >= 0.45 * options.imu_rate_hz)
            die("IMU filter cutoff too high");
        S.lpf.design(options.imu_lpf_hz, options.imu_rate_hz);
    }
    if (options.max_camera_queue == 0 || options.max_camera_queue > 100)
        die("camera queue must hold 1..100 frames");
    if (options.devices.size() != 2 || options.devices[0].kind == options.devices[1].kind)
        die("capture requires one accelerometer and one gyroscope");
    for (const auto& d : options.devices) {
        if ((d.kind != "accel" && d.kind != "gyro") || d.record_bytes < 8 ||
            d.record_bytes > 65536 || d.timestamp_offset < 0 ||
            d.timestamp_offset > d.record_bytes - 8 || !std::isfinite(d.scale) || d.scale <= 0)
            die("invalid IMU record description");
        for (int offset : d.axis_offsets)
            if (offset < 0 || offset > d.record_bytes - 2)
                die("invalid IMU axis offset");
    }
    CaptureFiles files;
    std::vector<ImuDev> devs;
    for (const auto& device : options.devices) {
        ImuDev d;
        d.kind = device.kind;
        d.chardev = device.chardev.string();
        d.rec = device.record_bytes;
        d.scale = device.scale;
        d.ts_off = device.timestamp_offset;
        d.x_off = device.axis_offsets[0];
        d.y_off = device.axis_offsets[1];
        d.z_off = device.axis_offsets[2];
        d.fd = files.input(d.chardev);
        if (!options.recording_prefix.empty())
            d.rec_fp =
                files.output(options.recording_prefix.string() + ".imu_" + d.kind + ".bin", "wb");
        devs.push_back(d);
    }
    FILE* out =
        options.estimate_log.empty() ? nullptr : files.output(options.estimate_log.string(), "w");
    if (out)
        fprintf(out, "# timestamp(s) q(JPL xyzw) p v bg ba\n");
    FILE* y16 = nullptr;
    FILE* meta_json = nullptr;
    if (!options.recording_prefix.empty()) {
        y16 = files.output(options.recording_prefix.string() + ".y16", "wb");
        setvbuf(y16, nullptr, _IOFBF, 8 << 20);
        meta_json = files.output(options.recording_prefix.string() + ".meta.json", "w");
        fprintf(meta_json, "[\n");
        S.features.start(options.recording_prefix.string() + ".features.jsonl");
    }
    int mfd = files.input(options.metadata.string());
    int ffd = files.input(options.frames.string());
    Workers workers(stop);
    workers.launch([&] { imu_thread(S, devs); });
    workers.launch([&] { update_thread(S, out); });
    workers.launch([&] { meta_thread(S, mfd); });
    workers.launch([&] { frame_thread(S, ffd, y16, meta_json, options.max_camera_queue); });
    const auto start = now_ns();
    bool imu_stalled = false;
    while (!stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        {
            std::lock_guard<std::mutex> lock(S.q_mtx);
            if (S.frames_done)
                break;
        }
        if (now_ns() - start > 3000000000LL &&
            (S.last_imu_ns == 0 || now_ns() - S.last_imu_ns > 2000000000LL)) {
            imu_stalled = true;
            stop = true;
            break;
        }
    }
    stop = true;
    S.q_cv.notify_all();
    S.meta_cv.notify_all();
    workers.join();
    S.features.close();
    if (meta_json)
        fprintf(meta_json, "\n]\n");
    if (workers.error)
        std::rethrow_exception(workers.error);
    if (imu_stalled)
        die("IMU samples stopped advancing");
    printf("vio_live: %ld frames received, %ld processed, %ld dropped\n", long(S.frames_in),
           long(S.frames_done_n), long(S.frames_dropped));
}

std::unique_ptr<EstimatorRunner> make_openvins_runner(const std::filesystem::path& config,
                                                      const std::string& verbosity) {
    return std::make_unique<OpenVinsRunner>(config, verbosity);
}
} // namespace vio
