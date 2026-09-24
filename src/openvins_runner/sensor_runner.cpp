// sensor_runner.cpp -- OpenVINS running live on viopi. No ROS.
//
// Started by src/capture_session.py, which configures the IIO devices, starts
// rpicam-raw and hands this program:
//   --imu FILE     one line per sensor: kind chardev record_bytes scale ts_off x_off y_off z_off
//   --frames FIFO  rpicam-raw -o: raw Y16 frames, 8-bit data in the high byte
//   --meta FIFO    rpicam-raw --metadata (json): one record per frame, in order
//   --config YAML  OpenVINS estimator_config.yaml (Kalibr chain files beside it)
//   --out FILE     state per processed frame: t q(JPL xyzw) p v bg ba, the column
//                  layout of OpenVINS's save_total_state, so vio_closure.py reads it
//   --record PFX   optional: write PFX.y16, PFX.meta.json, PFX.imu_{accel,gyro}.bin
//                  in the capture script's format, so vio_bag_from_raw.py can
//                  replay this exact run offline on the VM. The IMU is recorded
//                  raw, before the low-pass below.
//   --imu-lpf HZ   optional: low-pass the IMU at HZ before OpenVINS sees it
//                  (2nd-order Butterworth, designed at --imu-rate, default 440 Hz,
//                  the rate the part delivers at ODR 416). See ImuLowPass.
//
// Threads mirror OpenVINS's own ROS 2 node (ROS2Visualizer): IMU samples are fed
// as they arrive; a camera frame is processed once the IMU has passed its
// timestamp (minus the estimated camera-IMU offset), on a separate thread so
// IMU reading never waits on an update.
//
// Timestamps: IIO samples and SensorTimestamp are both CLOCK_MONOTONIC (udev
// rule + patched driver; libcamera natively). The only offset applied here is
// the low-pass's group delay, taken off the IMU timestamps.

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

#include "feature_log.h"
#include "track/TrackBase.h"
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "types/Vec.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop = true; }

static int64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

[[noreturn]] static void die(const std::string &msg) {
  fprintf(stderr, "vio_live: FAIL: %s\n", msg.c_str());
  fflush(stderr);
  _exit(1);
}

// Wait for readable data or stop. Returns bytes read, 0 on EOF, -1 on stop.
static ssize_t read_some(int fd, void *buf, size_t n) {
  while (!g_stop) {
    pollfd p{fd, POLLIN, 0};
    int r = poll(&p, 1, 200);
    if (r < 0 && errno != EINTR) die(std::string("poll: ") + strerror(errno));
    if (r <= 0) continue;
    ssize_t got = read(fd, buf, n);
    if (got < 0 && (errno == EINTR || errno == EAGAIN)) continue;
    if (got < 0) die(std::string("read: ") + strerror(errno));
    return got;
  }
  return -1;
}

static bool read_full(int fd, uint8_t *buf, size_t n) {
  size_t have = 0;
  while (have < n) {
    ssize_t got = read_some(fd, buf + have, n - have);
    if (got <= 0) return false;
    have += size_t(got);
  }
  return true;
}

struct ImuDev {
  std::string kind, chardev;
  int rec = 0, ts_off = 0, x_off = 0, y_off = 0, z_off = 0;
  double scale = 0;
  int fd = -1;
  FILE *rec_fp = nullptr;
};

// The motors shake the sensor plate at their rotation rate, 176-195 Hz in a
// hover, and the grommet mount passes about a third of it: 6-7 m/s^2 RMS per
// axis in flight against 0.01 at rest (docs/development-log.md, VIO alongside GPS). The
// filter's noise model is ~0.06 m/s^2 per sample, so it would trust the IMU
// ~100x beyond what it gets. Everything VIO needs sits below ~20 Hz, so a
// 50 Hz low-pass costs nothing in band (gain 0.99 at 20 Hz) and cuts 188 Hz
// 133x -- the bilinear transform steepens it near Nyquist.
//
// Its price is delay: ~4.3 ms at 50 Hz, 4.3-4.9 ms across 0-20 Hz. The DC
// value is taken off every IMU timestamp so filtered samples line up with the
// camera; OpenVINS's online time-offset calibration takes up the rest.
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

  Eigen::Vector3d step(const ImuLowPass &f, int64_t t, const Eigen::Vector3d &x) {
    if (!f.on) return x;
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
    const auto &uv = obs[0]; const auto &id = ids[0];
    if (uv.size() != id.size()) return points;
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
  std::shared_ptr<LoggedVioManager> sys;
  FeatureLog features;
  ImuLowPass lpf;
  int W = 1280, H = 800;

  std::mutex meta_mtx;
  std::condition_variable meta_cv;
  std::vector<int64_t> meta_ts;
  bool meta_done = false;

  std::mutex q_mtx;
  std::condition_variable q_cv;
  std::deque<CapturedFrame> cam_q;
  bool frames_done = false;

  std::atomic<int64_t> last_imu_ns{0};
  std::atomic<long> imu_n{0}, frames_in{0}, frames_done_n{0}, frames_dropped{0};
  std::atomic<double> upd_ms_sum{0}, upd_ms_max{0}, lag_ms_last{0};
  // IMU-path diagnostics. Bench run 1 (2026-09-10): IMU samples stopped
  // reaching this program 15 s in while the IMU interrupt kept firing at
  // 440/s. These say which side went quiet: the IMU thread (heartbeat age),
  // OpenVINS's feed (feed max), or the kernel buffer (kbuf fill).
  std::atomic<int64_t> imu_loop_ns{0};
  std::atomic<double> feed_ms_max{0}, read_gap_ms_max{0};

  // OpenVINS logs why initialisation is failing only at INFO, which after
  // initialisation is ~6 lines per frame. Run at INFO until initialised, then
  // drop to the requested level (walk2, 2026-09-10: a stuck init with the
  // reason invisible at WARNING).
  std::string run_verbosity = "WARNING";
  // OpenVINS's own initialisation (initialized_time() > 0). Distinct from
  // initialized(), which also needs one full update: with ZUPT on, every frame
  // at rest is a zero-velocity update that returns before the full update
  // (VioManager.cpp:299-304, timelastupdate set only at :651), so at rest
  // initialized() stays false however long the rig waits. walk2 (2026-09-10)
  // sat 16 s initialised, reporting "not initialized", waiting to be lifted.
  std::atomic<bool> vio_init{false};

  std::mutex st_mtx;  // last reported state, for the stats line and summary
  bool init = false;
  double init_t = 0;
  Eigen::Vector3d p_first = Eigen::Vector3d::Zero(), p_last = Eigen::Vector3d::Zero(), v_last = Eigen::Vector3d::Zero();
  double path_m = 0;
};

// ---------------------------------------------------------------- IMU

static void imu_thread(Shared &S, std::vector<ImuDev> &devs) {
  ImuDev *acc = nullptr, *gyr = nullptr;
  for (auto &d : devs) (d.kind == "accel" ? acc : gyr) = &d;
  if (!acc || !gyr) die("--imu must list both accel and gyro");

  std::deque<std::pair<int64_t, Eigen::Vector3d>> aq, gq;
  ImuLowPassState af, gf;
  std::vector<uint8_t> buf(64 * 1024);
  pollfd p[2] = {{acc->fd, POLLIN, 0}, {gyr->fd, POLLIN, 0}};

  auto parse = [](const ImuDev &d, const uint8_t *r, int64_t &t, Eigen::Vector3d &v) {
    int16_t x, y, z;
    memcpy(&t, r + d.ts_off, 8);
    memcpy(&x, r + d.x_off, 2);
    memcpy(&y, r + d.y_off, 2);
    memcpy(&z, r + d.z_off, 2);
    v << x * d.scale, y * d.scale, z * d.scale;
  };

  int64_t last_data = now_ns();
  while (!g_stop) {
    S.imu_loop_ns = now_ns();
    int r = poll(p, 2, 200);
    if (r < 0 && errno != EINTR) die(std::string("imu poll: ") + strerror(errno));
    if (r <= 0) continue;
    for (int i = 0; i < 2; i++) {
      if (!(p[i].revents & POLLIN)) continue;
      ImuDev &d = (i == 0) ? *acc : *gyr;
      ssize_t n = read(d.fd, buf.data(), (buf.size() / d.rec) * d.rec);
      if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
      if (n < 0) die("imu read " + d.chardev + ": " + strerror(errno));
      if (n % d.rec) die("IIO returned a partial record from " + d.chardev);
      if (n > 0) {
        int64_t t = now_ns();
        if ((t - last_data) * 1e-6 > S.read_gap_ms_max) S.read_gap_ms_max = (t - last_data) * 1e-6;
        last_data = t;
      }
      if (d.rec_fp) fwrite(buf.data(), 1, size_t(n), d.rec_fp);
      for (ssize_t o = 0; o < n; o += d.rec) {
        int64_t t;
        Eigen::Vector3d v;
        parse(d, buf.data() + o, t, v);
        v = (i == 0 ? af : gf).step(S.lpf, t, v);
        (i == 0 ? aq : gq).emplace_back(t - S.lpf.delay_ns, v);
      }
    }
    // Lay rows on the gyro's timestamps with accel interpolated onto them --
    // the same pairing kalibr_imu_csv.py used for calibration.
    while (!gq.empty() && !aq.empty() && aq.back().first >= gq.front().first) {
      auto [tg, w] = gq.front();
      gq.pop_front();
      if (tg < aq.front().first) continue;  // before the first accel sample
      while (aq.size() >= 2 && aq[1].first <= tg) aq.pop_front();
      Eigen::Vector3d a = aq[0].second;
      if (aq.size() >= 2 && aq[1].first > aq[0].first) {
        double f = double(tg - aq[0].first) / double(aq[1].first - aq[0].first);
        a = aq[0].second + f * (aq[1].second - aq[0].second);
      }
      ov_core::ImuData m;
      m.timestamp = tg * 1e-9;
      m.wm = w;
      m.am = a;
      int64_t f0 = now_ns();
      S.sys->feed_measurement_imu(m);
      double fm = (now_ns() - f0) * 1e-6;
      if (fm > S.feed_ms_max) S.feed_ms_max = fm;
      S.last_imu_ns = tg;
      S.imu_n++;
    }
    S.q_cv.notify_one();
  }
}

// ---------------------------------------------------------------- camera

static void meta_thread(Shared &S, int fd) {
  static const std::string key = "\"SensorTimestamp\"";
  std::string buf;
  char tmp[8192];
  ssize_t n;
  while ((n = read_some(fd, tmp, sizeof tmp)) > 0) {
    buf.append(tmp, size_t(n));
    size_t k;
    while ((k = buf.find(key)) != std::string::npos) {
      size_t c = buf.find(':', k);
      size_t e = (c == std::string::npos) ? c : buf.find_first_of(",}\n", c);
      if (e == std::string::npos) break;  // number not complete yet
      int64_t ts = std::stoll(buf.substr(c + 1, e - c - 1));
      buf.erase(0, e);
      std::lock_guard<std::mutex> lk(S.meta_mtx);
      S.meta_ts.push_back(ts);
      S.meta_cv.notify_all();
    }
    if (buf.size() > 64 && buf.find(key) == std::string::npos) buf.erase(0, buf.size() - 32);
  }
  std::lock_guard<std::mutex> lk(S.meta_mtx);
  S.meta_done = true;
  S.meta_cv.notify_all();
}

static void frame_thread(Shared &S, int fd, FILE *rec_fp, FILE *meta_fp, size_t max_queue) {
  const size_t npx = size_t(S.W) * S.H;
  std::vector<uint8_t> raw(npx * 2);
  size_t idx = 0;
  while (read_full(fd, raw.data(), raw.size())) {
    idx++;
    int64_t ts;
    {
      std::unique_lock<std::mutex> lk(S.meta_mtx);
      while (!(S.meta_ts.size() >= idx || S.meta_done || g_stop))
        S.meta_cv.wait_for(lk, std::chrono::milliseconds(200));  // g_stop is set by a signal, which notifies nothing
      if (S.meta_ts.size() < idx) break;
      ts = S.meta_ts[idx - 1];
    }
    if (rec_fp) fwrite(raw.data(), 1, raw.size(), rec_fp);
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
      if (raw[2 * i]) die("frame low byte nonzero -- not the R8 8-bit-in-16 layout");
    cv::Mat img(S.H, S.W, CV_8UC1);
    uint8_t *dst = img.data;
    for (size_t i = 0; i < npx; i++) dst[i] = raw[2 * i + 1];

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

static void update_thread(Shared &S, FILE *out) {
  double calib_dt = S.sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
  int64_t stall_since = 0;
  while (true) {
    CapturedFrame captured;
    {
      std::unique_lock<std::mutex> lk(S.q_mtx);
      auto ready = [&] { return !S.cam_q.empty() && S.cam_q.front().camera.timestamp < S.last_imu_ns * 1e-9 - calib_dt; };
      S.q_cv.wait_for(lk, std::chrono::milliseconds(100), [&] { return ready() || S.frames_done; });
      if (!ready()) {
        if (!S.frames_done) continue;
        if (S.cam_q.empty()) break;
        // Frames left that the IMU never passed: give the IMU a second, then stop.
        if (!stall_since) stall_since = now_ns();
        if (now_ns() - stall_since > 1000000000LL) break;
        continue;
      }
      captured = std::move(S.cam_q.front());
      S.cam_q.pop_front();
    }
    auto &c = captured.camera;
    int64_t t0 = now_ns();
    S.sys->feed_measurement_camera(c);
    if (S.features.active())
      S.features.push({captured.timestamp_ns, captured.frame_index, S.W, S.H,
                       S.sys->initialized_time() > 0, S.sys->snapshot()});

    int64_t t1 = now_ns();
    double ms = (t1 - t0) * 1e-6;
    S.upd_ms_sum = S.upd_ms_sum + ms;
    if (ms > S.upd_ms_max) S.upd_ms_max = ms;
    S.lag_ms_last = (t1 - int64_t(c.timestamp * 1e9)) * 1e-6;
    S.frames_done_n++;

    if (!S.vio_init && S.sys->initialized_time() > 0) {
      S.vio_init = true;
      ov_core::Printer::setPrintLevel(S.run_verbosity);
      printf("\n  *** INITIALIZED at t=%.3f s -- poses streaming; zero-velocity updates hold it while it rests ***\n\n",
             S.sys->initialized_time());
      fflush(stdout);
    }

    auto state = S.sys->get_state();
    calib_dt = state->_calib_dt_CAMtoIMU->value()(0);
    // Poses go out from OpenVINS's own initialisation, not from initialized(),
    // which also waits for a full visual update that ZUPT at rest never allows.
    // At rest the ZUPT-held state (still, gravity-aligned) is the right answer,
    // and ArduPilot will not arm under VISO_TYPE 2 until poses arrive: gating on
    // initialized() meant lifting the aircraft after INITIALIZED (2026-09-14).
    if (!S.vio_init) continue;
    static bool first_update = false;
    if (!first_update && S.sys->initialized()) {
      first_update = true;
      printf("\n  --- moving: first full visual update at t=%.3f s ---\n\n", state->_timestamp);
      fflush(stdout);
    }
    auto imu = state->_imu;
    Eigen::Vector4d q = imu->quat();
    Eigen::Vector3d p = imu->pos(), v = imu->vel(), bg = imu->bias_g(), ba = imu->bias_a();
    fprintf(out, "%.9f %.9f %.9f %.9f %.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
            state->_timestamp, q(0), q(1), q(2), q(3), p(0), p(1), p(2), v(0), v(1), v(2),
            bg(0), bg(1), bg(2), ba(0), ba(1), ba(2));
    fflush(out);  // mavlink_bridge.py follows this file live; unflushed, poses reach the FC in 4 KB bursts
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

// ---------------------------------------------------------------- diagnostics

static long read_long(const std::string &path) {
  FILE *f = fopen(path.c_str(), "r");
  if (!f) return -1;
  long v = -1;
  if (fscanf(f, "%ld", &v) != 1) v = -1;
  fclose(f);
  return v;
}

// Total st_lsm6dsx interrupts across CPUs, from /proc/interrupts.
static long lsm_irq_count() {
  std::ifstream f("/proc/interrupts");
  std::string line;
  while (std::getline(f, line)) {
    if (line.find("lsm6dsx") == std::string::npos) continue;
    std::istringstream ss(line);
    std::string irq;
    ss >> irq;
    long sum = 0, v;
    while (ss >> v) sum += v;  // stops at the chip name
    return sum;
  }
  return -1;
}

// ---------------------------------------------------------------- main

int main(int argc, char **argv) {
  std::string imu_cfg, frames_fifo, meta_fifo, config, out_path, record, verbosity = "WARNING", init_verbosity = "INFO";
  size_t max_queue = 10;
  double imu_lpf_hz = 0, imu_rate_hz = 440;
  Shared S;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { if (i + 1 >= argc) die("missing value for " + a); return argv[++i]; };
    if (a == "--imu") imu_cfg = next();
    else if (a == "--frames") frames_fifo = next();
    else if (a == "--meta") meta_fifo = next();
    else if (a == "--config") config = next();
    else if (a == "--out") out_path = next();
    else if (a == "--record") record = next();
    else if (a == "--verbosity") verbosity = next();
    else if (a == "--init-verbosity") init_verbosity = next();
    else if (a == "--width") S.W = std::stoi(next());
    else if (a == "--height") S.H = std::stoi(next());
    else if (a == "--max-queue") max_queue = std::stoul(next());
    else if (a == "--imu-lpf") imu_lpf_hz = std::stod(next());
    else if (a == "--imu-rate") imu_rate_hz = std::stod(next());
    else die("unknown argument " + a);
  }
  if (imu_cfg.empty() || frames_fifo.empty() || meta_fifo.empty() || config.empty() || out_path.empty())
    die("usage: vio_live --imu FILE --frames FIFO --meta FIFO --config YAML --out FILE [--record PREFIX]"
        " [--imu-lpf HZ [--imu-rate HZ]]");
  if (imu_lpf_hz > 0) {
    if (imu_lpf_hz >= 0.45 * imu_rate_hz) die("--imu-lpf must sit well below half of --imu-rate");
    S.lpf.design(imu_lpf_hz, imu_rate_hz);
    printf("IMU low-pass %.0f Hz (2nd-order Butterworth at %.0f Hz): IMU timestamps shifted by -%.2f ms\n",
           imu_lpf_hz, imu_rate_hz, S.lpf.delay_ns * 1e-6);
    fflush(stdout);
  }

  struct sigaction sa{};
  sa.sa_handler = on_signal;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // OpenVINS, loaded exactly as run_subscribe_msckf does it.
  auto parser = std::make_shared<ov_core::YamlParser>(config);
  ov_core::Printer::setPrintLevel(init_verbosity);
  S.run_verbosity = verbosity;
  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  if (!parser->successful()) die("OpenVINS could not parse " + config + " -- see the output above");
  S.sys = std::make_shared<LoggedVioManager>(params);
  if (params.camera_intrinsics.at(0)->w() != S.W || params.camera_intrinsics.at(0)->h() != S.H)
    die("config resolution does not match the camera stream");

  std::vector<ImuDev> devs;
  {
    std::ifstream f(imu_cfg);
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      ImuDev d;
      if (!(ss >> d.kind >> d.chardev >> d.rec >> d.scale >> d.ts_off >> d.x_off >> d.y_off >> d.z_off))
        die("bad --imu line: " + line);
      d.fd = open(d.chardev.c_str(), O_RDONLY | O_NONBLOCK);
      if (d.fd < 0) die("open " + d.chardev + ": " + strerror(errno) + " (run as root)");
      if (!record.empty()) {
        d.rec_fp = fopen((record + ".imu_" + d.kind + ".bin").c_str(), "wb");
        if (!d.rec_fp) die("cannot write recording for " + d.kind);
      }
      devs.push_back(d);
    }
  }

  FILE *out = fopen(out_path.c_str(), "w");
  if (!out) die("cannot write " + out_path);
  fprintf(out, "# timestamp(s) q(JPL xyzw) p v bg ba -- vio_live, OpenVINS state after each processed frame\n");

  FILE *y16 = nullptr, *meta_json = nullptr;
  if (!record.empty()) {
    y16 = fopen((record + ".y16").c_str(), "wb");
    if (!y16) die("cannot write " + record + ".y16");
    setvbuf(y16, nullptr, _IOFBF, 8 << 20);
    meta_json = fopen((record + ".meta.json").c_str(), "w");
    if (!meta_json) die("cannot write " + record + ".meta.json");
    fprintf(meta_json, "[\n");
  }

  if (!record.empty()) S.features.start(record + ".features.jsonl");

  std::thread ti(imu_thread, std::ref(S), std::ref(devs));
  std::thread tu(update_thread, std::ref(S), out);
  printf("vio_live: estimator up; waiting for the camera\n");
  fflush(stdout);
  // Non-blocking opens. A blocking open of one FIFO waits for its writer, and
  // rpicam-raw opens its two outputs in its own order, blocking on each until
  // there is a reader -- open them in the other order and both sides wait
  // forever. Linux reports no POLLHUP on a FIFO until a writer has come and
  // gone, so read_some's poll simply waits for rpicam-raw to connect.
  int mfd = open(meta_fifo.c_str(), O_RDONLY | O_NONBLOCK);
  int ffd = open(frames_fifo.c_str(), O_RDONLY | O_NONBLOCK);
  if (mfd < 0 || ffd < 0) die("cannot open the camera FIFOs");
  std::thread tm(meta_thread, std::ref(S), mfd);
  std::thread tf(frame_thread, std::ref(S), ffd, y16, meta_json, max_queue);

  const int64_t start = now_ns();
  long last_imu = 0, last_in = 0, last_done = 0, last_irq = lsm_irq_count();
  double last_sum = 0;
  int64_t last_t = start;
  bool frames_finished = false;
  while (!frames_finished) {
    for (int i = 0; i < 20 && !frames_finished; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::lock_guard<std::mutex> lk(S.q_mtx);
      frames_finished = S.frames_done;
    }
    int64_t t = now_ns();
    double dt = (t - last_t) * 1e-9;
    long imu = S.imu_n, in = S.frames_in, done = S.frames_done_n;
    double sum = S.upd_ms_sum;
    double avg = (done > last_done) ? (sum - last_sum) / double(done - last_done) : 0;
    std::lock_guard<std::mutex> lk(S.st_mtx);
    printf("[%6.1f s] imu %3.0f Hz | cam %4.1f in %4.1f done fps, %ld dropped | update %4.0f ms avg %4.0f max | lag %4.0f ms | %s",
           (t - start) * 1e-9, (imu - last_imu) / dt, (in - last_in) / dt, (done - last_done) / dt,
           long(S.frames_dropped), avg, double(S.upd_ms_max), double(S.lag_ms_last),
           S.init ? "" : (S.vio_init ? "INITIALIZED\n" : "not initialized yet -- keep the rig still\n"));
    if (S.init)
      printf("p %+6.2f %+6.2f %+6.2f m  |v| %.2f m/s  path %.1f m\n", S.p_last(0), S.p_last(1), S.p_last(2), S.v_last.norm(), S.path_m);
    long irq = lsm_irq_count();
    std::string kbuf;
    for (auto &d : devs) {
      std::string node = d.chardev.substr(d.chardev.rfind('/') + 1);
      kbuf += (kbuf.empty() ? "" : "/") + std::to_string(read_long("/sys/bus/iio/devices/" + node + "/buffer0/data_available"));
    }
    printf("           diag: irq %4.0f/s | kernel buffer %s samples | imu thread last seen %4.0f ms ago | feed max %5.2f ms | read gap max %4.0f ms\n",
           last_irq >= 0 && irq >= 0 ? (irq - last_irq) / dt : -1.0, kbuf.c_str(),
           (now_ns() - S.imu_loop_ns) * 1e-6, double(S.feed_ms_max), double(S.read_gap_ms_max));
    fflush(stdout);
    last_imu = imu; last_in = in; last_done = done; last_sum = sum; last_t = t; last_irq = irq;
    S.upd_ms_max = 0;
    S.feed_ms_max = 0;
    S.read_gap_ms_max = 0;
  }

  tf.join();
  tu.join();
  S.features.close();
  printf("  feature logging dropped %zu records\n", S.features.dropped.load());
  g_stop = true;  // camera is finished: stop the IMU and metadata readers
  ti.join();
  tm.join();

  fclose(out);
  for (auto &d : devs) {
    if (d.rec_fp) fclose(d.rec_fp);
    close(d.fd);
  }
  if (y16) fclose(y16);
  if (meta_json) {
    fprintf(meta_json, "\n]\n");
    fclose(meta_json);
  }

  printf("\n=== vio_live summary ===\n");
  printf("  frames   %ld received, %ld processed, %ld dropped; update %.0f ms avg\n", long(S.frames_in),
         long(S.frames_done_n), long(S.frames_dropped), S.frames_done_n ? S.upd_ms_sum / double(S.frames_done_n) : 0.0);
  if (S.init) {
    Eigen::Vector3d d = S.p_last - S.p_first;
    printf("  path     %.2f m since initialization\n", S.path_m);
    printf("  closure  %.3f m = %.2f %% of path (end minus first initialized position; run vio_closure.py for the at-rest average)\n",
           d.norm(), S.path_m > 0 ? 100 * d.norm() / S.path_m : 0.0);
  } else if (S.vio_init) {
    printf("  initialized, but no pose was written\n");
  } else {
    printf("  NEVER INITIALIZED -- the rig must rest still for a few seconds first\n");
  }
  printf("  estimate %s\n", out_path.c_str());
  return 0;
}
