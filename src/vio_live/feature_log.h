#pragma once
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <utility>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct FeaturePoint { size_t id; float x, y; };
struct FeatureFrame {
  int64_t timestamp_ns; size_t frame_index; int width, height; bool initialized;
  std::vector<FeaturePoint> points;
};

// Only snapshots cross this queue; disk I/O never holds the producer lock.
class FeatureLog {
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<FeatureFrame> queue;
  std::thread worker;
  bool done = false;
  std::atomic<bool> enabled{false};
public:
  std::atomic<size_t> dropped{0};
  void start(const std::string &path) {
    enabled = true;
    worker = std::thread([this, path] {
      FILE *f = fopen(path.c_str(), "w");
      if (!f) { enabled = false; fprintf(stderr, "feature log unavailable: %s\n", path.c_str()); return; }
      fprintf(f, "{\"type\":\"header\",\"version\":1,\"source\":\"live_tracker\",\"coordinates\":\"raw_pixels\",\"frame_index_base\":0,\"openvins_revision\":\"69488123ed9362dd44b6f28e7f4680abbff1442b\"}\n");
      while (true) {
        FeatureFrame frame;
        {
          std::unique_lock<std::mutex> lock(mutex);
          cv.wait(lock, [&] { return done || !queue.empty(); });
          if (queue.empty()) break;
          frame = std::move(queue.front()); queue.pop_front();
        }
        fprintf(f, "{\"type\":\"frame\",\"timestamp_ns\":%lld,\"frame_index\":%zu,\"camera_id\":0,\"width\":%d,\"height\":%d,\"initialized\":%s,\"features\":[",
                (long long)frame.timestamp_ns, frame.frame_index, frame.width, frame.height, frame.initialized ? "true" : "false");
        for (size_t i = 0; i < frame.points.size(); ++i) {
          auto &p = frame.points[i];
          fprintf(f, "%s[%zu,%.6f,%.6f]", i ? "," : "", p.id, p.x, p.y);
        }
        fprintf(f, "]}\n");
        if (fflush(f) != 0 || ferror(f)) {
          enabled = false; fprintf(stderr, "feature log write failed; pose delivery continues\n"); break;
        }
      }
      fprintf(f, "{\"type\":\"end\",\"dropped\":%zu}\n", dropped.load());
      fclose(f);
    });
  }
  bool active() const { return enabled; }
  void push(FeatureFrame frame) {
    if (!enabled) return;
    std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
    if (!lock.owns_lock() || queue.size() >= 64) { ++dropped; return; }
    queue.push_back(std::move(frame)); cv.notify_one();
  }
  void close() {
    { std::lock_guard<std::mutex> lock(mutex); done = true; }
    cv.notify_one();
    if (worker.joinable()) worker.join();
    enabled = false;
  }
  ~FeatureLog() { close(); }
};
