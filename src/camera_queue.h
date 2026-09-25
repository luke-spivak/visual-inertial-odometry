#pragma once

#include "camera_source.h"
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>

namespace vio {
/// Short, bounded handoff from camera callbacks; old frames yield to current ones.
class CameraQueue {
public:
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.clear();
        error_ = nullptr;
        closed_ = false;
        dropped_ = 0;
    }
    void push(CameraFrame frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_)
            return;
        if (frames_.size() == 4) {
            frames_.pop_front();
            ++dropped_;
        }
        frames_.push_back(std::move(frame));
        ready_.notify_one();
    }
    std::optional<CameraFrame> read(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait_for(lock, timeout, [&] { return closed_ || error_ || !frames_.empty(); });
        if (error_)
            std::rethrow_exception(error_);
        if (frames_.empty())
            return std::nullopt;
        auto frame = std::move(frames_.front());
        frames_.pop_front();
        return frame;
    }
    void fail(std::exception_ptr error) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!error_)
            error_ = error;
        frames_.clear();
        ready_.notify_all();
    }
    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        frames_.clear();
        ready_.notify_all();
    }
    std::uint64_t dropped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<CameraFrame> frames_;
    std::exception_ptr error_;
    bool closed_{true};
    std::uint64_t dropped_{0};
};
} // namespace vio
