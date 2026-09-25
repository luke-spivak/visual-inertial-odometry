// Bounded recording handoff with checked disk writes and explicit incomplete-recording reports.
#include "recording.h"
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <stdexcept>

namespace vio {
namespace {
// A stuck disk cannot accumulate one detached writer per capture restart.
std::atomic<bool> writer_busy{false};
void fail(RecordingStatus& status, RecordingFailure reason) {
    auto expected = RecordingFailure::None;
    status.failure.compare_exchange_strong(expected, reason);
}
const char* reason(RecordingFailure failure) {
    switch (failure) {
    case RecordingFailure::None:
        return "none";
    case RecordingFailure::Backlog:
        return "queue_full";
    case RecordingFailure::Storage:
        return "storage_error";
    case RecordingFailure::ShutdownTimeout:
        return "shutdown_timeout";
    case RecordingFailure::WriterBusy:
        return "previous_writer_still_running";
    }
    return "unknown";
}

class FileSink final : public RecordingSink {
public:
    FileSink(std::vector<RecordingFile> files, std::filesystem::path status)
        : files_(std::move(files)), status_(std::move(status)) {}
    void write(const RecordingRecord& record) override {
        open();
        for (const auto& chunk : record.chunks) {
            auto& file = outputs_.at(chunk.file);
            file.write(reinterpret_cast<const char*>(chunk.bytes.data()), chunk.bytes.size());
            file.flush();
            if (!file)
                throw std::runtime_error("recording write failed");
        }
    }
    void finish(RecordingStatus& status) override {
        if (outputs_.empty())
            open();
        bool good = true;
        for (std::size_t i = 0; i < outputs_.size(); ++i) {
            outputs_[i] << files_[i].footer;
            outputs_[i].close();
            good = good && bool(outputs_[i]);
        }
        if (!good)
            fail(status, RecordingFailure::Storage);
        // Counts are per stream: accel, gyro, camera, estimate, features. After the
        // first gap we retain only a recording prefix; we never hide a hole in replay.
        const auto temporary = status_.string() + ".tmp";
        std::ofstream report(temporary);
        report << "{\"version\":1,\"writer_drained\":"
               << (status.failure == RecordingFailure::None ? "true" : "false") << ",\"reason\":\""
               << reason(status.failure) << "\",\"streams\":{";
        const char* names[] = {"accel", "gyro", "camera", "estimate", "features"};
        for (unsigned i = 0; i < 5; ++i)
            report << (i ? "," : "") << '"' << names[i]
                   << "\":{\"submitted\":" << status.submitted[i]
                   << ",\"written\":" << status.written[i] << ",\"rejected\":" << status.rejected[i]
                   << '}';
        report << "}}\n";
        report.close();
        if (!report)
            throw std::runtime_error("recording status write failed");
        std::filesystem::rename(temporary, status_);
    }

private:
    void open() {
        if (opened_)
            return;
        // An initial incomplete report survives an interrupted process or later full disk.
        std::ofstream report(status_);
        report << "{\"version\":1,\"complete\":false,\"reason\":\"unfinished\"}\n";
        report.close();
        if (!report)
            throw std::runtime_error("cannot create recording status");
        outputs_.resize(files_.size());
        for (std::size_t i = 0; i < files_.size(); ++i) {
            outputs_[i].open(files_[i].path, std::ios::binary);
            outputs_[i] << files_[i].header;
            if (!outputs_[i])
                throw std::runtime_error("cannot open recording output");
        }
        opened_ = true;
    }
    std::vector<RecordingFile> files_;
    std::filesystem::path status_;
    std::vector<std::ofstream> outputs_;
    bool opened_{false};
};
} // namespace

struct Recording::State {
    std::unique_ptr<RecordingSink> sink;
    std::shared_ptr<RecordingStatus> status;
    std::mutex mutex;
    std::condition_variable ready, finished;
    std::deque<RecordingRecord> queue;
    std::size_t bytes{0}, byte_limit, record_limit;
    bool closing{false}, done{false};
};

Recording::Recording(std::unique_ptr<RecordingSink> sink, std::shared_ptr<RecordingStatus> status,
                     std::size_t byte_limit, std::size_t record_limit)
    : state_(std::make_shared<State>()) {
    if (!sink || !status || !byte_limit || !record_limit)
        throw std::invalid_argument("invalid recording configuration");
    auto state = state_;
    state->sink = std::move(sink);
    state->status = std::move(status);
    state->byte_limit = byte_limit;
    state->record_limit = record_limit;
    if (writer_busy.exchange(true)) {
        fail(*state->status, RecordingFailure::WriterBusy);
        state->done = true;
        state->status->finished = true;
        return;
    }
    try {
        worker_ = std::thread([state] {
            try {
                for (;;) {
                    RecordingRecord record;
                    {
                        std::unique_lock<std::mutex> lock(state->mutex);
                        state->ready.wait(lock,
                                          [&] { return state->closing || !state->queue.empty(); });
                        if (state->queue.empty())
                            break;
                        record = std::move(state->queue.front());
                        state->queue.pop_front();
                        for (const auto& chunk : record.chunks)
                            state->bytes -= chunk.bytes.size();
                    }
                    state->sink->write(record);
                    ++state->status->written[std::size_t(record.stream)];
                }
            } catch (...) {
                fail(*state->status, RecordingFailure::Storage);
            }
            // Wait until producers have stopped before summarizing their counts, even on error.
            {
                std::unique_lock<std::mutex> lock(state->mutex);
                state->ready.wait(lock, [&] { return state->closing; });
            }
            try {
                state->sink->finish(*state->status);
            } catch (...) {
                fail(*state->status, RecordingFailure::Storage);
            }
            // Destroy files on this thread too: close/destruction may block on storage.
            state->sink.reset();
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->queue.clear();
                state->done = true;
                state->status->finished = true;
            }
            writer_busy = false;
            state->finished.notify_all();
        });
    } catch (...) {
        writer_busy = false;
        throw;
    }
}
Recording::~Recording() {
    close();
}
bool Recording::active() const {
    return state_->status->failure == RecordingFailure::None && !state_->status->finished;
}
bool Recording::submit(RecordingRecord record) {
    const auto stream = std::size_t(record.stream);
    if (stream >= 5)
        throw std::invalid_argument("invalid recording stream");
    ++state_->status->submitted[stream];
    auto reject = [&] {
        ++state_->status->rejected[stream];
        return false;
    };
    if (!active())
        return reject();
    std::size_t bytes = 0;
    for (const auto& chunk : record.chunks) {
        if (chunk.bytes.size() > state_->byte_limit - bytes) {
            fail(*state_->status, RecordingFailure::Backlog);
            return reject();
        }
        bytes += chunk.bytes.size();
    }
    // The consumer never holds this mutex during file operations.
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!active())
        return reject();
    if (state_->closing || state_->queue.size() >= state_->record_limit ||
        bytes > state_->byte_limit - state_->bytes) {
        fail(*state_->status, RecordingFailure::Backlog);
        return reject();
    }
    state_->bytes += bytes;
    state_->queue.push_back(std::move(record));
    state_->ready.notify_one();
    return true;
}
bool Recording::close(std::chrono::milliseconds timeout) {
    if (!worker_.joinable())
        return state_->status->complete();
    std::unique_lock<std::mutex> lock(state_->mutex);
    state_->closing = true;
    state_->ready.notify_one();
    if (!state_->finished.wait_for(lock, timeout, [&] { return state_->done; })) {
        fail(*state_->status, RecordingFailure::ShutdownTimeout);
        lock.unlock();
        worker_.detach();
        return false;
    }
    lock.unlock();
    worker_.join();
    return state_->status->complete();
}
std::unique_ptr<RecordingSink> recording_files(std::vector<RecordingFile> files,
                                               std::filesystem::path status) {
    return std::make_unique<FileSink>(std::move(files), std::move(status));
}
RecordingChunk recording_text(std::size_t file, const std::string& text) {
    return {file, {text.begin(), text.end()}};
}
} // namespace vio
