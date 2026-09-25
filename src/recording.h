#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <vector>

namespace vio {
enum class RecordingStream {
    Accel,
    Gyro,
    Camera,
    Estimate,
    Features,
    Count
};
enum class RecordingFailure {
    None,
    Backlog,
    Storage,
    ShutdownTimeout,
    WriterBusy
};

/// Counts distinguish a complete recording from a prefix interrupted by lost records.
struct RecordingStatus {
    std::atomic<RecordingFailure> failure{RecordingFailure::None};
    std::atomic<bool> finished{false};
    std::array<std::atomic<std::uint64_t>, 5> submitted{}, written{}, rejected{};
    bool complete() const {
        return finished && failure == RecordingFailure::None;
    }
};
struct RecordingFile {
    std::filesystem::path path;
    std::string header, footer;
};
struct RecordingChunk {
    std::size_t file;
    std::vector<unsigned char> bytes;
};
struct RecordingRecord {
    RecordingStream stream;
    std::vector<RecordingChunk> chunks;
};

/// The worker alone calls the sink; tests can substitute slow or failing storage.
class RecordingSink {
public:
    virtual ~RecordingSink() = default;
    virtual void write(const RecordingRecord&) = 0;
    virtual void finish(RecordingStatus&) = 0;
};
std::unique_ptr<RecordingSink> recording_files(std::vector<RecordingFile>,
                                               std::filesystem::path status_path);
RecordingChunk recording_text(std::size_t file, const std::string& text);

/// Producers only enqueue owned bytes. The first lost record disables recording, not navigation.
class Recording {
public:
    Recording(std::unique_ptr<RecordingSink>, std::shared_ptr<RecordingStatus>,
              std::size_t byte_limit = 32 * 1024 * 1024, std::size_t record_limit = 1024);
    ~Recording();
    Recording(const Recording&) = delete;
    Recording& operator=(const Recording&) = delete;
    bool active() const;
    bool submit(RecordingRecord);
    /// Drain for at most this long. A stuck writer retains only its own heap-owned state.
    bool close(std::chrono::milliseconds timeout = std::chrono::seconds(2));

private:
    struct State;
    std::shared_ptr<State> state_;
    std::thread worker_;
};
} // namespace vio
