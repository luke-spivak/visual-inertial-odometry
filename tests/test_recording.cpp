#include "iio_fixture.h"
#include "openvins_runner/feature_snapshot.h"
#include "recording.h"
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
using namespace std::chrono_literals;
using namespace vio;
namespace {
struct Control {
    std::mutex mutex;
    std::condition_variable cv;
    bool entered{false}, release{false}, fail{false}, finished{false};
    unsigned records{0};
};
class SlowSink final : public RecordingSink {
public:
    explicit SlowSink(std::shared_ptr<Control> control) : c(std::move(control)) {}
    void write(const RecordingRecord&) override {
        std::unique_lock<std::mutex> lock(c->mutex);
        c->entered = true;
        c->cv.notify_all();
        c->cv.wait(lock, [&] { return c->release; });
        if (c->fail)
            throw std::runtime_error("disk full");
        ++c->records;
    }
    void finish(RecordingStatus&) override {
        std::lock_guard<std::mutex> lock(c->mutex);
        c->finished = true;
    }
    std::shared_ptr<Control> c;
};
void entered(Control& c) {
    std::unique_lock<std::mutex> lock(c.mutex);
    require(c.cv.wait_for(lock, 2s, [&] { return c.entered; }), "writer reached slow storage");
}
void release(Control& c) {
    std::lock_guard<std::mutex> lock(c.mutex);
    c.release = true;
    c.cv.notify_all();
}
RecordingRecord sample() {
    return {RecordingStream::Accel, {recording_text(0, "01234567")}};
}
void slow_storage() {
    auto control = std::make_shared<Control>();
    auto status = std::make_shared<RecordingStatus>();
    Recording recording(std::make_unique<SlowSink>(control), status, 16, 2);
    require(recording.submit(sample()), "first record queued");
    entered(*control);
    const auto start = std::chrono::steady_clock::now();
    require(recording.submit(sample()) && recording.submit(sample()), "bounded pending records");
    require(!recording.submit(sample()), "overflow rejects newest record");
    require(!recording.active(), "overflow disables recording");
    require(std::chrono::steady_clock::now() - start < 100ms, "producer does not wait on storage");
    release(*control);
    require(!recording.close(), "gapped recording cannot complete");
    require(status->written[0] == 3 && status->rejected[0] == 1, "exact overflow accounting");
}
void failed_storage() {
    auto control = std::make_shared<Control>();
    control->fail = true;
    auto status = std::make_shared<RecordingStatus>();
    Recording recording(std::make_unique<SlowSink>(control), status);
    recording.submit(sample());
    entered(*control);
    release(*control);
    require(!recording.close(), "failed write cannot complete");
    require(status->failure == RecordingFailure::Storage && status->written[0] == 0,
            "storage failure exposed");
}
void interrupted_shutdown() {
    auto control = std::make_shared<Control>();
    auto status = std::make_shared<RecordingStatus>();
    {
        Recording recording(std::make_unique<SlowSink>(control), status);
        recording.submit(sample());
        entered(*control);
        const auto start = std::chrono::steady_clock::now();
        require(!recording.close(20ms), "stuck writer times out");
        require(std::chrono::steady_clock::now() - start < 200ms, "bounded shutdown");
    } // Detached writer must not refer to this destroyed owner.
    auto second_status = std::make_shared<RecordingStatus>();
    {
        Recording second(std::make_unique<SlowSink>(std::make_shared<Control>()), second_status);
        require(!second.active() && second_status->failure == RecordingFailure::WriterBusy,
                "restart cannot accumulate stuck writers");
    }
    release(*control);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!status->finished && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    require(status->finished && !status->complete(), "late finish never reverses timeout");
    // Wait for the detached worker's final notification/slot release before the next test.
    std::this_thread::sleep_for(10ms);
}
void feature_snapshot() {
    auto text = feature_record({1234567890123456789LL, 42, 1280, 800, false, {{7, 100, 110}}});
    auto json = nlohmann::json::parse(text);
    require(json["timestamp_ns"] == 1234567890123456789LL && json["frame_index"] == 42 &&
                json["features"][0][0] == 7 && json["features"][0][1] == 100,
            "feature schema preserved");
}
void files() {
    Fixture fixture;
    auto status = std::make_shared<RecordingStatus>();
    auto report = fixture.root / "status.json";
    Recording recording(
        recording_files({{fixture.root / "raw", "", ""}, {fixture.root / "metadata", "[", "]"}},
                        report),
        status);
    recording.submit(
        {RecordingStream::Camera, {recording_text(0, "pixels"), recording_text(1, "17")}});
    require(recording.close(), "successful recording drained and closed");
    require(get(fixture.root / "raw") == "pixels" && get(fixture.root / "metadata") == "[17]",
            "paired frame and metadata written");
    nlohmann::json metadata;
    std::ifstream(report) >> metadata;
    require(metadata["writer_drained"] == true && metadata["streams"]["camera"]["written"] == 1,
            "checked recording report");
#ifdef __linux__
    status = std::make_shared<RecordingStatus>();
    Recording full(recording_files({{"/dev/full", "", ""}}, fixture.root / "full.json"), status);
    full.submit(sample());
    require(!full.close() && status->failure == RecordingFailure::Storage,
            "actual ENOSPC detected");
    std::ifstream(fixture.root / "full.json") >> metadata;
    require(metadata["reason"] == "storage_error" && metadata["streams"]["accel"]["written"] == 0,
            "failed storage leaves an explicit report when status storage still works");
#endif
}
} // namespace
int main() {
    try {
        feature_snapshot();
        slow_storage();
        failed_storage();
        interrupted_shutdown();
        files();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
