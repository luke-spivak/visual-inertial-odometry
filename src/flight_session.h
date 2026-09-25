#pragma once

#include "estimate.h"

#include <optional>

namespace vio {

enum class FlightSessionState {
    WaitingForController,
    Priming,
    AwaitingAlignment,
    Aligned,
    Failed
};

enum class AlignmentResult {
    Accepted,
    InProgress,
    Retry,
    Rejected
};

struct FlightSessionConfig {
    MonotonicTime heartbeat_timeout{std::chrono::seconds(3)};
    MonotonicTime alignment_timeout{std::chrono::seconds(2)};
    MonotonicTime progress_timeout{std::chrono::seconds(10)};
    MonotonicTime max_estimate_age{std::chrono::milliseconds(500)};
    unsigned alignment_attempts{3};
};

/// Publication policy for one estimator generation; all calls belong to one loop.
/// Time is injected so replay and timeout tests use the same policy as live flight.
class FlightSession {
public:
    explicit FlightSession(FlightSessionConfig config, std::uint8_t initial_reset_counter = 0);

    /// Start a distinct estimator frame. Reject reuse or restart during an outstanding
    /// command: COMMAND_ACK has no generation/transaction identifier to match it to.
    void start(SessionGeneration generation);
    void heartbeat(bool armed, MonotonicTime now);
    void tick(MonotonicTime now);
    void acknowledge(AlignmentResult result, MonotonicTime now);
    void fail();

    /// Before alignment, estimates may prime a fresh, disarmed controller only.
    bool can_publish(MonotonicTime now) const;
    /// Capture startup/restart requires a recently confirmed disarmed controller.
    bool controller_disarmed(MonotonicTime now) const {
        return controller_fresh(now) && !armed_;
    }
    bool alignment_due(MonotonicTime now) const;
    void measurement_sent(MonotonicTime timestamp);
    /// Called only after the entire command frame has been written to the transport.
    void alignment_sent(MonotonicTime now);

    FlightSessionState state() const {
        return state_;
    }
    SessionGeneration generation() const {
        return generation_.value();
    }
    std::uint8_t reset_counter() const {
        return reset_counter_;
    }
    unsigned alignment_attempts() const {
        return attempts_;
    }
    MonotonicTime max_estimate_age() const {
        return config_.max_estimate_age;
    }

private:
    bool controller_fresh(MonotonicTime now) const;
    FlightSessionConfig config_;
    FlightSessionState state_{FlightSessionState::WaitingForController};
    std::optional<SessionGeneration> generation_;
    std::optional<MonotonicTime> heartbeat_time_;
    std::optional<MonotonicTime> measurement_time_;
    MonotonicTime request_time_{0};
    MonotonicTime next_request_time_{0};
    bool armed_{true};
    bool in_progress_{false};
    unsigned attempts_{0};
    std::uint8_t reset_counter_;
};

} // namespace vio
