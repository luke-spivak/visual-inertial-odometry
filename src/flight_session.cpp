#include "flight_session.h"

#include <stdexcept>

namespace vio {

/// Store the timing/retry limits and the reset byte supplied by the application.
/// Reject settings that would disable time limits or exceed MAVLink's retry byte.
FlightSession::FlightSession(FlightSessionConfig config, std::uint8_t initial_reset_counter)
    : config_(config), reset_counter_(initial_reset_counter) {
    if (config.heartbeat_timeout.count() <= 0 || config.alignment_timeout.count() <= 0 ||
        config.progress_timeout.count() <= 0 || config.max_estimate_age.count() <= 0 ||
        config.alignment_attempts == 0 || config.alignment_attempts > 256) {
        throw std::invalid_argument("session requires positive timeouts and 1..256 attempts");
    }
}

/// Begin a new estimator coordinate frame and revoke the previous alignment.
/// Advance the reset byte after the first session; reject an unresolved old command.
void FlightSession::start(SessionGeneration generation) {
    if (state_ == FlightSessionState::Failed ||
        (attempts_ != 0 && state_ != FlightSessionState::Aligned))
        throw std::logic_error("recreate the link after an unresolved alignment or failure");
    if (generation_ && generation.value <= generation_->value)
        throw std::invalid_argument("estimator generations must increase");
    if (generation_)
        ++reset_counter_; // MAVLink's byte wraps; the internal generation does not.
    generation_ = generation;
    measurement_time_.reset();
    attempts_ = 0;
    in_progress_ = false;
    next_request_time_ = MonotonicTime{0};
    state_ = FlightSessionState::WaitingForController;
}

/// Check whether the last controller heartbeat is recent enough to trust.
bool FlightSession::controller_fresh(MonotonicTime now) const {
    return heartbeat_time_ && now >= *heartbeat_time_ &&
           now - *heartbeat_time_ < config_.heartbeat_timeout;
}

/// Record whether the controller is armed and when we last heard from it.
/// Arming before an outstanding alignment is acknowledged stops this session.
void FlightSession::heartbeat(bool armed, MonotonicTime now) {
    // Expire the old session before refreshing it, even if the loop was delayed.
    tick(now);
    heartbeat_time_ = now;
    armed_ = armed;
    if (state_ == FlightSessionState::AwaitingAlignment && armed)
        fail();
    tick(now);
}

/// Update the state as time passes, even when no new messages arrive.
/// Revoke alignment on heartbeat loss and retry or fail expired alignment requests.
void FlightSession::tick(MonotonicTime now) {
    if (!generation_ || state_ == FlightSessionState::Failed)
        return;
    if (!controller_fresh(now)) {
        // We cannot know whether a timed-out FC restarted. Never retain alignment.
        if (state_ == FlightSessionState::AwaitingAlignment) {
            fail();
        } else {
            if (state_ == FlightSessionState::Aligned)
                attempts_ = 0;
            state_ = FlightSessionState::WaitingForController;
            measurement_time_.reset();
        }
        return;
    }
    if (state_ == FlightSessionState::WaitingForController)
        state_ = FlightSessionState::Priming;
    if (state_ == FlightSessionState::AwaitingAlignment &&
        now - request_time_ >=
            (in_progress_ ? config_.progress_timeout : config_.alignment_timeout)) {
        if (in_progress_ || attempts_ >= config_.alignment_attempts) {
            fail();
        } else {
            state_ = FlightSessionState::Priming;
            next_request_time_ = now;
        }
    }
}

/// Allow measurements with a fresh controller: only disarmed until aligned,
/// then armed or disarmed. A failed or unstarted session cannot publish.
bool FlightSession::can_publish(MonotonicTime now) const {
    return generation_ && controller_fresh(now) && state_ != FlightSessionState::Failed &&
           (state_ == FlightSessionState::Aligned || !armed_);
}

/// Check whether we may request alignment now: disarmed, a recent measurement
/// already sent, retry delay elapsed, and attempts still available.
bool FlightSession::alignment_due(MonotonicTime now) const {
    return state_ == FlightSessionState::Priming && can_publish(now) && !armed_ &&
           attempts_ < config_.alignment_attempts && now >= next_request_time_ &&
           measurement_time_ && now >= *measurement_time_ &&
           now - *measurement_time_ <= config_.max_estimate_age;
}

/// Remember the sensor timestamp of the last complete measurement written.
/// This is evidence for priming, not confirmation that the controller received it.
void FlightSession::measurement_sent(MonotonicTime timestamp) {
    measurement_time_ = timestamp;
}

/// Start the acknowledgment timer after the whole alignment command is written.
/// Count this attempt only after transmission, not when the command is queued.
void FlightSession::alignment_sent(MonotonicTime now) {
    if (!alignment_due(now))
        throw std::logic_error("alignment sent without a fresh disarmed measurement");
    ++attempts_;
    request_time_ = now;
    in_progress_ = false;
    state_ = FlightSessionState::AwaitingAlignment;
}

/// Apply the controller's reply to an outstanding alignment request.
/// Acceptance permits armed publication; progress waits, busy retries, and rejection stops.
void FlightSession::acknowledge(AlignmentResult result, MonotonicTime now) {
    tick(now);
    if (state_ != FlightSessionState::AwaitingAlignment || !controller_fresh(now) || armed_)
        return;
    switch (result) {
    case AlignmentResult::Accepted:
        state_ = FlightSessionState::Aligned;
        break;
    case AlignmentResult::InProgress:
        // Repeated progress reports must not extend the deadline indefinitely.
        in_progress_ = true;
        break;
    case AlignmentResult::Retry:
        if (attempts_ >= config_.alignment_attempts) {
            fail();
        } else {
            state_ = FlightSessionState::Priming;
            next_request_time_ = now + config_.alignment_timeout;
        }
        break;
    case AlignmentResult::Rejected:
        fail();
        break;
    }
}

/// Latch a failure so this session cannot publish or automatically recover.
void FlightSession::fail() {
    state_ = FlightSessionState::Failed;
    measurement_time_.reset();
}

} // namespace vio
