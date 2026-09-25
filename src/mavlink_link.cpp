#include "mavlink_link.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vio {
namespace {
constexpr float visual_odometry_align = 80;
constexpr float switch_high = 2;

/// Check that each double can become a finite MAVLink float without overflowing.
bool fits_wire(const Eigen::Vector3d& vector) {
    return vector.allFinite() &&
           (vector.array().abs() <= static_cast<double>(std::numeric_limits<float>::max())).all();
}
} // namespace

/// Take ownership of the byte connection and store the mounting and controller IDs.
/// Each link keeps its own parser, packet sequence, and flight-session state.
MavlinkLink::MavlinkLink(std::unique_ptr<ByteStream> stream, MavlinkLinkConfig config,
                         FrameTransform transform, std::uint8_t initial_reset_counter)
    : stream_(std::move(stream)), config_(config), transform_(std::move(transform)),
      session_(config.session, initial_reset_counter) {
    if (!stream_ || config.source.system == 0 || config.source.component == 0 ||
        config.controller.system == 0 || config.controller.component == 0 ||
        (config.source.system == config.controller.system &&
         config.source.component == config.controller.component)) {
        throw std::invalid_argument("MAVLink requires a stream and distinct, nonzero addresses");
    }
}

/// Start a new estimator generation, discarding unsent data from the previous one.
/// A partially written packet cannot be abandoned safely, so that case closes the link.
void MavlinkLink::start_session(SessionGeneration generation) {
    if (pending_offset_ != 0) {
        fail("session restart during a partial transmission");
        throw std::logic_error("cannot replace a session during a partial transmission");
    }
    session_.start(generation);
    clear_pending();
    last_estimate_time_.reset();
    started_ = true;
}

/// Withdraw queued capture data before cleanup; a partial packet forces link recovery.
void MavlinkLink::end_session() {
    started_ = false;
    if (pending_offset_ != 0)
        fail("capture ended during partial transmission");
    else
        clear_pending();
}

/// Handle one decoded packet from the configured flight controller.
/// Feed its heartbeat or relevant alignment acknowledgment into the session policy.
void MavlinkLink::receive(const mavlink_message_t& message, MonotonicTime now) {
    if (message.sysid != config_.controller.system ||
        message.compid != config_.controller.component)
        return;
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (heartbeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA && heartbeat.type != MAV_TYPE_GCS)
            session_.heartbeat((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0, now);
    } else if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&message, &ack);
        // Zero addresses are permitted for MAVLink 1 ACKs and omitted extensions.
        if (ack.command != MAV_CMD_DO_AUX_FUNCTION ||
            (ack.target_system != 0 && ack.target_system != config_.source.system) ||
            (ack.target_component != 0 && ack.target_component != config_.source.component))
            return;
        AlignmentResult result = AlignmentResult::Rejected;
        if (ack.result == MAV_RESULT_ACCEPTED)
            result = AlignmentResult::Accepted;
        else if (ack.result == MAV_RESULT_IN_PROGRESS)
            result = AlignmentResult::InProgress;
        else if (ack.result == MAV_RESULT_TEMPORARILY_REJECTED)
            result = AlignmentResult::Retry;
        session_.acknowledge(result, now);
    }
}

/// Encode one MAVLink packet into the fixed outgoing buffer; do not send it yet.
void MavlinkLink::append(const mavlink_message_t& message) {
    // Only a pose/velocity pair or one control message occupies this fixed buffer.
    if (pending_size_ + MAVLINK_MAX_PACKET_LEN > pending_.size())
        throw std::logic_error("MAVLink transmit buffer capacity exceeded");
    pending_size_ += mavlink_msg_to_send_buffer(pending_.data() + pending_size_, &message);
}

/// Validate a snapshot, convert its coordinates, and queue pose/velocity packets.
/// Return why it was held or rejected if it cannot be queued. poll() does the I/O;
/// the application communication loop supplies copied estimator snapshots.
PublishResult MavlinkLink::publish(const EstimatorEstimate& estimate, MonotonicTime now) {
    session_.tick(now);
    if (!started_ || !stream_ || !session_.can_publish(now))
        return PublishResult::Held;
    if (validate_estimate(estimate, session_.generation(), now, session_.max_estimate_age()) !=
        EstimateRejection::None)
        return PublishResult::InvalidEstimate;
    if (last_estimate_time_ && estimate.timestamp <= *last_estimate_time_)
        return PublishResult::OutOfOrder;
    // Reserve the next poll for due control traffic instead of letting a fast
    // estimator continually refill the buffer and starve alignment/heartbeats.
    if (pending_kind_ != PendingKind::None || session_.alignment_due(now) || now >= next_heartbeat_)
        return PublishResult::Busy;
    const auto measurement = transform_.transform(estimate);
    if (!fits_wire(measurement.position_ned_m) || !fits_wire(measurement.velocity_ned_mps) ||
        !fits_wire(measurement.roll_pitch_yaw_rad))
        return PublishResult::InvalidEstimate;

    const auto usec = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(measurement.timestamp).count());
    const auto& p = measurement.position_ned_m;
    const auto& v = measurement.velocity_ned_mps;
    const auto& rpy = measurement.roll_pitch_yaw_rad;
    // MAVLink specifies NaN in the first covariance entry for unknown covariance.
    float pose_covariance[21]{};
    float velocity_covariance[9]{};
    pose_covariance[0] = velocity_covariance[0] = std::numeric_limits<float>::quiet_NaN();
    mavlink_message_t message{};
    mavlink_msg_vision_position_estimate_pack_status(
        config_.source.system, config_.source.component, &transmit_status_, &message, usec, p.x(),
        p.y(), p.z(), rpy.x(), rpy.y(), rpy.z(), pose_covariance, session_.reset_counter());
    append(message);
    if (config_.send_velocity) {
        mavlink_msg_vision_speed_estimate_pack_status(
            config_.source.system, config_.source.component, &transmit_status_, &message, usec,
            v.x(), v.y(), v.z(), velocity_covariance, session_.reset_counter());
        append(message);
    }
    pending_kind_ = PendingKind::Measurement;
    pending_timestamp_ = measurement.timestamp;
    pending_deadline_ = now + (session_.max_estimate_age() - (now - measurement.timestamp));
    last_estimate_time_ = estimate.timestamp;
    return PublishResult::Queued;
}

/// Queue ArduPilot's visual-odometry alignment command for the configured controller.
/// The acknowledgment timer starts later, after poll() finishes writing the packet.
void MavlinkLink::queue_alignment(MonotonicTime now) {
    mavlink_message_t message{};
    mavlink_msg_command_long_pack_status(config_.source.system, config_.source.component,
                                         &transmit_status_, &message, config_.controller.system,
                                         config_.controller.component, MAV_CMD_DO_AUX_FUNCTION,
                                         static_cast<std::uint8_t>(session_.alignment_attempts()),
                                         visual_odometry_align, switch_high, 0, 0, 0, 0, 0);
    append(message);
    pending_kind_ = PendingKind::Alignment;
    pending_deadline_ = now + config_.session.alignment_timeout;
}

/// Queue our own "I am here" message and schedule the next one one second later.
void MavlinkLink::queue_heartbeat(MonotonicTime now) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(config_.source.system, config_.source.component,
                                      &transmit_status_, &message, MAV_TYPE_ONBOARD_CONTROLLER,
                                      MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    append(message);
    pending_kind_ = PendingKind::Heartbeat;
    pending_deadline_ = now + std::chrono::seconds(1);
    next_heartbeat_ = pending_deadline_;
}

/// Empty the application's outgoing buffer; this cannot retract bytes already written.
void MavlinkLink::clear_pending() {
    pending_kind_ = PendingKind::None;
    pending_size_ = pending_offset_ = 0;
}

/// Save the failure reason, stop publication, and release the byte connection.
void MavlinkLink::fail(std::string reason) {
    failure_reason_ = std::move(reason);
    session_.fail();
    clear_pending();
    stream_.reset();
}

/// Service the connection once: read replies, update policy, and advance a write.
/// Work is bounded so a slow or noisy connection cannot keep this call running forever.
/// Drop expired unsent data; close the link on I/O errors or an unsafe partial packet.
void MavlinkLink::poll(MonotonicTime now) {
    if (!stream_)
        return;
    try {
        session_.tick(now);
        std::array<std::uint8_t, 4096> input{};
        const auto count = stream_->read(input.data(), input.size());
        for (std::size_t i = 0; i < count; ++i) {
            mavlink_message_t message{};
            mavlink_status_t status{};
            const auto framing = mavlink_frame_char_buffer(&parser_message_, &parser_status_,
                                                           input[i], &message, &status);
            if (framing == MAVLINK_FRAMING_OK)
                receive(message, now);
            else if (framing == MAVLINK_FRAMING_BAD_CRC || framing == MAVLINK_FRAMING_BAD_SIGNATURE)
                ++stats_.invalid_frames;
        }
        if (session_.state() == FlightSessionState::Failed) {
            fail("alignment failed or controller state changed during alignment");
            return;
        }
        bool eligible = true;
        if (pending_kind_ == PendingKind::Measurement)
            eligible = session_.can_publish(now);
        else if (pending_kind_ == PendingKind::Alignment)
            eligible = session_.alignment_due(now);
        if (pending_kind_ != PendingKind::None && (!eligible || now > pending_deadline_)) {
            if (pending_kind_ == PendingKind::Measurement)
                ++stats_.measurements_dropped;
            // Never splice a new frame onto an old partial packet, nor finish a stale
            // measurement. Close the link and require deliberate recovery instead.
            if (pending_offset_ != 0) {
                fail("partial transmission expired or publication permission was revoked");
                return;
            }
            clear_pending();
        }
        // A full receive budget may hide a queued arming heartbeat. Drain on the
        // next poll before allowing any transmission; input floods cannot block us.
        if (count == input.size())
            return;
        if (pending_kind_ == PendingKind::None) {
            if (started_ && session_.alignment_due(now))
                queue_alignment(now);
            else if (now >= next_heartbeat_)
                queue_heartbeat(now);
        }
        if (pending_kind_ == PendingKind::None)
            return;
        pending_offset_ +=
            stream_->write(pending_.data() + pending_offset_, pending_size_ - pending_offset_);
        if (pending_offset_ != pending_size_)
            return;
        if (pending_kind_ == PendingKind::Measurement) {
            ++stats_.measurements_sent;
            session_.measurement_sent(pending_timestamp_);
        } else if (pending_kind_ == PendingKind::Alignment) {
            session_.alignment_sent(now);
        }
        clear_pending();
    } catch (const std::runtime_error& error) {
        fail(error.what());
    }
}

} // namespace vio
