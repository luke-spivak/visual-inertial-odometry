#pragma once

#include "flight_session.h"
#include "frames.h"
#include "serial_port.h"

#include <ardupilotmega/mavlink.h>

#include <array>
#include <memory>
#include <optional>

namespace vio {

struct MavlinkAddress {
    std::uint8_t system;
    std::uint8_t component;
};

struct MavlinkLinkConfig {
    MavlinkAddress source{1, 197};
    MavlinkAddress controller{1, 1};
    FlightSessionConfig session;
    bool send_velocity{true};
};

enum class PublishResult {
    Queued,
    Held,
    InvalidEstimate,
    OutOfOrder,
    Busy
};

struct MavlinkLinkStats {
    std::uint64_t measurements_sent{0};
    std::uint64_t measurements_dropped{0};
    std::uint64_t invalid_frames{0};
};

/// One owner for serial I/O, MAVLink parser/sequence state, and publication policy.
/// Call start_session(), then poll()/publish() on one communication loop. Hand in
/// copied estimates; this class never accesses OpenVINS or reads estimate files.
class MavlinkLink {
public:
    MavlinkLink(std::unique_ptr<ByteStream> stream, MavlinkLinkConfig config,
                FrameTransform transform, std::uint8_t initial_reset_counter = 0);
    void start_session(SessionGeneration generation);
    /// Stop publishing during capture cleanup; retain FC monitoring for a disarmed restart.
    void end_session();

    /// Bounded work: at most 4096 input bytes and one nonblocking write per call.
    /// I/O failure latches Failed and releases the stream; inspect state() to stop.
    void poll(MonotonicTime now);
    bool controller_disarmed(MonotonicTime now) const {
        return session_.controller_disarmed(now);
    }

    /// Queue one pose/velocity pair. Busy applies backpressure without a backlog.
    /// The caller should offer its newest snapshot again after poll() makes space.
    PublishResult publish(const EstimatorEstimate& estimate, MonotonicTime now);
    FlightSessionState state() const {
        return session_.state();
    }
    const std::string& failure_reason() const {
        return failure_reason_;
    }
    const MavlinkLinkStats& stats() const {
        return stats_;
    }

private:
    enum class PendingKind {
        None,
        Measurement,
        Alignment,
        Heartbeat
    };
    void receive(const mavlink_message_t& message, MonotonicTime now);
    void append(const mavlink_message_t& message);
    void queue_alignment(MonotonicTime now);
    void queue_heartbeat(MonotonicTime now);
    void clear_pending();
    void fail(std::string reason);

    std::unique_ptr<ByteStream> stream_;
    MavlinkLinkConfig config_;
    FrameTransform transform_;
    FlightSession session_;
    bool started_{false};
    mavlink_message_t parser_message_{};
    mavlink_status_t parser_status_{};
    mavlink_status_t transmit_status_{};
    std::array<std::uint8_t, 2 * MAVLINK_MAX_PACKET_LEN> pending_{};
    std::size_t pending_size_{0};
    std::size_t pending_offset_{0};
    PendingKind pending_kind_{PendingKind::None};
    MonotonicTime pending_timestamp_{0};
    MonotonicTime pending_deadline_{0};
    MonotonicTime next_heartbeat_{0};
    std::optional<MonotonicTime> last_estimate_time_;
    MavlinkLinkStats stats_;
    std::string failure_reason_;
};

} // namespace vio
