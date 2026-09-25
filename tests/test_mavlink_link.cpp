#include "mavlink_link.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;
using namespace vio;

namespace {
void require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

struct Wire {
    std::deque<std::uint8_t> input;
    std::vector<std::uint8_t> output;
    std::size_t write_limit{4096};
    std::size_t read_limit{4096};
    bool disconnected{false};
};

class FakeStream final : public ByteStream {
public:
    explicit FakeStream(std::shared_ptr<Wire> wire) : wire_(std::move(wire)) {}
    std::size_t read(std::uint8_t* data, std::size_t capacity) override {
        if (wire_->disconnected)
            throw std::runtime_error("injected disconnect");
        const auto count = std::min({capacity, wire_->input.size(), wire_->read_limit});
        for (std::size_t i = 0; i < count; ++i) {
            data[i] = wire_->input.front();
            wire_->input.pop_front();
        }
        return count;
    }
    std::size_t write(const std::uint8_t* data, std::size_t size) override {
        const auto count = std::min(size, wire_->write_limit);
        wire_->output.insert(wire_->output.end(), data, data + count);
        return count;
    }

private:
    std::shared_ptr<Wire> wire_;
};

struct Fixture {
    std::shared_ptr<Wire> wire{std::make_shared<Wire>()};
    MavlinkLink link;
    MonotonicTime now{10s};
    mavlink_status_t fc_tx{};
    mavlink_status_t fc_rx{};
    mavlink_message_t fc_parser{};

    explicit Fixture(MavlinkLinkConfig config = {}, std::uint8_t counter = 0)
        : link(std::make_unique<FakeStream>(wire), config,
               FrameTransform(Eigen::Matrix3d::Identity(), {0, false}), counter) {
        link.start_session({1});
    }

    void inject(const mavlink_message_t& message, bool corrupt = false) {
        std::uint8_t data[MAVLINK_MAX_PACKET_LEN];
        auto size = mavlink_msg_to_send_buffer(data, &message);
        if (corrupt)
            data[size - 1] ^= 1;
        wire->input.insert(wire->input.end(), data, data + size);
    }
    void heartbeat(bool armed = false, std::uint8_t sys = 1, std::uint8_t comp = 1,
                   std::uint8_t type = MAV_TYPE_QUADROTOR,
                   std::uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA) {
        mavlink_message_t message{};
        mavlink_msg_heartbeat_pack_status(sys, comp, &fc_tx, &message, type, autopilot,
                                          armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, 0,
                                          MAV_STATE_ACTIVE);
        inject(message);
    }
    void ack(std::uint8_t result = MAV_RESULT_ACCEPTED, std::uint8_t sys = 1, std::uint8_t comp = 1,
             std::uint8_t target_sys = 1, std::uint8_t target_comp = 197,
             std::uint16_t command = MAV_CMD_DO_AUX_FUNCTION) {
        mavlink_message_t message{};
        mavlink_msg_command_ack_pack_status(sys, comp, &fc_tx, &message, command, result, 0, 0,
                                            target_sys, target_comp);
        inject(message);
    }
    std::vector<mavlink_message_t> take() {
        std::vector<mavlink_message_t> messages;
        for (auto byte : wire->output) {
            mavlink_message_t message{};
            mavlink_status_t status{};
            auto framing = mavlink_frame_char_buffer(&fc_parser, &fc_rx, byte, &message, &status);
            require(framing != MAVLINK_FRAMING_BAD_CRC, "bad output CRC");
            if (framing == MAVLINK_FRAMING_OK)
                messages.push_back(message);
        }
        wire->output.clear();
        return messages;
    }
    EstimatorEstimate estimate() const {
        EstimatorEstimate result;
        result.generation = {1};
        result.status = EstimatorStatus::Ready;
        result.timestamp = now;
        result.position_world_m = {1, 2, 3};
        result.velocity_world_mps = {4, 5, 6};
        return result;
    }
    void connect() {
        heartbeat();
        link.poll(now);
        take();
    }
    void request_alignment() {
        connect();
        require(link.publish(estimate(), now) == PublishResult::Queued, "prime estimate");
        link.poll(now);
        take();
        link.poll(now);
        auto messages = take();
        require(messages.size() == 1 && messages[0].msgid == MAVLINK_MSG_ID_COMMAND_LONG,
                "one alignment command after priming");
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&messages[0], &command);
        require(command.target_system == 1 && command.target_component == 1 &&
                    command.command == MAV_CMD_DO_AUX_FUNCTION && command.param1 == 80 &&
                    command.param2 == 2 && command.confirmation == 0,
                "alignment wire parameters");
    }
    void align() {
        request_alignment();
        ack();
        link.poll(now);
        require(link.state() == FlightSessionState::Aligned, "alignment accepted");
        take();
    }
};

void identity_and_wire_contract() {
    Fixture f({}, 255);
    require(f.link.publish(f.estimate(), f.now) == PublishResult::Held, "no FC holds poses");
    f.heartbeat(false, 2);
    f.heartbeat(false, 1, 42);
    f.heartbeat(false, 1, 1, MAV_TYPE_GCS);
    f.heartbeat(false, 1, 1, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4);
    f.link.poll(f.now);
    require(f.link.state() == FlightSessionState::WaitingForController,
            "reject foreign heartbeats");
    f.take();
    f.heartbeat(true);
    f.link.poll(f.now);
    require(f.link.publish(f.estimate(), f.now) == PublishResult::Held, "armed new session holds");
    f.heartbeat();
    f.ack(); // An unsolicited ACK must not align anything.
    f.link.poll(f.now);
    require(f.link.state() == FlightSessionState::Priming, "unsolicited ACK ignored");
    auto estimate = f.estimate();
    estimate.timestamp += 123ns;
    f.now = estimate.timestamp;
    require(f.link.publish(estimate, f.now) == PublishResult::Queued, "queue valid estimate");
    f.link.poll(f.now);
    auto messages = f.take();
    require(messages.size() == 2, "pose and velocity pair");
    require(messages[0].magic == MAVLINK_STX && messages[0].sysid == 1 &&
                messages[0].compid == 197 && messages[1].seq == std::uint8_t(messages[0].seq + 1),
            "MAVLink 2 identity and sequence");
    mavlink_vision_position_estimate_t pose{};
    mavlink_vision_speed_estimate_t speed{};
    mavlink_msg_vision_position_estimate_decode(&messages[0], &pose);
    mavlink_msg_vision_speed_estimate_decode(&messages[1], &speed);
    require(pose.usec == 10000000 && speed.usec == pose.usec, "nanoseconds to microseconds");
    require(pose.x == 2 && pose.y == 1 && pose.z == -3 && speed.x == 5 && speed.y == 4 &&
                speed.z == -6,
            "NED wire coordinates");
    require(pose.reset_counter == 255 && speed.reset_counter == 255 &&
                std::isnan(pose.covariance[0]) && std::isnan(speed.covariance[0]),
            "reset extension and unknown covariance");
    f.link.poll(f.now);
    f.take();
    f.ack(MAV_RESULT_ACCEPTED, 2);
    f.ack(MAV_RESULT_ACCEPTED, 1, 42);
    f.ack(MAV_RESULT_ACCEPTED, 1, 1, 2);
    f.ack(MAV_RESULT_ACCEPTED, 1, 1, 1, 42);
    f.ack(MAV_RESULT_ACCEPTED, 1, 1, 1, 197, MAV_CMD_COMPONENT_ARM_DISARM);
    f.link.poll(f.now);
    require(f.link.state() == FlightSessionState::AwaitingAlignment, "foreign ACKs ignored");
    f.ack(MAV_RESULT_IN_PROGRESS);
    f.link.poll(f.now);
    require(f.link.state() == FlightSessionState::AwaitingAlignment, "progress is not acceptance");
    f.ack();
    f.heartbeat(true);
    f.link.poll(f.now);
    require(f.link.state() == FlightSessionState::Aligned, "aligned session may stay armed");
    f.now += 10ms;
    require(f.link.publish(f.estimate(), f.now) == PublishResult::Queued,
            "aligned armed publication");
    f.link.poll(f.now);
    f.take();
    f.link.start_session({2});
    require(f.link.publish(f.estimate(), f.now) == PublishResult::Held,
            "restart while armed holds");
    f.heartbeat();
    f.link.poll(f.now);
    auto next = f.estimate();
    require(f.link.publish(next, f.now) == PublishResult::InvalidEstimate,
            "old generation rejected");
    next.generation = {2};
    require(f.link.publish(next, f.now) == PublishResult::Queued,
            "new generation accepted disarmed");
    f.link.poll(f.now);
    messages = f.take();
    mavlink_msg_vision_position_estimate_decode(&messages[0], &pose);
    require(pose.reset_counter == 0, "reset byte wraps on explicit generation only");
}

void invalid_estimates_and_backpressure() {
    Fixture f;
    f.connect();
    auto e = f.estimate();
    e.timestamp = f.now - 501ms;
    require(f.link.publish(e, f.now) == PublishResult::InvalidEstimate, "stale estimate rejected");
    e = f.estimate();
    e.timestamp += 1ns;
    require(f.link.publish(e, f.now) == PublishResult::InvalidEstimate, "future estimate rejected");
    e = f.estimate();
    e.status = EstimatorStatus::Initializing;
    require(f.link.publish(e, f.now) == PublishResult::InvalidEstimate, "uninitialized rejected");
    e = f.estimate();
    e.position_world_m.x() = std::numeric_limits<double>::max();
    require(f.link.publish(e, f.now) == PublishResult::InvalidEstimate, "float overflow rejected");
    e = f.estimate();
    e.velocity_world_mps.y() = std::numeric_limits<double>::quiet_NaN();
    require(f.link.publish(e, f.now) == PublishResult::InvalidEstimate, "NaN rejected");
    e = f.estimate();
    e.world_from_imu.coeffs().setZero();
    require(f.link.publish(e, f.now) == PublishResult::InvalidEstimate, "zero quaternion rejected");
    e = f.estimate();
    require(f.link.publish(e, f.now) == PublishResult::Queued, "valid estimate queued");
    require(f.link.publish(e, f.now) == PublishResult::OutOfOrder, "duplicate rejected");
    e.timestamp -= 1ns;
    require(f.link.publish(e, f.now) == PublishResult::OutOfOrder, "regression rejected");
    e.timestamp = f.now + 1ms;
    require(f.link.publish(e, e.timestamp) == PublishResult::Busy, "one pair capacity");
    f.wire->write_limit = 0;
    f.now += 501ms;
    f.link.poll(f.now);
    require(f.link.stats().measurements_dropped == 1 && f.wire->output.empty(),
            "unsent stale pair dropped");
    f.wire->write_limit = 4096;
    f.now += 1ms;
    require(f.link.publish(f.estimate(), f.now) == PublishResult::Queued,
            "recovery after unsent drop");
}

void fragmentation_and_faults() {
    Fixture f;
    f.wire->read_limit = 1;
    f.heartbeat();
    while (!f.wire->input.empty())
        f.link.poll(f.now);
    require(f.link.state() == FlightSessionState::Priming, "fragmented heartbeat parsed");
    f.take();
    f.wire->write_limit = 1;
    require(f.link.publish(f.estimate(), f.now) == PublishResult::Queued,
            "fragmented output queued");
    while (f.link.stats().measurements_sent == 0)
        f.link.poll(f.now);
    require(f.take().size() == 2, "partial writes preserve packet boundaries");

    Fixture stalled;
    stalled.connect();
    stalled.link.publish(stalled.estimate(), stalled.now);
    stalled.wire->write_limit = 1;
    stalled.link.poll(stalled.now);
    stalled.now += 501ms;
    stalled.link.poll(stalled.now);
    require(stalled.link.state() == FlightSessionState::Failed && stalled.wire->output.size() == 1,
            "expired partial transmission closes link");

    Fixture disconnected;
    disconnected.connect();
    disconnected.wire->disconnected = true;
    disconnected.link.poll(disconnected.now);
    require(disconnected.link.state() == FlightSessionState::Failed, "disconnect fails closed");

    Fixture armed;
    armed.request_alignment();
    armed.heartbeat(true);
    armed.ack();
    armed.link.poll(armed.now);
    require(armed.link.state() == FlightSessionState::Failed,
            "arming during alignment fails closed");

    Fixture corrupt;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(1, 1, &corrupt.fc_tx, &message, MAV_TYPE_QUADROTOR,
                                      MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_ACTIVE);
    corrupt.inject(message, true);
    corrupt.link.poll(corrupt.now);
    require(corrupt.link.stats().invalid_frames == 1 &&
                corrupt.link.state() == FlightSessionState::WaitingForController,
            "bad CRC rejected");
    corrupt.connect();
    require(corrupt.link.state() == FlightSessionState::Priming, "parser recovers after bad CRC");
}

void alignment_timeouts() {
    Fixture f;
    f.request_alignment();
    for (unsigned attempt = 1; attempt <= 3; ++attempt) {
        f.now += 2s;
        f.heartbeat();
        f.link.poll(f.now);
        f.take();
        if (attempt == 3)
            break;
        require(f.link.state() == FlightSessionState::Priming, "timeout requires fresh priming");
        require(f.link.publish(f.estimate(), f.now) == PublishResult::Queued,
                "retry fresh estimate");
        f.link.poll(f.now);
        f.take();
        f.link.poll(f.now);
        auto messages = f.take();
        mavlink_command_long_t command{};
        require(messages.size() == 1, "one retry command");
        mavlink_msg_command_long_decode(&messages[0], &command);
        require(command.confirmation == attempt, "retry confirmation increments");
    }
    require(f.link.state() == FlightSessionState::Failed, "retry count bounded");

    Fixture refused;
    refused.request_alignment();
    refused.ack(MAV_RESULT_DENIED);
    refused.link.poll(refused.now);
    require(refused.link.state() == FlightSessionState::Failed,
            "denied alignment stops publication");

    Fixture timeout;
    timeout.align();
    timeout.now += 3s;
    timeout.link.poll(timeout.now);
    require(timeout.link.publish(timeout.estimate(), timeout.now) == PublishResult::Held,
            "heartbeat timeout revokes publication");
    timeout.heartbeat(true);
    timeout.link.poll(timeout.now);
    require(timeout.link.publish(timeout.estimate(), timeout.now) == PublishResult::Held,
            "reconnected armed FC requires alignment again");

    Fixture pending;
    pending.request_alignment();
    bool rejected = false;
    try {
        pending.link.start_session({2});
    } catch (const std::logic_error&) {
        rejected = true;
    }
    require(rejected, "cannot restart across an unresolved ACK");
}

void additional_policy_edges() {
    Fixture progress;
    progress.request_alignment();
    for (int second = 1; second <= 10; ++second) {
        progress.now += 1s;
        progress.heartbeat();
        progress.ack(MAV_RESULT_IN_PROGRESS);
        progress.link.poll(progress.now);
        progress.take();
        require(progress.link.state() == (second < 10 ? FlightSessionState::AwaitingAlignment
                                                      : FlightSessionState::Failed),
                "progress cannot extend the deadline forever");
    }

    Fixture busy;
    busy.request_alignment();
    busy.ack(MAV_RESULT_TEMPORARILY_REJECTED);
    busy.link.poll(busy.now);
    busy.take();
    require(busy.link.state() == FlightSessionState::Priming, "temporary rejection permits retry");
    busy.now += 1s;
    busy.heartbeat();
    busy.link.poll(busy.now);
    auto messages = busy.take();
    for (const auto& message : messages)
        require(message.msgid != MAVLINK_MSG_ID_COMMAND_LONG, "retry respects backoff");
    busy.now += 1s;
    busy.heartbeat();
    busy.link.poll(busy.now);
    busy.take();
    busy.link.publish(busy.estimate(), busy.now);
    busy.link.poll(busy.now);
    busy.take();
    busy.link.poll(busy.now);
    messages = busy.take();
    require(messages.size() == 1 && messages[0].msgid == MAVLINK_MSG_ID_COMMAND_LONG,
            "temporary rejection retries after fresh measurement");

    Fixture lost;
    lost.request_alignment();
    lost.now += 3s;
    lost.heartbeat();
    lost.ack();
    lost.link.poll(lost.now);
    require(lost.link.state() == FlightSessionState::Failed,
            "late heartbeat cannot hide link loss");

    Fixture queued;
    queued.connect();
    queued.link.publish(queued.estimate(), queued.now);
    queued.heartbeat(true);
    queued.link.poll(queued.now);
    require(queued.link.stats().measurements_dropped == 1 && queued.take().empty(),
            "arming cancels an unsent priming measurement");

    Fixture restart;
    restart.request_alignment();
    restart.now += 2s;
    restart.heartbeat();
    restart.link.poll(restart.now);
    bool rejected = false;
    try {
        restart.link.start_session({2});
    } catch (const std::logic_error&) {
        rejected = true;
    }
    require(rejected, "timed-out command cannot leak into a new generation");

    MavlinkLinkConfig config;
    config.send_velocity = false;
    Fixture pose_only(config);
    pose_only.connect();
    pose_only.link.publish(pose_only.estimate(), pose_only.now);
    pose_only.link.poll(pose_only.now);
    messages = pose_only.take();
    require(messages.size() == 1 && messages[0].msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE,
            "velocity publication can be disabled");

    Fixture flooded;
    flooded.connect();
    flooded.link.publish(flooded.estimate(), flooded.now);
    flooded.wire->input.insert(flooded.wire->input.end(), 4096, 0);
    flooded.heartbeat(true);
    flooded.link.poll(flooded.now);
    require(flooded.take().empty() && !flooded.wire->input.empty(), "receive work is bounded");
    flooded.link.poll(flooded.now);
    require(flooded.link.stats().measurements_dropped == 1, "queued arming beats transmission");
}

void recorded_replay(const char* path) {
    std::ifstream file(path);
    require(file.good(), "open recorded estimates");
    Fixture f;
    f.now = 13s;
    f.align();
    std::string line;
    unsigned count = 0;
    while (std::getline(file, line)) {
        if (line.empty() || line.front() == '#')
            continue;
        std::istringstream row(line);
        double seconds, x, y, z, w;
        auto estimate = f.estimate();
        row >> seconds >> x >> y >> z >> w;
        for (int i = 0; i < 3; ++i)
            row >> estimate.position_world_m[i];
        for (int i = 0; i < 3; ++i)
            row >> estimate.velocity_world_mps[i];
        require(!row.fail(), "parse recorded estimate");
        estimate.timestamp = MonotonicTime{static_cast<std::int64_t>(std::llround(seconds * 1e9))};
        estimate.world_from_imu = Eigen::Quaterniond(w, x, y, z);
        f.now = estimate.timestamp;
        f.heartbeat();
        f.link.poll(f.now);
        f.take();
        require(f.link.publish(estimate, f.now) == PublishResult::Queued,
                "queue recorded estimate");
        f.link.poll(f.now);
        auto packets = f.take();
        require(packets.size() == 2, "recorded estimate delivers pair");
        mavlink_vision_position_estimate_t pose{};
        mavlink_msg_vision_position_estimate_decode(&packets[0], &pose);
        require(pose.usec == static_cast<std::uint64_t>(estimate.timestamp.count() / 1000) &&
                    std::abs(pose.x - estimate.position_world_m.y()) < 1e-4 &&
                    std::abs(pose.y - estimate.position_world_m.x()) < 1e-4 &&
                    std::abs(pose.z + estimate.position_world_m.z()) < 1e-4,
                "recorded timestamp and position preserved on wire");
        ++count;
    }
    require(count > 7000, "full recorded flight replayed");
    std::cout << "Replayed " << count << " recorded estimates\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "recorded estimate path required");
        identity_and_wire_contract();
        invalid_estimates_and_backpressure();
        fragmentation_and_faults();
        alignment_timeouts();
        additional_policy_edges();
        recorded_replay(argv[1]);
        std::cout << "MAVLink and flight session checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
