#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>

#include "dc/flight/ports.hpp"
#include "dc/mqtt/mission_codec.hpp"

namespace dc::mqtt {

enum class IngestError {
    none,
    closed,
    topic_mismatch,
    invalid_qos,
    retained_message,
    empty_payload,
    payload_too_large,
    decode_failed,
    queue_full,
};

struct IngestResult {
    IngestError error{IngestError::none};
    std::string detail;
    std::string command_id;

    [[nodiscard]] bool ok() const noexcept
    {
        return error == IngestError::none;
    }
};

struct CommandSourceLimits {
    std::size_t max_payload_bytes{};
    std::size_t queue_capacity{};
    // STOP/RTL/E-stop have a separately bounded reserve so a full mission
    // queue cannot prevent preemption.
    std::size_t safety_queue_capacity{};
    MissionCodecLimits codec;
};

[[nodiscard]] bool valid_vehicle_id(std::string_view vehicle_id) noexcept;
[[nodiscard]] std::string command_topic_for(std::string_view vehicle_id);
[[nodiscard]] std::string command_ack_topic_for(std::string_view vehicle_id);

// Broker callbacks call ingest() and return. Decode happens before the short
// queue lock; no PX4 or status-network call exists on this path.
class MqttCommandSource final : public flight::CommandSource {
public:
    MqttCommandSource(std::string vehicle_id, CommandSourceLimits limits);

    [[nodiscard]] IngestResult ingest(
        std::string_view topic,
        std::string_view payload,
        int qos,
        bool retained) noexcept;

    bool wait_next(
        flight::RemoteCommand& command,
        std::chrono::milliseconds timeout) override;
    void close() noexcept override;

    [[nodiscard]] const std::string& vehicle_id() const noexcept;
    [[nodiscard]] const std::string& command_topic() const noexcept;
    [[nodiscard]] std::size_t queued() const noexcept;
    [[nodiscard]] std::size_t queued_safety() const noexcept;

private:
    std::string vehicle_id_;
    std::string command_topic_;
    CommandSourceLimits limits_;

    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<flight::RemoteCommand> safety_queue_;
    std::deque<flight::RemoteCommand> normal_queue_;
    bool closed_{};
};

}  // namespace dc::mqtt
