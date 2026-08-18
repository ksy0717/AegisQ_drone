#include "dc/mqtt/mqtt_command_source.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace dc::mqtt {

bool valid_vehicle_id(const std::string_view vehicle_id) noexcept
{
    if (vehicle_id.empty() || vehicle_id.size() > 128U) {
        return false;
    }
    return std::all_of(vehicle_id.begin(), vehicle_id.end(), [](const char character) {
        return (character >= 'a' && character <= 'z')
            || (character >= 'A' && character <= 'Z')
            || (character >= '0' && character <= '9')
            || character == '.' || character == '_' || character == '-';
    });
}

std::string command_topic_for(const std::string_view vehicle_id)
{
    if (!valid_vehicle_id(vehicle_id)) {
        throw std::invalid_argument("invalid MQTT vehicle id");
    }
    return "aegisq/cmd/" + std::string{vehicle_id};
}

std::string command_ack_topic_for(const std::string_view vehicle_id)
{
    if (!valid_vehicle_id(vehicle_id)) {
        throw std::invalid_argument("invalid MQTT vehicle id");
    }
    return "aegisq/cmd-ack/" + std::string{vehicle_id};
}

MqttCommandSource::MqttCommandSource(
    std::string vehicle_id, CommandSourceLimits limits)
    : vehicle_id_(std::move(vehicle_id)),
      command_topic_(command_topic_for(vehicle_id_)),
      limits_(limits)
{
    if (limits_.max_payload_bytes == 0U
        || limits_.queue_capacity == 0U
        || limits_.safety_queue_capacity == 0U) {
        throw std::invalid_argument("MQTT source limits must be configured");
    }
    if (!flight::valid_route_limits(limits_.codec.route)
        || limits_.codec.max_issued_by_bytes == 0U
        || limits_.codec.max_json_depth < 3U) {
        throw std::invalid_argument("MQTT codec limits must be configured");
    }
}

IngestResult MqttCommandSource::ingest(
    const std::string_view topic,
    const std::string_view payload,
    const int qos,
    const bool retained) noexcept
{
    try {
        if (topic != command_topic_) {
            return {IngestError::topic_mismatch, "topic_mismatch", {}};
        }
        if (qos != 1) {
            return {IngestError::invalid_qos, "qos_must_be_1", {}};
        }
        if (retained) {
            return {IngestError::retained_message, "retained_command_rejected", {}};
        }
        if (payload.empty()) {
            return {IngestError::empty_payload, "empty_payload", {}};
        }
        if (payload.size() > limits_.max_payload_bytes) {
            return {IngestError::payload_too_large, "payload_too_large", {}};
        }
        {
            std::lock_guard<std::mutex> lock{mutex_};
            if (closed_) {
                return {IngestError::closed, "source_closed", {}};
            }
        }

        DecodeResult decoded = decode_command(payload, vehicle_id_, limits_.codec);
        if (!decoded.ok()) {
            return {
                IngestError::decode_failed,
                std::move(decoded.error),
                std::move(decoded.command_id),
            };
        }
        std::string command_id = decoded.command->command_id;
        const bool safety = flight::is_safety_command(decoded.command->type);

        {
            std::lock_guard<std::mutex> lock{mutex_};
            if (closed_) {
                return {IngestError::closed, "source_closed", std::move(command_id)};
            }
            std::deque<flight::RemoteCommand>& queue =
                safety ? safety_queue_ : normal_queue_;
            const std::size_t capacity =
                safety ? limits_.safety_queue_capacity : limits_.queue_capacity;
            if (queue.size() >= capacity) {
                return {IngestError::queue_full, "command_queue_full", std::move(command_id)};
            }
            queue.push_back(std::move(*decoded.command));
        }
        ready_.notify_one();
        return {IngestError::none, {}, std::move(command_id)};
    } catch (const std::exception& error) {
        return {IngestError::decode_failed, error.what(), {}};
    } catch (...) {
        return {IngestError::decode_failed, "unknown_ingest_error", {}};
    }
}

bool MqttCommandSource::wait_next(
    flight::RemoteCommand& command,
    const std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock{mutex_};
    ready_.wait_for(lock, timeout, [this] {
        return closed_ || !safety_queue_.empty() || !normal_queue_.empty();
    });
    if (safety_queue_.empty() && normal_queue_.empty()) {
        return false;
    }
    std::deque<flight::RemoteCommand>& queue =
        safety_queue_.empty() ? normal_queue_ : safety_queue_;
    command = std::move(queue.front());
    queue.pop_front();
    return true;
}

void MqttCommandSource::close() noexcept
{
    {
        std::lock_guard<std::mutex> lock{mutex_};
        closed_ = true;
        safety_queue_.clear();
        normal_queue_.clear();
    }
    ready_.notify_all();
}

const std::string& MqttCommandSource::vehicle_id() const noexcept
{
    return vehicle_id_;
}

const std::string& MqttCommandSource::command_topic() const noexcept
{
    return command_topic_;
}

std::size_t MqttCommandSource::queued() const noexcept
{
    std::lock_guard<std::mutex> lock{mutex_};
    return safety_queue_.size() + normal_queue_.size();
}

std::size_t MqttCommandSource::queued_safety() const noexcept
{
    std::lock_guard<std::mutex> lock{mutex_};
    return safety_queue_.size();
}

}  // namespace dc::mqtt
