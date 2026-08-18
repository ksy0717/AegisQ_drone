#include "dc/mqtt/mqtt_command_source.hpp"
#include "fixtures.hpp"
#include "test_support.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace {

using dc::flight::RemoteCommand;
using dc::flight::RouteLimits;
using dc::mqtt::CommandSourceLimits;
using dc::mqtt::IngestError;
using dc::mqtt::MissionCodecLimits;
using dc::mqtt::MqttCommandSource;

constexpr CommandSourceLimits kLimits{
    16U * 1024U,
    2U,
    3U,
    MissionCodecLimits{
        RouteLimits{20U, 2.0, 120.0, 60U, 180U, 0.5, 8.0},
        128U,
        4U,
    },
};

void test_topic_derivation_and_fifo()
{
    MqttCommandSource source{"DRONE-01", kLimits};
    test_support::check(
        source.command_topic() == "aegisq/cmd/DRONE-01",
        "command topic is derived from vehicle id");
    test_support::check(
        dc::mqtt::command_ack_topic_for("DRONE-01")
            == "aegisq/cmd-ack/DRONE-01",
        "ACK topic is derived from vehicle id");

    const auto first = source.ingest(
        source.command_topic(),
        mqtt_fixtures::execute_mission_payload(),
        1,
        false);
    const std::string second_payload = mqtt_fixtures::replace_once(
        mqtt_fixtures::execute_mission_payload(),
        "2ac105b7-2233-4455-aabb-66778899aabb",
        "2ac105b7-2233-4455-aabb-66778899aabc");
    const auto second =
        source.ingest(source.command_topic(), second_payload, 1, false);
    test_support::check(first.ok() && second.ok(), "valid commands enter bounded queue");

    RemoteCommand command;
    test_support::check(
        source.wait_next(command, std::chrono::milliseconds{0}),
        "first command is available");
    test_support::check(
        command.command_id == "2ac105b7-2233-4455-aabb-66778899aabb",
        "queue preserves FIFO order");
    test_support::check(
        source.wait_next(command, std::chrono::milliseconds{0}),
        "second command is available");
    test_support::check(
        command.command_id == "2ac105b7-2233-4455-aabb-66778899aabc",
        "second FIFO command is preserved");
}

void test_transport_metadata_rejections()
{
    MqttCommandSource source{"DRONE-01", kLimits};
    const std::string payload = mqtt_fixtures::execute_mission_payload();

    auto result = source.ingest("aegisq/cmd/WHEEL-01", payload, 1, false);
    test_support::check(
        result.error == IngestError::topic_mismatch,
        "ground robot topic is rejected by drone connector");

    result = source.ingest(source.command_topic(), payload, 0, false);
    test_support::check(result.error == IngestError::invalid_qos, "QoS 0 is rejected");

    result = source.ingest(source.command_topic(), payload, 1, true);
    test_support::check(
        result.error == IngestError::retained_message,
        "retained command is rejected");

    result = source.ingest(source.command_topic(), {}, 1, false);
    test_support::check(result.error == IngestError::empty_payload, "empty payload is rejected");
    test_support::check(source.queued() == 0U, "metadata rejection never enqueues");
}

void test_payload_and_queue_bounds()
{
    CommandSourceLimits one = kLimits;
    one.queue_capacity = 1U;
    MqttCommandSource source{"DRONE-01", one};
    const std::string payload = mqtt_fixtures::execute_mission_payload();
    test_support::check(
        source.ingest(source.command_topic(), payload, 1, false).ok(),
        "first command fills queue");

    const std::string second_payload = mqtt_fixtures::replace_once(
        payload,
        "2ac105b7-2233-4455-aabb-66778899aabb",
        "2ac105b7-2233-4455-aabb-66778899aabc");
    const auto full =
        source.ingest(source.command_topic(), second_payload, 1, false);
    test_support::check(full.error == IngestError::queue_full, "newest command is rejected");
    test_support::check(source.queued() == 1U, "full queue keeps older command");

    CommandSourceLimits tiny = kLimits;
    tiny.max_payload_bytes = payload.size() - 1U;
    MqttCommandSource small_source{"DRONE-01", tiny};
    const auto oversized =
        small_source.ingest(small_source.command_topic(), payload, 1, false);
    test_support::check(
        oversized.error == IngestError::payload_too_large,
        "oversized payload is rejected before JSON parsing");
}

void test_safety_queue_preempts_full_mission_queue()
{
    CommandSourceLimits limits = kLimits;
    limits.queue_capacity = 1U;
    MqttCommandSource source{"DRONE-01", limits};

    test_support::check(
        source.ingest(
            source.command_topic(), mqtt_fixtures::execute_mission_payload(), 1, false).ok(),
        "normal queue accepts the mission");
    const auto normal_full = source.ingest(
        source.command_topic(),
        mqtt_fixtures::replace_once(
            mqtt_fixtures::execute_mission_payload(),
            "2ac105b7-2233-4455-aabb-66778899aabb",
            "2ac105b7-2233-4455-aabb-66778899aabc"),
        1,
        false);
    test_support::check(
        normal_full.error == IngestError::queue_full,
        "normal mission queue is independently bounded");

    test_support::check(
        source.ingest(
            source.command_topic(), mqtt_fixtures::stop_mission_payload(), 1, false).ok(),
        "STOP still enters its reserved queue");
    test_support::check(source.queued_safety() == 1U, "reserved safety queue is observable");

    RemoteCommand command;
    test_support::check(
        source.wait_next(command, std::chrono::milliseconds{0})
            && command.type == dc::flight::RemoteCommandType::stop_mission,
        "STOP preempts an older queued mission");
    test_support::check(
        source.wait_next(command, std::chrono::milliseconds{0})
            && command.type == dc::flight::RemoteCommandType::execute_mission,
        "mission remains queued after the safety command");
}

void test_decode_rejection_preserves_safe_command_id()
{
    MqttCommandSource source{"DRONE-01", kLimits};
    auto root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    root["coordinateFrame"] = "UNSUPPORTED";
    const auto result = source.ingest(source.command_topic(), root.dump(), 1, false);
    test_support::check(result.error == IngestError::decode_failed, "bad frame is rejected");
    test_support::check(
        result.command_id == "2ac105b7-2233-4455-aabb-66778899aabb",
        "later schema failure retains canonical id for negative ACK");
}

void test_close_clears_and_rejects()
{
    MqttCommandSource source{"DRONE-01", kLimits};
    const std::string payload = mqtt_fixtures::execute_mission_payload();
    static_cast<void>(source.ingest(source.command_topic(), payload, 1, false));
    source.close();
    test_support::check(source.queued() == 0U, "close drops pending flight commands");

    RemoteCommand command;
    test_support::check(
        !source.wait_next(command, std::chrono::milliseconds{0}),
        "closed source wakes with no command");
    const auto result =
        source.ingest(source.command_topic(), payload, 1, false);
    test_support::check(result.error == IngestError::closed, "closed source rejects ingest");
}

void test_configuration_fails_closed()
{
    bool invalid_topic_rejected = false;
    try {
        MqttCommandSource invalid_vehicle{"DRONE/#", kLimits};
    } catch (const std::invalid_argument&) {
        invalid_topic_rejected = true;
    }
    test_support::check(
        invalid_topic_rejected,
        "wildcard vehicle id cannot create a subscription topic");

    CommandSourceLimits shallow = kLimits;
    shallow.codec.max_json_depth = 2U;
    bool shallow_depth_rejected = false;
    try {
        MqttCommandSource invalid_depth{"DRONE-01", shallow};
    } catch (const std::invalid_argument&) {
        shallow_depth_rejected = true;
    }
    test_support::check(
        shallow_depth_rejected,
        "source rejects JSON depth below the valid envelope shape");

    CommandSourceLimits no_safety_reserve = kLimits;
    no_safety_reserve.safety_queue_capacity = 0U;
    bool reserve_rejected = false;
    try {
        MqttCommandSource invalid_reserve{"DRONE-01", no_safety_reserve};
    } catch (const std::invalid_argument&) {
        reserve_rejected = true;
    }
    test_support::check(
        reserve_rejected,
        "source requires a bounded safety-command reserve");
}

}  // namespace

int main()
{
    test_topic_derivation_and_fifo();
    test_transport_metadata_rejections();
    test_payload_and_queue_bounds();
    test_safety_queue_preempts_full_mission_queue();
    test_decode_rejection_preserves_safe_command_id();
    test_close_clears_and_rejects();
    test_configuration_fails_closed();
    return test_support::report("mqtt_command_source_tests");
}
