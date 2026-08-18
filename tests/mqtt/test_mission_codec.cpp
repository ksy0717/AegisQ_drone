#include "dc/mqtt/mission_codec.hpp"
#include "fixtures.hpp"
#include "test_support.hpp"

#include <chrono>
#include <string>

#include <nlohmann/json.hpp>

namespace {

using dc::flight::MissionStatus;
using dc::flight::RemoteCommandType;
using dc::flight::RouteLimits;
using dc::flight::WaypointAction;
using dc::flight::WaypointPositionMode;
using dc::mqtt::MissionCodecLimits;

constexpr MissionCodecLimits kLimits{
    RouteLimits{20U, 2.0, 120.0, 60U, 180U, 0.5, 8.0},
    128U,
    4U,
};

void test_prototype_v2_execute_decodes_without_data_loss()
{
    const auto result = dc::mqtt::decode_command(
        mqtt_fixtures::execute_mission_payload(), "DRONE-01", kLimits);
    test_support::check(result.ok(), "prototype-v2 EXECUTE_MISSION decodes");
    if (!result.ok()) return;

    const auto& command = *result.command;
    test_support::check(
        command.command_id == "2ac105b7-2233-4455-aabb-66778899aabb",
        "canonical commandId is preserved");
    test_support::check(
        command.type == RemoteCommandType::execute_mission,
        "command maps to execute_mission");
    test_support::check(command.vehicle_id == "DRONE-01", "vehicle comes from exact topic");
    test_support::check(
        command.mission_id == "f7c5c855-6677-4889-aabb-112233445566",
        "missionId is preserved");
    test_support::check(command.route.has_value(), "mission route is present");
    test_support::check(command.expires_at.has_value(), "expiresAt is preserved");
    test_support::check(!command.canonical_payload.empty(), "canonical payload is retained");
    if (!command.route.has_value()) return;

    const auto& route = *command.route;
    test_support::check(route.coordinate_frame == "ABSOLUTE", "coordinateFrame is explicit");
    test_support::check(
        route.altitude_reference == "HOME_RELATIVE",
        "altitudeReference is explicit");
    test_support::check(
        route.launch.mode == "CURRENT_POSITION"
            && route.launch.takeoff_altitude_m == 10.0,
        "launch binds current position to TAKEOFF altitude");
    test_support::check(route.waypoints.size() == 4U, "all waypoints are preserved");
    test_support::check(
        !route.waypoints.front().latitude_deg.has_value()
            && route.waypoints.front().position_mode == WaypointPositionMode::current,
        "positionless TAKEOFF is current-position control");
    test_support::check(
        route.waypoints.back().action == WaypointAction::return_to_launch
            && route.waypoints.back().position_mode == WaypointPositionMode::home
            && !route.waypoints.back().relative_altitude_m.has_value(),
        "terminal RTL targets PX4 Home without an altitude");
}

void test_safety_commands_decode()
{
    auto result = dc::mqtt::decode_command(
        mqtt_fixtures::stop_mission_payload(), "DRONE-01", kLimits);
    test_support::check(
        result.ok() && result.command->type == RemoteCommandType::stop_mission,
        "STOP_MISSION decodes");

    result = dc::mqtt::decode_command(
        mqtt_fixtures::rtl_payload(), "DRONE-01", kLimits);
    test_support::check(
        result.ok() && result.command->type == RemoteCommandType::return_to_launch,
        "mission STOP follow-up RTL decodes");
    if (result.ok()) {
        test_support::check(
            result.command->reason == "OPERATOR_STOP", "RTL reason is preserved");
    }

    result = dc::mqtt::decode_command(
        mqtt_fixtures::emergency_stop_payload(), "DRONE-01", kLimits);
    test_support::check(
        result.ok() && result.command->type == RemoteCommandType::emergency_stop,
        "standalone EMERGENCY_STOP decodes");

    const auto mission_only = dc::mqtt::decode_execute_mission(
        mqtt_fixtures::rtl_payload(), "DRONE-01", kLimits);
    test_support::check(
        mission_only.error == "unsupported_command",
        "mission-only compatibility entry point still rejects RTL");
}

void test_server_control_only_and_land_forms()
{
    auto root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    root["waypoints"] = nlohmann::json::array({{
        {"seq", 1},
        {"action", "RTL"},
        {"dwellSec", 0},
        {"positionMode", "HOME"},
    }});
    root["launch"].erase("takeoffAltitudeM");
    auto result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(result.ok(), "standalone RTL route step is valid");

    root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    root["waypoints"][3]["action"] = "LAND";
    root["waypoints"][3]["positionMode"] = "CURRENT";
    result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(result.ok(), "LAND is also a valid terminal action");

    root["waypoints"][3]["altitude"] = 3.0;
    result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(!result.ok(), "LAND altitude is rejected rather than causing hover");
}

void test_sequence_and_position_mode_fail_closed()
{
    auto root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    std::swap(root["waypoints"][0], root["waypoints"][3]);
    auto result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(!result.ok(), "array order cannot be silently sorted by seq");

    root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    root["waypoints"][1]["positionMode"] = "CURRENT";
    result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(!result.ok(), "MOVE cannot ignore its absolute coordinates");

    root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    root["coordinateFrame"] = "RELATIVE";
    result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(!result.ok(), "unknown coordinate frame is rejected");
}

void test_unknown_fields_duplicates_and_direct_goto_fail_closed()
{
    auto root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    root["vehicleId"] = "WHEEL-01";
    auto result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(!result.ok(), "payload identity field is not trusted");

    root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    root["cruiseSpeedUnknown"] = 0.8;
    result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(!result.ok(), "unknown root field is rejected");

    // Waypoints are the one place unknown keys are tolerated. AegisQ preserves
    // any field an edge device started writing, so rejecting them here would
    // break every route the moment either side adds one. Root fields carry
    // command-level meaning and stay strict, as the check above pins.
    root = nlohmann::json::parse(mqtt_fixtures::execute_mission_payload());
    root["waypoints"][1]["operatorLabel"] = "gate-A";
    result = dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits);
    test_support::check(
        result.ok(), "unknown waypoint field is carried, not rejected");

    const std::string duplicate_key = mqtt_fixtures::replace_once(
        mqtt_fixtures::execute_mission_payload(),
        R"json("command": "EXECUTE_MISSION",)json",
        R"json("command": "EXECUTE_MISSION", "command": "EXECUTE_MISSION",)json");
    result = dc::mqtt::decode_command(duplicate_key, "DRONE-01", kLimits);
    test_support::check(!result.ok(), "duplicate JSON key is rejected");

    nlohmann::json direct_goto{
        {"commandId", "6ac105b7-2233-4455-aabb-66778899aabb"},
        {"command", "EXECUTE_MISSION"},
        {"waypoints", nlohmann::json::array({{
            {"seq", 1}, {"lat", 35.3}, {"lng", 129.0},
            {"action", "MOVE"}, {"dwellSec", 0},
        }})},
        {"issuedBy", "ksy"},
        {"issuedAt", "2026-08-06T03:04:05.123Z"},
    };
    result = dc::mqtt::decode_command(direct_goto.dump(), "DRONE-01", kLimits);
    test_support::check(
        !result.ok(),
        "DroneOps GOTO without altitude/home contract is rejected safely");
    test_support::check(
        result.command_id == "6ac105b7-2233-4455-aabb-66778899aabb",
        "canonical commandId survives a later schema rejection");
}

// Pins the envelope aegisq-plus publishes today. Before cruiseSpeedMps and the
// per-leg speed were accepted, every mission from the current server failed the
// root whitelist with invalid_root_schema — the aircraft could not fly one.
void test_current_server_envelope_decodes()
{
    const auto result = dc::mqtt::decode_command(
        mqtt_fixtures::server_execute_mission_payload(), "DRONE-01", kLimits);
    test_support::check(
        result.ok(), "the envelope the current server publishes decodes");
    if (!result.ok()) return;

    const auto& route = *result.command->route;
    test_support::check(
        route.cruise_speed_m_s.has_value()
            && *route.cruise_speed_m_s == 2.0,
        "mission-level cruiseSpeedMps is carried to the executor");
    test_support::check(
        route.waypoints.at(1).speed_m_s.has_value()
            && *route.waypoints.at(1).speed_m_s == 1.5,
        "a per-leg speed overrides the route cruise speed");
    test_support::check(
        !route.waypoints.at(2).speed_m_s.has_value(),
        "an omitted per-leg speed stays absent rather than defaulting");
    test_support::check(
        route.waypoints.back().action == WaypointAction::land
            && route.waypoints.back().position_mode
                == WaypointPositionMode::current,
        "the server-expanded LAND keeps its CURRENT position mode");

    // The speed range is vehicle policy, not the server's.
    auto root = nlohmann::json::parse(
        mqtt_fixtures::server_execute_mission_payload());
    root["cruiseSpeedMps"] = 9.0;
    test_support::check(
        dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits).error
            == "speed_out_of_range",
        "a cruise speed above the vehicle limit is rejected, not clamped");

    root = nlohmann::json::parse(
        mqtt_fixtures::server_execute_mission_payload());
    root["waypoints"][1]["speed"] = 0.1;
    test_support::check(
        !dc::mqtt::decode_command(root.dump(), "DRONE-01", kLimits).ok(),
        "a per-leg speed below the vehicle limit is rejected");
}

void test_rfc3339_and_ack_codec()
{
    const auto utc = dc::mqtt::parse_rfc3339("2026-08-06T03:04:05.123Z");
    const auto offset = dc::mqtt::parse_rfc3339("2026-08-06T12:04:05.123+09:00");
    test_support::check(utc.has_value() && offset.has_value(), "RFC3339 forms parse");
    test_support::check(utc == offset, "RFC3339 offset is normalized to UTC");
    test_support::check(
        !dc::mqtt::parse_rfc3339("2026-02-30T00:00:00Z").has_value(),
        "invalid calendar date is rejected");

    MissionStatus status;
    status.command_id = "2ac105b7-2233-4455-aabb-66778899aabb";
    status.vehicle_id = "DRONE-01";
    status.mission_id = "f7c5c855-6677-4889-aabb-112233445566";
    status.state = "REJECTED";
    status.detail = "CONTROL_DISABLED";
    const auto ack = nlohmann::json::parse(dc::mqtt::encode_command_ack(status, *utc));
    test_support::check(ack["status"] == "REJECTED", "ACK uses terminal rejection status");
    test_support::check(ack["error"] == "CONTROL_DISABLED", "ACK carries Pi verdict");
    test_support::check(
        ack["ackedAt"] == "2026-08-06T03:04:05.123Z",
        "ACK timestamp is UTC RFC3339");
}

void test_json_depth_boundary()
{
    MissionCodecLimits exact_depth = kLimits;
    exact_depth.max_json_depth = 3U;
    test_support::check(
        dc::mqtt::decode_command(
            mqtt_fixtures::execute_mission_payload(), "DRONE-01", exact_depth).ok(),
        "valid mission fits JSON depth three");

    exact_depth.max_json_depth = 2U;
    test_support::check(
        dc::mqtt::decode_command(
            mqtt_fixtures::execute_mission_payload(), "DRONE-01", exact_depth).error
            == "invalid_codec_limits",
        "depth below the envelope shape is rejected as configuration");
}

}  // namespace

int main()
{
    test_prototype_v2_execute_decodes_without_data_loss();
    test_safety_commands_decode();
    test_server_control_only_and_land_forms();
    test_sequence_and_position_mode_fail_closed();
    test_unknown_fields_duplicates_and_direct_goto_fail_closed();
    test_current_server_envelope_decodes();
    test_rfc3339_and_ack_codec();
    test_json_depth_boundary();
    return test_support::report("mqtt_mission_codec_tests");
}
