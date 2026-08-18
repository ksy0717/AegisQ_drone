#include "dc/mqtt/aegisq_state_codec.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

#include "dc/mqtt/mission_codec.hpp"
#include "dc/mqtt/mqtt_command_source.hpp"

namespace dc::mqtt {
namespace {

using Json = nlohmann::json;

bool finite_position(const flight::GlobalPosition& position) noexcept
{
    return std::isfinite(position.latitude_deg)
        && std::isfinite(position.longitude_deg)
        && std::isfinite(position.absolute_altitude_m)
        && std::isfinite(position.relative_altitude_m)
        && position.latitude_deg >= -90.0 && position.latitude_deg <= 90.0
        && position.longitude_deg >= -180.0 && position.longitude_deg <= 180.0
        && !(position.latitude_deg == 0.0 && position.longitude_deg == 0.0);
}

void require_registration_text(
    const std::string& value, const char* field)
{
    if (value.empty() || value.size() > 256U) {
        throw std::invalid_argument(
            "invalid vehicle registration field: " + std::string{field});
    }
}

Json encode_position(const flight::GlobalPosition& position)
{
    return Json{
        {"lat", position.latitude_deg},
        {"lng", position.longitude_deg},
        {"absoluteAltitude", position.absolute_altitude_m},
        {"relativeAltitude", position.relative_altitude_m},
    };
}

void add_finite_optional(
    Json& object, const char* key, const std::optional<double>& value)
{
    if (value.has_value() && std::isfinite(*value)) {
        object[key] = *value;
    }
}

}  // namespace

std::string telemetry_topic_for(const std::string_view vehicle_id)
{
    if (!valid_vehicle_id(vehicle_id)) {
        throw std::invalid_argument("invalid MQTT vehicle id");
    }
    return "aegisq/telemetry/" + std::string{vehicle_id};
}

std::string registration_topic_for(const std::string_view vehicle_id)
{
    if (!valid_vehicle_id(vehicle_id)) {
        throw std::invalid_argument("invalid MQTT vehicle id");
    }
    return "aegisq/robot-register/" + std::string{vehicle_id};
}

std::string lwt_topic_for(const std::string_view vehicle_id)
{
    if (!valid_vehicle_id(vehicle_id)) {
        throw std::invalid_argument("invalid MQTT vehicle id");
    }
    return "aegisq/lwt/" + std::string{vehicle_id};
}

std::string encode_vehicle_registration(
    const VehicleRegistration& registration)
{
    if (!valid_vehicle_id(registration.robot_id)) {
        throw std::invalid_argument("invalid registration robot id");
    }
    require_registration_text(registration.name, "name");
    require_registration_text(registration.model, "model");
    require_registration_text(registration.firmware, "firmware");
    require_registration_text(registration.site_id, "siteId");
    return Json{
        {"robotId", registration.robot_id},
        {"mobilityClass", "DRONE"},
        {"name", registration.name},
        {"model", registration.model},
        {"firmware", registration.firmware},
        {"siteId", registration.site_id},
    }.dump();
}

std::string encode_vehicle_telemetry(
    const flight::VehicleTelemetrySnapshot& snapshot)
{
    if (!valid_vehicle_id(snapshot.vehicle_id)) {
        throw std::invalid_argument("invalid telemetry vehicle id");
    }

    Json payload{
        {"occurredAt", format_rfc3339(snapshot.occurred_at)},
        {"robotId", snapshot.vehicle_id},
        {"status", !snapshot.connected
            ? "ERROR"
            : (snapshot.armed || snapshot.in_air ? "ACTIVE" : "STANDBY")},
        {"mobilityClass", "DRONE"},
        {"homeValid", snapshot.home_valid},
        {"globalPositionValid", snapshot.global_position_valid},
        {"armed", snapshot.armed},
        {"inAir", snapshot.in_air},
        {"controlOwner", flight::control_owner_name(snapshot.control_owner)},
        {"readyForMission", snapshot.ready_for_mission},
    };
    add_finite_optional(payload, "battery", snapshot.battery_percent);
    add_finite_optional(payload, "batteryTemp", snapshot.battery_temperature_c);

    if (snapshot.position.has_value()
        && finite_position(*snapshot.position)) {
        Json position = encode_position(*snapshot.position);
        add_finite_optional(position, "heading", snapshot.heading_deg);
        add_finite_optional(position, "speed", snapshot.horizontal_speed_m_s);
        payload["position"] = std::move(position);
    }
    if (snapshot.home_valid
        && snapshot.home_position.has_value()
        && finite_position(*snapshot.home_position)) {
        payload["homePosition"] = encode_position(*snapshot.home_position);
    }

    Json hardware{
        {"px4Connected", snapshot.connected},
        {"flightMode", snapshot.flight_mode.empty()
            ? "UNKNOWN"
            : snapshot.flight_mode},
        {"ownershipSwitch",
         flight::ownership_switch_name(snapshot.ownership_switch)},
        {"armSwitch", flight::arm_switch_position_name(snapshot.arm_switch)},
        {"ownershipEpoch", snapshot.ownership_epoch},
    };
    add_finite_optional(
        hardware, "batteryVoltage", snapshot.battery_voltage_v);
    if (snapshot.ownership_pwm.has_value()) {
        hardware["ownershipPwm"] = *snapshot.ownership_pwm;
    }
    if (snapshot.arm_switch_pwm.has_value()) {
        hardware["armSwitchPwm"] = *snapshot.arm_switch_pwm;
    }
    payload["hardware"] = std::move(hardware);
    return payload.dump();
}

std::string encode_vehicle_lwt(
    const std::string_view vehicle_id,
    const std::string_view reason,
    const std::chrono::system_clock::time_point occurred_at)
{
    if (!valid_vehicle_id(vehicle_id)
        || reason.empty() || reason.size() > 128U) {
        throw std::invalid_argument("invalid vehicle LWT");
    }
    return Json{
        {"robotId", vehicle_id},
        {"status", "OFFLINE"},
        {"reason", reason},
        {"ts", format_rfc3339(occurred_at)},
    }.dump();
}

}  // namespace dc::mqtt
