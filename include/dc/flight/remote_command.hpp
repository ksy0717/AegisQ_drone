#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace dc::flight {

// Only verbs with an implemented, safety-reviewed PX4 meaning belong here.
// Other AegisQ robot/ICD verbs are rejected by the drone codec.
enum class RemoteCommandType : std::uint8_t {
    execute_mission,
    stop_mission,
    return_to_launch,
    emergency_stop,
};

enum class WaypointAction : std::uint8_t {
    takeoff,
    move,
    wait,
    hover,
    lpr,
    watch,
    land,
    return_to_launch,
};

enum class WaypointPositionMode : std::uint8_t {
    waypoint,
    current,
    home,
};

struct GeoWaypoint {
    // TAKEOFF/LAND/RTL are control actions rather than map points. Their
    // coordinates may be absent, and coordinates preserved by the server for
    // display are ignored according to position_mode.
    std::optional<double> latitude_deg;
    std::optional<double> longitude_deg;
    std::optional<double> relative_altitude_m;
    std::uint32_t sequence{};
    WaypointAction action{WaypointAction::move};
    std::uint32_t dwell_seconds{};
    WaypointPositionMode position_mode{WaypointPositionMode::waypoint};
    // Speed for the leg that arrives at this point. Absent means "use the
    // route's cruise speed"; the server omits the key rather than sending a
    // default, so absent and "explicitly the same as cruise" stay distinct.
    std::optional<double> speed_m_s;
};

struct LaunchSpec {
    std::string mode;
    std::optional<double> takeoff_altitude_m;
};

struct RoutePlan {
    std::vector<GeoWaypoint> waypoints;
    std::string coordinate_frame;
    std::string altitude_reference;
    LaunchSpec launch;
    // Route-wide speed chosen by the operator. Optional so a server that has
    // not adopted the field still decodes; the executor falls back to its
    // configured speed when it is absent.
    std::optional<double> cruise_speed_m_s;
};

struct RemoteCommand {
    // AegisQ generates this with UUID.randomUUID(). Keep the UUID string
    // intact: hashing it into a numeric id would weaken idempotency.
    std::string command_id;
    std::string vehicle_id;
    RemoteCommandType type{RemoteCommandType::execute_mission};
    std::optional<std::string> mission_id;
    std::optional<RoutePlan> route;
    std::string issued_by;
    std::optional<std::chrono::system_clock::time_point> issued_at;
    std::optional<std::chrono::system_clock::time_point> expires_at;
    std::string reason;

    // Local-only stamp captured synchronously when the executor accepts START.
    // It is not decoded, encoded, or included in canonical_payload.
    std::optional<std::uint64_t> dispatch_ownership_epoch;

    // The strict codec writes a deterministic JSON representation here. The
    // durable ledger compares it byte-for-byte when the same UUID reappears.
    std::string canonical_payload;
};

// Limits are injected by the application runtime configuration. There are no
// safety-critical vehicle defaults in this transport contract.
struct RouteLimits {
    std::size_t max_waypoints{};
    double min_relative_altitude_m{};
    double max_relative_altitude_m{};
    std::uint32_t max_dwell_seconds{};
    std::uint64_t max_total_dwell_seconds{};
    // Bounds for the server's speed fields. The vehicle owns these: the server
    // enforces its own range, but nothing on that side knows this airframe.
    double min_speed_m_s{};
    double max_speed_m_s{};
};

struct CommandFreshnessLimits {
    std::chrono::seconds max_age{};
    std::chrono::seconds max_future_skew{};
};

enum class ValidationError : std::uint8_t {
    none,
    invalid_command_id,
    empty_vehicle_id,
    empty_issued_by,
    missing_issued_at,
    missing_expires_at,
    invalid_expiry,
    expired_command,
    missing_mission_id,
    missing_route,
    unexpected_route,
    empty_waypoints,
    too_many_waypoints,
    invalid_limits,
    invalid_coordinate_frame,
    invalid_altitude_reference,
    invalid_launch,
    missing_coordinate,
    partial_coordinate,
    non_finite_coordinate,
    latitude_out_of_range,
    longitude_out_of_range,
    missing_altitude,
    unexpected_altitude,
    altitude_out_of_range,
    invalid_position_mode,
    invalid_waypoint_sequence,
    invalid_action_order,
    dwell_out_of_range,
    total_dwell_exceeded,
    speed_out_of_range,
    stale_command,
    command_from_future,
};

struct ValidationResult {
    static constexpr std::size_t no_waypoint = std::numeric_limits<std::size_t>::max();

    ValidationError error{ValidationError::none};
    std::size_t waypoint_index{no_waypoint};

    [[nodiscard]] bool ok() const noexcept
    {
        return error == ValidationError::none;
    }
};

inline bool valid_route_limits(const RouteLimits& limits) noexcept
{
    return limits.max_waypoints > 0U
        && std::isfinite(limits.min_relative_altitude_m)
        && std::isfinite(limits.max_relative_altitude_m)
        && limits.min_relative_altitude_m <= limits.max_relative_altitude_m
        && limits.max_dwell_seconds > 0U
        && limits.max_total_dwell_seconds > 0U
        && std::isfinite(limits.min_speed_m_s)
        && std::isfinite(limits.max_speed_m_s)
        && limits.min_speed_m_s > 0.0
        && limits.min_speed_m_s <= limits.max_speed_m_s;
}

inline bool speed_within(
    const std::optional<double>& speed, const RouteLimits& limits) noexcept
{
    return !speed.has_value()
        || (std::isfinite(*speed)
            && *speed >= limits.min_speed_m_s
            && *speed <= limits.max_speed_m_s);
}

inline bool is_terminal_action(const WaypointAction action) noexcept
{
    return action == WaypointAction::land
        || action == WaypointAction::return_to_launch;
}

inline bool is_control_action(const WaypointAction action) noexcept
{
    return action == WaypointAction::takeoff || is_terminal_action(action);
}

inline bool requires_altitude(const WaypointAction action) noexcept
{
    return !is_terminal_action(action);
}

inline bool is_safety_command(const RemoteCommandType type) noexcept
{
    return type == RemoteCommandType::stop_mission
        || type == RemoteCommandType::return_to_launch
        || type == RemoteCommandType::emergency_stop;
}

inline ValidationResult validate_route(
    const RoutePlan& route,
    const RouteLimits& limits) noexcept
{
    if (!valid_route_limits(limits)) {
        return {ValidationError::invalid_limits, ValidationResult::no_waypoint};
    }
    if (route.coordinate_frame != "ABSOLUTE") {
        return {ValidationError::invalid_coordinate_frame, ValidationResult::no_waypoint};
    }
    if (route.altitude_reference != "HOME_RELATIVE") {
        return {ValidationError::invalid_altitude_reference, ValidationResult::no_waypoint};
    }
    if (route.launch.mode != "CURRENT_POSITION") {
        return {ValidationError::invalid_launch, ValidationResult::no_waypoint};
    }
    if (route.launch.takeoff_altitude_m.has_value()
        && (!std::isfinite(*route.launch.takeoff_altitude_m)
            || *route.launch.takeoff_altitude_m < limits.min_relative_altitude_m
            || *route.launch.takeoff_altitude_m > limits.max_relative_altitude_m)) {
        return {ValidationError::invalid_launch, ValidationResult::no_waypoint};
    }
    if (!speed_within(route.cruise_speed_m_s, limits)) {
        return {ValidationError::speed_out_of_range, ValidationResult::no_waypoint};
    }
    if (route.waypoints.empty()) {
        return {ValidationError::empty_waypoints, ValidationResult::no_waypoint};
    }
    if (route.waypoints.size() > limits.max_waypoints) {
        return {ValidationError::too_many_waypoints, ValidationResult::no_waypoint};
    }

    std::size_t takeoff_count = 0U;
    std::size_t terminal_count = 0U;
    bool routed = false;
    std::uint64_t total_dwell = 0U;

    for (std::size_t index = 0U; index < route.waypoints.size(); ++index) {
        const GeoWaypoint& point = route.waypoints[index];
        const std::size_t expected_sequence = index + 1U;
        if (expected_sequence > std::numeric_limits<std::uint32_t>::max()
            || point.sequence != static_cast<std::uint32_t>(expected_sequence)) {
            return {ValidationError::invalid_waypoint_sequence, index};
        }

        const bool has_latitude = point.latitude_deg.has_value();
        const bool has_longitude = point.longitude_deg.has_value();
        if (has_latitude != has_longitude) {
            return {ValidationError::partial_coordinate, index};
        }
        const bool has_coordinates = has_latitude;
        if (!has_coordinates && !is_control_action(point.action)) {
            return {ValidationError::missing_coordinate, index};
        }
        if (has_coordinates) {
            if (!std::isfinite(*point.latitude_deg)
                || !std::isfinite(*point.longitude_deg)) {
                return {ValidationError::non_finite_coordinate, index};
            }
            if (*point.latitude_deg < -90.0 || *point.latitude_deg > 90.0) {
                return {ValidationError::latitude_out_of_range, index};
            }
            if (*point.longitude_deg < -180.0 || *point.longitude_deg > 180.0) {
                return {ValidationError::longitude_out_of_range, index};
            }
        }

        const WaypointPositionMode expected_mode =
            point.action == WaypointAction::return_to_launch
                ? WaypointPositionMode::home
                : (point.action == WaypointAction::takeoff
                       || point.action == WaypointAction::land)
                    ? WaypointPositionMode::current
                    : WaypointPositionMode::waypoint;
        if (point.position_mode != expected_mode) {
            return {ValidationError::invalid_position_mode, index};
        }

        if (requires_altitude(point.action)) {
            if (!point.relative_altitude_m.has_value()) {
                return {ValidationError::missing_altitude, index};
            }
            if (!std::isfinite(*point.relative_altitude_m)) {
                return {ValidationError::non_finite_coordinate, index};
            }
            if (*point.relative_altitude_m < limits.min_relative_altitude_m
                || *point.relative_altitude_m > limits.max_relative_altitude_m) {
                return {ValidationError::altitude_out_of_range, index};
            }
        } else if (point.relative_altitude_m.has_value()) {
            return {ValidationError::unexpected_altitude, index};
        }

        if (!speed_within(point.speed_m_s, limits)) {
            return {ValidationError::speed_out_of_range, index};
        }
        if (point.dwell_seconds > limits.max_dwell_seconds) {
            return {ValidationError::dwell_out_of_range, index};
        }
        if (total_dwell > limits.max_total_dwell_seconds
            || point.dwell_seconds > limits.max_total_dwell_seconds - total_dwell) {
            return {ValidationError::total_dwell_exceeded, index};
        }
        total_dwell += point.dwell_seconds;

        if (point.action == WaypointAction::takeoff) {
            ++takeoff_count;
            if (index != 0U) {
                return {ValidationError::invalid_action_order, index};
            }
        }
        if (is_terminal_action(point.action)) {
            ++terminal_count;
            if (index + 1U != route.waypoints.size()) {
                return {ValidationError::invalid_action_order, index};
            }
        }
        routed = routed || !is_control_action(point.action);
    }

    if (takeoff_count > 1U || terminal_count > 1U
        || (takeoff_count == 1U && terminal_count != 1U)
        || (takeoff_count == 0U && terminal_count == 0U)) {
        return {ValidationError::invalid_action_order, ValidationResult::no_waypoint};
    }
    if (routed
        && (route.waypoints.front().action != WaypointAction::takeoff
            || !is_terminal_action(route.waypoints.back().action))) {
        return {ValidationError::invalid_action_order, ValidationResult::no_waypoint};
    }

    const auto first_takeoff = route.waypoints.front().action == WaypointAction::takeoff
        ? route.waypoints.front().relative_altitude_m
        : std::optional<double>{};
    if (route.launch.takeoff_altitude_m.has_value() != first_takeoff.has_value()
        || (first_takeoff.has_value()
            && std::abs(*route.launch.takeoff_altitude_m - *first_takeoff) > 1.0e-6)) {
        return {ValidationError::invalid_launch, ValidationResult::no_waypoint};
    }
    return {};
}

inline ValidationResult validate_command(
    const RemoteCommand& command, const RouteLimits& limits) noexcept
{
    if (command.command_id.empty()) {
        return {ValidationError::invalid_command_id, ValidationResult::no_waypoint};
    }
    if (command.vehicle_id.empty()) {
        return {ValidationError::empty_vehicle_id, ValidationResult::no_waypoint};
    }
    if (command.issued_by.empty()) {
        return {ValidationError::empty_issued_by, ValidationResult::no_waypoint};
    }
    if (!command.issued_at.has_value()) {
        return {ValidationError::missing_issued_at, ValidationResult::no_waypoint};
    }

    if (command.type == RemoteCommandType::execute_mission) {
        if (!command.mission_id.has_value() || command.mission_id->empty()) {
            return {ValidationError::missing_mission_id, ValidationResult::no_waypoint};
        }
        if (!command.expires_at.has_value()) {
            return {ValidationError::missing_expires_at, ValidationResult::no_waypoint};
        }
        if (!command.route.has_value()) {
            return {ValidationError::missing_route, ValidationResult::no_waypoint};
        }
        return validate_route(*command.route, limits);
    }
    if (command.route.has_value()) {
        return {ValidationError::unexpected_route, ValidationResult::no_waypoint};
    }
    if (command.expires_at.has_value()) {
        return {ValidationError::invalid_expiry, ValidationResult::no_waypoint};
    }
    return {};
}

inline ValidationResult validate_freshness(
    const RemoteCommand& command,
    const CommandFreshnessLimits& limits,
    const std::chrono::system_clock::time_point now) noexcept
{
    if (limits.max_age <= std::chrono::seconds::zero()
        || limits.max_future_skew < std::chrono::seconds::zero()) {
        return {ValidationError::invalid_limits, ValidationResult::no_waypoint};
    }
    if (!command.issued_at.has_value()) {
        return {ValidationError::missing_issued_at, ValidationResult::no_waypoint};
    }
    if (*command.issued_at > now
        && *command.issued_at - now > limits.max_future_skew) {
        return {ValidationError::command_from_future, ValidationResult::no_waypoint};
    }
    if (*command.issued_at <= now && now - *command.issued_at > limits.max_age) {
        return {ValidationError::stale_command, ValidationResult::no_waypoint};
    }
    if (command.expires_at.has_value()) {
        if (*command.expires_at <= *command.issued_at) {
            return {ValidationError::invalid_expiry, ValidationResult::no_waypoint};
        }
        if (now >= *command.expires_at) {
            return {ValidationError::expired_command, ValidationResult::no_waypoint};
        }
    }
    return {};
}

inline const char* validation_error_name(const ValidationError error) noexcept
{
    switch (error) {
        case ValidationError::none: return "none";
        case ValidationError::invalid_command_id: return "invalid_command_id";
        case ValidationError::empty_vehicle_id: return "empty_vehicle_id";
        case ValidationError::empty_issued_by: return "empty_issued_by";
        case ValidationError::missing_issued_at: return "missing_issued_at";
        case ValidationError::missing_expires_at: return "missing_expires_at";
        case ValidationError::invalid_expiry: return "invalid_expiry";
        case ValidationError::expired_command: return "expired_command";
        case ValidationError::missing_mission_id: return "missing_mission_id";
        case ValidationError::missing_route: return "missing_route";
        case ValidationError::unexpected_route: return "unexpected_route";
        case ValidationError::empty_waypoints: return "empty_waypoints";
        case ValidationError::too_many_waypoints: return "too_many_waypoints";
        case ValidationError::invalid_limits: return "invalid_limits";
        case ValidationError::invalid_coordinate_frame: return "invalid_coordinate_frame";
        case ValidationError::invalid_altitude_reference: return "invalid_altitude_reference";
        case ValidationError::invalid_launch: return "invalid_launch";
        case ValidationError::missing_coordinate: return "missing_coordinate";
        case ValidationError::partial_coordinate: return "partial_coordinate";
        case ValidationError::non_finite_coordinate: return "non_finite_coordinate";
        case ValidationError::latitude_out_of_range: return "latitude_out_of_range";
        case ValidationError::longitude_out_of_range: return "longitude_out_of_range";
        case ValidationError::missing_altitude: return "missing_altitude";
        case ValidationError::unexpected_altitude: return "unexpected_altitude";
        case ValidationError::altitude_out_of_range: return "altitude_out_of_range";
        case ValidationError::invalid_position_mode: return "invalid_position_mode";
        case ValidationError::invalid_waypoint_sequence: return "invalid_waypoint_sequence";
        case ValidationError::invalid_action_order: return "invalid_action_order";
        case ValidationError::dwell_out_of_range: return "dwell_out_of_range";
        case ValidationError::total_dwell_exceeded: return "total_dwell_exceeded";
        case ValidationError::speed_out_of_range: return "speed_out_of_range";
        case ValidationError::stale_command: return "stale_command";
        case ValidationError::command_from_future: return "command_from_future";
    }
    return "unknown";
}

}  // namespace dc::flight
