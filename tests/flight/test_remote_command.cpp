#include "dc/flight/remote_command.hpp"
#include "dc/flight/telemetry.hpp"
#include "test_support.hpp"

#include <chrono>
#include <limits>
#include <optional>

namespace {

using dc::flight::CommandFreshnessLimits;
using dc::flight::GeoWaypoint;
using dc::flight::RemoteCommand;
using dc::flight::RemoteCommandType;
using dc::flight::RouteLimits;
using dc::flight::RoutePlan;
using dc::flight::ValidationError;
using dc::flight::WaypointAction;
using dc::flight::WaypointPositionMode;

constexpr RouteLimits kLimits{10U, 2.0, 120.0, 60U, 120U, 0.5, 8.0};
constexpr CommandFreshnessLimits kFreshness{
    std::chrono::seconds{300},
    std::chrono::seconds{10},
};
const auto kNow =
    std::chrono::system_clock::time_point{std::chrono::seconds{1'800'000'000}};

GeoWaypoint waypoint(
    const std::uint32_t sequence,
    const WaypointAction action,
    const WaypointPositionMode mode,
    const std::optional<double> latitude,
    const std::optional<double> longitude,
    const std::optional<double> altitude,
    const std::uint32_t dwell = 0U)
{
    GeoWaypoint result;
    result.sequence = sequence;
    result.action = action;
    result.position_mode = mode;
    result.latitude_deg = latitude;
    result.longitude_deg = longitude;
    result.relative_altitude_m = altitude;
    result.dwell_seconds = dwell;
    return result;
}

RoutePlan valid_route()
{
    RoutePlan route;
    route.coordinate_frame = "ABSOLUTE";
    route.altitude_reference = "HOME_RELATIVE";
    route.launch.mode = "CURRENT_POSITION";
    route.launch.takeoff_altitude_m = 10.0;
    route.waypoints = {
        waypoint(1U, WaypointAction::takeoff, WaypointPositionMode::current,
                 std::nullopt, std::nullopt, 10.0),
        waypoint(2U, WaypointAction::move, WaypointPositionMode::waypoint,
                 36.6615082, 126.3422854, 25.0),
        waypoint(3U, WaypointAction::hover, WaypointPositionMode::waypoint,
                 36.6617082, 126.3424854, 25.0, 15U),
        waypoint(4U, WaypointAction::return_to_launch, WaypointPositionMode::home,
                 std::nullopt, std::nullopt, std::nullopt),
    };
    return route;
}

RemoteCommand valid_execute()
{
    RemoteCommand command;
    command.command_id = "2ac105b7-2233-4455-aabb-66778899aabb";
    command.vehicle_id = "DRONE-01";
    command.type = RemoteCommandType::execute_mission;
    command.mission_id = "f7c5c855-6677-4889-aabb-112233445566";
    command.route = valid_route();
    command.issued_by = "ksy";
    command.issued_at = kNow;
    command.expires_at = kNow + std::chrono::seconds{10};
    return command;
}

void test_prototype_v2_route_is_valid()
{
    const auto result = dc::flight::validate_command(valid_execute(), kLimits);
    test_support::check(result.ok(), "prototype-v2 TAKEOFF/MOVE/HOVER/RTL route is valid");
}

void test_control_actions_allow_omitted_position_and_altitude()
{
    RemoteCommand command = valid_execute();
    RoutePlan route;
    route.coordinate_frame = "ABSOLUTE";
    route.altitude_reference = "HOME_RELATIVE";
    route.launch.mode = "CURRENT_POSITION";
    route.waypoints = {
        waypoint(1U, WaypointAction::return_to_launch, WaypointPositionMode::home,
                 std::nullopt, std::nullopt, std::nullopt),
    };
    command.route = route;
    test_support::check(
        dc::flight::validate_command(command, kLimits).ok(),
        "standalone RTL is a valid control-only mission");

    command.route->waypoints[0].relative_altitude_m = 3.0;
    test_support::check(
        dc::flight::validate_command(command, kLimits).error
            == ValidationError::unexpected_altitude,
        "RTL rejects an altitude that would prevent grounding");
}

void test_route_shape_and_launch_binding()
{
    RemoteCommand command = valid_execute();
    command.route->waypoints[1].sequence = 3U;
    test_support::check(
        dc::flight::validate_command(command, kLimits).error
            == ValidationError::invalid_waypoint_sequence,
        "seq must equal array index plus one");

    command = valid_execute();
    command.route->waypoints[1].position_mode = WaypointPositionMode::current;
    test_support::check(
        dc::flight::validate_command(command, kLimits).error
            == ValidationError::invalid_position_mode,
        "MOVE must use its absolute waypoint position");

    command = valid_execute();
    command.route->launch.takeoff_altitude_m = 11.0;
    test_support::check(
        dc::flight::validate_command(command, kLimits).error
            == ValidationError::invalid_launch,
        "launch altitude must match the TAKEOFF step");

    command = valid_execute();
    command.route->waypoints[1].latitude_deg.reset();
    test_support::check(
        dc::flight::validate_command(command, kLimits).error
            == ValidationError::partial_coordinate,
        "one-sided lat/lng is rejected");
}

void test_server_dwell_policy_is_bounded()
{
    RemoteCommand command = valid_execute();
    command.route->waypoints[1].dwell_seconds = 1U;
    test_support::check(
        dc::flight::validate_command(command, kLimits).ok(),
        "prototype-v2 permits bounded dwell on a MOVE step");

    command.route->waypoints[1].dwell_seconds = 61U;
    test_support::check(
        dc::flight::validate_command(command, kLimits).error
            == ValidationError::dwell_out_of_range,
        "per-step dwell limit is enforced");

    command = valid_execute();
    const RouteLimits total_limit{10U, 2.0, 120.0, 60U, 10U, 0.5, 8.0};
    test_support::check(
        dc::flight::validate_command(command, total_limit).error
            == ValidationError::total_dwell_exceeded,
        "total dwell limit is enforced without unsigned underflow");
}

void test_safety_commands_cannot_smuggle_routes()
{
    RemoteCommand command = valid_execute();
    command.type = RemoteCommandType::stop_mission;
    command.expires_at.reset();
    test_support::check(
        dc::flight::validate_command(command, kLimits).error
            == ValidationError::unexpected_route,
        "STOP cannot smuggle a replacement route");

    command.route.reset();
    test_support::check(
        dc::flight::validate_command(command, kLimits).ok(),
        "STOP with mission metadata and no route is valid");
}

void test_freshness_and_expiry()
{
    RemoteCommand command = valid_execute();
    command.issued_at = kNow - std::chrono::seconds{301};
    test_support::check(
        dc::flight::validate_freshness(command, kFreshness, kNow).error
            == ValidationError::stale_command,
        "stale command is rejected");

    command = valid_execute();
    command.issued_at = kNow + std::chrono::seconds{11};
    command.expires_at = kNow + std::chrono::seconds{21};
    test_support::check(
        dc::flight::validate_freshness(command, kFreshness, kNow).error
            == ValidationError::command_from_future,
        "command beyond clock skew is rejected");

    command = valid_execute();
    command.issued_at = kNow - std::chrono::seconds{1};
    command.expires_at = kNow;
    test_support::check(
        dc::flight::validate_freshness(command, kFreshness, kNow).error
            == ValidationError::expired_command,
        "expiresAt is enforced independently of max age");
}

}  // namespace

void test_horizontal_distance_from_launch()
{
    // The geofence supervisor compares this against a radius in metres, so a
    // wrong scale factor would silently disable containment rather than fail.
    const dc::flight::GlobalPosition home{35.338500, 129.030500, 10.0, 0.0};
    dc::flight::GlobalPosition same = home;
    test_support::check(
        dc::flight::horizontal_distance_m(home, same) < 0.001,
        "distance to the launch point itself is zero");

    // 0.001 degrees of latitude is ~111.19 m anywhere on the globe.
    dc::flight::GlobalPosition north = home;
    north.latitude_deg += 0.001;
    test_support::check_close(
        dc::flight::horizontal_distance_m(home, north), 111.19, 0.5,
        "a thousandth of a degree north is ~111 m");

    // Altitude must not leak into a horizontal limit: a vehicle climbing
    // straight up has not left the fence.
    dc::flight::GlobalPosition above = home;
    above.absolute_altitude_m += 500.0;
    above.relative_altitude_m += 500.0;
    test_support::check(
        dc::flight::horizontal_distance_m(home, above) < 0.001,
        "climbing does not count against a horizontal radius");

    // The value the aircraft actually runs: 50 m must sit between these.
    dc::flight::GlobalPosition inside = home;
    inside.latitude_deg += 0.0004;   // ~44 m
    dc::flight::GlobalPosition outside = home;
    outside.latitude_deg += 0.0006;  // ~67 m
    test_support::check(
        dc::flight::horizontal_distance_m(home, inside) < 50.0
            && dc::flight::horizontal_distance_m(home, outside) > 50.0,
        "the configured 50 m radius separates these two points");
}

int main()
{
    test_prototype_v2_route_is_valid();
    test_control_actions_allow_omitted_position_and_altitude();
    test_route_shape_and_launch_binding();
    test_server_dwell_policy_is_bounded();
    test_safety_commands_cannot_smuggle_routes();
    test_freshness_and_expiry();
    test_horizontal_distance_from_launch();
    return test_support::report("remote_command_tests");
}
