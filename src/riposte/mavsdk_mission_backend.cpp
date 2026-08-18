#include "mavsdk_mission_backend.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "geo.hpp"
#include "trajectory.hpp"

namespace dc::flight {
namespace {

bool finite_position(const mavsdk::Telemetry::Position& position) noexcept
{
    return std::isfinite(position.latitude_deg)
        && std::isfinite(position.longitude_deg)
        && std::isfinite(position.absolute_altitude_m)
        && std::isfinite(position.relative_altitude_m)
        && position.latitude_deg >= -90.0 && position.latitude_deg <= 90.0
        && position.longitude_deg >= -180.0 && position.longitude_deg <= 180.0
        && !(position.latitude_deg == 0.0 && position.longitude_deg == 0.0);
}

bool finite_local_position(
    const mavsdk::Telemetry::PositionVelocityNed& position) noexcept
{
    return std::isfinite(position.position.north_m)
        && std::isfinite(position.position.east_m)
        && std::isfinite(position.position.down_m);
}

bool finite_positive(const double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

std::string flight_mode_name(const mavsdk::Telemetry::FlightMode mode)
{
    switch (mode) {
        case mavsdk::Telemetry::FlightMode::Ready: return "READY";
        case mavsdk::Telemetry::FlightMode::Takeoff: return "TAKEOFF";
        case mavsdk::Telemetry::FlightMode::Hold: return "HOLD";
        case mavsdk::Telemetry::FlightMode::Mission: return "MISSION";
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch: return "RTL";
        case mavsdk::Telemetry::FlightMode::Land: return "LAND";
        case mavsdk::Telemetry::FlightMode::Offboard: return "OFFBOARD";
        case mavsdk::Telemetry::FlightMode::FollowMe: return "FOLLOW_ME";
        case mavsdk::Telemetry::FlightMode::Manual: return "MANUAL";
        case mavsdk::Telemetry::FlightMode::Altctl: return "ALTCTL";
        case mavsdk::Telemetry::FlightMode::Posctl: return "POSCTL";
        case mavsdk::Telemetry::FlightMode::Acro: return "ACRO";
        case mavsdk::Telemetry::FlightMode::Stabilized: return "STABILIZED";
        case mavsdk::Telemetry::FlightMode::Rattitude: return "RATTITUDE";
        case mavsdk::Telemetry::FlightMode::Unknown: return "UNKNOWN";
    }
    return "UNKNOWN";
}

GlobalPosition global_position(const mavsdk::Telemetry::Position& position)
{
    return {
        position.latitude_deg,
        position.longitude_deg,
        static_cast<double>(position.absolute_altitude_m),
        static_cast<double>(position.relative_altitude_m),
    };
}

bullet_mission::TrajectorySetpoint stationary_setpoint(
    const bullet_mission::NedPoint& point, const double yaw_deg)
{
    bullet_mission::TrajectorySetpoint setpoint{};
    setpoint.position = point;
    setpoint.yaw_deg = yaw_deg;
    return setpoint;
}

}  // namespace

struct MavsdkMissionBackend::RouteContext {
    bullet_mission::NedPoint ground{};
    riposte_test::GeoPoint launch_global{};
    double takeoff_climb_m{};
    std::vector<std::optional<bullet_mission::NedPoint>> targets;
};

MavsdkMissionBackend::MavsdkMissionBackend(
    std::string vehicle_id,
    std::shared_ptr<mavsdk::System> system,
    mavsdk::Action& action,
    mavsdk::Telemetry& telemetry,
    const RcOwnershipProvider& ownership_provider,
    MavsdkMissionBackendConfig config)
    : vehicle_id_(std::move(vehicle_id)),
      system_(std::move(system)),
      action_(action),
      telemetry_(telemetry),
      ownership_provider_(ownership_provider),
      config_(config),
      flight_control_(
          system_,
          action_,
          telemetry_,
          config_.offboard,
          [this] {
              if (external_preemption_requested()) {
                  return true;
              }
              if (internal_failsafe_) {
                  return false;
              }
              if (low_battery_triggered()) {
                  low_battery_abort_ = true;
                  return true;
              }
              if (battery_telemetry_lost()) {
                  battery_loss_abort_ = true;
                  return true;
              }
              return false;
          },
          [](const std::string& detail) {
              std::cerr << "[mqtt-px4] " << detail << '\n';
          },
          {},
          {},
          &ownership_provider)
{
    if (vehicle_id_.empty() || !system_) {
        throw std::invalid_argument("vehicle id and MAVSDK system are required");
    }
    validate_config();
}

MavsdkMissionBackend::~MavsdkMissionBackend() = default;

std::optional<std::uint64_t>
MavsdkMissionBackend::mission_authorization_epoch() const noexcept
{
    return ownership_provider_.mission_authorization_epoch();
}

void MavsdkMissionBackend::validate_config() const
{
    if (!finite_positive(config_.cruise_speed_m_s)
        || !finite_positive(config_.vertical_speed_m_s)
        || !finite_positive(config_.horizontal_tolerance_m)
        || !finite_positive(config_.vertical_tolerance_floor_m)
        || !finite_positive(config_.vertical_tolerance_ratio)
        || !finite_positive(config_.arrival_speed_tolerance_m_s)
        || !finite_positive(config_.arrival_settle_s)
        || !finite_positive(config_.arrival_timeout_s)
        || !finite_positive(config_.health_timeout_s)
        || !finite_positive(config_.position_timeout_s)
        || !finite_positive(config_.battery_telemetry_timeout_s)
        || !finite_positive(config_.max_route_radius_m)
        || !finite_positive(config_.max_home_launch_horizontal_m)
        || !finite_positive(config_.max_home_launch_altitude_delta_m)
        || !std::isfinite(config_.preflight_min_battery_fraction)
        || !std::isfinite(config_.low_battery_rtl_fraction)
        || config_.preflight_min_battery_fraction <= 0.0
        || config_.preflight_min_battery_fraction > 1.0
        || config_.low_battery_rtl_fraction <= 0.0
        || config_.low_battery_rtl_fraction >= config_.preflight_min_battery_fraction) {
        throw std::invalid_argument("invalid MAVSDK mission backend configuration");
    }
}

bool MavsdkMissionBackend::external_preemption_requested() const noexcept
{
    return active_token_ != nullptr
        && active_token_->requested() != PreemptionRequest::none;
}

std::string MavsdkMissionBackend::preemption_detail() const
{
    return "preempted_by_"
        + std::string{preemption_request_name(
            active_token_ == nullptr
                ? PreemptionRequest::none
                : active_token_->requested())};
}

bool MavsdkMissionBackend::low_battery_triggered()
{
    const double fraction = flight_control_.battery_remaining_fraction();
    if (fraction < 0.0) {
        return false;
    }
    last_valid_battery_at_ = std::chrono::steady_clock::now();
    return fraction <= config_.low_battery_rtl_fraction;
}

bool MavsdkMissionBackend::battery_telemetry_lost()
{
    const double fraction = flight_control_.battery_remaining_fraction();
    if (fraction >= 0.0) {
        last_valid_battery_at_ = std::chrono::steady_clock::now();
        return false;
    }
    return last_valid_battery_at_.has_value()
        && std::chrono::duration<double>(
            std::chrono::steady_clock::now() - *last_valid_battery_at_).count()
            >= config_.battery_telemetry_timeout_s;
}

void MavsdkMissionBackend::check_route_abort()
{
    if (active_start_ownership_epoch_.has_value()
        && ownership_provider_.takeover_epoch()
            != *active_start_ownership_epoch_) {
        throw riposte_runtime::PilotOverrideDetected(
            "rc_position_epoch_changed_during_preflight");
    }
    if (external_preemption_requested()) {
        throw std::runtime_error(preemption_detail());
    }
    if (low_battery_triggered()) {
        low_battery_abort_ = true;
        throw std::runtime_error("low_battery_rtl_required");
    }
    if (battery_telemetry_lost()) {
        battery_loss_abort_ = true;
        throw std::runtime_error("battery_telemetry_lost_rtl_required");
    }
}

double MavsdkMissionBackend::vertical_tolerance_for(
    const double altitude_m) const noexcept
{
    return std::max(
        config_.vertical_tolerance_floor_m,
        altitude_m * config_.vertical_tolerance_ratio);
}

VehicleTelemetrySnapshot MavsdkMissionBackend::snapshot() noexcept
{
    VehicleTelemetrySnapshot snapshot;
    snapshot.vehicle_id = vehicle_id_;
    snapshot.occurred_at = std::chrono::system_clock::now();
    try {
        snapshot.connected = system_->is_connected();
        if (!snapshot.connected) {
            snapshot.flight_mode = "DISCONNECTED";
            return snapshot;
        }

        snapshot.armed = telemetry_.armed();
        snapshot.in_air = telemetry_.in_air();
        const RcOwnershipSnapshot ownership =
            flight_control_.ownership_snapshot();
        snapshot.control_owner = ownership.owner;
        snapshot.ownership_switch = ownership.switch_position;
        snapshot.ownership_pwm = ownership.raw_pwm;
        snapshot.arm_switch = ownership.arm_switch.position;
        snapshot.arm_switch_pwm = ownership.arm_switch.raw_pwm;
        snapshot.ownership_epoch = ownership.takeover_epoch;
        const mavsdk::Telemetry::Health health = telemetry_.health();
        const mavsdk::Telemetry::Position position = telemetry_.position();
        const mavsdk::Telemetry::Position home = telemetry_.home();
        snapshot.global_position_valid =
            health.is_global_position_ok && finite_position(position);
        snapshot.home_valid = health.is_home_position_ok && finite_position(home);
        if (finite_position(position)) {
            snapshot.position = global_position(position);
        }
        if (finite_position(home)) {
            snapshot.home_position = global_position(home);
        }

        const mavsdk::Telemetry::Battery battery = telemetry_.battery();
        if (std::isfinite(battery.remaining_percent)
            && battery.remaining_percent >= 0.0F
            && battery.remaining_percent <= 100.0F) {
            snapshot.battery_percent =
                static_cast<double>(battery.remaining_percent);
        }
        if (std::isfinite(battery.voltage_v)) {
            snapshot.battery_voltage_v = static_cast<double>(battery.voltage_v);
        }
        if (std::isfinite(battery.temperature_degc)) {
            snapshot.battery_temperature_c =
                static_cast<double>(battery.temperature_degc);
        }

        const double heading = telemetry_.heading().heading_deg;
        if (std::isfinite(heading)) {
            snapshot.heading_deg = heading;
        }
        const auto local = telemetry_.position_velocity_ned();
        if (std::isfinite(local.velocity.north_m_s)
            && std::isfinite(local.velocity.east_m_s)) {
            snapshot.horizontal_speed_m_s = std::hypot(
                static_cast<double>(local.velocity.north_m_s),
                static_cast<double>(local.velocity.east_m_s));
        }
        snapshot.flight_mode = flight_mode_name(telemetry_.flight_mode());

        // HOLD is the deliberate ground handback position. Selecting OFFBOARD
        // on the transmitter before START bypasses the companion's required
        // current-position priming and, on the field airframe, also prevents a
        // usable navigation lock. The Pi enters Offboard itself only after the
        // new session has a fresh setpoint stream.
        snapshot.ready_for_mission = ownership.fresh
            && ownership.owner == ControlOwner::pi
            && ownership.switch_position == OwnershipSwitch::hold
            && ownership.arm_switch.fresh
            && ownership.arm_switch.position == ArmSwitchPosition::disarm
            && !snapshot.armed
            && !snapshot.in_air
            && telemetry_.landed_state()
                == mavsdk::Telemetry::LandedState::OnGround
            && !flight_control_.watchdog_triggered();
        if (snapshot.ready_for_mission) {
            // A completed manual landing plus CH6=HOLD is the explicit
            // handback. The next START still recreates and primes a brand-new
            // Offboard plugin before arming; clearing this command gate here
            // only lets the server show that the vehicle is ready again.
            pilot_override_latched_.store(false, std::memory_order_release);
        }
    } catch (...) {
        snapshot.connected = false;
        snapshot.home_valid = false;
        snapshot.global_position_valid = false;
        snapshot.control_owner = ControlOwner::unknown;
        snapshot.ownership_switch = OwnershipSwitch::unknown;
        snapshot.arm_switch = ArmSwitchPosition::unknown;
        snapshot.ready_for_mission = false;
        snapshot.flight_mode = "TELEMETRY_ERROR";
    }
    return snapshot;
}

void MavsdkMissionBackend::wait_for_vehicle_health()
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration<double>(config_.health_timeout_s);
    const auto poll =
        std::chrono::duration<double>(config_.offboard.command_poll_s);
    while (std::chrono::steady_clock::now() < deadline) {
        check_route_abort();
        if (telemetry_.health_all_ok()) {
            return;
        }
        std::this_thread::sleep_for(poll);
    }
    throw std::runtime_error("vehicle_health_check_timed_out");
}

void MavsdkMissionBackend::check_preflight_battery()
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration<double>(config_.battery_telemetry_timeout_s);
    const auto poll =
        std::chrono::duration<double>(config_.offboard.command_poll_s);
    while (std::chrono::steady_clock::now() < deadline) {
        if (external_preemption_requested()) {
            throw std::runtime_error(preemption_detail());
        }
        const double fraction = flight_control_.battery_remaining_fraction();
        if (fraction >= 0.0) {
            last_valid_battery_at_ = std::chrono::steady_clock::now();
            if (fraction < config_.preflight_min_battery_fraction) {
                throw std::runtime_error("preflight_battery_below_minimum");
            }
            return;
        }
        std::this_thread::sleep_for(poll);
    }
    throw std::runtime_error("preflight_battery_telemetry_timed_out");
}

MavsdkMissionBackend::RouteContext MavsdkMissionBackend::prepare_route(
    const RoutePlan& route)
{
    if (telemetry_.armed() || telemetry_.in_air()) {
        throw std::runtime_error("new_mission_requires_landed_disarmed_vehicle");
    }
    wait_for_vehicle_health();
    check_preflight_battery();

    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration<double>(config_.position_timeout_s);
    const auto poll =
        std::chrono::duration<double>(config_.offboard.command_poll_s);
    mavsdk::Telemetry::PositionVelocityNed local{};
    mavsdk::Telemetry::Position current{};
    mavsdk::Telemetry::Position home{};
    bool ready = false;
    while (std::chrono::steady_clock::now() < deadline) {
        check_route_abort();
        const mavsdk::Telemetry::Health health = telemetry_.health();
        local = telemetry_.position_velocity_ned();
        current = telemetry_.position();
        home = telemetry_.home();
        if (health.is_global_position_ok
            && health.is_home_position_ok
            && finite_local_position(local)
            && finite_position(current)
            && finite_position(home)) {
            ready = true;
            break;
        }
        std::this_thread::sleep_for(poll);
    }
    if (!ready) {
        throw std::runtime_error("local_global_or_home_position_not_valid");
    }

    const riposte_test::GeoPoint launch_geo{
        current.latitude_deg, current.longitude_deg};
    const riposte_test::GeoPoint home_geo{
        home.latitude_deg, home.longitude_deg};
    const double home_horizontal_m =
        riposte_test::geodetic_distance_m(launch_geo, home_geo);
    const double home_altitude_delta_m = std::abs(
        static_cast<double>(current.absolute_altitude_m)
        - static_cast<double>(home.absolute_altitude_m));
    if (home_horizontal_m > config_.max_home_launch_horizontal_m
        || home_altitude_delta_m > config_.max_home_launch_altitude_delta_m) {
        throw std::runtime_error("px4_home_does_not_match_current_launch_position");
    }

    RouteContext context;
    context.ground = {
        static_cast<double>(local.position.north_m),
        static_cast<double>(local.position.east_m),
        static_cast<double>(local.position.down_m),
    };
    context.launch_global = launch_geo;
    context.targets.resize(route.waypoints.size());

    const double current_absolute_m =
        static_cast<double>(current.absolute_altitude_m);
    const double home_absolute_m = static_cast<double>(home.absolute_altitude_m);
    for (std::size_t index = 0U; index < route.waypoints.size(); ++index) {
        const GeoWaypoint& waypoint = route.waypoints[index];
        if (waypoint.action == WaypointAction::takeoff) {
            const double desired_absolute_m =
                home_absolute_m + *waypoint.relative_altitude_m;
            context.takeoff_climb_m = desired_absolute_m - current_absolute_m;
            if (!finite_positive(context.takeoff_climb_m)) {
                throw std::runtime_error("takeoff_altitude_not_above_launch_position");
            }
            context.targets[index] = bullet_mission::NedPoint{
                context.ground.north_m,
                context.ground.east_m,
                context.ground.down_m - context.takeoff_climb_m,
            };
            continue;
        }
        if (is_terminal_action(waypoint.action)) {
            continue;
        }

        const riposte_test::GeoPoint waypoint_geo{
            *waypoint.latitude_deg, *waypoint.longitude_deg};
        const riposte_test::NedOffset offset =
            riposte_test::geodetic_to_ned_offset(launch_geo, waypoint_geo);
        const double radius_m = std::hypot(offset.north_m, offset.east_m);
        if (radius_m > config_.max_route_radius_m) {
            throw std::runtime_error("route_exceeds_configured_horizontal_radius");
        }
        const double desired_absolute_m =
            home_absolute_m + *waypoint.relative_altitude_m;
        context.targets[index] = bullet_mission::NedPoint{
            context.ground.north_m + offset.north_m,
            context.ground.east_m + offset.east_m,
            context.ground.down_m - (desired_absolute_m - current_absolute_m),
        };
    }
    flight_control_.set_flight_volume(context.ground, config_.max_route_radius_m);
    return context;
}

double MavsdkMissionBackend::leg_speed_m_s(
    const RoutePlan& route, const GeoWaypoint& waypoint) const noexcept
{
    const double requested = waypoint.speed_m_s.value_or(
        route.cruise_speed_m_s.value_or(config_.cruise_speed_m_s));
    return finite_positive(requested) ? requested : config_.cruise_speed_m_s;
}

void MavsdkMissionBackend::move_to(
    const std::string& phase,
    const bullet_mission::NedPoint& start,
    const bullet_mission::NedPoint& finish,
    const double speed_m_s)
{
    const double delta_north = finish.north_m - start.north_m;
    const double delta_east = finish.east_m - start.east_m;
    const double delta_down = finish.down_m - start.down_m;
    const double horizontal_m = std::hypot(delta_north, delta_east);
    const double vertical_m = std::abs(delta_down);
    const double horizontal_time_s = horizontal_m / speed_m_s;
    const double vertical_time_s = vertical_m / config_.vertical_speed_m_s;
    const double duration_s = std::max(horizontal_time_s, vertical_time_s);
    const double yaw_deg = bullet_mission::line_heading_deg(start, finish);
    if (duration_s <= 1.0e-6) {
        static_cast<void>(flight_control_.send_setpoint(
            phase, stationary_setpoint(finish, yaw_deg)));
        return;
    }

    const auto period = std::chrono::duration<double>(
        1.0 / config_.offboard.setpoint_rate_hz);
    const auto begin = std::chrono::steady_clock::now();
    auto next = begin;
    while (true) {
        check_route_abort();
        const double elapsed_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
        const double progress = elapsed_s / duration_s;
        if (progress >= 1.0) {
            break;
        }
        const double p = bullet_mission::clamp01(progress);
        bullet_mission::TrajectorySetpoint setpoint{};
        setpoint.position = {
            start.north_m + p * delta_north,
            start.east_m + p * delta_east,
            start.down_m + p * delta_down,
        };
        setpoint.velocity = {
            delta_north / duration_s,
            delta_east / duration_s,
            delta_down / duration_s,
        };
        setpoint.yaw_deg = yaw_deg;
        static_cast<void>(flight_control_.send_setpoint(phase, setpoint));
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
    }
    static_cast<void>(flight_control_.send_setpoint(
        phase, stationary_setpoint(finish, yaw_deg)));
}

void MavsdkMissionBackend::settle_at(
    const std::string& phase,
    const bullet_mission::NedPoint& target,
    const double yaw_deg,
    const double altitude_m)
{
    const double vertical_tolerance_m = vertical_tolerance_for(altitude_m);
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration<double>(config_.arrival_timeout_s);
    const auto period = std::chrono::duration<double>(
        1.0 / config_.offboard.setpoint_rate_hz);
    std::optional<std::chrono::steady_clock::time_point> within_tolerance_since;
    auto next = std::chrono::steady_clock::now();
    const auto setpoint = stationary_setpoint(target, yaw_deg);

    while (std::chrono::steady_clock::now() < deadline) {
        check_route_abort();
        const auto state = flight_control_.send_setpoint(phase, setpoint);
        const bullet_mission::NedPoint actual{
            static_cast<double>(state.position.north_m),
            static_cast<double>(state.position.east_m),
            static_cast<double>(state.position.down_m),
        };
        const bool horizontal_ok = bullet_mission::horizontal_distance(actual, target)
            <= config_.horizontal_tolerance_m;
        const bool vertical_ok =
            std::abs(actual.down_m - target.down_m) <= vertical_tolerance_m;
        const bool speed_ok = std::hypot(
            static_cast<double>(state.velocity.north_m_s),
            static_cast<double>(state.velocity.east_m_s))
            <= config_.arrival_speed_tolerance_m_s;
        if (horizontal_ok && vertical_ok && speed_ok) {
            if (!within_tolerance_since.has_value()) {
                within_tolerance_since = std::chrono::steady_clock::now();
            }
            if (std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - *within_tolerance_since).count()
                >= config_.arrival_settle_s) {
                return;
            }
        } else {
            within_tolerance_since.reset();
        }
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
    }
    throw std::runtime_error("arrival_settling_timed_out_" + phase);
}

void MavsdkMissionBackend::hold_at(
    const std::string& phase,
    const bullet_mission::NedPoint& target,
    const double yaw_deg,
    const std::chrono::seconds duration)
{
    if (duration <= std::chrono::seconds::zero()) {
        return;
    }
    const auto period = std::chrono::duration<double>(
        1.0 / config_.offboard.setpoint_rate_hz);
    const auto deadline = std::chrono::steady_clock::now() + duration;
    auto next = std::chrono::steady_clock::now();
    const auto setpoint = stationary_setpoint(target, yaw_deg);
    while (std::chrono::steady_clock::now() < deadline) {
        check_route_abort();
        static_cast<void>(flight_control_.send_setpoint(phase, setpoint));
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
    }
}

FlightOperationResult MavsdkMissionBackend::execute_route(
    const RemoteCommand& command)
{
    if (!command.route.has_value()) {
        throw std::runtime_error("execute_mission_route_missing");
    }
    const RoutePlan& route = *command.route;
    if (route.waypoints.size() == 1U) {
        const WaypointAction action = route.waypoints.front().action;
        if (action == WaypointAction::land) {
            if (telemetry_.armed() || telemetry_.in_air()) {
                flight_control_.land_and_wait();
            }
            return {ExecutionOutcome::completed, "landed"};
        }
        if (action == WaypointAction::return_to_launch) {
            flight_control_.return_to_launch_and_wait();
            return {ExecutionOutcome::completed, "rtl_landed"};
        }
    }

    RouteContext context = prepare_route(route);
    bullet_mission::NedPoint current = context.ground;
    bool terminal_executed = false;
    for (std::size_t index = 0U; index < route.waypoints.size(); ++index) {
        check_route_abort();
        const GeoWaypoint& waypoint = route.waypoints[index];
        const std::string phase = "WP" + std::to_string(waypoint.sequence);
        if (waypoint.action == WaypointAction::takeoff) {
            flight_control_.takeoff_to(
                context.ground, context.takeoff_climb_m, phase + "_TAKEOFF");
            current = *context.targets[index];
            settle_at(
                phase + "_SETTLE",
                current,
                0.0,
                *waypoint.relative_altitude_m);
            continue;
        }
        if (waypoint.action == WaypointAction::land) {
            flight_control_.land_and_wait();
            terminal_executed = true;
            break;
        }
        if (waypoint.action == WaypointAction::return_to_launch) {
            flight_control_.return_to_launch_and_wait();
            terminal_executed = true;
            break;
        }

        if (!context.targets[index].has_value()) {
            throw std::runtime_error("navigation_waypoint_target_missing");
        }
        const bullet_mission::NedPoint target = *context.targets[index];
        const double yaw_deg = bullet_mission::line_heading_deg(current, target);
        move_to(
            phase + "_MOVE", current, target, leg_speed_m_s(route, waypoint));
        settle_at(
            phase + "_SETTLE",
            target,
            yaw_deg,
            *waypoint.relative_altitude_m);
        hold_at(
            phase + "_DWELL",
            target,
            yaw_deg,
            std::chrono::seconds{
                static_cast<std::chrono::seconds::rep>(waypoint.dwell_seconds)});
        current = target;
    }
    if (!terminal_executed) {
        throw std::runtime_error("mission_finished_without_land_or_rtl");
    }
    return {ExecutionOutcome::completed, "mission_terminal_action_completed"};
}

FlightOperationResult MavsdkMissionBackend::execute_safety(
    const RemoteCommand& command)
{
    switch (command.type) {
        case RemoteCommandType::stop_mission:
            flight_control_.stop_and_hold();
            return {ExecutionOutcome::completed, "mission_stopped_in_hold"};
        case RemoteCommandType::return_to_launch:
            flight_control_.return_to_launch_and_wait();
            return {ExecutionOutcome::completed, "rtl_landed"};
        case RemoteCommandType::emergency_stop:
            // For a multirotor, motor termination in air is a crash command.
            // EMERGENCY_STOP therefore uses the verified immediate Land path.
            flight_control_.emergency_land_and_wait();
            if (flight_control_.in_air()) {
                return {ExecutionOutcome::failed, "emergency_land_not_confirmed"};
            }
            return {ExecutionOutcome::completed, "emergency_land_completed"};
        case RemoteCommandType::execute_mission:
            break;
    }
    return {ExecutionOutcome::failed, "not_a_safety_command"};
}

FlightOperationResult MavsdkMissionBackend::execute(
    const RemoteCommand& command, const PreemptionToken& token)
{
    // A new route is only ever a new ground session. There is intentionally no
    // airborne resume: the previous mission terminalized when the pilot took
    // over, and accepting a new route while armed used to fall through the
    // generic exception path and command an emergency Land.
    if (command.type == RemoteCommandType::execute_mission) {
        const RcOwnershipSnapshot ownership =
            flight_control_.ownership_snapshot();
        if (!dispatch_ownership_epoch_matches(
                command.dispatch_ownership_epoch,
                ownership.takeover_epoch)) {
            return {
                ExecutionOutcome::failed,
                "mission_start_invalidated_by_rc_takeover"};
        }
        if (!ownership.fresh
            || ownership.owner != ControlOwner::pi
            || ownership.switch_position != OwnershipSwitch::hold
            || !ownership.arm_switch.fresh
            || ownership.arm_switch.position
                != ArmSwitchPosition::disarm) {
            return {
                ExecutionOutcome::failed,
                "mission_start_requires_fresh_rc_hold_and_disarm"};
        }
        if (!flight_control_.ground_disarmed()) {
            return {
                ExecutionOutcome::failed,
                "new_mission_requires_landed_disarmed_vehicle"};
        }
        try {
            // This also destroys any cached Offboard sender from an earlier
            // session and creates a clean plugin. It is permitted only after
            // the positive OnGround+DISARMED check above.
            flight_control_.prepare_for_new_ground_mission(
                *command.dispatch_ownership_epoch);
            active_start_ownership_epoch_ =
                *command.dispatch_ownership_epoch;
        } catch (const std::exception& error) {
            return {
                ExecutionOutcome::failed,
                std::string{"new_mission_session_prepare_failed_"}
                    + error.what()};
        } catch (...) {
            return {
                ExecutionOutcome::failed,
                "new_mission_session_prepare_failed_unknown"};
        }
        pilot_override_latched_.store(false, std::memory_order_release);
    }

    // Safety commands remain available to the Pi while airborne, but never
    // after an explicit or ambiguous pilot takeover. START has already passed
    // the stricter ground/HOLD gate above.
    if (pilot_override_latched_.load(std::memory_order_acquire)) {
        return {ExecutionOutcome::failed, "pilot_override_active"};
    }
    if (flight_control_.pilot_in_control()) {
        pilot_override_latched_.store(true, std::memory_order_release);
        return {ExecutionOutcome::failed, "pilot_override_active"};
    }
    active_token_ = &token;
    if (command.type != RemoteCommandType::execute_mission) {
        active_start_ownership_epoch_.reset();
    }
    internal_failsafe_ = false;
    low_battery_abort_ = false;
    battery_loss_abort_ = false;
    // Safety commands do not create a new Offboard plugin, but they still must
    // not inherit a completed prior operation's watchdog latch.
    flight_control_.reset_watchdog_latch();
    last_valid_battery_at_ = std::chrono::steady_clock::now();
    try {
        FlightOperationResult result = command.type == RemoteCommandType::execute_mission
            ? execute_route(command)
            : execute_safety(command);
        active_token_ = nullptr;
        active_start_ownership_epoch_.reset();
        return result;
    } catch (const riposte_runtime::PilotOverrideDetected& override_error) {
        // Stop MAVSDK's cached setpoint sender by destroying its plugin, not
        // by Offboard::stop(): stop() commands HOLD and would take the mode
        // back from the pilot. yield_to_pilot also invalidates and joins the
        // exact watchdog generation before this operation is terminalized.
        flight_control_.yield_to_pilot();
        pilot_override_latched_.store(true, std::memory_order_release);
        active_token_ = nullptr;
        active_start_ownership_epoch_.reset();
        return {ExecutionOutcome::failed, override_error.what()};
    } catch (const std::exception& error) {
        std::string detail = error.what();
        // Close the sampling race between the last check_abort() and a MAVSDK
        // failure. Once CH6 says POSITION (or mode is pilot-like while RC is
        // unknown), no failure path may send Land/RTL/Hold/Disarm.
        if (flight_control_.pilot_in_control()) {
            flight_control_.yield_to_pilot();
            pilot_override_latched_.store(true, std::memory_order_release);
            active_token_ = nullptr;
            active_start_ownership_epoch_.reset();
            return {ExecutionOutcome::failed, "pilot_override_during_" + detail};
        }
        if (external_preemption_requested()) {
            detail = preemption_detail();
            // Prime setpoints start MAVSDK's cached sender before
            // Offboard::start() succeeds. The preempting safety command runs
            // next, but the obsolete sender must be removed immediately and
            // without injecting a mode change here.
            flight_control_.cancel_for_external_preemption();
            active_token_ = nullptr;
            active_start_ownership_epoch_.reset();
            return {ExecutionOutcome::failed, std::move(detail)};
        }

        if (low_battery_abort_ || battery_loss_abort_) {
            const std::string cause = low_battery_abort_
                ? "low_battery_rtl"
                : "battery_telemetry_lost_rtl";
            internal_failsafe_ = true;
            try {
                flight_control_.return_to_launch_and_wait();
                detail = cause + "_landed";
            } catch (const std::exception& rtl_error) {
                detail = cause + "_failed_" + rtl_error.what();
                if (!external_preemption_requested()) {
                    flight_control_.emergency_land_and_wait();
                }
            }
            internal_failsafe_ = false;
            active_token_ = nullptr;
            active_start_ownership_epoch_.reset();
            return {ExecutionOutcome::failed, std::move(detail)};
        }

        internal_failsafe_ = true;
        flight_control_.emergency_land_and_wait();
        internal_failsafe_ = false;
        active_token_ = nullptr;
        active_start_ownership_epoch_.reset();
        return {ExecutionOutcome::failed, std::move(detail)};
    } catch (...) {
        if (flight_control_.pilot_in_control()) {
            flight_control_.yield_to_pilot();
            pilot_override_latched_.store(true, std::memory_order_release);
            active_token_ = nullptr;
            active_start_ownership_epoch_.reset();
            return {ExecutionOutcome::failed, "pilot_override_during_unknown_error"};
        }
        internal_failsafe_ = true;
        flight_control_.emergency_land_and_wait();
        internal_failsafe_ = false;
        active_token_ = nullptr;
        active_start_ownership_epoch_.reset();
        return {ExecutionOutcome::failed, "unknown_mavsdk_backend_error"};
    }
}

CommandResult MavsdkMissionBackend::recover_after_restart(
    const PreemptionToken& token)
{
    active_token_ = &token;
    internal_failsafe_ = true;
    try {
        flight_control_.recover_after_restart();
        internal_failsafe_ = false;
        active_token_ = nullptr;
        return {true, "vehicle_safe_after_restart"};
    } catch (const riposte_runtime::PilotOverrideDetected& override_error) {
        // Recovery succeeded in the only sense that matters: the aircraft is
        // under a pilot and this process has stopped trying to fly it. Report
        // success so the executor opens, then refuse every command that
        // arrives until it is on the ground and disarmed.
        flight_control_.yield_to_pilot();
        pilot_override_latched_.store(true, std::memory_order_release);
        internal_failsafe_ = false;
        active_token_ = nullptr;
        return {true, override_error.what()};
    } catch (const std::exception& error) {
        internal_failsafe_ = false;
        active_token_ = nullptr;
        return {false, error.what()};
    } catch (...) {
        internal_failsafe_ = false;
        active_token_ = nullptr;
        return {false, "unknown_vehicle_recovery_error"};
    }
}

FlightOperationResult MavsdkMissionBackend::shutdown_safely(
    const PreemptionToken&)
{
    active_token_ = nullptr;
    internal_failsafe_ = true;
    try {
        // Stopping this service must not fly the aircraft home when somebody
        // is holding the transmitter. It is the reason an operator was told
        // never to restart the agent during manual flight; that should be a
        // property of the code, not an instruction people have to remember.
        if (pilot_override_latched_.load(std::memory_order_acquire)
            || flight_control_.pilot_in_control()) {
            internal_failsafe_ = false;
            return {
                ExecutionOutcome::completed, "executor_shutdown_pilot_in_control"};
        }
        if (flight_control_.in_air() || flight_control_.armed()) {
            flight_control_.return_to_launch_and_wait();
        }
        internal_failsafe_ = false;
        return {ExecutionOutcome::completed, "executor_shutdown_vehicle_safe"};
    } catch (const std::exception& error) {
        flight_control_.emergency_land_and_wait();
        internal_failsafe_ = false;
        return {ExecutionOutcome::failed, error.what()};
    } catch (...) {
        flight_control_.emergency_land_and_wait();
        internal_failsafe_ = false;
        return {ExecutionOutcome::failed, "unknown_shutdown_recovery_error"};
    }
}

}  // namespace dc::flight
