// mqtt_px4_control - the AegisQ MQTT command path for exactly one PX4 vehicle.
//
// This translation unit is wiring. Every rule that matters lives behind a port:
// strict decoding in dc_mqtt_codec, bounded ingress in MqttCommandSource,
// durable UUID claim and ACK outbox in SqliteCommandLedger, serialization and
// preemption in PreemptiveMissionExecutor, and the PX4 mechanics in
// MavsdkMissionBackend. Nothing here re-implements any of that.
//
// REMOTE_FLIGHT_EXECUTE=0 keeps the process off PX4 completely: no MAVSDK
// connection is opened, no MAVLink is emitted, and a refusing backend answers
// every command with a truthful ACCEPTED -> FAILED round trip. That is the
// supported way to exercise the broker path before SITL and vehicle checks are
// done. The gate sits above the backend on purpose - MavsdkOffboardControl's
// restart recovery issues real disarm/RTL commands, so it must never be reached
// by a process that is only meant to observe.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "dc/flight/preemptive_mission_executor.hpp"
#include "dc/flight/remote_mission_coordinator.hpp"
#include "dc/flight/telemetry.hpp"
#include "dc/mqtt/agent_config.hpp"
#include "dc/mqtt/mosquitto_session.hpp"
#include "dc/mqtt/mqtt_command_source.hpp"
#include "dc/mqtt/sqlite_command_ledger.hpp"

#include "mavsdk_mission_backend.hpp"
#include "mavsdk_rc_ownership_monitor.hpp"

namespace {

// Only a lock-free integer may be touched from a signal handler. The shutdown
// itself takes mutexes and joins threads, so it runs on the supervisor thread.
volatile std::sig_atomic_t g_stop_requested = 0;

extern "C" void on_signal(int) noexcept
{
    g_stop_requested = 1;
}

void log_line(const char* const level, const std::string& text) noexcept
{
    std::fprintf(stderr, "[%s] %s\n", level, text.c_str());
    std::fflush(stderr);
}

// Honours REMOTE_FLIGHT_EXECUTE=0. Every port method is implemented and none
// of them can reach MAVSDK, because this object never holds a MAVSDK handle.
class ObserveOnlyBackend final : public dc::flight::MissionBackend,
                                 public dc::flight::VehicleTelemetryProvider {
public:
    explicit ObserveOnlyBackend(std::string vehicle_id)
        : vehicle_id_(std::move(vehicle_id))
    {
    }

    dc::flight::CommandResult recover_after_restart(
        const dc::flight::PreemptionToken&) override
    {
        // The executor gates every dispatch behind a successful recovery, so
        // refusing here would make the observe-only mode untestable. Nothing
        // was recovered and the detail says so; see the unresolved-command
        // check in run() for why that stays safe.
        return {true, "observe_only_no_px4_recovery"};
    }

    dc::flight::FlightOperationResult execute(
        const dc::flight::RemoteCommand&,
        const dc::flight::PreemptionToken&) override
    {
        return {
            dc::flight::ExecutionOutcome::failed,
            "remote_flight_execute_disabled"};
    }

    dc::flight::FlightOperationResult shutdown_safely(
        const dc::flight::PreemptionToken&) override
    {
        return {
            dc::flight::ExecutionOutcome::completed, "observe_only_shutdown"};
    }

    [[nodiscard]] std::optional<std::uint64_t>
    mission_authorization_epoch() const noexcept override
    {
        // This backend owns no MAVSDK handles and can never initiate flight.
        // Keep observe-only commands flowing to its explicit terminal result.
        return 0U;
    }

    dc::flight::VehicleTelemetrySnapshot snapshot() noexcept override
    {
        dc::flight::VehicleTelemetrySnapshot snapshot;
        snapshot.vehicle_id = vehicle_id_;
        snapshot.occurred_at = std::chrono::system_clock::now();
        // A ground operator must not be able to mistake this process for a
        // live one, so the published flight mode names the gate.
        snapshot.flight_mode = "REMOTE_FLIGHT_DISABLED";
        return snapshot;
    }

private:
    std::string vehicle_id_;
};

struct Px4LinkConfig {
    std::string connection;
    double discovery_timeout_s{};
    std::uint8_t component_id{};
    double rate_position_velocity_ned_hz{};
    double rate_battery_hz{};
    double rate_position_hz{};
    double rate_rc_status_hz{};
};

[[nodiscard]] Px4LinkConfig load_px4_link_config()
{
    Px4LinkConfig config;
    config.connection = dc::mqtt::env_text("PX4_CONNECTION");
    config.discovery_timeout_s =
        dc::mqtt::env_real("PX4_DISCOVERY_TIMEOUT_S", 1.0, 600.0);
    // MAVSDK v3.17 defaults CompanionComputer to 195; the Riposte agent pins
    // 191 so ground requests addressed to 191 arrive. Keep it configurable and
    // keep the vehicle's two agents consistent.
    config.component_id = static_cast<std::uint8_t>(
        dc::mqtt::env_integer("PX4_COMPONENT_ID", 1, 255));
    config.rate_position_velocity_ned_hz = dc::mqtt::env_real(
        "PX4_RATE_POSITION_VELOCITY_NED_HZ", 1.0, 200.0);
    config.rate_battery_hz =
        dc::mqtt::env_real("PX4_RATE_BATTERY_HZ", 0.1, 50.0);
    config.rate_position_hz =
        dc::mqtt::env_real("PX4_RATE_POSITION_HZ", 0.1, 50.0);
    config.rate_rc_status_hz =
        dc::mqtt::env_real("PX4_RATE_RC_STATUS_HZ", 1.0, 50.0);
    return config;
}

[[nodiscard]] riposte_runtime::OffboardControlConfig load_offboard_config()
{
    riposte_runtime::OffboardControlConfig config;
    config.setpoint_rate_hz =
        dc::mqtt::env_real("FLIGHT_SETPOINT_RATE_HZ", 2.0, 200.0);
    config.setpoint_watchdog_timeout_s =
        dc::mqtt::env_real("FLIGHT_SETPOINT_WATCHDOG_TIMEOUT_S", 0.05, 10.0);
    config.setpoint_watchdog_poll_s =
        dc::mqtt::env_real("FLIGHT_SETPOINT_WATCHDOG_POLL_S", 0.005, 1.0);
    config.watchdog_owner_grace_s = dc::mqtt::env_real(
        "FLIGHT_SETPOINT_WATCHDOG_OWNER_GRACE_S", 0.05, 5.0);
    config.offboard_mode_confirmation_s = dc::mqtt::env_real(
        "FLIGHT_OFFBOARD_MODE_CONFIRMATION_S", 0.1, 5.0);
    config.action_takeoff_min_altitude_m =
        dc::mqtt::env_real("FLIGHT_ACTION_TAKEOFF_MIN_ALTITUDE_M", 0.1, 100.0);
    config.offboard_climb_speed_m_s =
        dc::mqtt::env_real("FLIGHT_OFFBOARD_CLIMB_SPEED_M_S", 0.05, 10.0);
    config.offboard_prime_seconds =
        dc::mqtt::env_real("FLIGHT_OFFBOARD_PRIME_SECONDS", 0.1, 30.0);
    config.offboard_prime_retry_increment_s = dc::mqtt::env_real(
        "FLIGHT_OFFBOARD_PRIME_RETRY_INCREMENT_S", 0.01, 10.0);
    config.offboard_start_attempts = static_cast<std::uint32_t>(
        dc::mqtt::env_integer("FLIGHT_OFFBOARD_START_ATTEMPTS", 1, 20));
    config.takeoff_altitude_tolerance_m =
        dc::mqtt::env_real("FLIGHT_TAKEOFF_ALTITUDE_TOLERANCE_M", 0.05, 20.0);
    config.takeoff_vertical_speed_tolerance_m_s = dc::mqtt::env_real(
        "FLIGHT_TAKEOFF_VERTICAL_SPEED_TOLERANCE_M_S", 0.05, 20.0);
    config.takeoff_settle_s =
        dc::mqtt::env_real("FLIGHT_TAKEOFF_SETTLE_S", 0.1, 60.0);
    config.takeoff_timeout_s =
        dc::mqtt::env_real("FLIGHT_TAKEOFF_TIMEOUT_S", 5.0, 600.0);
    config.landing_timeout_s =
        dc::mqtt::env_real("FLIGHT_LANDING_TIMEOUT_S", 5.0, 600.0);
    config.rtl_timeout_s =
        dc::mqtt::env_real("FLIGHT_RTL_TIMEOUT_S", 5.0, 1800.0);
    config.command_poll_s =
        dc::mqtt::env_real("FLIGHT_COMMAND_POLL_S", 0.01, 5.0);
    config.disarm_timeout_s =
        dc::mqtt::env_real("FLIGHT_DISARM_TIMEOUT_S", 1.0, 600.0);
    return config;
}

[[nodiscard]] dc::flight::MavsdkRcOwnershipMonitorConfig
load_rc_ownership_config()
{
    const auto pwm = [](const char* const name) {
        return static_cast<std::uint16_t>(
            dc::mqtt::env_integer(name, 800, 2200));
    };

    dc::flight::MavsdkRcOwnershipMonitorConfig config;
    config.channel = static_cast<std::uint8_t>(
        dc::mqtt::env_integer("FLIGHT_OWNERSHIP_RC_CHANNEL", 1, 18));
    config.arm_channel = static_cast<std::uint8_t>(
        dc::mqtt::env_integer("FLIGHT_ARM_SWITCH_RC_CHANNEL", 1, 18));
    config.tracker.offboard_pwm = {
        pwm("FLIGHT_OWNERSHIP_OFFBOARD_PWM_MIN"),
        pwm("FLIGHT_OWNERSHIP_OFFBOARD_PWM_MAX")};
    config.tracker.hold_pwm = {
        pwm("FLIGHT_OWNERSHIP_HOLD_PWM_MIN"),
        pwm("FLIGHT_OWNERSHIP_HOLD_PWM_MAX")};
    config.tracker.position_pwm = {
        pwm("FLIGHT_OWNERSHIP_POSITION_PWM_MIN"),
        pwm("FLIGHT_OWNERSHIP_POSITION_PWM_MAX")};
    config.disarm_pwm = {
        pwm("FLIGHT_ARM_SWITCH_DISARM_PWM_MIN"),
        pwm("FLIGHT_ARM_SWITCH_DISARM_PWM_MAX")};
    config.arm_pwm = {
        pwm("FLIGHT_ARM_SWITCH_ARM_PWM_MIN"),
        pwm("FLIGHT_ARM_SWITCH_ARM_PWM_MAX")};
    config.tracker.handback_debounce = std::chrono::milliseconds{
        dc::mqtt::env_integer("FLIGHT_OWNERSHIP_DEBOUNCE_MS", 1, 5000)};
    config.tracker.stale_timeout = std::chrono::milliseconds{
        dc::mqtt::env_integer("FLIGHT_OWNERSHIP_STALE_MS", 2, 10000)};
    config.message_rate_hz =
        dc::mqtt::env_real("PX4_RATE_RC_CHANNELS_HZ", 2.0, 100.0);
    return config;
}

[[nodiscard]] dc::flight::MavsdkMissionBackendConfig load_backend_config()
{
    dc::flight::MavsdkMissionBackendConfig config;
    config.offboard = load_offboard_config();
    config.cruise_speed_m_s =
        dc::mqtt::env_real("FLIGHT_CRUISE_SPEED_M_S", 0.1, 30.0);
    config.vertical_speed_m_s =
        dc::mqtt::env_real("FLIGHT_VERTICAL_SPEED_M_S", 0.1, 15.0);
    config.horizontal_tolerance_m =
        dc::mqtt::env_real("FLIGHT_HORIZONTAL_TOLERANCE_M", 0.05, 50.0);
    config.vertical_tolerance_floor_m =
        dc::mqtt::env_real("FLIGHT_VERTICAL_TOLERANCE_FLOOR_M", 0.05, 50.0);
    config.vertical_tolerance_ratio =
        dc::mqtt::env_real("FLIGHT_VERTICAL_TOLERANCE_RATIO", 0.001, 1.0);
    config.arrival_speed_tolerance_m_s =
        dc::mqtt::env_real("FLIGHT_ARRIVAL_SPEED_TOLERANCE_M_S", 0.05, 10.0);
    config.arrival_settle_s =
        dc::mqtt::env_real("FLIGHT_ARRIVAL_SETTLE_S", 0.1, 60.0);
    config.arrival_timeout_s =
        dc::mqtt::env_real("FLIGHT_ARRIVAL_TIMEOUT_S", 1.0, 600.0);
    config.health_timeout_s =
        dc::mqtt::env_real("FLIGHT_HEALTH_TIMEOUT_S", 1.0, 600.0);
    config.position_timeout_s =
        dc::mqtt::env_real("FLIGHT_POSITION_TIMEOUT_S", 1.0, 600.0);
    config.battery_telemetry_timeout_s =
        dc::mqtt::env_real("FLIGHT_BATTERY_TELEMETRY_TIMEOUT_S", 1.0, 600.0);
    // Fractions, not percent. The backend rejects anything outside (0, 1] and
    // requires the RTL threshold to sit strictly below the preflight gate.
    config.preflight_min_battery_fraction = dc::mqtt::env_real(
        "FLIGHT_PREFLIGHT_MIN_BATTERY_FRACTION", 0.01, 1.0);
    config.low_battery_rtl_fraction =
        dc::mqtt::env_real("FLIGHT_LOW_BATTERY_RTL_FRACTION", 0.01, 1.0);
    config.max_route_radius_m =
        dc::mqtt::env_real("FLIGHT_MAX_ROUTE_RADIUS_M", 1.0, 10000.0);
    config.max_home_launch_horizontal_m =
        dc::mqtt::env_real("FLIGHT_MAX_HOME_LAUNCH_HORIZONTAL_M", 0.5, 1000.0);
    config.max_home_launch_altitude_delta_m = dc::mqtt::env_real(
        "FLIGHT_MAX_HOME_LAUNCH_ALTITUDE_DELTA_M", 0.5, 1000.0);
    return config;
}

[[nodiscard]] bool rate_accepted(
    const mavsdk::Telemetry::Result result, const char* const stream)
{
    if (result == mavsdk::Telemetry::Result::Success) {
        return true;
    }
    log_line(
        "FATAL", std::string{"cannot set PX4 telemetry rate: "} + stream);
    return false;
}

// Neither MAVSDK class requests its own stream rates. Without these, arrival
// judgement, the takeoff height wait and both battery gates read default-rate
// or stale data - a silent runtime failure rather than a compile error.
[[nodiscard]] bool set_telemetry_rates(
    mavsdk::Telemetry& telemetry, const Px4LinkConfig& link)
{
    const bool ned = rate_accepted(
        telemetry.set_rate_position_velocity_ned(
            link.rate_position_velocity_ned_hz),
        "position_velocity_ned");
    const bool battery = rate_accepted(
        telemetry.set_rate_battery(link.rate_battery_hz), "battery");
    const bool position = rate_accepted(
        telemetry.set_rate_position(link.rate_position_hz), "position");
    const bool rc_status = rate_accepted(
        telemetry.set_rate_rc_status(link.rate_rc_status_hz), "rc_status");
    return ned && battery && position && rc_status;
}

int run(const dc::mqtt::AgentConfig& config)
{
    // Declaration order below is the destruction contract. Nothing owns
    // anything else: every object holds bare references to the ones above it.
    dc::mqtt::MqttCommandSource source{
        config.process.vehicle_id, config.source};
    dc::mqtt::SqliteCommandLedger ledger{config.ledger};
    dc::mqtt::MosquittoSession session{source, ledger, config.session};

    std::string ledger_error;
    const std::vector<dc::mqtt::UnresolvedCommand> unresolved =
        ledger.unresolved_commands(ledger_error);
    if (!ledger_error.empty()) {
        log_line("FATAL", "cannot read the command ledger: " + ledger_error);
        return 4;
    }

    std::unique_ptr<mavsdk::Mavsdk> mavsdk;
    std::shared_ptr<mavsdk::System> system;
    std::unique_ptr<mavsdk::Action> action;
    std::unique_ptr<mavsdk::Telemetry> telemetry;
    std::unique_ptr<dc::flight::MavsdkRcOwnershipMonitor> rc_ownership;
    std::unique_ptr<dc::flight::MavsdkMissionBackend> px4_backend;
    ObserveOnlyBackend observe_only{config.process.vehicle_id};

    dc::flight::MissionBackend* backend = &observe_only;
    dc::flight::VehicleTelemetryProvider* provider = &observe_only;
    double low_battery_rtl_fraction = 0.0;
    std::uint8_t ownership_channel = 0U;
    std::uint8_t arm_switch_channel = 0U;

    if (config.process.execute_flight) {
        const Px4LinkConfig link = load_px4_link_config();
        const dc::flight::MavsdkMissionBackendConfig backend_config =
            load_backend_config();
        const dc::flight::MavsdkRcOwnershipMonitorConfig ownership_config =
            load_rc_ownership_config();
        ownership_channel = ownership_config.channel;
        arm_switch_channel = ownership_config.arm_channel;
        low_battery_rtl_fraction = backend_config.low_battery_rtl_fraction;

        mavsdk::Mavsdk::Configuration mavsdk_configuration{
            mavsdk::ComponentType::CompanionComputer};
        mavsdk_configuration.set_component_id(link.component_id);
        mavsdk = std::make_unique<mavsdk::Mavsdk>(mavsdk_configuration);

        if (mavsdk->add_any_connection(link.connection)
            != mavsdk::ConnectionResult::Success) {
            log_line("FATAL", "cannot open the PX4 link: " + link.connection);
            return 3;
        }
        log_line("INFO", "waiting for the PX4 autopilot on " + link.connection);
        const auto discovered =
            mavsdk->first_autopilot(link.discovery_timeout_s);
        if (!discovered.has_value()) {
            log_line("FATAL", "no autopilot was discovered before the timeout");
            return 3;
        }
        system = *discovered;

        action = std::make_unique<mavsdk::Action>(system);
        telemetry = std::make_unique<mavsdk::Telemetry>(system);
        if (!set_telemetry_rates(*telemetry, link)) {
            return 3;
        }
        rc_ownership =
            std::make_unique<dc::flight::MavsdkRcOwnershipMonitor>(
                system, ownership_config);
        rc_ownership->require_message_rate();

        px4_backend = std::make_unique<dc::flight::MavsdkMissionBackend>(
            config.process.vehicle_id,
            system,
            *action,
            *telemetry,
            *rc_ownership,
            backend_config);
        backend = px4_backend.get();
        provider = px4_backend.get();
    } else if (!unresolved.empty()) {
        // Terminalizing these rows asserts that a PX4 recovery action really
        // happened. In observe-only mode it did not, and the database belongs
        // to a run that really was flying.
        log_line(
            "FATAL",
            "the ledger holds "
                + std::to_string(unresolved.size())
                + " unresolved command(s) from a previous flight; run with "
                  "REMOTE_FLIGHT_EXECUTE=1 to recover the vehicle, or point "
                  "LEDGER_DB_PATH at a separate observe-only database");
        return 4;
    }

    dc::flight::PreemptiveMissionExecutor executor{*backend, config.executor};

    // This blocks with no deadline. On the MAVSDK path it stops offboard and
    // then either disarms or flies RTL and waits, uninterruptibly, for up to
    // FLIGHT_RTL_TIMEOUT_S. Until it returns accepted, every dispatch - even
    // EMERGENCY_STOP - is refused with vehicle_recovery_required.
    log_line("INFO", "requesting vehicle recovery before serving commands");
    const dc::flight::CommandResult recovery = executor.recover_after_restart();
    if (!recovery.accepted) {
        log_line("FATAL", "vehicle recovery refused: " + recovery.detail);
        executor.shutdown();
        return 4;
    }
    log_line("INFO", "vehicle recovery accepted: " + recovery.detail);

    if (!unresolved.empty()) {
        const bool recovery_yielded_without_vehicle_command =
            recovery.detail.rfind("pilot_override_", 0U) == 0U;
        const std::string terminal_detail =
            recovery_yielded_without_vehicle_command
            ? "restart_owner_ambiguous_no_px4_command"
            : "vehicle_safe_after_agent_restart";
        if (!ledger.fail_unresolved_after_vehicle_recovery(
                terminal_detail, ledger_error)) {
            log_line(
                "FATAL",
                "cannot terminalize unresolved commands: " + ledger_error);
            executor.shutdown();
            return 4;
        }
        log_line(
            "INFO",
            "terminalized " + std::to_string(unresolved.size())
                + " unresolved command(s) after recovery");
    }

    std::string session_error;
    if (!session.start(session_error)) {
        log_line("FATAL", "cannot start the MQTT session: " + session_error);
        executor.shutdown();
        return 5;
    }

    dc::flight::RemoteMissionCoordinator coordinator{
        source,
        executor,
        ledger,
        session,
        config.process.vehicle_id,
        config.route,
        config.freshness};

    std::atomic_bool coordinator_failed{false};
    std::thread worker{[&coordinator, &coordinator_failed] {
        // An escape from run() leaves nobody draining commands or publishing
        // status, so it is a vehicle-safing condition rather than a log line.
        try {
            coordinator.run();
        } catch (const std::exception& error) {
            log_line(
                "FATAL", std::string{"coordinator aborted: "} + error.what());
            coordinator_failed.store(true);
        } catch (...) {
            log_line("FATAL", "coordinator aborted with an unknown error");
            coordinator_failed.store(true);
        }
    }};

    const auto tick = std::chrono::milliseconds{100};
    const auto started_at = std::chrono::steady_clock::now();
    auto next_telemetry = started_at;
    auto next_battery = started_at + config.process.battery_supervisor_interval;
    bool ever_connected = false;
    bool telemetry_publish_logged = false;
    auto last_connected = started_at;
    auto next_link_check = started_at;
    bool link_loss_return_requested = false;
    bool geofence_return_requested = false;
    std::string last_flight_mode;
    dc::flight::ControlOwner last_control_owner =
        dc::flight::ControlOwner::unknown;
    dc::flight::OwnershipSwitch last_ownership_switch =
        dc::flight::OwnershipSwitch::unknown;
    dc::flight::ArmSwitchPosition last_arm_switch =
        dc::flight::ArmSwitchPosition::unknown;
    bool ownership_logged = false;
    auto next_state_log = started_at;
    bool link_loss_logged = false;
    int exit_code = EXIT_SUCCESS;

    while (g_stop_requested == 0 && !coordinator_failed.load()) {
        const auto now = std::chrono::steady_clock::now();

        if (session.connected()) {
            ever_connected = true;
            last_connected = now;
            if (link_loss_logged) {
                log_line("INFO", "broker session restored");
                link_loss_logged = false;
            }
            link_loss_return_requested = false;
        } else if (!ever_connected
                   && now - started_at >= config.process.initial_connect_timeout) {
            // connected() means "QoS 1 subscription granted". A broker that
            // never grants it produces no error at all, so silence past the
            // deadline has to be treated as a failure.
            log_line(
                "FATAL",
                "no QoS 1 subscription was granted within the initial connect "
                "timeout");
            exit_code = EXIT_FAILURE;
            break;
        } else if (ever_connected
                   && !link_loss_return_requested
                   && now - last_connected >= config.process.link_loss_timeout
                   && now >= next_link_check) {
            // Losing the session is not fatal to the process -- the outbox
            // still works and the broker comes back. It is fatal to an
            // airborne route, because STOP and RTL can no longer reach us and
            // PX4 sees a healthy MAVLink link, so its own datalink failsafe
            // never fires. Re-checked on the supervisor cadence rather than
            // latched, because a mission already queued locally can put the
            // vehicle in the air after the outage began.
            next_link_check = now + config.process.battery_supervisor_interval;
            const dc::flight::VehicleTelemetrySnapshot snapshot =
                provider->snapshot();
            const bool first_report = !link_loss_logged;
            link_loss_logged = true;

            if (!config.process.execute_flight) {
                if (first_report) {
                    log_line(
                        "WARN",
                        "broker link lost past the timeout; observe-only mode "
                        "cannot command a return");
                }
            } else if (!snapshot.in_air) {
                if (first_report) {
                    log_line(
                        "WARN",
                        "broker link lost past the timeout; vehicle is not "
                        "airborne, so no return was commanded");
                }
            } else if (!snapshot.home_valid) {
                // Home is the launch point PX4 latched at arming. Without it a
                // return has no destination, so PX4's own failsafe is all that
                // is left and an operator has to be told.
                if (first_report) {
                    log_line(
                        "FATAL",
                        "broker link lost while airborne but home is not "
                        "valid; no return could be commanded");
                }
            } else {
                const dc::flight::CommandResult requested =
                    executor.request_link_loss_return_to_launch();
                link_loss_return_requested = requested.accepted;
                log_line(
                    requested.accepted ? "WARN" : "FATAL",
                    std::string{"broker link lost while airborne; return to "
                                "launch "}
                        + (requested.accepted ? "queued: " : "refused: ")
                        + requested.detail);
            }
        }

        const bool telemetry_due = now >= next_telemetry;
        const bool battery_due =
            config.process.execute_flight && now >= next_battery;
        if (telemetry_due || battery_due) {
            const dc::flight::VehicleTelemetrySnapshot snapshot =
                provider->snapshot();

            // Flight mode remains diagnostic evidence. Ownership itself is
            // logged separately from raw CH5/CH6 below so an intended switch
            // movement can be distinguished from an Offboard-loss mode change.
            if (snapshot.flight_mode != last_flight_mode) {
                log_line(
                    "INFO",
                    "flight mode: "
                        + (last_flight_mode.empty() ? std::string{"(none)"}
                                                    : last_flight_mode)
                        + " -> " + snapshot.flight_mode);
                last_flight_mode = snapshot.flight_mode;
            }

            if (!ownership_logged
                || snapshot.control_owner != last_control_owner
                || snapshot.ownership_switch != last_ownership_switch
                || snapshot.arm_switch != last_arm_switch) {
                std::string line = "ownership channel="
                    + std::to_string(ownership_channel)
                    + " switch="
                    + dc::flight::ownership_switch_name(last_ownership_switch)
                    + "->"
                    + dc::flight::ownership_switch_name(
                        snapshot.ownership_switch)
                    + " owner="
                    + dc::flight::control_owner_name(last_control_owner)
                    + "->"
                    + dc::flight::control_owner_name(snapshot.control_owner);
                if (snapshot.ownership_pwm.has_value()) {
                    line += " pwm="
                        + std::to_string(*snapshot.ownership_pwm);
                }
                line += " arm_channel="
                    + std::to_string(arm_switch_channel)
                    + " arm_switch="
                    + dc::flight::arm_switch_position_name(last_arm_switch)
                    + "->"
                    + dc::flight::arm_switch_position_name(
                        snapshot.arm_switch);
                if (snapshot.arm_switch_pwm.has_value()) {
                    line += " arm_pwm="
                        + std::to_string(*snapshot.arm_switch_pwm);
                }
                line += " ready="
                    + std::string{snapshot.ready_for_mission ? "1" : "0"}
                    + " epoch=" + std::to_string(snapshot.ownership_epoch);
                log_line("RC", line);
                last_control_owner = snapshot.control_owner;
                last_ownership_switch = snapshot.ownership_switch;
                last_arm_switch = snapshot.arm_switch;
                ownership_logged = true;
            }

            // Silent on the ground, one line a second once it matters, so a
            // flight can be reconstructed afterwards without drowning the
            // journal while the aircraft sits powered on.
            if ((snapshot.armed || snapshot.in_air) && now >= next_state_log) {
                next_state_log = now + config.process.telemetry_interval;
                std::string line = "state mode=" + snapshot.flight_mode
                    + " armed=" + (snapshot.armed ? "1" : "0")
                    + " in_air=" + (snapshot.in_air ? "1" : "0")
                    + " owner="
                    + dc::flight::control_owner_name(snapshot.control_owner)
                    + " switch="
                    + dc::flight::ownership_switch_name(
                        snapshot.ownership_switch)
                    + " home_valid=" + (snapshot.home_valid ? "1" : "0");
                if (snapshot.position.has_value()) {
                    line += " rel_alt="
                        + std::to_string(static_cast<int>(
                            snapshot.position->relative_altitude_m));
                }
                if (snapshot.position.has_value()
                    && snapshot.home_position.has_value()) {
                    line += " home_dist="
                        + std::to_string(static_cast<int>(
                            dc::flight::horizontal_distance_m(
                                *snapshot.home_position, *snapshot.position)))
                        + "/"
                        + std::to_string(static_cast<int>(
                            config.process.geofence_radius_m));
                }
                if (snapshot.battery_percent.has_value()) {
                    line += " batt="
                        + std::to_string(
                            static_cast<int>(*snapshot.battery_percent));
                }
                log_line("STATE", line);
            }

            if (telemetry_due) {
                std::string telemetry_error;
                if (!session.publish_telemetry(snapshot, telemetry_error)
                    && !telemetry_publish_logged) {
                    // Telemetry is QoS 0 with no outbox; failing while offline
                    // is normal. Log the first one only, so an outage cannot
                    // turn into a log flood.
                    log_line(
                        "WARN",
                        "telemetry publish unavailable: " + telemetry_error);
                    telemetry_publish_logged = true;
                }
                next_telemetry = now + config.process.telemetry_interval;
            }

            if (battery_due) {
                // The backend's own failsafe only runs while a route is
                // active. This covers the idle vehicle.
                if (snapshot.battery_percent.has_value()
                    && (*snapshot.battery_percent / 100.0)
                        <= low_battery_rtl_fraction) {
                    const dc::flight::CommandResult requested =
                        executor.request_low_battery_return_to_launch();
                    if (!requested.accepted) {
                        log_line(
                            "WARN",
                            "low battery return refused: " + requested.detail);
                    }
                }

                // Temporary containment until the server delivers a geofence
                // the codec can accept. Measured from PX4's home, which is the
                // launch point latched at arming, so it matches where an RTL
                // would actually take the aircraft.
                if (snapshot.in_air && snapshot.home_valid
                    && snapshot.position.has_value()
                    && snapshot.home_position.has_value()) {
                    const double distance = dc::flight::horizontal_distance_m(
                        *snapshot.home_position, *snapshot.position);
                    if (distance > config.process.geofence_radius_m) {
                        if (!geofence_return_requested) {
                            const dc::flight::CommandResult requested =
                                executor.request_geofence_return_to_launch();
                            geofence_return_requested = requested.accepted;
                            log_line(
                                requested.accepted ? "WARN" : "FATAL",
                                "geofence breached at "
                                    + std::to_string(
                                        static_cast<int>(distance))
                                    + " m from launch; return "
                                    + (requested.accepted ? "queued: "
                                                          : "refused: ")
                                    + requested.detail);
                        }
                    } else {
                        // Only clear once the aircraft is back inside, so a
                        // return still in progress cannot re-arm the trigger.
                        geofence_return_requested = false;
                    }
                }
                next_battery = now + config.process.battery_supervisor_interval;
            }
        }

        std::this_thread::sleep_for(tick);
    }

    if (coordinator_failed.load()) {
        exit_code = EXIT_FAILURE;
    }

    log_line("INFO", "shutting down");
    std::atomic_bool shutdown_complete{false};
    const auto shutdown_deadline =
        std::chrono::steady_clock::now() + config.process.shutdown_watchdog;
    std::thread shutdown_watchdog{[&shutdown_complete, shutdown_deadline] {
        while (!shutdown_complete.load()) {
            if (std::chrono::steady_clock::now() >= shutdown_deadline) {
                // run()'s post-stop drain has no bail-out, and a backend wedged
                // in shutdown_safely() would keep this process alive forever.
                // systemd Restart= is the recovery.
                log_line("FATAL", "shutdown watchdog expired; forcing exit");
                std::_Exit(EXIT_FAILURE);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }
    }};

    // request_stop() must come first: MosquittoSession::stop() also closes the
    // command source, which would leave run() spinning on a source that never
    // yields while its own stop flag is still clear.
    coordinator.request_stop();
    worker.join();
    session.stop();
    shutdown_complete.store(true);
    shutdown_watchdog.join();

    return exit_code;
}

}  // namespace

int main()
{
    // libmosquitto writes to a TLS socket that can go away underneath it.
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    dc::mqtt::AgentConfig config;
    try {
        config = dc::mqtt::load_agent_config();
    } catch (const std::exception& error) {
        log_line("FATAL", std::string{"configuration: "} + error.what());
        return 2;
    }

    std::fprintf(
        stdout,
        "[mqtt_px4_control] vehicle=%s broker=%s:%u flight_execution=%s\n",
        config.process.vehicle_id.c_str(),
        config.session.broker_host.c_str(),
        static_cast<unsigned int>(config.session.broker_port),
        config.process.execute_flight ? "ENABLED" : "DISABLED (observe only)");
    if (config.process.execute_flight) {
        std::fprintf(
            stdout,
            "[mqtt_px4_control] REMOTE_FLIGHT_EXECUTE=1 - this process will "
            "command PX4.\n");
    }
    std::fflush(stdout);

    try {
        return run(config);
    } catch (const std::exception& error) {
        log_line("FATAL", std::string{"startup: "} + error.what());
        return 6;
    } catch (...) {
        log_line("FATAL", "startup failed with an unknown error");
        return 6;
    }
}
