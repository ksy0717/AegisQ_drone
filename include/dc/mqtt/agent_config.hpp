#pragma once

#include <chrono>
#include <cstddef>
#include <string>

#include "dc/flight/preemptive_mission_executor.hpp"
#include "dc/flight/remote_command.hpp"
#include "dc/mqtt/mosquitto_session.hpp"
#include "dc/mqtt/mqtt_command_source.hpp"
#include "dc/mqtt/sqlite_command_ledger.hpp"

namespace dc::mqtt {

// Environment readers shared by the agent and its configuration tests. Each one
// throws std::runtime_error naming the offending variable. There are no
// fallbacks on purpose: a compiled default for a vehicle limit is a vehicle
// policy nobody reviewed. An empty variable is treated as unset, because
// systemd's EnvironmentFile= exports empty values rather than dropping them.
[[nodiscard]] std::string env_text(const char* name);
[[nodiscard]] bool env_flag(const char* name);
[[nodiscard]] long long env_integer(
    const char* name, long long minimum, long long maximum);
[[nodiscard]] double env_real(const char* name, double minimum, double maximum);
[[nodiscard]] std::size_t env_size(
    const char* name, long long minimum, long long maximum);

struct AgentProcessConfig {
    // False keeps the process off PX4 entirely: no MAVSDK connection is opened
    // and a refusing backend answers every command.
    bool execute_flight{};
    std::string vehicle_id;
    std::chrono::milliseconds telemetry_interval{};
    std::chrono::milliseconds battery_supervisor_interval{};
    // The broker session reports "connected" only after a QoS 1 SUBACK. A
    // broker that never grants it produces no error of any kind, so the
    // supervisor treats silence past this deadline as fatal.
    std::chrono::seconds initial_connect_timeout{};
    // Once the session has been granted, losing it is not fatal to the process
    // -- the outbox keeps working and the broker will be back. It is fatal to
    // an airborne route, because the server can no longer send STOP or RTL. A
    // gap this long while in the air makes the aircraft return on its own.
    std::chrono::seconds link_loss_timeout{};
    // Temporary software containment until the server's own geofence is
    // delivered and enforced: horizontal metres from the launch point beyond
    // which the aircraft returns on its own. The codec does not accept a
    // geofence field yet, so this is the only containment the agent has.
    double geofence_radius_m{};
    // A backend wedged inside shutdown_safely() would otherwise keep the
    // process alive forever; systemd Restart= is the recovery.
    std::chrono::seconds shutdown_watchdog{};
};

struct AgentConfig {
    AgentProcessConfig process;
    flight::RouteLimits route;
    flight::CommandFreshnessLimits freshness;
    flight::PreemptiveExecutorConfig executor;
    CommandSourceLimits source;
    SqliteCommandLedgerConfig ledger;
    MosquittoSessionConfig session;
};

// Reads and cross-validates the transport, ledger and executor configuration.
// The MAVSDK flight tuning is loaded separately by the agent because those
// structs pull in MAVSDK headers this translation unit deliberately avoids;
// keeping them out is what lets the configuration be unit tested on a host
// without a flight stack.
[[nodiscard]] AgentConfig load_agent_config();

}  // namespace dc::mqtt
