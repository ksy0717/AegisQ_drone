#include "dc/mqtt/agent_config.hpp"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace dc::mqtt {
namespace {

constexpr std::uint16_t kRequiredBrokerPort = 8883U;

[[noreturn]] void fail(const std::string& name, const std::string& reason)
{
    throw std::runtime_error(name + ": " + reason);
}

[[nodiscard]] const char* raw_env(const char* const name) noexcept
{
    const char* const value = std::getenv(name);
    if (value == nullptr || *value == '\0') {
        return nullptr;
    }
    return value;
}

// Rejects a dotted quad only when it is not one. The broker host must be a
// literal address because the WireGuard route check compares it to the routing
// decision; a name that resolves elsewhere would silently defeat that check.
[[nodiscard]] bool ipv4_literal(const std::string& value) noexcept
{
    std::size_t octets = 0U;
    std::size_t index = 0U;
    while (index < value.size()) {
        std::size_t digits = 0U;
        unsigned int octet = 0U;
        while (index < value.size()
               && std::isdigit(static_cast<unsigned char>(value[index])) != 0) {
            octet = (octet * 10U)
                + static_cast<unsigned int>(value[index] - '0');
            ++digits;
            ++index;
            if (digits > 3U || octet > 255U) {
                return false;
            }
        }
        if (digits == 0U) {
            return false;
        }
        ++octets;
        if (index == value.size()) {
            break;
        }
        if (value[index] != '.') {
            return false;
        }
        ++index;
        if (index == value.size()) {
            return false;
        }
    }
    return octets == 4U;
}

struct BrokerEndpoint {
    std::string host;
    std::uint16_t port{};
};

[[nodiscard]] BrokerEndpoint parse_broker_uri(const std::string& uri)
{
    const std::string scheme = "mqtts://";
    if (uri.rfind(scheme, 0U) != 0U) {
        fail("MQTT_BROKER_URI", "must use the mqtts:// scheme");
    }
    const std::string authority = uri.substr(scheme.size());
    if (authority.find('/') != std::string::npos
        || authority.find('@') != std::string::npos) {
        fail("MQTT_BROKER_URI", "must be mqtts://<ipv4>:<port> with no path or userinfo");
    }
    const std::size_t colon = authority.rfind(':');
    if (colon == std::string::npos || colon == 0U
        || colon + 1U == authority.size()) {
        fail("MQTT_BROKER_URI", "must be mqtts://<ipv4>:<port>");
    }

    BrokerEndpoint endpoint;
    endpoint.host = authority.substr(0U, colon);
    if (!ipv4_literal(endpoint.host)) {
        fail("MQTT_BROKER_URI", "host must be an IPv4 literal, not a name");
    }

    const std::string port_text = authority.substr(colon + 1U);
    unsigned long port = 0UL;
    for (const char character : port_text) {
        if (std::isdigit(static_cast<unsigned char>(character)) == 0) {
            fail("MQTT_BROKER_URI", "port must be numeric");
        }
        port = (port * 10UL)
            + static_cast<unsigned long>(character - '0');
        if (port > 65535UL) {
            fail("MQTT_BROKER_URI", "port is out of range");
        }
    }
    endpoint.port = static_cast<std::uint16_t>(port);
    if (endpoint.port != kRequiredBrokerPort) {
        fail("MQTT_BROKER_URI", "MQTT control requires TLS port 8883");
    }
    return endpoint;
}

// The template carries the topics so an operator can read them, but the code
// derives them from the vehicle id. A mismatch means the two sources of truth
// have diverged, which is exactly the case that must never boot.
void require_derived_topic(
    const char* const name,
    const std::string& derived)
{
    const std::string configured = env_text(name);
    if (configured != derived) {
        fail(name, "must be the derived topic " + derived);
    }
}

[[nodiscard]] std::string require_absolute_path(const char* const name)
{
    const std::string value = env_text(name);
    if (value.front() != '/') {
        fail(name, "must be an absolute path");
    }
    return value;
}

[[nodiscard]] std::string require_bounded_text(
    const char* const name, const std::size_t maximum)
{
    const std::string value = env_text(name);
    if (value.size() > maximum) {
        fail(name, "exceeds " + std::to_string(maximum) + " bytes");
    }
    return value;
}

// The cast is explicit because the flight targets build with -Wconversion and
// a duration's rep is not the same integer type on every platform.
template <typename Duration>
[[nodiscard]] Duration env_duration(
    const char* const name, const long long minimum, const long long maximum)
{
    return Duration{
        static_cast<typename Duration::rep>(
            env_integer(name, minimum, maximum))};
}

}  // namespace

std::string env_text(const char* const name)
{
    const char* const value = raw_env(name);
    if (value == nullptr) {
        fail(name, "must be set and non-empty");
    }
    return std::string{value};
}

bool env_flag(const char* const name)
{
    const std::string value = env_text(name);
    if (value == "1") {
        return true;
    }
    if (value == "0") {
        return false;
    }
    fail(name, "must be exactly 0 or 1");
}

long long env_integer(
    const char* const name, const long long minimum, const long long maximum)
{
    const std::string value = env_text(name);
    std::size_t consumed = 0U;
    long long parsed = 0;
    try {
        parsed = std::stoll(value, &consumed);
    } catch (const std::exception&) {
        fail(name, "is not an integer: " + value);
    }
    if (consumed != value.size()) {
        fail(name, "has trailing characters: " + value);
    }
    if (parsed < minimum || parsed > maximum) {
        fail(
            name,
            "must be within [" + std::to_string(minimum) + ", "
                + std::to_string(maximum) + "]: " + value);
    }
    return parsed;
}

double env_real(
    const char* const name, const double minimum, const double maximum)
{
    const std::string value = env_text(name);
    std::size_t consumed = 0U;
    double parsed = 0.0;
    try {
        parsed = std::stod(value, &consumed);
    } catch (const std::exception&) {
        fail(name, "is not a number: " + value);
    }
    if (consumed != value.size()) {
        fail(name, "has trailing characters: " + value);
    }
    // stod accepts "nan" and "inf"; a non-finite limit would disable the very
    // comparison it is supposed to bound.
    if (!(parsed >= minimum && parsed <= maximum)) {
        fail(name, "must be a finite value within its documented range: " + value);
    }
    return parsed;
}

std::size_t env_size(
    const char* const name, const long long minimum, const long long maximum)
{
    return static_cast<std::size_t>(env_integer(name, minimum, maximum));
}

AgentConfig load_agent_config()
{
    AgentConfig config;

    config.process.execute_flight = env_flag("REMOTE_FLIGHT_EXECUTE");
    config.process.vehicle_id = env_text("MQTT_VEHICLE_ID");
    if (!valid_vehicle_id(config.process.vehicle_id)) {
        fail("MQTT_VEHICLE_ID", "is not a valid AegisQ vehicle id");
    }
    require_derived_topic(
        "MQTT_COMMAND_TOPIC", command_topic_for(config.process.vehicle_id));
    require_derived_topic(
        "MQTT_COMMAND_ACK_TOPIC",
        command_ack_topic_for(config.process.vehicle_id));

    config.process.telemetry_interval =
        env_duration<std::chrono::milliseconds>(
            "TELEMETRY_PUBLISH_INTERVAL_MS", 200, 3600000);
    config.process.battery_supervisor_interval =
        env_duration<std::chrono::milliseconds>(
            "BATTERY_SUPERVISOR_INTERVAL_MS", 200, 3600000);
    config.process.initial_connect_timeout =
        env_duration<std::chrono::seconds>(
            "MQTT_INITIAL_CONNECT_TIMEOUT_S", 5, 3600);
    config.process.link_loss_timeout = env_duration<std::chrono::seconds>(
        "MQTT_LINK_LOSS_TIMEOUT_S", 5, 3600);
    config.process.geofence_radius_m =
        env_real("GEOFENCE_RADIUS_M", 10.0, 10000.0);
    config.process.shutdown_watchdog = env_duration<std::chrono::seconds>(
        "SHUTDOWN_WATCHDOG_S", 5, 3600);

    config.route.max_waypoints = env_size("ROUTE_MAX_WAYPOINTS", 1, 4096);
    config.route.min_relative_altitude_m =
        env_real("ROUTE_MIN_REL_ALT_M", 0.0, 10000.0);
    config.route.max_relative_altitude_m =
        env_real("ROUTE_MAX_REL_ALT_M", 0.0, 10000.0);
    if (config.route.min_relative_altitude_m
        > config.route.max_relative_altitude_m) {
        fail("ROUTE_MAX_REL_ALT_M", "must not be below ROUTE_MIN_REL_ALT_M");
    }
    config.route.max_dwell_seconds = static_cast<std::uint32_t>(
        env_integer("ROUTE_MAX_DWELL_SEC", 1, 86400));
    config.route.max_total_dwell_seconds = static_cast<std::uint64_t>(
        env_integer("ROUTE_MAX_TOTAL_DWELL_SEC", 1, 604800));
    // The server enforces its own speed range, but nothing on that side knows
    // this airframe. These bound what the operator's choice may become here.
    config.route.min_speed_m_s = env_real("ROUTE_MIN_SPEED_MPS", 0.1, 50.0);
    config.route.max_speed_m_s = env_real("ROUTE_MAX_SPEED_MPS", 0.1, 50.0);
    if (config.route.min_speed_m_s > config.route.max_speed_m_s) {
        fail("ROUTE_MAX_SPEED_MPS", "must not be below ROUTE_MIN_SPEED_MPS");
    }
    if (!flight::valid_route_limits(config.route)) {
        fail("ROUTE_MAX_WAYPOINTS", "route limits are not a usable set");
    }

    config.freshness.max_age = env_duration<std::chrono::seconds>(
        "MQTT_COMMAND_MAX_AGE_SEC", 1, 86400);
    config.freshness.max_future_skew = env_duration<std::chrono::seconds>(
        "MQTT_MAX_FUTURE_SKEW_SEC", 0, 3600);

    config.executor.mission_queue_capacity =
        env_size("EXECUTOR_MISSION_QUEUE_CAPACITY", 1, 1024);
    config.executor.safety_queue_capacity =
        env_size("EXECUTOR_SAFETY_QUEUE_CAPACITY", 1, 1024);
    config.executor.terminal_event_capacity =
        env_size("EXECUTOR_TERMINAL_EVENT_CAPACITY", 3, 4096);
    if (config.executor.terminal_event_capacity
        < config.executor.mission_queue_capacity
            + config.executor.safety_queue_capacity + 1U) {
        fail(
            "EXECUTOR_TERMINAL_EVENT_CAPACITY",
            "must be at least mission + safety + 1");
    }

    config.source.max_payload_bytes =
        env_size("MQTT_MAX_PAYLOAD_BYTES", 1, 16777216);
    config.source.queue_capacity =
        env_size("MQTT_COMMAND_QUEUE_CAPACITY", 1, 4096);
    config.source.safety_queue_capacity =
        env_size("MQTT_SAFETY_QUEUE_CAPACITY", 1, 4096);
    config.source.codec.route = config.route;
    config.source.codec.max_issued_by_bytes =
        env_size("MQTT_MAX_ISSUED_BY_BYTES", 1, 4096);
    config.source.codec.max_json_depth =
        env_size("MQTT_MAX_JSON_DEPTH", 3, 64);

    config.ledger.database_path = require_absolute_path("LEDGER_DB_PATH");
    config.ledger.vehicle_id = config.process.vehicle_id;
    config.ledger.max_pending_outbox =
        env_size("LEDGER_MAX_PENDING_OUTBOX", 4, 1000000);
    config.ledger.outbox_claim_reserve =
        env_size("LEDGER_OUTBOX_CLAIM_RESERVE", 2, 1000000);
    if (config.ledger.outbox_claim_reserve >= config.ledger.max_pending_outbox) {
        fail(
            "LEDGER_OUTBOX_CLAIM_RESERVE",
            "must be below LEDGER_MAX_PENDING_OUTBOX");
    }
    config.ledger.max_status_detail_bytes =
        env_size("LEDGER_MAX_STATUS_DETAIL_BYTES", 1, 65536);
    config.ledger.busy_timeout = env_duration<std::chrono::milliseconds>(
        "LEDGER_BUSY_TIMEOUT_MS", 1, 60000);

    const BrokerEndpoint broker = parse_broker_uri(env_text("MQTT_BROKER_URI"));
    config.session.broker_host = broker.host;
    config.session.broker_port = broker.port;
    config.session.wireguard_server_ip = env_text("WIREGUARD_SERVER_IP");
    if (config.session.wireguard_server_ip != broker.host) {
        fail(
            "WIREGUARD_SERVER_IP",
            "must equal the MQTT_BROKER_URI host " + broker.host);
    }
    config.session.wireguard_interface = env_text("WIREGUARD_INTERFACE");
    config.session.client_id = env_text("MQTT_CLIENT_ID");
    if (config.session.client_id != config.process.vehicle_id) {
        fail("MQTT_CLIENT_ID", "must equal MQTT_VEHICLE_ID");
    }
    config.session.ca_file = require_absolute_path("MQTT_CA_FILE");
    config.session.client_certificate_file =
        require_absolute_path("MQTT_CERT_FILE");
    config.session.client_key_file = require_absolute_path("MQTT_KEY_FILE");
    config.session.qos = static_cast<int>(env_integer("MQTT_QOS", 1, 1));
    config.session.keepalive_seconds =
        static_cast<int>(env_integer("MQTT_KEEPALIVE_SEC", 5, 65535));
    config.session.reconnect_min_seconds = static_cast<unsigned int>(
        env_integer("MQTT_RECONNECT_MIN_SEC", 1, 3600));
    config.session.reconnect_max_seconds = static_cast<unsigned int>(
        env_integer("MQTT_RECONNECT_MAX_SEC", 1, 3600));
    if (config.session.reconnect_max_seconds
        < config.session.reconnect_min_seconds) {
        fail(
            "MQTT_RECONNECT_MAX_SEC",
            "must not be below MQTT_RECONNECT_MIN_SEC");
    }
    config.session.outbox_batch_size =
        env_size("MQTT_OUTBOX_BATCH_SIZE", 1, 4096);
    config.session.graceful_offline_timeout =
        env_duration<std::chrono::milliseconds>(
            "MQTT_GRACEFUL_OFFLINE_TIMEOUT_MS", 1, 5000);
    config.session.registration.robot_id = config.process.vehicle_id;
    config.session.registration.name = require_bounded_text("VEHICLE_NAME", 256U);
    config.session.registration.model =
        require_bounded_text("VEHICLE_MODEL", 256U);
    config.session.registration.firmware =
        require_bounded_text("VEHICLE_FIRMWARE", 256U);
    config.session.registration.site_id = require_bounded_text("SITE_ID", 256U);

    return config;
}

}  // namespace dc::mqtt
