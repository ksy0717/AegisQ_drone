#include <chrono>
#include <cstdlib>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "dc/mqtt/agent_config.hpp"
#include "test_support.hpp"

namespace {

// A configuration that must load. Every rejection case below starts from this
// set and breaks exactly one variable, so a failure names the rule that broke.
const std::vector<std::pair<const char*, const char*>>& valid_environment()
{
    static const std::vector<std::pair<const char*, const char*>> values = {
        {"REMOTE_FLIGHT_EXECUTE", "0"},
        {"MQTT_VEHICLE_ID", "DRONE-01"},
        {"MQTT_CLIENT_ID", "DRONE-01"},
        {"MQTT_COMMAND_TOPIC", "aegisq/cmd/DRONE-01"},
        {"MQTT_COMMAND_ACK_TOPIC", "aegisq/cmd-ack/DRONE-01"},
        {"MQTT_BROKER_URI", "mqtts://10.100.0.1:8883"},
        {"WIREGUARD_SERVER_IP", "10.100.0.1"},
        {"WIREGUARD_INTERFACE", "wg0"},
        {"MQTT_QOS", "1"},
        {"MQTT_KEEPALIVE_SEC", "30"},
        {"MQTT_RECONNECT_MIN_SEC", "1"},
        {"MQTT_RECONNECT_MAX_SEC", "30"},
        {"MQTT_CA_FILE", "/etc/aegisq/certs/ca.crt"},
        {"MQTT_CERT_FILE", "/etc/aegisq/certs/DRONE-01.crt"},
        {"MQTT_KEY_FILE", "/etc/aegisq/certs/DRONE-01.key"},
        {"MQTT_MAX_PAYLOAD_BYTES", "65536"},
        {"MQTT_COMMAND_QUEUE_CAPACITY", "8"},
        {"MQTT_SAFETY_QUEUE_CAPACITY", "4"},
        {"MQTT_MAX_ISSUED_BY_BYTES", "128"},
        {"MQTT_MAX_JSON_DEPTH", "4"},
        {"MQTT_OUTBOX_BATCH_SIZE", "16"},
        {"MQTT_GRACEFUL_OFFLINE_TIMEOUT_MS", "1000"},
        {"MQTT_COMMAND_MAX_AGE_SEC", "300"},
        {"MQTT_MAX_FUTURE_SKEW_SEC", "10"},
        {"MQTT_INITIAL_CONNECT_TIMEOUT_S", "60"},
        {"TELEMETRY_PUBLISH_INTERVAL_MS", "1000"},
        {"BATTERY_SUPERVISOR_INTERVAL_MS", "2000"},
        {"SHUTDOWN_WATCHDOG_S", "30"},
        {"MQTT_LINK_LOSS_TIMEOUT_S", "15"},
        {"GEOFENCE_RADIUS_M", "50"},
        {"VEHICLE_NAME", "Riposte 01"},
        {"VEHICLE_MODEL", "skyhunter-x"},
        {"VEHICLE_FIRMWARE", "px4-1.15"},
        {"SITE_ID", "site-01"},
        {"ROUTE_MAX_WAYPOINTS", "20"},
        {"ROUTE_MIN_REL_ALT_M", "2.0"},
        {"ROUTE_MAX_REL_ALT_M", "60.0"},
        {"ROUTE_MAX_DWELL_SEC", "120"},
        {"ROUTE_MAX_TOTAL_DWELL_SEC", "600"},
        {"ROUTE_MIN_SPEED_MPS", "0.5"},
        {"ROUTE_MAX_SPEED_MPS", "8.0"},
        {"LEDGER_DB_PATH", "/var/lib/drone-companion/commands.sqlite3"},
        {"LEDGER_MAX_PENDING_OUTBOX", "256"},
        {"LEDGER_OUTBOX_CLAIM_RESERVE", "32"},
        {"LEDGER_MAX_STATUS_DETAIL_BYTES", "512"},
        {"LEDGER_BUSY_TIMEOUT_MS", "5000"},
        {"EXECUTOR_MISSION_QUEUE_CAPACITY", "1"},
        {"EXECUTOR_SAFETY_QUEUE_CAPACITY", "4"},
        {"EXECUTOR_TERMINAL_EVENT_CAPACITY", "16"},
    };
    return values;
}

void apply_valid_environment()
{
    for (const auto& entry : valid_environment()) {
        static_cast<void>(::setenv(entry.first, entry.second, 1));
    }
}

[[nodiscard]] bool loads()
{
    try {
        static_cast<void>(dc::mqtt::load_agent_config());
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void check_rejected(
    const char* const name, const char* const value, const std::string& what)
{
    apply_valid_environment();
    static_cast<void>(::setenv(name, value, 1));
    test_support::check(!loads(), what);
    apply_valid_environment();
}

void check_missing_rejected(const char* const name)
{
    apply_valid_environment();
    static_cast<void>(::unsetenv(name));
    test_support::check(
        !loads(), std::string{name} + " is required");
    apply_valid_environment();

    // systemd exports an empty EnvironmentFile entry rather than dropping it,
    // so empty must fail exactly like unset.
    static_cast<void>(::setenv(name, "", 1));
    test_support::check(
        !loads(), std::string{name} + " must not be accepted empty");
    apply_valid_environment();
}

void test_valid_configuration_loads()
{
    apply_valid_environment();
    dc::mqtt::AgentConfig config;
    bool threw = false;
    try {
        config = dc::mqtt::load_agent_config();
    } catch (const std::exception&) {
        threw = true;
    }
    test_support::check(!threw, "the documented template loads");
    if (threw) {
        return;
    }

    test_support::check(
        !config.process.execute_flight, "flight execution defaults to disabled");
    test_support::check(
        config.process.vehicle_id == "DRONE-01", "vehicle id is carried");
    test_support::check(
        config.session.broker_host == "10.100.0.1"
            && config.session.broker_port == 8883U,
        "broker URI is split into host and port");
    test_support::check(
        config.session.client_id == config.process.vehicle_id,
        "client id is bound to the vehicle id");
    test_support::check(
        config.session.registration.robot_id == config.process.vehicle_id,
        "registration robot id is derived, not configured");
    test_support::check(
        config.ledger.vehicle_id == config.process.vehicle_id,
        "ledger vehicle id is derived, not configured");
    test_support::check(
        config.source.codec.route.max_waypoints == config.route.max_waypoints
            && config.source.codec.route.max_relative_altitude_m
                == config.route.max_relative_altitude_m,
        "decode-time and dispatch-time route limits are the same object");
    test_support::check(
        config.freshness.max_age == std::chrono::seconds{300}
            && config.freshness.max_future_skew == std::chrono::seconds{10},
        "freshness limits are carried");
    test_support::check(
        config.process.telemetry_interval == std::chrono::milliseconds{1000},
        "telemetry interval is carried");
}

void test_every_variable_is_required()
{
    for (const auto& entry : valid_environment()) {
        check_missing_rejected(entry.first);
    }
}

void test_identity_and_topic_bindings()
{
    check_rejected(
        "MQTT_CLIENT_ID", "DRONE-02", "client id differing from vehicle id");
    check_rejected(
        "MQTT_VEHICLE_ID", "DRONE 01", "vehicle id with an illegal character");
    check_rejected(
        "MQTT_COMMAND_TOPIC",
        "aegisq/cmd/DRONE-02",
        "command topic that is not the derived topic");
    check_rejected(
        "MQTT_COMMAND_ACK_TOPIC",
        "aegisq/ack/DRONE-01",
        "ack topic that is not the derived topic");
}

void test_broker_uri_rules()
{
    check_rejected(
        "MQTT_BROKER_URI", "mqtt://10.100.0.1:8883", "a non-TLS broker scheme");
    check_rejected(
        "MQTT_BROKER_URI", "mqtts://10.100.0.1:1883", "a broker port that is not 8883");
    check_rejected(
        "MQTT_BROKER_URI",
        "mqtts://broker.internal:8883",
        "a broker host name instead of an IPv4 literal");
    check_rejected(
        "MQTT_BROKER_URI",
        "mqtts://10.100.0.1:8883/mqtt",
        "a broker URI carrying a path");
    check_rejected(
        "MQTT_BROKER_URI",
        "mqtts://user@10.100.0.1:8883",
        "a broker URI carrying userinfo");
    check_rejected(
        "WIREGUARD_SERVER_IP",
        "10.100.0.2",
        "a WireGuard peer that is not the broker host");
}

void test_numeric_and_cross_field_rules()
{
    check_rejected("MQTT_QOS", "0", "QoS 0 for flight commands");
    check_rejected("MQTT_QOS", "2", "QoS 2 for flight commands");
    check_rejected(
        "MQTT_RECONNECT_MAX_SEC", "0", "a reconnect ceiling below the floor");
    check_rejected(
        "MQTT_MAX_JSON_DEPTH", "2", "a JSON depth below the envelope's needs");
    check_rejected("MQTT_COMMAND_MAX_AGE_SEC", "0", "a zero freshness window");
    check_rejected(
        "MQTT_MAX_FUTURE_SKEW_SEC", "-1", "a negative clock skew allowance");
    check_rejected("ROUTE_MAX_WAYPOINTS", "0", "a zero waypoint budget");
    check_rejected(
        "ROUTE_MAX_REL_ALT_M", "1.0", "a ceiling below ROUTE_MIN_REL_ALT_M");
    check_rejected(
        "ROUTE_MAX_SPEED_MPS", "0.4", "a speed ceiling below ROUTE_MIN_SPEED_MPS");
    check_rejected("ROUTE_MAX_DWELL_SEC", "0", "a zero dwell budget");
    check_rejected(
        "ROUTE_MIN_REL_ALT_M", "nan", "a non-finite altitude limit");
    check_rejected(
        "ROUTE_MAX_REL_ALT_M", "inf", "an infinite altitude limit");
    check_rejected(
        "EXECUTOR_TERMINAL_EVENT_CAPACITY",
        "5",
        "a terminal event capacity below mission + safety + 1");
    check_rejected(
        "LEDGER_OUTBOX_CLAIM_RESERVE",
        "256",
        "an outbox reserve that is not below the outbox bound");
    check_rejected(
        "LEDGER_DB_PATH", "commands.sqlite3", "a relative ledger path");
    check_rejected("MQTT_CA_FILE", "certs/ca.crt", "a relative CA path");
    check_rejected(
        "REMOTE_FLIGHT_EXECUTE", "true", "a flight gate that is not 0 or 1");
    check_rejected(
        "REMOTE_FLIGHT_EXECUTE", "2", "a flight gate outside 0 or 1");
    check_rejected(
        "MQTT_COMMAND_QUEUE_CAPACITY", "8x", "a bound with trailing characters");
}

}  // namespace

int main()
{
    test_valid_configuration_loads();
    test_every_variable_is_required();
    test_identity_and_topic_bindings();
    test_broker_uri_rules();
    test_numeric_and_cross_field_rules();
    return test_support::report("mqtt_agent_config_tests");
}
