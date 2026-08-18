#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "dc/mqtt/aegisq_state_codec.hpp"
#include "test_support.hpp"

namespace {

using namespace dc;

flight::VehicleTelemetrySnapshot snapshot()
{
    flight::VehicleTelemetrySnapshot value;
    value.vehicle_id = "DRONE-01";
    value.connected = true;
    value.armed = true;
    value.in_air = true;
    value.home_valid = true;
    value.global_position_valid = true;
    value.control_owner = flight::ControlOwner::pi;
    value.ownership_switch = flight::OwnershipSwitch::hold;
    value.ownership_pwm = 1500U;
    value.arm_switch = flight::ArmSwitchPosition::disarm;
    value.arm_switch_pwm = 1000U;
    value.ownership_epoch = 3U;
    value.ready_for_mission = false;
    value.battery_percent = 30.0;
    value.battery_voltage_v = 22.4;
    value.battery_temperature_c = 31.5;
    value.heading_deg = 91.0;
    value.horizontal_speed_m_s = 4.2;
    value.position = flight::GlobalPosition{37.1, 127.2, 118.4, 10.0};
    value.home_position = flight::GlobalPosition{37.0, 127.0, 108.4, 0.0};
    value.flight_mode = "OFFBOARD";
    value.occurred_at =
        std::chrono::system_clock::time_point{std::chrono::seconds{1'800'000'000}};
    return value;
}

void test_topics_and_registration()
{
    test_support::check(
        mqtt::telemetry_topic_for("DRONE-01")
            == "aegisq/telemetry/DRONE-01",
        "telemetry topic is exact");
    test_support::check(
        mqtt::registration_topic_for("DRONE-01")
            == "aegisq/robot-register/DRONE-01",
        "registration topic is exact");
    test_support::check(
        mqtt::lwt_topic_for("DRONE-01") == "aegisq/lwt/DRONE-01",
        "LWT topic is exact");

    const nlohmann::json encoded = nlohmann::json::parse(
        mqtt::encode_vehicle_registration({
            "DRONE-01", "AegisQ Drone 01", "Holybro X500 V2",
            "mqtt-px4-control", "site-poc-01"}));
    test_support::check(
        encoded.at("robotId") == "DRONE-01"
            && encoded.at("mobilityClass") == "DRONE"
            && encoded.at("siteId") == "site-poc-01",
        "registration matches prototype-v2");
}

void test_flight_telemetry_contract()
{
    const nlohmann::json encoded =
        nlohmann::json::parse(mqtt::encode_vehicle_telemetry(snapshot()));
    test_support::check(
        encoded.at("robotId") == "DRONE-01"
            && encoded.at("status") == "ACTIVE"
            && encoded.at("mobilityClass") == "DRONE",
        "telemetry identifies an active drone");
    test_support::check(
        encoded.at("battery") == 30.0
            && encoded.at("homeValid") == true
            && encoded.at("globalPositionValid") == true
            && encoded.at("armed") == true
            && encoded.at("inAir") == true
            && encoded.at("controlOwner") == "PI"
            && encoded.at("readyForMission") == false,
        "flight gates and battery are present");
    test_support::check(
        encoded.at("position").at("lat") == 37.1
            && encoded.at("position").at("speed") == 4.2
            && encoded.at("homePosition").at("absoluteAltitude") == 108.4,
        "current and PX4 Home positions are distinct");
    test_support::check(
        encoded.at("hardware").at("ownershipSwitch") == "HOLD"
            && encoded.at("hardware").at("ownershipPwm") == 1500
            && encoded.at("hardware").at("armSwitch") == "DISARM"
            && encoded.at("hardware").at("armSwitchPwm") == 1000
            && encoded.at("hardware").at("ownershipEpoch") == 3
            && encoded.at("hardware").at("flightMode") == "OFFBOARD",
        "ownership, arm switch, and PX4 mode are reported independently");
}

void test_invalid_home_is_not_published()
{
    auto value = snapshot();
    value.home_valid = false;
    value.global_position_valid = false;
    value.position = flight::GlobalPosition{0.0, 0.0, 0.0, 0.0};
    const nlohmann::json encoded =
        nlohmann::json::parse(mqtt::encode_vehicle_telemetry(value));
    test_support::check(
        !encoded.contains("homePosition") && !encoded.contains("position"),
        "unverified navigation positions are omitted");
}

void test_ground_handover_and_unknown_owner_are_explicit()
{
    auto ready = snapshot();
    ready.armed = false;
    ready.in_air = false;
    ready.ready_for_mission = true;
    const nlohmann::json ready_encoded = nlohmann::json::parse(
        mqtt::encode_vehicle_telemetry(ready));
    test_support::check(
        ready_encoded.at("status") == "STANDBY"
            && ready_encoded.at("controlOwner") == "PI"
            && ready_encoded.at("readyForMission") == true,
        "a confirmed ground HOLD handback is an explicit ready state");

    ready.control_owner = flight::ControlOwner::unknown;
    ready.ownership_switch = flight::OwnershipSwitch::unknown;
    ready.ownership_pwm.reset();
    ready.arm_switch = flight::ArmSwitchPosition::unknown;
    ready.arm_switch_pwm.reset();
    ready.ready_for_mission = false;
    const nlohmann::json unknown_encoded = nlohmann::json::parse(
        mqtt::encode_vehicle_telemetry(ready));
    test_support::check(
        unknown_encoded.at("controlOwner") == "UNKNOWN"
            && unknown_encoded.at("readyForMission") == false
            && unknown_encoded.at("hardware").at("ownershipSwitch")
                == "UNKNOWN"
            && unknown_encoded.at("hardware").at("armSwitch") == "UNKNOWN"
            && !unknown_encoded.at("hardware").contains("ownershipPwm")
            && !unknown_encoded.at("hardware").contains("armSwitchPwm"),
        "missing ownership proof is published fail-closed");
}

void test_lwt_contract()
{
    const nlohmann::json encoded = nlohmann::json::parse(
        mqtt::encode_vehicle_lwt(
            "DRONE-01",
            "abnormal-disconnect",
            std::chrono::system_clock::time_point{
                std::chrono::seconds{1'800'000'000}}));
    test_support::check(
        encoded.at("status") == "OFFLINE"
            && encoded.at("reason") == "abnormal-disconnect",
        "LWT marks the exact vehicle offline");
}

}  // namespace

int main()
{
    test_topics_and_registration();
    test_flight_telemetry_contract();
    test_invalid_home_is_not_published();
    test_ground_handover_and_unknown_owner_are_explicit();
    test_lwt_contract();
    std::cout << "aegisq state codec tests passed\n";
    return 0;
}
