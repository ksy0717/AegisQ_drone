#pragma once

#include <chrono>
#include <string>
#include <string_view>

#include "dc/flight/telemetry.hpp"

namespace dc::mqtt {

struct VehicleRegistration {
    std::string robot_id;
    std::string name;
    std::string model;
    std::string firmware;
    std::string site_id;
};

[[nodiscard]] std::string telemetry_topic_for(std::string_view vehicle_id);
[[nodiscard]] std::string registration_topic_for(std::string_view vehicle_id);
[[nodiscard]] std::string lwt_topic_for(std::string_view vehicle_id);

[[nodiscard]] std::string encode_vehicle_registration(
    const VehicleRegistration& registration);
[[nodiscard]] std::string encode_vehicle_telemetry(
    const flight::VehicleTelemetrySnapshot& snapshot);
[[nodiscard]] std::string encode_vehicle_lwt(
    std::string_view vehicle_id,
    std::string_view reason,
    std::chrono::system_clock::time_point occurred_at);

}  // namespace dc::mqtt
