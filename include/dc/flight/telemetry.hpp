#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include "dc/flight/rc_ownership.hpp"

namespace dc::flight {

struct GlobalPosition {
    double latitude_deg{};
    double longitude_deg{};
    double absolute_altitude_m{};
    double relative_altitude_m{};
};

// Great-circle distance, ignoring altitude. Deliberately duplicated here
// rather than taken from include/riposte/geo.hpp: the MQTT layer must stay
// buildable and unit-testable without the flight-core geometry headers, which
// is the whole reason for the dependency direction.
[[nodiscard]] inline double horizontal_distance_m(
    const GlobalPosition& from, const GlobalPosition& to) noexcept
{
    constexpr double earth_radius_m = 6371000.0;
    constexpr double deg_to_rad = 3.14159265358979323846 / 180.0;
    const double lat_from = from.latitude_deg * deg_to_rad;
    const double lat_to = to.latitude_deg * deg_to_rad;
    const double delta_lat = lat_to - lat_from;
    const double delta_lon = (to.longitude_deg - from.longitude_deg) * deg_to_rad;
    const double half_lat = std::sin(delta_lat / 2.0);
    const double half_lon = std::sin(delta_lon / 2.0);
    const double h = (half_lat * half_lat)
        + (std::cos(lat_from) * std::cos(lat_to) * half_lon * half_lon);
    // clamp: rounding can push h just past 1 for antipodal inputs, and asin
    // would return NaN, which would read as "inside the fence" downstream.
    return 2.0 * earth_radius_m * std::asin(std::sqrt(std::min(1.0, h)));
}

struct VehicleTelemetrySnapshot {
    std::string vehicle_id;
    bool connected{};
    bool armed{};
    bool in_air{};
    bool home_valid{};
    bool global_position_valid{};
    ControlOwner control_owner{ControlOwner::unknown};
    OwnershipSwitch ownership_switch{OwnershipSwitch::unknown};
    std::optional<std::uint16_t> ownership_pwm;
    ArmSwitchPosition arm_switch{ArmSwitchPosition::unknown};
    std::optional<std::uint16_t> arm_switch_pwm;
    std::uint64_t ownership_epoch{};
    // True means a new mission may safely create a fresh Offboard session.
    // It is deliberately computed on the vehicle, where RC freshness and
    // PX4's OnGround state are both available, rather than inferred by the UI.
    bool ready_for_mission{};
    std::optional<double> battery_percent;
    std::optional<double> battery_voltage_v;
    std::optional<double> battery_temperature_c;
    std::optional<double> heading_deg;
    std::optional<double> horizontal_speed_m_s;
    std::optional<GlobalPosition> position;
    std::optional<GlobalPosition> home_position;
    std::string flight_mode;
    std::chrono::system_clock::time_point occurred_at{};
};

class VehicleTelemetryProvider {
public:
    virtual ~VehicleTelemetryProvider() = default;
    virtual VehicleTelemetrySnapshot snapshot() noexcept = 0;
};

}  // namespace dc::flight
