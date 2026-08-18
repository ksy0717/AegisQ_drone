#pragma once

#include <chrono>
#include <atomic>
#include <memory>
#include <optional>
#include <string>

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/system.h>

#include "dc/flight/preemptive_mission_executor.hpp"
#include "dc/flight/telemetry.hpp"
#include "mavsdk_offboard_control.hpp"

namespace dc::flight {

struct MavsdkMissionBackendConfig {
    riposte_runtime::OffboardControlConfig offboard;
    double cruise_speed_m_s{};
    double vertical_speed_m_s{};
    double horizontal_tolerance_m{};
    double vertical_tolerance_floor_m{};
    double vertical_tolerance_ratio{};
    double arrival_speed_tolerance_m_s{};
    double arrival_settle_s{};
    double arrival_timeout_s{};
    double health_timeout_s{};
    double position_timeout_s{};
    double battery_telemetry_timeout_s{};
    double preflight_min_battery_fraction{};
    double low_battery_rtl_fraction{};
    double max_route_radius_m{};
    double max_home_launch_horizontal_m{};
    double max_home_launch_altitude_delta_m{};
};

// The only ordinary owner of Action/Offboard commands. It adapts AegisQ's
// absolute WGS84 route into the same local-NED setpoint mechanics used by the
// verified Riposte test runner.
class MavsdkMissionBackend final : public MissionBackend,
                                   public VehicleTelemetryProvider {
public:
    MavsdkMissionBackend(
        std::string vehicle_id,
        std::shared_ptr<mavsdk::System> system,
        mavsdk::Action& action,
        mavsdk::Telemetry& telemetry,
        const RcOwnershipProvider& ownership_provider,
        MavsdkMissionBackendConfig config);
    ~MavsdkMissionBackend() override;

    MavsdkMissionBackend(const MavsdkMissionBackend&) = delete;
    MavsdkMissionBackend& operator=(const MavsdkMissionBackend&) = delete;

    CommandResult recover_after_restart(const PreemptionToken& token) override;
    FlightOperationResult execute(
        const RemoteCommand& command,
        const PreemptionToken& token) override;
    FlightOperationResult shutdown_safely(
        const PreemptionToken& token) override;
    [[nodiscard]] std::optional<std::uint64_t>
    mission_authorization_epoch() const noexcept override;
    VehicleTelemetrySnapshot snapshot() noexcept override;

private:
    struct RouteContext;

    [[nodiscard]] FlightOperationResult execute_route(const RemoteCommand& command);
    [[nodiscard]] FlightOperationResult execute_safety(const RemoteCommand& command);
    [[nodiscard]] RouteContext prepare_route(const RoutePlan& route);
    void wait_for_vehicle_health();
    void check_preflight_battery();
    void check_route_abort();
    // The operator's speed for this leg, in metres per second. Per-waypoint
    // wins over the route's cruise speed, which wins over the configured
    // default. Nothing is clamped here: a speed outside the vehicle's
    // RouteLimits was already rejected, and quietly flying slower than the
    // console shows would be its own kind of wrong.
    [[nodiscard]] double leg_speed_m_s(
        const RoutePlan& route, const GeoWaypoint& waypoint) const noexcept;
    void move_to(
        const std::string& phase,
        const bullet_mission::NedPoint& start,
        const bullet_mission::NedPoint& finish,
        double speed_m_s);
    void settle_at(
        const std::string& phase,
        const bullet_mission::NedPoint& target,
        double yaw_deg,
        double altitude_m);
    void hold_at(
        const std::string& phase,
        const bullet_mission::NedPoint& target,
        double yaw_deg,
        std::chrono::seconds duration);
    [[nodiscard]] bool external_preemption_requested() const noexcept;
    [[nodiscard]] bool low_battery_triggered();
    [[nodiscard]] bool battery_telemetry_lost();
    [[nodiscard]] std::string preemption_detail() const;
    [[nodiscard]] double vertical_tolerance_for(double altitude_m) const noexcept;
    void validate_config() const;

    std::string vehicle_id_;
    std::shared_ptr<mavsdk::System> system_;
    mavsdk::Action& action_;
    mavsdk::Telemetry& telemetry_;
    const RcOwnershipProvider& ownership_provider_;
    MavsdkMissionBackendConfig config_;
    const PreemptionToken* active_token_{};
    // Set by fresh CH6 POSITION or ambiguous RC ownership. Once set, this
    // backend issues nothing to the aircraft -- not a mission, STOP, RTL, or
    // emergency Land. Ground+PX4 DISARMED+fresh CH6 HOLD+CH5 DISARM makes a
    // new session eligible; moving the switches while airborne never does.
    std::atomic_bool pilot_override_latched_{false};
    bool internal_failsafe_{};
    bool low_battery_abort_{};
    bool battery_loss_abort_{};
    std::optional<std::chrono::steady_clock::time_point> last_valid_battery_at_;
    std::optional<std::uint64_t> active_start_ownership_epoch_;
    riposte_runtime::MavsdkOffboardControl flight_control_;
};

}  // namespace dc::flight
