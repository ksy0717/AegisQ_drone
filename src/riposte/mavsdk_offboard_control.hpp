#pragma once

#include <stdexcept>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/system.h>

#include "dc/flight/rc_ownership.hpp"
#include "trajectory.hpp"

namespace riposte_runtime {

// Thrown when fresh raw RC evidence says CH6 selected POSITION, or when RC
// ownership becomes ambiguous while we are driving. It is a separate type
// because ordinary Offboard faults use a controlled emergency action, while
// sending any new PX4 mode/action command under a pilot must never happen.
class PilotOverrideDetected final : public std::runtime_error {
public:
    explicit PilotOverrideDetected(const std::string& mode)
        : std::runtime_error("pilot_override_" + mode)
    {
    }
};

struct OffboardControlConfig {
    double setpoint_rate_hz{};
    double setpoint_watchdog_timeout_s{};
    double setpoint_watchdog_poll_s{};
    double watchdog_owner_grace_s{};
    double offboard_mode_confirmation_s{};
    double action_takeoff_min_altitude_m{};
    double offboard_climb_speed_m_s{};
    double offboard_prime_seconds{};
    double offboard_prime_retry_increment_s{};
    std::uint32_t offboard_start_attempts{};
    double takeoff_altitude_tolerance_m{};
    double takeoff_vertical_speed_tolerance_m_s{};
    double takeoff_settle_s{};
    double takeoff_timeout_s{};
    double landing_timeout_s{};
    double rtl_timeout_s{};
    double command_poll_s{};
    double disarm_timeout_s{};
};

using AbortProbe = std::function<bool()>;
using FaultCallback = std::function<void(const std::string&)>;
using SetpointObserver = std::function<void(
    const std::string&,
    const bullet_mission::TrajectorySetpoint&,
    const mavsdk::Telemetry::PositionVelocityNed&,
    float)>;
using FlushObserver = std::function<void()>;

// MAVSDK-dependent flight mechanics shared by riposte_test and the MQTT
// adapter. All ordinary calls are made by one owner thread. The watchdog is an
// intentionally narrow emergency interlock: if that owner stops generating
// setpoints, it is allowed to request Offboard stop + Land.
class MavsdkOffboardControl final {
public:
    MavsdkOffboardControl(
        std::shared_ptr<mavsdk::System> system,
        mavsdk::Action& action,
        mavsdk::Telemetry& telemetry,
        OffboardControlConfig config,
        AbortProbe abort_probe,
        FaultCallback fault_callback = {},
        SetpointObserver setpoint_observer = {},
        FlushObserver flush_observer = {},
        const dc::flight::RcOwnershipProvider* ownership_provider = nullptr);
    ~MavsdkOffboardControl();

    MavsdkOffboardControl(const MavsdkOffboardControl&) = delete;
    MavsdkOffboardControl& operator=(const MavsdkOffboardControl&) = delete;

    void set_flight_volume(
        const bullet_mission::NedPoint& ground_reference,
        double radius_limit_m);
    void takeoff_to(
        const bullet_mission::NedPoint& ground,
        double altitude_m,
        const std::string& phase);
    mavsdk::Telemetry::PositionVelocityNed send_setpoint(
        const std::string& phase,
        const bullet_mission::TrajectorySetpoint& target);

    void land_and_wait();
    void stop_and_hold();
    void return_to_launch_and_wait();
    void emergency_land_and_wait() noexcept;
    void recover_after_restart();

    // A manual takeover is the one shutdown path where even Offboard::stop()
    // is unsafe: MAVSDK implements it by commanding Hold. Invalidate and join
    // the watchdog first, then destroy the plugin to stop its cached 20 Hz
    // sender without transmitting a PX4 mode/action command.
    void yield_to_pilot() noexcept;
    // A queued STOP/RTL preempts the route before it runs its own safety
    // action. Remove any cached setpoint stream from the obsolete route
    // without changing PX4 mode or latching pilot ownership.
    void cancel_for_external_preemption() noexcept;
    // Recreates the Offboard plugin only after a fresh RC handback and a
    // positively confirmed landed/disarmed state. A destroyed plugin can never
    // carry the previous session's cached target into a new arming cycle.
    void prepare_for_new_ground_mission(std::uint64_t dispatch_takeover_epoch);

    [[nodiscard]] double battery_remaining_fraction() const noexcept;
    [[nodiscard]] bool connected() const noexcept;
    [[nodiscard]] bool armed() const noexcept;
    [[nodiscard]] bool in_air() const noexcept;
    [[nodiscard]] bool ground_disarmed() const noexcept;
    [[nodiscard]] dc::flight::RcOwnershipSnapshot ownership_snapshot() const noexcept;
    [[nodiscard]] bool watchdog_triggered() const noexcept;
    // Clears a watchdog latch left over from a previous operation. Refuses to
    // clear while the watchdog thread is running: mid-flight the latch is the
    // abort signal every 20 Hz loop relies on, and only a fresh command on an
    // idle vehicle may discard it.
    void reset_watchdog_latch() noexcept;
    // True when PX4 is in one of its own pilot modes with the aircraft armed
    // or airborne: someone is flying it right now. Distinct from the override
    // thrown mid-flight, which only fires once we were already driving.
    [[nodiscard]] bool pilot_in_control() const noexcept;

private:
    using SteadyRep = std::chrono::steady_clock::duration::rep;

    enum class OwnershipAssessment {
        autonomous,
        pilot,
        ambiguous,
        offboard_fault,
    };

    static bullet_mission::TrajectorySetpoint stationary_setpoint(
        const bullet_mission::NedPoint& point, double yaw_deg);
    static SteadyRep steady_now_ticks() noexcept;

    void validate_config() const;
    void check_abort();
    void require_ground_start_switches() const;
    // CH6 is authoritative for a handover. Once Offboard has been confirmed,
    // a non-Offboard PX4 mode under the unchanged PI switch is a flight fault.
    void check_pilot_override();
    [[nodiscard]] OwnershipAssessment assess_ownership(
        mavsdk::Telemetry::FlightMode mode) const noexcept;
    [[nodiscard]] bool watchdog_session_active(std::uint64_t generation) const noexcept;
    void finish_watchdog_session(std::uint64_t generation) noexcept;
    void latch_watchdog_pilot_takeover(
        std::uint64_t generation, const std::string& detail) noexcept;
    void prime_and_start_offboard(const bullet_mission::NedPoint& point);
    void wait_for_takeoff_height(
        const bullet_mission::NedPoint& ground, double altitude_m);
    void climb_to(
        const std::string& phase,
        const bullet_mission::NedPoint& start,
        const bullet_mission::NedPoint& finish,
        double speed_m_s,
        double yaw_deg);
    void stop_offboard(const std::string& operation, bool require_success);
    void wait_until_landed(double timeout_s, bool interruptible);
    void start_setpoint_watchdog();
    void stop_setpoint_watchdog() noexcept;
    void report_fault(const std::string& detail) noexcept;

    template<typename Callable>
    decltype(auto) run_px4_command(Callable&& callable)
    {
        std::lock_guard<std::mutex> lock{px4_command_mutex_};
        return std::forward<Callable>(callable)();
    }

    std::shared_ptr<mavsdk::System> system_;
    mavsdk::Action& action_;
    mavsdk::Telemetry& telemetry_;
    std::unique_ptr<mavsdk::Offboard> offboard_;
    const dc::flight::RcOwnershipProvider* ownership_provider_{};
    OffboardControlConfig config_;
    AbortProbe abort_probe_;
    FaultCallback fault_callback_;
    SetpointObserver setpoint_observer_;
    FlushObserver flush_observer_;

    bullet_mission::NedPoint ground_reference_{};
    double radius_limit_m_{};
    bool flight_volume_configured_{};
    // Distinct from offboard_started_ on purpose: the setpoint watchdog clears
    // that flag itself before the mission thread ever observes the fault,
    // which would let a pilot takeover be reported as a watchdog expiry. This
    // one is cleared only by a deliberate stop.
    std::atomic_bool offboard_intended_{false};
    std::atomic_bool offboard_started_{false};
    std::atomic<SteadyRep> offboard_started_ticks_{0};
    // These are set immediately before the blocking MAVSDK call. If PX4
    // applies a command but its ACK is lost, cleanup must not mistake that
    // fail-uncertain state for "never issued".
    std::atomic_bool arm_command_may_have_applied_{false};
    std::atomic_bool takeoff_command_may_have_applied_{false};
    std::atomic_bool offboard_start_may_have_applied_{false};
    // MAVSDK begins its cached 20 Hz sender on the first prime setpoint, before
    // Offboard::start() succeeds. Keep that lifetime separate from flight mode.
    std::atomic_bool offboard_sender_active_{false};
    std::atomic_bool pilot_takeover_latched_{false};
    std::atomic_bool mission_takeover_epoch_valid_{false};
    std::atomic<std::uint64_t> mission_takeover_epoch_{0U};
    std::atomic_bool watchdog_running_{false};
    std::atomic<std::uint64_t> watchdog_generation_{0U};
    std::atomic<std::uint64_t> watchdog_triggered_generation_{0U};
    std::atomic<SteadyRep> last_setpoint_generation_ticks_{0};
    std::mutex px4_command_mutex_{};
    std::mutex watchdog_wait_mutex_{};
    std::condition_variable watchdog_wake_{};
    std::thread watchdog_thread_{};
};

}  // namespace riposte_runtime
