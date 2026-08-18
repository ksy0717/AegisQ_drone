#include "mavsdk_offboard_control.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "safety_policy.hpp"

using namespace std::chrono_literals;

namespace riposte_runtime {
namespace {

void require_action_success(
    const mavsdk::Action::Result result, const std::string& operation)
{
    if (result != mavsdk::Action::Result::Success) {
        std::ostringstream message;
        message << operation << " failed, MAVSDK Action result=" << result;
        throw std::runtime_error(message.str());
    }
}

void require_offboard_success(
    const mavsdk::Offboard::Result result, const std::string& operation)
{
    if (result != mavsdk::Offboard::Result::Success) {
        std::ostringstream message;
        message << operation << " failed, MAVSDK Offboard result=" << result;
        throw std::runtime_error(message.str());
    }
}

bool finite_positive(const double value) noexcept
{
    return std::isfinite(value) && value > 0.0;
}

}  // namespace

MavsdkOffboardControl::MavsdkOffboardControl(
    std::shared_ptr<mavsdk::System> system,
    mavsdk::Action& action,
    mavsdk::Telemetry& telemetry,
    OffboardControlConfig config,
    AbortProbe abort_probe,
    FaultCallback fault_callback,
    SetpointObserver setpoint_observer,
    FlushObserver flush_observer,
    const dc::flight::RcOwnershipProvider* ownership_provider)
    : system_(std::move(system)),
      action_(action),
      telemetry_(telemetry),
      ownership_provider_(ownership_provider),
      config_(config),
      abort_probe_(std::move(abort_probe)),
      fault_callback_(std::move(fault_callback)),
      setpoint_observer_(std::move(setpoint_observer)),
      flush_observer_(std::move(flush_observer))
{
    if (!system_) {
        throw std::invalid_argument("MAVSDK system is required");
    }
    if (!abort_probe_) {
        throw std::invalid_argument("abort probe is required");
    }
    validate_config();
    offboard_ = std::make_unique<mavsdk::Offboard>(system_);
}

MavsdkOffboardControl::~MavsdkOffboardControl()
{
    stop_setpoint_watchdog();
    try {
        std::lock_guard<std::mutex> lock{px4_command_mutex_};
        offboard_.reset();
    } catch (...) {
    }
}

void MavsdkOffboardControl::validate_config() const
{
    if (!finite_positive(config_.setpoint_rate_hz)
        || !finite_positive(config_.setpoint_watchdog_timeout_s)
        || !finite_positive(config_.setpoint_watchdog_poll_s)
        || !finite_positive(config_.watchdog_owner_grace_s)
        || !finite_positive(config_.offboard_mode_confirmation_s)
        || config_.setpoint_watchdog_timeout_s
            <= 2.0 / config_.setpoint_rate_hz
        || config_.watchdog_owner_grace_s
            <= 2.0 / config_.setpoint_rate_hz
        || !finite_positive(config_.action_takeoff_min_altitude_m)
        || !finite_positive(config_.offboard_climb_speed_m_s)
        || !finite_positive(config_.offboard_prime_seconds)
        || !finite_positive(config_.offboard_prime_retry_increment_s)
        || config_.offboard_start_attempts == 0U
        || !finite_positive(config_.takeoff_altitude_tolerance_m)
        || !finite_positive(config_.takeoff_vertical_speed_tolerance_m_s)
        || !finite_positive(config_.takeoff_settle_s)
        || !finite_positive(config_.takeoff_timeout_s)
        || !finite_positive(config_.landing_timeout_s)
        || !finite_positive(config_.rtl_timeout_s)
        || !finite_positive(config_.command_poll_s)
        || !finite_positive(config_.disarm_timeout_s)) {
        throw std::invalid_argument("invalid MAVSDK offboard control configuration");
    }
}

void MavsdkOffboardControl::set_flight_volume(
    const bullet_mission::NedPoint& ground_reference,
    const double radius_limit_m)
{
    if (!std::isfinite(ground_reference.north_m)
        || !std::isfinite(ground_reference.east_m)
        || !std::isfinite(ground_reference.down_m)
        || !finite_positive(radius_limit_m)) {
        throw std::invalid_argument("invalid flight volume");
    }
    ground_reference_ = ground_reference;
    radius_limit_m_ = radius_limit_m;
    flight_volume_configured_ = true;
}

bullet_mission::TrajectorySetpoint MavsdkOffboardControl::stationary_setpoint(
    const bullet_mission::NedPoint& point, const double yaw_deg)
{
    bullet_mission::TrajectorySetpoint setpoint{};
    setpoint.position = point;
    setpoint.yaw_deg = yaw_deg;
    return setpoint;
}

namespace {

// The modes a human flies in. Hold, Land, RTL, Takeoff and Mission are PX4
// driving itself: losing Offboard to one of those is a fault, not a handover,
// and calling it an override latches the vehicle out of service for a reason
// that never involved a pilot.
[[nodiscard]] bool is_pilot_mode(
    const mavsdk::Telemetry::FlightMode mode) noexcept
{
    return mode == mavsdk::Telemetry::FlightMode::Manual
        || mode == mavsdk::Telemetry::FlightMode::Posctl
        || mode == mavsdk::Telemetry::FlightMode::Altctl
        || mode == mavsdk::Telemetry::FlightMode::Acro
        || mode == mavsdk::Telemetry::FlightMode::Stabilized
        || mode == mavsdk::Telemetry::FlightMode::Rattitude;
}

}  // namespace

bool MavsdkOffboardControl::pilot_in_control() const noexcept
{
    try {
        if (pilot_takeover_latched_.load(std::memory_order_acquire)) {
            return true;
        }
        if (ownership_provider_ != nullptr
            && mission_takeover_epoch_valid_.load(std::memory_order_acquire)
            && ownership_provider_->takeover_epoch()
                != mission_takeover_epoch_.load(std::memory_order_acquire)) {
            return true;
        }
        const mavsdk::Telemetry::FlightMode mode = telemetry_.flight_mode();
        if (ownership_provider_ == nullptr) {
            // riposte_test has no RC_CHANNELS adapter. Preserve its historical
            // mode-only interlock instead of silently weakening that runner.
            return is_pilot_mode(mode);
        }
        const dc::flight::RcOwnershipSnapshot ownership = ownership_snapshot();
        if (ownership.fresh
            && (ownership.owner == dc::flight::ControlOwner::pilot
                || ownership.switch_position
                    == dc::flight::OwnershipSwitch::position)) {
            return true;
        }
        if (!telemetry_.armed() && !telemetry_.in_air()) {
            return false;
        }
        // Missing/stale RC while armed or airborne is never evidence that the
        // Pi may issue a new mode command. Yielding here also makes restart and
        // shutdown fail closed before the first RC_CHANNELS sample arrives.
        return ownership.owner == dc::flight::ControlOwner::unknown;
    } catch (...) {
        return ownership_provider_ != nullptr;
    }
}

dc::flight::RcOwnershipSnapshot
MavsdkOffboardControl::ownership_snapshot() const noexcept
{
    if (ownership_provider_ == nullptr) {
        return {};
    }
    try {
        dc::flight::RcOwnershipSnapshot ownership =
            ownership_provider_->snapshot();
        // RC_CHANNELS can retain the last PWM while a receiver is in failsafe.
        // A fresh packet therefore is not sufficient proof by itself: PX4's
        // independent RC_STATUS must also say that the receiver is available.
        if (!telemetry_.rc_status().is_available) {
            ownership.owner = dc::flight::ControlOwner::unknown;
            ownership.switch_position = dc::flight::OwnershipSwitch::unknown;
            ownership.fresh = false;
        }
        return ownership;
    } catch (...) {
        return {};
    }
}

bool MavsdkOffboardControl::ground_disarmed() const noexcept
{
    try {
        return system_->is_connected()
            && !telemetry_.armed()
            && !telemetry_.in_air()
            && telemetry_.landed_state()
                == mavsdk::Telemetry::LandedState::OnGround;
    } catch (...) {
        return false;
    }
}

MavsdkOffboardControl::OwnershipAssessment
MavsdkOffboardControl::assess_ownership(
    const mavsdk::Telemetry::FlightMode mode) const noexcept
{
    if (ownership_provider_ == nullptr) {
        return offboard_intended_.load(std::memory_order_acquire)
                && is_pilot_mode(mode)
            ? OwnershipAssessment::pilot
            : OwnershipAssessment::autonomous;
    }

    const dc::flight::RcOwnershipSnapshot ownership = ownership_snapshot();
    if (ownership.fresh
        && (ownership.switch_position == dc::flight::OwnershipSwitch::position
            || ownership.owner == dc::flight::ControlOwner::pilot)) {
        return OwnershipAssessment::pilot;
    }
    if (ownership.fresh
        && ownership.owner == dc::flight::ControlOwner::pi) {
        if (!offboard_started_.load(std::memory_order_acquire)
            || mode == mavsdk::Telemetry::FlightMode::Offboard) {
            return OwnershipAssessment::autonomous;
        }

        // Offboard::start() can return before the next PX4 heartbeat updates
        // MAVSDK's cached flight mode. Ignore only that bounded startup cache
        // window; once it expires, every non-Offboard mode with CH6 still
        // authorizing the Pi is an Offboard-loss fault, never a pilot handover.
        const SteadyRep started_ticks =
            offboard_started_ticks_.load(std::memory_order_acquire);
        const SteadyRep now_ticks = steady_now_ticks();
        const SteadyRep confirmation_ticks =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(
                    config_.offboard_mode_confirmation_s))
                .count();
        if (started_ticks != 0 && now_ticks >= started_ticks
            && now_ticks - started_ticks < confirmation_ticks) {
            return OwnershipAssessment::autonomous;
        }
        return OwnershipAssessment::offboard_fault;
    }
    // During an active route, stale/missing RC is an ownership ambiguity even
    // if PX4 still reports Offboard. Continuing would silently treat loss of
    // the explicit handover signal as permission to keep flying.
    return offboard_intended_.load(std::memory_order_acquire)
            || is_pilot_mode(mode)
        ? OwnershipAssessment::ambiguous
        : OwnershipAssessment::autonomous;
}

void MavsdkOffboardControl::check_pilot_override()
{
    if (pilot_takeover_latched_.load(std::memory_order_acquire)) {
        throw PilotOverrideDetected("rc_latched");
    }
    if (ownership_provider_ != nullptr
        && mission_takeover_epoch_valid_.load(std::memory_order_acquire)
        && ownership_provider_->takeover_epoch()
            != mission_takeover_epoch_.load(std::memory_order_acquire)) {
        pilot_takeover_latched_.store(true, std::memory_order_release);
        throw PilotOverrideDetected("rc_position_epoch_changed");
    }
    if (ownership_provider_ != nullptr) {
        const dc::flight::RcOwnershipSnapshot ownership = ownership_snapshot();
        if (!ownership.fresh) {
            bool active_or_unknown = true;
            try {
                active_or_unknown = telemetry_.armed() || telemetry_.in_air();
            } catch (...) {
            }
            if (active_or_unknown) {
                pilot_takeover_latched_.store(
                    true, std::memory_order_release);
                throw PilotOverrideDetected("rc_ownership_unknown");
            }
        }
    }

    const mavsdk::Telemetry::FlightMode mode = telemetry_.flight_mode();
    const OwnershipAssessment assessment = assess_ownership(mode);
    if (assessment == OwnershipAssessment::pilot
        || assessment == OwnershipAssessment::ambiguous) {
        pilot_takeover_latched_.store(true, std::memory_order_release);
        std::ostringstream name;
        name << (assessment == OwnershipAssessment::pilot
                ? "rc_position_"
                : "ambiguous_")
             << mode;
        throw PilotOverrideDetected(name.str());
    }
    if (assessment == OwnershipAssessment::offboard_fault) {
        std::ostringstream detail;
        detail << "offboard_mode_lost_with_pi_owner_" << mode;
        throw std::runtime_error(detail.str());
    }
}

void MavsdkOffboardControl::check_abort()
{
    // Checked first: every other abort below ends in a landing, and if the
    // pilot already has the aircraft that is the one thing we must not do.
    check_pilot_override();
    if (watchdog_triggered()) {
        throw std::runtime_error("setpoint_generation_watchdog_expired");
    }
    if (abort_probe_()) {
        throw std::runtime_error("flight_operation_preempted");
    }
    if (!system_->is_connected()) {
        throw std::runtime_error("vehicle_heartbeat_lost");
    }
}

void MavsdkOffboardControl::require_ground_start_switches() const
{
    if (ownership_provider_ == nullptr) {
        return;
    }
    const dc::flight::RcOwnershipSnapshot ownership = ownership_snapshot();
    if (!mission_takeover_epoch_valid_.load(std::memory_order_acquire)
        || ownership.takeover_epoch
            != mission_takeover_epoch_.load(std::memory_order_acquire)
        || !ownership.fresh
        || ownership.owner != dc::flight::ControlOwner::pi
        || ownership.switch_position != dc::flight::OwnershipSwitch::hold
        || !ownership.arm_switch.fresh
        || ownership.arm_switch.position
            != dc::flight::ArmSwitchPosition::disarm
        || !ground_disarmed()) {
        throw PilotOverrideDetected(
            "ground_start_authorization_lost");
    }
}

double MavsdkOffboardControl::battery_remaining_fraction() const noexcept
{
    try {
        const float remaining = telemetry_.battery().remaining_percent;
        if (!std::isfinite(remaining) || remaining < 0.0F || remaining > 100.0F) {
            return -1.0;
        }
        return static_cast<double>(remaining) / 100.0;
    } catch (...) {
        return -1.0;
    }
}

bool MavsdkOffboardControl::connected() const noexcept
{
    try {
        return system_->is_connected();
    } catch (...) {
        return false;
    }
}

bool MavsdkOffboardControl::armed() const noexcept
{
    try {
        return connected() && telemetry_.armed();
    } catch (...) {
        return false;
    }
}

bool MavsdkOffboardControl::in_air() const noexcept
{
    try {
        return connected() && telemetry_.in_air();
    } catch (...) {
        return false;
    }
}

bool MavsdkOffboardControl::watchdog_triggered() const noexcept
{
    const std::uint64_t generation =
        watchdog_generation_.load(std::memory_order_acquire);
    return generation != 0U
        && watchdog_triggered_generation_.load(std::memory_order_acquire)
            == generation;
}

void MavsdkOffboardControl::takeoff_to(
    const bullet_mission::NedPoint& ground,
    const double altitude_m,
    const std::string& phase)
{
    if (!flight_volume_configured_ || !finite_positive(altitude_m)) {
        throw std::invalid_argument("takeoff requires configured flight volume and altitude");
    }
    // Both physical switches must remain in the reviewed ground-start state
    // until the software arm request. CH6 POSITION is still the ownership
    // handover after arming; CH5 is not used as a flight-mode proxy.
    require_ground_start_switches();
    // RC POSITION must be able to veto the operation before arm/takeoff, not
    // only after Offboard::start() succeeds.
    offboard_intended_.store(true, std::memory_order_release);
    check_abort();
    const bullet_mission::NedPoint air_point{
        ground.north_m, ground.east_m, ground.down_m - altitude_m};

    if (altitude_m >= config_.action_takeoff_min_altitude_m) {
        require_action_success(
            run_px4_command([this, altitude_m] {
                check_pilot_override();
                return action_.set_takeoff_altitude(
                    static_cast<float>(altitude_m));
            }),
            "set_takeoff_altitude");
        require_action_success(
            run_px4_command([this] {
                require_ground_start_switches();
                check_abort();
                arm_command_may_have_applied_.store(
                    true, std::memory_order_release);
                return action_.arm();
            }),
            "arm");
        require_action_success(
            run_px4_command([this] {
                check_abort();
                takeoff_command_may_have_applied_.store(
                    true, std::memory_order_release);
                return action_.takeoff();
            }),
            "takeoff");
        wait_for_takeoff_height(ground, altitude_m);
        prime_and_start_offboard(air_point);
        return;
    }

    // PX4's MIS_TAKEOFF_ALT floor makes Action::takeoff unsuitable for low
    // altitude tests. Prime Offboard first, then arm and climb explicitly.
    prime_and_start_offboard(ground);
    require_action_success(
        run_px4_command([this] {
            require_ground_start_switches();
            check_abort();
            arm_command_may_have_applied_.store(
                true, std::memory_order_release);
            return action_.arm();
        }),
        "arm");
    // The explicit Offboard climb is the takeoff operation for this path.
    // From here onward a failure is conservatively treated as possibly airborne.
    takeoff_command_may_have_applied_.store(true, std::memory_order_release);
    climb_to(
        phase + "_OFFBOARD_CLIMB",
        ground,
        air_point,
        config_.offboard_climb_speed_m_s,
        0.0);
}

void MavsdkOffboardControl::prime_and_start_offboard(
    const bullet_mission::NedPoint& point)
{
    const auto setpoint = stationary_setpoint(point, 0.0);
    const auto period = std::chrono::duration<double>(1.0 / config_.setpoint_rate_hz);

    for (std::uint32_t attempt = 1U;
         attempt <= config_.offboard_start_attempts;
         ++attempt) {
        const double prime_seconds = config_.offboard_prime_seconds
            + config_.offboard_prime_retry_increment_s
                * static_cast<double>(attempt - 1U);
        const double count_value = std::ceil(prime_seconds * config_.setpoint_rate_hz);
        if (!std::isfinite(count_value) || count_value < 1.0
            || count_value > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
            throw std::runtime_error("offboard_prime_count_out_of_range");
        }
        const auto prime_count = static_cast<std::uint32_t>(count_value);
        auto next = std::chrono::steady_clock::now();
        for (std::uint32_t index = 0U; index < prime_count; ++index) {
            static_cast<void>(send_setpoint("OFFBOARD_PRIME", setpoint));
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next);
        }

        // Keep the last setpoint adjacent to start(); a 1 Hz heartbeat between
        // the two can otherwise make MAVSDK declare the cached setpoint stale.
        static_cast<void>(send_setpoint("OFFBOARD_PRIME", setpoint));
        const mavsdk::Offboard::Result result = run_px4_command([this] {
            check_abort();
            if (!arm_command_may_have_applied_.load(std::memory_order_acquire)) {
                require_ground_start_switches();
            }
            if (!offboard_) {
                throw std::runtime_error("offboard_plugin_not_prepared");
            }
            offboard_start_may_have_applied_.store(
                true, std::memory_order_release);
            return offboard_->start();
        });
        if (result == mavsdk::Offboard::Result::Success) {
            offboard_started_ticks_.store(
                steady_now_ticks(), std::memory_order_release);
            offboard_started_.store(true, std::memory_order_release);
            offboard_intended_.store(true, std::memory_order_release);
            start_setpoint_watchdog();
            return;
        }
        if (attempt == config_.offboard_start_attempts) {
            require_offboard_success(result, "offboard.start");
        }
        std::cerr << "[flight] offboard.start failed; repriming ("
                  << attempt << '/' << config_.offboard_start_attempts << ")\n";
    }
    throw std::runtime_error("offboard_start_attempts_exhausted");
}

void MavsdkOffboardControl::wait_for_takeoff_height(
    const bullet_mission::NedPoint& ground, const double altitude_m)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration<double>(config_.takeoff_timeout_s);
    const auto poll = std::chrono::duration<double>(config_.command_poll_s);
    std::optional<std::chrono::steady_clock::time_point> stable_since;

    while (std::chrono::steady_clock::now() < deadline) {
        check_abort();
        const auto state = telemetry_.position_velocity_ned();
        const double climbed_m = ground.down_m
            - static_cast<double>(state.position.down_m);
        const bool high_enough = climbed_m
            >= altitude_m - config_.takeoff_altitude_tolerance_m;
        const bool vertical_speed_small =
            std::abs(static_cast<double>(state.velocity.down_m_s))
            < config_.takeoff_vertical_speed_tolerance_m_s;

        if (high_enough && vertical_speed_small) {
            if (!stable_since.has_value()) {
                stable_since = std::chrono::steady_clock::now();
            }
            if (std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - *stable_since).count()
                >= config_.takeoff_settle_s) {
                return;
            }
        } else {
            stable_since.reset();
        }
        std::this_thread::sleep_for(poll);
    }
    throw std::runtime_error("takeoff_altitude_not_reached");
}

void MavsdkOffboardControl::climb_to(
    const std::string& phase,
    const bullet_mission::NedPoint& start,
    const bullet_mission::NedPoint& finish,
    const double speed_m_s,
    const double yaw_deg)
{
    const double delta_down_m = std::abs(finish.down_m - start.down_m);
    if (delta_down_m <= 1.0e-6) {
        static_cast<void>(send_setpoint(phase, stationary_setpoint(finish, yaw_deg)));
        return;
    }
    if (!finite_positive(speed_m_s)) {
        throw std::invalid_argument("vertical speed must be positive");
    }

    const double duration_s = delta_down_m / speed_m_s;
    const auto period = std::chrono::duration<double>(1.0 / config_.setpoint_rate_hz);
    const auto begin = std::chrono::steady_clock::now();
    auto next = begin;
    while (true) {
        const double elapsed_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
        const double progress = elapsed_s / duration_s;
        if (progress >= 1.0) {
            break;
        }

        const double p = bullet_mission::clamp01(progress);
        bullet_mission::TrajectorySetpoint target{};
        target.position = {
            start.north_m + p * (finish.north_m - start.north_m),
            start.east_m + p * (finish.east_m - start.east_m),
            start.down_m + p * (finish.down_m - start.down_m),
        };
        target.velocity.down_m_s = finish.down_m < start.down_m
            ? -speed_m_s
            : speed_m_s;
        target.yaw_deg = yaw_deg;
        static_cast<void>(send_setpoint(phase, target));
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
    }
    static_cast<void>(send_setpoint(phase, stationary_setpoint(finish, yaw_deg)));
}

mavsdk::Telemetry::PositionVelocityNed MavsdkOffboardControl::send_setpoint(
    const std::string& phase,
    const bullet_mission::TrajectorySetpoint& target)
{
    check_abort();
    if (!flight_volume_configured_) {
        throw std::runtime_error("flight_volume_not_configured");
    }
    if (!bullet_mission::finite_setpoint(target)) {
        throw std::runtime_error("generated_setpoint_not_finite");
    }
    const double radius_m = bullet_mission::horizontal_distance(
        target.position, ground_reference_);
    if (radius_m > radius_limit_m_) {
        std::ostringstream message;
        message << "setpoint_radius_" << std::fixed << std::setprecision(1)
                << radius_m << "_exceeds_" << radius_limit_m_;
        throw std::runtime_error(message.str());
    }

    const mavsdk::Offboard::PositionNedYaw position{
        static_cast<float>(target.position.north_m),
        static_cast<float>(target.position.east_m),
        static_cast<float>(target.position.down_m),
        static_cast<float>(target.yaw_deg),
    };
    const mavsdk::Offboard::VelocityNedYaw velocity{
        static_cast<float>(target.velocity.north_m_s),
        static_cast<float>(target.velocity.east_m_s),
        static_cast<float>(target.velocity.down_m_s),
        static_cast<float>(target.yaw_deg),
    };
    require_offboard_success(
        run_px4_command([this, position, velocity] {
            // A watchdog fallback can stop Offboard while this owner waits for
            // the command mutex. Re-check after acquiring it so no setpoint is
            // emitted after the serialized stop+land sequence has begun.
            check_abort();
            if (!offboard_) {
                throw std::runtime_error("offboard_plugin_not_prepared");
            }
            offboard_sender_active_.store(true, std::memory_order_release);
            return offboard_->set_position_velocity_ned(position, velocity);
        }),
        "set_position_velocity_ned");
    last_setpoint_generation_ticks_.store(
        steady_now_ticks(), std::memory_order_release);

    const auto actual = telemetry_.position_velocity_ned();
    if (setpoint_observer_) {
        try {
            setpoint_observer_(
                phase,
                target,
                actual,
                static_cast<float>(battery_remaining_fraction()));
        } catch (...) {
            // Logging/observation is deliberately non-fatal to flight control.
        }
    }
    return actual;
}

void MavsdkOffboardControl::stop_offboard(
    const std::string& operation, const bool require_success)
{
    const bool start_may_have_applied =
        offboard_started_.load(std::memory_order_acquire)
        || offboard_start_may_have_applied_.load(std::memory_order_acquire);
    if (!start_may_have_applied) {
        // Priming alone starts MAVSDK's cached sender. There is no PX4 mode to
        // stop in this branch, so destroy the plugin instead of sending HOLD.
        // Recreate immediately so subsequent cycles do not see offboard_==nullptr.
        if (offboard_sender_active_.load(std::memory_order_acquire)) {
            run_px4_command([this] {
                offboard_.reset();
                offboard_ = std::make_unique<mavsdk::Offboard>(system_);
            });
        }
        offboard_intended_.store(false, std::memory_order_release);
        offboard_sender_active_.store(false, std::memory_order_release);
        offboard_started_ticks_.store(0, std::memory_order_release);
        return;
    }
    const mavsdk::Offboard::Result result = run_px4_command([this] {
        // Keep the active-session flags intact until ownership is proven.
        // Clearing them first used to turn stale RC + current OFFBOARD into
        // an apparent autonomous state and allowed stop() to command HOLD.
        check_pilot_override();
        if (!offboard_) {
            return mavsdk::Offboard::Result::NoSystem;
        }
        return offboard_->stop();
    });
    // Whether the ACK succeeded or not, do not retain a cached setpoint stream
    // after this route has entered a terminal/safety path. If the stop result
    // is uncertain, the may-have-applied flag remains set for conservative
    // emergency action selection. Recreate immediately for subsequent cycles.
    run_px4_command([this] {
        offboard_.reset();
        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
    });
    offboard_intended_.store(false, std::memory_order_release);
    offboard_sender_active_.store(false, std::memory_order_release);
    offboard_started_.store(false, std::memory_order_release);
    offboard_started_ticks_.store(0, std::memory_order_release);
    if (result == mavsdk::Offboard::Result::Success) {
        offboard_start_may_have_applied_.store(
            false, std::memory_order_release);
    }
    if (result != mavsdk::Offboard::Result::Success) {
        if (require_success) {
            require_offboard_success(result, operation);
        }
        std::cerr << "[flight] " << operation << " failed: " << result << '\n';
    }
}

void MavsdkOffboardControl::wait_until_landed(
    const double timeout_s, const bool interruptible)
{
    const auto deadline = std::chrono::steady_clock::now()
        + std::chrono::duration<double>(timeout_s);
    const auto poll = std::chrono::duration<double>(config_.command_poll_s);
    while (std::chrono::steady_clock::now() < deadline) {
        if (interruptible) {
            check_abort();
        } else if (!system_->is_connected()) {
            throw std::runtime_error("vehicle_heartbeat_lost_during_landing");
        }
        if (!telemetry_.in_air()) {
            const auto disarm_deadline = std::chrono::steady_clock::now()
                + std::chrono::duration<double>(config_.disarm_timeout_s);
            while (telemetry_.armed()
                   && std::chrono::steady_clock::now() < disarm_deadline) {
                std::this_thread::sleep_for(poll);
            }
            if (telemetry_.armed()) {
                require_action_success(
                    run_px4_command([this] {
                        check_pilot_override();
                        return action_.disarm();
                    }),
                    "disarm after landing");
            }
            arm_command_may_have_applied_.store(false);
            takeoff_command_may_have_applied_.store(false);
            offboard_start_may_have_applied_.store(false);
            offboard_sender_active_.store(false);
            return;
        }
        std::this_thread::sleep_for(poll);
    }
    throw std::runtime_error("landing_confirmation_timed_out");
}

void MavsdkOffboardControl::land_and_wait()
{
    stop_setpoint_watchdog();
    stop_offboard("offboard.stop before land", true);
    if (flush_observer_) {
        flush_observer_();
    }
    require_action_success(
        run_px4_command([this] {
            check_pilot_override();
            return action_.land();
        }),
        "land");
    wait_until_landed(config_.landing_timeout_s, true);
}

void MavsdkOffboardControl::stop_and_hold()
{
    stop_setpoint_watchdog();
    stop_offboard("offboard.stop before hold", false);
    if (flush_observer_) {
        flush_observer_();
    }
    check_abort();
    if (telemetry_.in_air()) {
        require_action_success(
            run_px4_command([this] {
                check_pilot_override();
                return action_.hold();
            }),
            "hold");
        return;
    }
    if (telemetry_.armed()
        || arm_command_may_have_applied_.load(std::memory_order_acquire)) {
        const auto landed_state = telemetry_.landed_state();
        if (landed_state == mavsdk::Telemetry::LandedState::OnGround
            && !takeoff_command_may_have_applied_.load(
                std::memory_order_acquire)) {
            require_action_success(
                run_px4_command([this] {
                    check_pilot_override();
                    return action_.disarm();
                }),
                "disarm stopped ground vehicle");
            arm_command_may_have_applied_.store(false);
            takeoff_command_may_have_applied_.store(false);
            offboard_start_may_have_applied_.store(false);
            offboard_sender_active_.store(false);
        } else {
            require_action_success(
                run_px4_command([this] {
                    check_pilot_override();
                    return action_.land();
                }),
                "land during stop transition");
            wait_until_landed(config_.landing_timeout_s, true);
        }
    }
}

void MavsdkOffboardControl::return_to_launch_and_wait()
{
    stop_setpoint_watchdog();
    stop_offboard("offboard.stop before RTL", false);
    if (flush_observer_) {
        flush_observer_();
    }
    check_abort();
    if (!telemetry_.in_air()) {
        if (telemetry_.armed()
            || arm_command_may_have_applied_.load(
                std::memory_order_acquire)) {
            const auto landed_state = telemetry_.landed_state();
            if (landed_state == mavsdk::Telemetry::LandedState::OnGround
                && !takeoff_command_may_have_applied_.load(
                    std::memory_order_acquire)) {
                require_action_success(
                    run_px4_command([this] {
                        check_pilot_override();
                        return action_.disarm();
                    }),
                    "disarm ground vehicle before RTL");
                arm_command_may_have_applied_.store(false);
                takeoff_command_may_have_applied_.store(false);
                offboard_start_may_have_applied_.store(false);
                offboard_sender_active_.store(false);
                return;
            }
        } else {
            return;
        }
    }
    require_action_success(
        run_px4_command([this] {
            check_pilot_override();
            return action_.return_to_launch();
        }),
        "return_to_launch");
    wait_until_landed(config_.rtl_timeout_s, true);
}

void MavsdkOffboardControl::emergency_land_and_wait() noexcept
{
    stop_setpoint_watchdog();
    try {
        const bool vehicle_connected = system_->is_connected();
        const bool vehicle_in_air = vehicle_connected && telemetry_.in_air();
        const bool vehicle_armed = vehicle_connected
            && (telemetry_.armed()
                || arm_command_may_have_applied_.load());
        const auto emergency_action = bullet_mission::choose_emergency_action(
            vehicle_connected,
            vehicle_armed,
            vehicle_in_air,
            takeoff_command_may_have_applied_.load());

        // Even a pre-arm Offboard failure can leave MAVSDK's cached sender
        // alive. Stop it only after ownership is positively checked; UNKNOWN
        // or POSITION throws to the pilot-yield catch below without a command.
        if (offboard_started_.load(std::memory_order_acquire)
            || offboard_start_may_have_applied_.load(
                std::memory_order_acquire)
            || offboard_sender_active_.load(std::memory_order_acquire)) {
            stop_offboard("offboard.stop during emergency", false);
        } else {
            offboard_intended_.store(false, std::memory_order_release);
        }

        if (emergency_action == bullet_mission::EmergencyAction::Land) {
            if (flush_observer_) {
                flush_observer_();
            }
            const mavsdk::Action::Result result =
                run_px4_command([this] {
                    check_pilot_override();
                    return action_.land();
                });
            if (result != mavsdk::Action::Result::Success) {
                std::cerr << "[flight] emergency land failed: " << result << '\n';
            }
            try {
                wait_until_landed(config_.landing_timeout_s, false);
            } catch (const std::exception& error) {
                std::cerr << "[flight] emergency landing confirmation failed: "
                          << error.what() << '\n';
            }
        } else if (emergency_action
                   == bullet_mission::EmergencyAction::DisarmOnGround) {
            const mavsdk::Action::Result result =
                run_px4_command([this] {
                    check_pilot_override();
                    return action_.disarm();
                });
            if (result != mavsdk::Action::Result::Success) {
                std::cerr << "[flight] emergency ground disarm failed: " << result << '\n';
            } else {
                arm_command_may_have_applied_.store(false);
                takeoff_command_may_have_applied_.store(false);
                offboard_start_may_have_applied_.store(false);
                offboard_sender_active_.store(false);
            }
        }
    } catch (const PilotOverrideDetected&) {
        yield_to_pilot();
    } catch (...) {
        // PX4's configured Offboard-loss failsafe is the final protection when
        // the link itself is unavailable.
    }
}

void MavsdkOffboardControl::recover_after_restart()
{
    if (!system_->is_connected()) {
        throw std::runtime_error("vehicle_not_connected_for_recovery");
    }
    const bool vehicle_in_air = telemetry_.in_air();
    const bool vehicle_armed = telemetry_.armed();
    if (!vehicle_in_air && !vehicle_armed) {
        return;
    }
    // The pilot latch and mission generation are process memory. After a
    // restart an armed/airborne CH6=HOLD sample cannot prove whether this was
    // an autonomous crash or a pilot who already took over and moved the
    // switch back. Treat every such recovery as ambiguous and emit no PX4
    // command. PX4's configured Offboard-loss action remains the independent
    // safety layer; a future durable ownership journal could safely refine
    // this policy.
    throw PilotOverrideDetected("restart_armed_or_airborne_owner_unknown");
}

void MavsdkOffboardControl::yield_to_pilot() noexcept
{
    // Publish the handover before waiting on either thread or mutex. A watchdog
    // in its grace window observes these flags and exits without issuing Hold
    // or Land for the obsolete session.
    pilot_takeover_latched_.store(true, std::memory_order_release);
    offboard_intended_.store(false, std::memory_order_release);
    stop_setpoint_watchdog();

    try {
        std::lock_guard<std::mutex> lock{px4_command_mutex_};
        // Offboard::~Offboard() removes MAVSDK's call_every sender. Unlike
        // Offboard::stop(), it does not request FlightMode::Hold from PX4.
        // Immediately recreate a fresh empty plugin so this object remains
        // usable for the next cycle (e.g. riposte_test --repeat >1). The new
        // plugin carries no cached setpoint, so Offboard cannot start without
        // a fresh prime. AegisQ's prepare_for_new_ground_mission() recreates
        // again unconditionally, so this is not a double-allocate there.
        offboard_.reset();
        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
    } catch (...) {
        report_fault("pilot_yield_offboard_plugin_destroy_failed");
    }

    offboard_started_.store(false, std::memory_order_release);
    offboard_started_ticks_.store(0, std::memory_order_release);
    offboard_sender_active_.store(false, std::memory_order_release);
    offboard_start_may_have_applied_.store(false, std::memory_order_release);
    watchdog_triggered_generation_.store(0U, std::memory_order_release);
    arm_command_may_have_applied_.store(false, std::memory_order_release);
    takeoff_command_may_have_applied_.store(false, std::memory_order_release);
}

void MavsdkOffboardControl::cancel_for_external_preemption() noexcept
{
    offboard_intended_.store(false, std::memory_order_release);
    stop_setpoint_watchdog();
    try {
        std::lock_guard<std::mutex> lock{px4_command_mutex_};
        // No mode/action command here: the preempting STOP/RTL/E-stop is
        // already queued and will choose that action on the sole worker.
        // Recreate a fresh empty plugin so this object remains usable after
        // the queued command completes; prepare_for_new_ground_mission() will
        // recreate again before the next autonomous session, which is fine.
        offboard_.reset();
        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
    } catch (...) {
        report_fault("preemption_offboard_plugin_destroy_failed");
    }
    offboard_sender_active_.store(false, std::memory_order_release);
    offboard_started_.store(false, std::memory_order_release);
    offboard_started_ticks_.store(0, std::memory_order_release);
    // Do not clear *_may_have_applied_. A lost ACK means the queued safety
    // command must still assume that arming/takeoff/Offboard may have happened.
}

void MavsdkOffboardControl::prepare_for_new_ground_mission(
    const std::uint64_t dispatch_takeover_epoch)
{
    if (!ground_disarmed()) {
        throw std::runtime_error(
            "new_mission_requires_confirmed_on_ground_disarmed_vehicle");
    }
    if (ownership_provider_ != nullptr) {
        const dc::flight::RcOwnershipSnapshot ownership = ownership_snapshot();
        if (!ownership.fresh
            || ownership.takeover_epoch != dispatch_takeover_epoch
            || ownership.owner != dc::flight::ControlOwner::pi
            || ownership.switch_position
                != dc::flight::OwnershipSwitch::hold
            || !ownership.arm_switch.fresh
            || ownership.arm_switch.position
                != dc::flight::ArmSwitchPosition::disarm) {
            throw std::runtime_error(
                "new_mission_requires_fresh_rc_hold_and_disarm");
        }
    }

    mission_takeover_epoch_.store(
        dispatch_takeover_epoch, std::memory_order_release);
    mission_takeover_epoch_valid_.store(true, std::memory_order_release);

    stop_setpoint_watchdog();
    {
        std::lock_guard<std::mutex> lock{px4_command_mutex_};
        // Recreate even when a previous normal path left the plugin alive. The
        // next prime therefore starts from an empty cache in every case.
        offboard_.reset();
        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
    }
    offboard_intended_.store(false, std::memory_order_release);
    offboard_started_.store(false, std::memory_order_release);
    offboard_started_ticks_.store(0, std::memory_order_release);
    offboard_sender_active_.store(false, std::memory_order_release);
    offboard_start_may_have_applied_.store(false, std::memory_order_release);
    watchdog_triggered_generation_.store(0U, std::memory_order_release);
    last_setpoint_generation_ticks_.store(0, std::memory_order_release);
    arm_command_may_have_applied_.store(false, std::memory_order_release);
    takeoff_command_may_have_applied_.store(false, std::memory_order_release);
    pilot_takeover_latched_.store(false, std::memory_order_release);
    flight_volume_configured_ = false;
}

MavsdkOffboardControl::SteadyRep MavsdkOffboardControl::steady_now_ticks() noexcept
{
    return std::chrono::steady_clock::now().time_since_epoch().count();
}

void MavsdkOffboardControl::report_fault(const std::string& detail) noexcept
{
    try {
        std::cerr << "[flight] safety fault: " << detail << '\n';
        if (fault_callback_) {
            fault_callback_(detail);
        }
    } catch (...) {
    }
}

bool MavsdkOffboardControl::watchdog_session_active(
    const std::uint64_t generation) const noexcept
{
    return watchdog_running_.load(std::memory_order_acquire)
        && watchdog_generation_.load(std::memory_order_acquire) == generation;
}

void MavsdkOffboardControl::finish_watchdog_session(
    const std::uint64_t generation) noexcept
{
    if (watchdog_generation_.load(std::memory_order_acquire) == generation) {
        watchdog_running_.store(false, std::memory_order_release);
    }
    watchdog_wake_.notify_all();
}

void MavsdkOffboardControl::latch_watchdog_pilot_takeover(
    const std::uint64_t generation, const std::string& detail) noexcept
{
    if (!watchdog_session_active(generation)) {
        return;
    }
    pilot_takeover_latched_.store(true, std::memory_order_release);
    offboard_intended_.store(false, std::memory_order_release);
    std::uint64_t expected = generation;
    static_cast<void>(watchdog_triggered_generation_.compare_exchange_strong(
        expected, 0U, std::memory_order_acq_rel));
    report_fault(detail);
    try {
        std::lock_guard<std::mutex> command_lock{px4_command_mutex_};
        // The mission owner may be the component that stalled. Destroying the
        // plugin here is the only sender-only cancellation MAVSDK 3.17 offers;
        // unlike Offboard::stop(), it emits no HOLD command under the pilot.
        // Recreate a fresh empty plugin immediately so the same object can be
        // used for a subsequent cycle without hitting offboard_ == nullptr.
        offboard_.reset();
        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
        offboard_started_.store(false, std::memory_order_release);
        offboard_started_ticks_.store(0, std::memory_order_release);
        offboard_sender_active_.store(false, std::memory_order_release);
        offboard_start_may_have_applied_.store(
            false, std::memory_order_release);
    } catch (...) {
        report_fault("watchdog_pilot_sender_destroy_failed");
    }
    finish_watchdog_session(generation);
}

void MavsdkOffboardControl::start_setpoint_watchdog()
{
    stop_setpoint_watchdog();
    const std::uint64_t generation =
        watchdog_generation_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
    watchdog_triggered_generation_.store(0U, std::memory_order_release);
    last_setpoint_generation_ticks_.store(
        steady_now_ticks(), std::memory_order_release);
    watchdog_running_.store(true, std::memory_order_release);
    const std::uint64_t takeover_epoch =
        mission_takeover_epoch_valid_.load(std::memory_order_acquire)
        ? mission_takeover_epoch_.load(std::memory_order_acquire)
        : ownership_provider_ != nullptr
            ? ownership_provider_->takeover_epoch()
            : 0U;

    try {
        watchdog_thread_ = std::thread([this, generation, takeover_epoch] {
            const auto timeout = std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(
                    config_.setpoint_watchdog_timeout_s));
            const auto poll =
                std::chrono::duration<double>(config_.setpoint_watchdog_poll_s);
            const auto owner_grace =
                std::chrono::duration<double>(config_.watchdog_owner_grace_s);

            const auto current_ownership = [this]() noexcept {
                mavsdk::Telemetry::FlightMode mode =
                    mavsdk::Telemetry::FlightMode::Unknown;
                try {
                    mode = telemetry_.flight_mode();
                } catch (...) {
                }
                return assess_ownership(mode);
            };
            const auto ownership_is_pilot = [](const OwnershipAssessment value) {
                return value == OwnershipAssessment::pilot
                    || value == OwnershipAssessment::ambiguous;
            };
            const auto takeover_changed = [this, takeover_epoch]() noexcept {
                return ownership_provider_ != nullptr
                    && ownership_provider_->takeover_epoch() != takeover_epoch;
            };

            while (watchdog_session_active(generation)) {
                const auto last = std::chrono::steady_clock::time_point{
                    std::chrono::steady_clock::duration{
                        last_setpoint_generation_ticks_.load(
                            std::memory_order_acquire)}};
                if (!bullet_mission::setpoint_deadline_expired(
                        std::chrono::steady_clock::now(), last, timeout)) {
                    std::unique_lock<std::mutex> wait_lock{watchdog_wait_mutex_};
                    static_cast<void>(watchdog_wake_.wait_for(
                        wait_lock,
                        poll,
                        [this, generation] {
                            return !watchdog_session_active(generation);
                        }));
                    continue;
                }

                const OwnershipAssessment first_ownership = current_ownership();
                if (takeover_changed()
                    || ownership_is_pilot(first_ownership)) {
                    latch_watchdog_pilot_takeover(
                        generation,
                        takeover_changed()
                            ? "watchdog_yielded_to_rc_takeover_epoch"
                        : first_ownership == OwnershipAssessment::pilot
                            ? "watchdog_yielded_to_rc_pilot"
                            : "watchdog_yielded_to_ambiguous_owner");
                    return;
                }

                watchdog_triggered_generation_.store(
                    generation, std::memory_order_release);
                report_fault("setpoint_generation_watchdog_expired");

                // A responsive owner sees the generation-scoped fault in
                // check_abort(), performs normal cleanup, and invalidates this
                // session. Only an owner which remains stalled for the whole
                // grace interval reaches the independent fallback below.
                {
                    std::unique_lock<std::mutex> wait_lock{watchdog_wait_mutex_};
                    static_cast<void>(watchdog_wake_.wait_for(
                        wait_lock,
                        owner_grace,
                        [this, generation] {
                            return !watchdog_session_active(generation)
                                || pilot_takeover_latched_.load(
                                    std::memory_order_acquire);
                        }));
                }
                if (!watchdog_session_active(generation)) {
                    return;
                }

                const OwnershipAssessment second_ownership = current_ownership();
                if (takeover_changed()
                    || ownership_is_pilot(second_ownership)) {
                    latch_watchdog_pilot_takeover(
                        generation,
                        takeover_changed()
                            ? "watchdog_yielded_to_rc_takeover_epoch_during_grace"
                        : second_ownership == OwnershipAssessment::pilot
                            ? "watchdog_yielded_to_rc_pilot_during_grace"
                            : "watchdog_yielded_to_ambiguous_owner_during_grace");
                    return;
                }

                try {
                    std::lock_guard<std::mutex> command_lock{px4_command_mutex_};
                    if (!watchdog_session_active(generation)
                        || !offboard_intended_.load(std::memory_order_acquire)
                        || pilot_takeover_latched_.load(
                            std::memory_order_acquire)) {
                        finish_watchdog_session(generation);
                        return;
                    }
                    if (takeover_changed()) {
                        pilot_takeover_latched_.store(
                            true, std::memory_order_release);
                        offboard_intended_.store(
                            false, std::memory_order_release);
                        offboard_.reset();
                        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
                        offboard_started_.store(
                            false, std::memory_order_release);
                        offboard_started_ticks_.store(
                            0, std::memory_order_release);
                        offboard_sender_active_.store(
                            false, std::memory_order_release);
                        offboard_start_may_have_applied_.store(
                            false, std::memory_order_release);
                        report_fault(
                            "watchdog_yielded_to_rc_takeover_epoch_at_command_gate");
                        finish_watchdog_session(generation);
                        return;
                    }

                    // Re-evaluate under the same mutex that serializes all PX4
                    // commands. This closes the old check-then-stop race for
                    // every handover already visible to the RC adapter.
                    const OwnershipAssessment final_ownership =
                        current_ownership();
                    if (takeover_changed()
                        || ownership_is_pilot(final_ownership)) {
                        pilot_takeover_latched_.store(
                            true, std::memory_order_release);
                        offboard_intended_.store(
                            false, std::memory_order_release);
                        watchdog_triggered_generation_.store(
                            0U, std::memory_order_release);
                        offboard_.reset();
                        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
                        offboard_started_.store(
                            false, std::memory_order_release);
                        offboard_started_ticks_.store(
                            0, std::memory_order_release);
                        offboard_sender_active_.store(
                            false, std::memory_order_release);
                        offboard_start_may_have_applied_.store(
                            false, std::memory_order_release);
                        report_fault(
                            takeover_changed()
                                ? "watchdog_yielded_to_rc_takeover_epoch_at_final_gate"
                                : "watchdog_yielded_to_pilot_at_command_gate");
                        finish_watchdog_session(generation);
                        return;
                    }

                    mavsdk::Offboard::Result stop_result =
                        mavsdk::Offboard::Result::NoSystem;
                    if (offboard_started_.exchange(false)
                        && offboard_ != nullptr) {
                        stop_result = offboard_->stop();
                        if (stop_result != mavsdk::Offboard::Result::Success) {
                            std::cerr << "[flight] watchdog offboard.stop failed: "
                                      << stop_result << '\n';
                        }
                    }
                    // Never retain the timed-out route's cached sender. The
                    // plugin destructor sends no mode command. Recreate a fresh
                    // empty plugin so this object remains usable for the next
                    // cycle (riposte_test --repeat >1).
                    offboard_.reset();
                    offboard_ = std::make_unique<mavsdk::Offboard>(system_);
                    offboard_sender_active_.store(
                        false, std::memory_order_release);
                    if (stop_result == mavsdk::Offboard::Result::Success) {
                        offboard_start_may_have_applied_.store(
                            false, std::memory_order_release);
                    }
                    offboard_started_ticks_.store(
                        0, std::memory_order_release);

                    // Offboard::stop() waits for a mode-command ACK. CH6 may
                    // move to POSITION while that wait is in progress. The
                    // ACK is not permission to emit a later LAND under the
                    // newly observed pilot, so close that blocking-call race
                    // before issuing the independent Action command.
                    const OwnershipAssessment after_stop_ownership =
                        current_ownership();
                    const dc::flight::RcOwnershipSnapshot after_stop_rc =
                        ownership_snapshot();
                    const bool rc_no_longer_authorizes_pi =
                        ownership_provider_ != nullptr
                        && (!after_stop_rc.fresh
                            || after_stop_rc.owner
                                != dc::flight::ControlOwner::pi);
                    if (!watchdog_session_active(generation)
                        || pilot_takeover_latched_.load(
                            std::memory_order_acquire)
                        || ownership_is_pilot(after_stop_ownership)
                        || rc_no_longer_authorizes_pi
                        || takeover_changed()) {
                        if (ownership_is_pilot(after_stop_ownership)
                            || rc_no_longer_authorizes_pi
                            || takeover_changed()) {
                            pilot_takeover_latched_.store(
                                true, std::memory_order_release);
                            report_fault(
                                takeover_changed()
                                    ? "watchdog_yielded_to_rc_takeover_epoch_after_stop"
                                : after_stop_rc.fresh
                                        && after_stop_rc.owner
                                            == dc::flight::ControlOwner::pilot
                                    ? "watchdog_yielded_to_rc_pilot_after_stop"
                                    : "watchdog_yielded_to_ambiguous_owner_after_stop");
                        }
                        offboard_intended_.store(
                            false, std::memory_order_release);
                        watchdog_triggered_generation_.store(
                            0U, std::memory_order_release);
                        offboard_.reset();
                        offboard_ = std::make_unique<mavsdk::Offboard>(system_);
                        offboard_sender_active_.store(
                            false, std::memory_order_release);
                        offboard_start_may_have_applied_.store(
                            false, std::memory_order_release);
                        finish_watchdog_session(generation);
                        return;
                    }
                    offboard_intended_.store(false, std::memory_order_release);
                    const mavsdk::Action::Result land_result = action_.land();
                    if (land_result != mavsdk::Action::Result::Success) {
                        std::cerr << "[flight] watchdog land failed: "
                                  << land_result << '\n';
                    }
                } catch (...) {
                    report_fault("watchdog_emergency_command_exception");
                }
                finish_watchdog_session(generation);
                return;
            }
        });
    } catch (...) {
        finish_watchdog_session(generation);
        throw;
    }
}

void MavsdkOffboardControl::reset_watchdog_latch() noexcept
{
    // The latch is only reset inside start_setpoint_watchdog(), but a new
    // mission calls check_abort() before it ever starts the watchdog — so a
    // latch left by the previous flight failed every later mission within
    // milliseconds ("setpoint_generation_watchdog_expired" 7 ms after ACCEPTED,
    // 2026-08-14 field day), and only an agent restart cleared it. A running
    // watchdog means the latch may be the live abort signal of an operation in
    // progress; in that case leave it alone.
    if (watchdog_running_.load(std::memory_order_acquire)) {
        return;
    }
    watchdog_triggered_generation_.store(0U, std::memory_order_release);
}

void MavsdkOffboardControl::stop_setpoint_watchdog() noexcept
{
    static_cast<void>(
        watchdog_generation_.fetch_add(1U, std::memory_order_acq_rel));
    watchdog_running_.store(false, std::memory_order_release);
    watchdog_wake_.notify_all();
    if (watchdog_thread_.joinable()
        && watchdog_thread_.get_id() != std::this_thread::get_id()) {
        try {
            watchdog_thread_.join();
        } catch (...) {
            report_fault("watchdog_join_failed");
        }
    }
}

}  // namespace riposte_runtime
