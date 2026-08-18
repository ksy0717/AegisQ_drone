#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace dc::flight {

enum class ControlOwner {
    unknown,
    pi,
    pilot,
};

enum class OwnershipSwitch {
    unknown,
    offboard,
    hold,
    position,
};

enum class ArmSwitchPosition {
    unknown,
    disarm,
    arm,
};

[[nodiscard]] constexpr const char* control_owner_name(
    const ControlOwner owner) noexcept
{
    switch (owner) {
        case ControlOwner::unknown: return "UNKNOWN";
        case ControlOwner::pi: return "PI";
        case ControlOwner::pilot: return "PILOT";
    }
    return "UNKNOWN";
}

[[nodiscard]] constexpr const char* ownership_switch_name(
    const OwnershipSwitch position) noexcept
{
    switch (position) {
        case OwnershipSwitch::unknown: return "UNKNOWN";
        case OwnershipSwitch::offboard: return "OFFBOARD";
        case OwnershipSwitch::hold: return "HOLD";
        case OwnershipSwitch::position: return "POSITION";
    }
    return "UNKNOWN";
}

[[nodiscard]] constexpr const char* arm_switch_position_name(
    const ArmSwitchPosition position) noexcept
{
    switch (position) {
        case ArmSwitchPosition::unknown: return "UNKNOWN";
        case ArmSwitchPosition::disarm: return "DISARM";
        case ArmSwitchPosition::arm: return "ARM";
    }
    return "UNKNOWN";
}

[[nodiscard]] constexpr bool dispatch_ownership_epoch_matches(
    const std::optional<std::uint64_t> dispatch_epoch,
    const std::uint64_t current_epoch) noexcept
{
    return dispatch_epoch.has_value() && *dispatch_epoch == current_epoch;
}

struct RcPwmRange {
    std::uint16_t minimum{};
    std::uint16_t maximum{};

    [[nodiscard]] constexpr bool contains(const std::uint16_t pwm) const noexcept
    {
        return pwm >= minimum && pwm <= maximum;
    }
};

struct RcOwnershipTrackerConfig {
    RcPwmRange offboard_pwm;
    RcPwmRange hold_pwm;
    RcPwmRange position_pwm;
    std::chrono::milliseconds handback_debounce{};
    std::chrono::milliseconds stale_timeout{};
};

struct RcArmSwitchTrackerConfig {
    RcPwmRange disarm_pwm;
    RcPwmRange arm_pwm;
    std::chrono::milliseconds debounce{};
    std::chrono::milliseconds stale_timeout{};
};

struct RcArmSwitchSnapshot {
    ArmSwitchPosition position{ArmSwitchPosition::unknown};
    bool fresh{};
    std::optional<std::uint16_t> raw_pwm;
};

struct RcOwnershipSnapshot {
    ControlOwner owner{ControlOwner::unknown};
    OwnershipSwitch switch_position{OwnershipSwitch::unknown};
    bool fresh{};
    std::optional<std::uint16_t> raw_pwm;
    RcArmSwitchSnapshot arm_switch;
    // Monotonic within this process. A single POSITION observation increments
    // it even if the switch returns to HOLD before the command worker polls.
    std::uint64_t takeover_epoch{};
};

// The callable may enqueue an asynchronous MAVSDK request, set an atomic, or
// perform another bounded nonblocking initiation step. It must never wait for
// a command ACK while the RC publication gate is held.
using NonblockingCommandInitiator = std::function<void()>;

class RcOwnershipProvider {
public:
    virtual ~RcOwnershipProvider() = default;

    [[nodiscard]] virtual RcOwnershipSnapshot snapshot() const noexcept = 0;
    [[nodiscard]] virtual std::uint64_t takeover_epoch() const noexcept = 0;
    // START may be queued only from the exact jointly-published sample that
    // proves CH6=HOLD and CH5=DISARM. The epoch invalidates the queued command
    // if POSITION is observed before its worker reaches PX4.
    [[nodiscard]] virtual std::optional<std::uint64_t>
    mission_authorization_epoch() const noexcept = 0;
    // Re-checks the stamped epoch and the user's fresh HOLD+DISARM contract
    // while excluding the RC callback, then performs only the caller's
    // nonblocking initiation.
    // Waiting for MAVSDK completion must happen after this method returns.
    [[nodiscard]] virtual bool initiate_authorized_command(
        std::uint64_t expected_epoch,
        const NonblockingCommandInitiator& initiator) const noexcept = 0;
};

// Sticky edge detector used inside RcCommandAuthorizationGate. Its atomics
// keep cheap epoch-only reads race-free, while the gate provides the stronger
// epoch+switch snapshot consistency needed for START and command initiation.
class RcTakeoverEpochTracker final {
public:
    void observe(const bool position_selected) noexcept
    {
        const bool was_selected = position_selected_.exchange(
            position_selected, std::memory_order_acq_rel);
        if (position_selected && !was_selected) {
            static_cast<void>(epoch_.fetch_add(1U, std::memory_order_acq_rel));
        }
    }

    [[nodiscard]] std::uint64_t current() const noexcept
    {
        return epoch_.load(std::memory_order_acquire);
    }

private:
    std::atomic_bool position_selected_{false};
    std::atomic<std::uint64_t> epoch_{0U};
};

// Pure, clock-injected ownership state machine. The MAVLink adapter serializes
// access to it; keeping synchronization out of this class makes every timing
// edge directly testable without MAVSDK, a receiver, or sleeping tests.
class RcOwnershipTracker final {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit RcOwnershipTracker(const RcOwnershipTrackerConfig config)
        : config_(config)
    {
        validate_config();
    }

    void observe(
        const std::optional<std::uint16_t> pwm,
        const TimePoint observed_at) noexcept
    {
        // A stale gap revokes the previous handback. The first new HOLD or
        // OFFBOARD sample must start a new debounce interval rather than
        // reviving an authorization learned before the gap.
        if (last_observed_at_.has_value()
            && (observed_at < *last_observed_at_
                || observed_at - *last_observed_at_ > config_.stale_timeout)) {
            reset_classification();
        }

        last_observed_at_ = observed_at;
        raw_pwm_ = pwm;
        if (!pwm.has_value()) {
            reset_classification();
            return;
        }

        const OwnershipSwitch observed = classify(*pwm);
        if (observed == OwnershipSwitch::unknown) {
            reset_classification();
            return;
        }

        // Taking control must win over switch debounce. Waiting here would let
        // the Pi emit several more setpoints after the pilot selected POSITION.
        if (observed == OwnershipSwitch::position) {
            stable_switch_ = OwnershipSwitch::position;
            reset_candidate();
            return;
        }

        if (observed == stable_switch_) {
            reset_candidate();
            return;
        }

        // Returning control permits a new autonomous flight, so it deliberately
        // requires both elapsed stability and more than one actual RC sample.
        // While that proof is incomplete the public state is UNKNOWN.
        stable_switch_ = OwnershipSwitch::unknown;
        if (observed != candidate_switch_) {
            candidate_switch_ = observed;
            candidate_since_ = observed_at;
            candidate_samples_ = 1U;
            return;
        }

        if (candidate_samples_ < std::numeric_limits<std::uint32_t>::max()) {
            ++candidate_samples_;
        }
        if (candidate_samples_ >= 2U
            && observed_at >= candidate_since_
            && observed_at - candidate_since_ >= config_.handback_debounce) {
            stable_switch_ = candidate_switch_;
            reset_candidate();
        }
    }

    [[nodiscard]] RcOwnershipSnapshot snapshot(const TimePoint now) const noexcept
    {
        RcOwnershipSnapshot result;
        result.raw_pwm = raw_pwm_;
        if (!last_observed_at_.has_value()
            || now < *last_observed_at_
            || now - *last_observed_at_ > config_.stale_timeout
            || stable_switch_ == OwnershipSwitch::unknown) {
            return result;
        }

        result.fresh = true;
        result.switch_position = stable_switch_;
        result.owner = stable_switch_ == OwnershipSwitch::position
            ? ControlOwner::pilot
            : ControlOwner::pi;
        return result;
    }

private:
    [[nodiscard]] static constexpr bool overlaps(
        const RcPwmRange lhs, const RcPwmRange rhs) noexcept
    {
        return lhs.minimum <= rhs.maximum && rhs.minimum <= lhs.maximum;
    }

    void validate_config() const
    {
        const auto valid_range = [](const RcPwmRange range) {
            return range.minimum <= range.maximum
                && range.maximum < std::numeric_limits<std::uint16_t>::max();
        };
        if (!valid_range(config_.offboard_pwm)
            || !valid_range(config_.hold_pwm)
            || !valid_range(config_.position_pwm)
            || overlaps(config_.offboard_pwm, config_.hold_pwm)
            || overlaps(config_.offboard_pwm, config_.position_pwm)
            || overlaps(config_.hold_pwm, config_.position_pwm)
            || config_.handback_debounce <= std::chrono::milliseconds::zero()
            || config_.stale_timeout <= config_.handback_debounce) {
            throw std::invalid_argument("invalid RC ownership tracker configuration");
        }
    }

    [[nodiscard]] OwnershipSwitch classify(const std::uint16_t pwm) const noexcept
    {
        if (config_.offboard_pwm.contains(pwm)) {
            return OwnershipSwitch::offboard;
        }
        if (config_.hold_pwm.contains(pwm)) {
            return OwnershipSwitch::hold;
        }
        if (config_.position_pwm.contains(pwm)) {
            return OwnershipSwitch::position;
        }
        return OwnershipSwitch::unknown;
    }

    void reset_candidate() noexcept
    {
        candidate_switch_ = OwnershipSwitch::unknown;
        candidate_since_ = TimePoint{};
        candidate_samples_ = 0U;
    }

    void reset_classification() noexcept
    {
        stable_switch_ = OwnershipSwitch::unknown;
        reset_candidate();
    }

    RcOwnershipTrackerConfig config_;
    OwnershipSwitch stable_switch_{OwnershipSwitch::unknown};
    OwnershipSwitch candidate_switch_{OwnershipSwitch::unknown};
    TimePoint candidate_since_{};
    std::uint32_t candidate_samples_{};
    std::optional<TimePoint> last_observed_at_;
    std::optional<std::uint16_t> raw_pwm_;
};

// CH5 is evidence about the transmitter's physical ARM switch, not PX4's
// actual arming state. Both positions are debounced: Telemetry::armed remains
// authoritative for vehicle state, while this tracker proves the switch was
// deliberately left in DISARM before a remote arm request is considered.
class RcArmSwitchTracker final {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit RcArmSwitchTracker(const RcArmSwitchTrackerConfig config)
        : config_(config)
    {
        validate_config();
    }

    void observe(
        const std::optional<std::uint16_t> pwm,
        const TimePoint observed_at) noexcept
    {
        if (last_observed_at_.has_value()
            && (observed_at < *last_observed_at_
                || observed_at - *last_observed_at_ > config_.stale_timeout)) {
            reset_classification();
        }

        last_observed_at_ = observed_at;
        raw_pwm_ = pwm;
        if (!pwm.has_value()) {
            reset_classification();
            return;
        }

        const ArmSwitchPosition observed = classify(*pwm);
        if (observed == ArmSwitchPosition::unknown) {
            reset_classification();
            return;
        }
        if (observed == stable_position_) {
            reset_candidate();
            return;
        }

        stable_position_ = ArmSwitchPosition::unknown;
        if (observed != candidate_position_) {
            candidate_position_ = observed;
            candidate_since_ = observed_at;
            candidate_samples_ = 1U;
            return;
        }

        if (candidate_samples_ < std::numeric_limits<std::uint32_t>::max()) {
            ++candidate_samples_;
        }
        if (candidate_samples_ >= 2U
            && observed_at >= candidate_since_
            && observed_at - candidate_since_ >= config_.debounce) {
            stable_position_ = candidate_position_;
            reset_candidate();
        }
    }

    [[nodiscard]] RcArmSwitchSnapshot snapshot(const TimePoint now) const noexcept
    {
        RcArmSwitchSnapshot result;
        result.raw_pwm = raw_pwm_;
        if (!last_observed_at_.has_value()
            || now < *last_observed_at_
            || now - *last_observed_at_ > config_.stale_timeout
            || stable_position_ == ArmSwitchPosition::unknown) {
            return result;
        }
        result.position = stable_position_;
        result.fresh = true;
        return result;
    }

private:
    [[nodiscard]] static constexpr bool overlaps(
        const RcPwmRange lhs, const RcPwmRange rhs) noexcept
    {
        return lhs.minimum <= rhs.maximum && rhs.minimum <= lhs.maximum;
    }

    void validate_config() const
    {
        const auto valid_range = [](const RcPwmRange range) {
            return range.minimum <= range.maximum
                && range.maximum < std::numeric_limits<std::uint16_t>::max();
        };
        if (!valid_range(config_.disarm_pwm)
            || !valid_range(config_.arm_pwm)
            || overlaps(config_.disarm_pwm, config_.arm_pwm)
            || config_.debounce <= std::chrono::milliseconds::zero()
            || config_.stale_timeout <= config_.debounce) {
            throw std::invalid_argument("invalid RC arm-switch tracker configuration");
        }
    }

    [[nodiscard]] ArmSwitchPosition classify(const std::uint16_t pwm) const noexcept
    {
        if (config_.disarm_pwm.contains(pwm)) {
            return ArmSwitchPosition::disarm;
        }
        if (config_.arm_pwm.contains(pwm)) {
            return ArmSwitchPosition::arm;
        }
        return ArmSwitchPosition::unknown;
    }

    void reset_candidate() noexcept
    {
        candidate_position_ = ArmSwitchPosition::unknown;
        candidate_since_ = TimePoint{};
        candidate_samples_ = 0U;
    }

    void reset_classification() noexcept
    {
        stable_position_ = ArmSwitchPosition::unknown;
        reset_candidate();
    }

    RcArmSwitchTrackerConfig config_;
    ArmSwitchPosition stable_position_{ArmSwitchPosition::unknown};
    ArmSwitchPosition candidate_position_{ArmSwitchPosition::unknown};
    TimePoint candidate_since_{};
    std::uint32_t candidate_samples_{};
    std::optional<TimePoint> last_observed_at_;
    std::optional<std::uint16_t> raw_pwm_;
};

// Serializes one complete RC_CHANNELS publication with authorization reads.
// This prevents a reader from combining the previous HOLD/DISARM trackers
// with the epoch from a newly-arrived POSITION packet. The mutex is purposely
// held across only a caller-supplied nonblocking initiation, never an ACK wait.
class RcCommandAuthorizationGate final {
public:
    using Clock = RcOwnershipTracker::Clock;
    using TimePoint = Clock::time_point;

    RcCommandAuthorizationGate(
        const RcOwnershipTrackerConfig ownership_config,
        const RcArmSwitchTrackerConfig arm_config)
        : position_pwm_(ownership_config.position_pwm),
          ownership_tracker_(ownership_config),
          arm_tracker_(arm_config)
    {
    }

    void observe(
        const std::optional<std::uint16_t> ownership_pwm,
        const std::optional<std::uint16_t> arm_pwm,
        const TimePoint observed_at) noexcept
    {
        std::lock_guard<std::mutex> lock{mutex_};
        const bool position_selected = ownership_pwm.has_value()
            && position_pwm_.contains(*ownership_pwm);
        takeover_epoch_.observe(position_selected);
        ownership_tracker_.observe(ownership_pwm, observed_at);
        arm_tracker_.observe(arm_pwm, observed_at);
    }

    [[nodiscard]] RcOwnershipSnapshot snapshot(
        const TimePoint now) const noexcept
    {
        try {
            std::lock_guard<std::mutex> lock{mutex_};
            return snapshot_locked(now);
        } catch (...) {
            return {};
        }
    }

    [[nodiscard]] std::uint64_t takeover_epoch() const noexcept
    {
        try {
            std::lock_guard<std::mutex> lock{mutex_};
            return takeover_epoch_.current();
        } catch (...) {
            return 0U;
        }
    }

    [[nodiscard]] std::optional<std::uint64_t>
    mission_authorization_epoch(const TimePoint now) const noexcept
    {
        try {
            std::lock_guard<std::mutex> lock{mutex_};
            const RcOwnershipSnapshot current = snapshot_locked(now);
            if (!current.fresh
                || current.owner != ControlOwner::pi
                || current.switch_position != OwnershipSwitch::hold
                || !current.arm_switch.fresh
                || current.arm_switch.position != ArmSwitchPosition::disarm) {
                return std::nullopt;
            }
            return current.takeover_epoch;
        } catch (...) {
            return std::nullopt;
        }
    }

    [[nodiscard]] bool initiate_authorized_command(
        const std::uint64_t expected_epoch,
        const TimePoint now,
        const NonblockingCommandInitiator& initiator) const noexcept
    {
        if (!initiator) {
            return false;
        }
        try {
            std::lock_guard<std::mutex> lock{mutex_};
            const RcOwnershipSnapshot current = snapshot_locked(now);
            if (current.takeover_epoch != expected_epoch
                || !current.fresh
                || current.owner != ControlOwner::pi
                || current.switch_position != OwnershipSwitch::hold
                || !current.arm_switch.fresh
                || current.arm_switch.position != ArmSwitchPosition::disarm) {
                return false;
            }
            initiator();
            return true;
        } catch (...) {
            return false;
        }
    }

private:
    [[nodiscard]] RcOwnershipSnapshot snapshot_locked(
        const TimePoint now) const noexcept
    {
        RcOwnershipSnapshot result = ownership_tracker_.snapshot(now);
        result.arm_switch = arm_tracker_.snapshot(now);
        result.takeover_epoch = takeover_epoch_.current();
        return result;
    }

    RcPwmRange position_pwm_;
    mutable std::mutex mutex_;
    RcTakeoverEpochTracker takeover_epoch_;
    RcOwnershipTracker ownership_tracker_;
    RcArmSwitchTracker arm_tracker_;
};

}  // namespace dc::flight
