#include "dc/flight/rc_ownership.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

using dc::flight::ControlOwner;
using dc::flight::ArmSwitchPosition;
using dc::flight::OwnershipSwitch;
using dc::flight::RcCommandAuthorizationGate;
using dc::flight::RcArmSwitchSnapshot;
using dc::flight::RcArmSwitchTracker;
using dc::flight::RcArmSwitchTrackerConfig;
using dc::flight::RcOwnershipSnapshot;
using dc::flight::RcOwnershipTracker;
using dc::flight::RcOwnershipTrackerConfig;
using dc::flight::RcPwmRange;
using dc::flight::RcTakeoverEpochTracker;

using namespace std::chrono_literals;

constexpr RcOwnershipTrackerConfig kConfig{
    RcPwmRange{900U, 1100U},
    RcPwmRange{1400U, 1600U},
    RcPwmRange{1900U, 2100U},
    100ms,
    500ms,
};

constexpr RcArmSwitchTrackerConfig kArmConfig{
    RcPwmRange{900U, 1100U},
    RcPwmRange{1900U, 2100U},
    100ms,
    500ms,
};

const RcOwnershipTracker::TimePoint kStart{1s};

void expect_unknown(
    const RcOwnershipSnapshot& snapshot,
    const std::string& message)
{
    test_support::check(
        !snapshot.fresh
            && snapshot.owner == ControlOwner::unknown
            && snapshot.switch_position == OwnershipSwitch::unknown,
        message);
}

void expect_arm_unknown(
    const RcArmSwitchSnapshot& snapshot,
    const std::string& message)
{
    test_support::check(
        !snapshot.fresh
            && snapshot.position == ArmSwitchPosition::unknown,
        message);
}

void test_names_are_wire_stable()
{
    test_support::check(
        std::string{dc::flight::control_owner_name(ControlOwner::unknown)} == "UNKNOWN"
            && std::string{dc::flight::control_owner_name(ControlOwner::pi)} == "PI"
            && std::string{dc::flight::control_owner_name(ControlOwner::pilot)} == "PILOT",
        "control-owner names are stable telemetry values");
    test_support::check(
        std::string{dc::flight::ownership_switch_name(OwnershipSwitch::unknown)}
                == "UNKNOWN"
            && std::string{
                   dc::flight::ownership_switch_name(OwnershipSwitch::offboard)}
                == "OFFBOARD"
            && std::string{dc::flight::ownership_switch_name(OwnershipSwitch::hold)}
                == "HOLD"
            && std::string{
                   dc::flight::ownership_switch_name(OwnershipSwitch::position)}
                == "POSITION",
        "ownership-switch names are stable telemetry values");
    test_support::check(
        std::string{
            dc::flight::arm_switch_position_name(ArmSwitchPosition::unknown)}
                == "UNKNOWN"
            && std::string{
                   dc::flight::arm_switch_position_name(ArmSwitchPosition::disarm)}
                == "DISARM"
            && std::string{
                   dc::flight::arm_switch_position_name(ArmSwitchPosition::arm)}
                == "ARM",
        "arm-switch names are stable telemetry values");
}

void test_position_takes_control_immediately()
{
    RcOwnershipTracker tracker{kConfig};
    expect_unknown(tracker.snapshot(kStart), "ownership starts unknown");

    tracker.observe(std::uint16_t{2000U}, kStart);
    const RcOwnershipSnapshot snapshot = tracker.snapshot(kStart);
    test_support::check(
        snapshot.fresh
            && snapshot.owner == ControlOwner::pilot
            && snapshot.switch_position == OwnershipSwitch::position
            && snapshot.raw_pwm == std::optional<std::uint16_t>{2000U},
        "one POSITION sample transfers control to the pilot immediately");
}

void test_hold_handback_needs_time_and_multiple_samples()
{
    RcOwnershipTracker tracker{kConfig};
    tracker.observe(std::uint16_t{2000U}, kStart);
    tracker.observe(std::uint16_t{1500U}, kStart + 10ms);
    expect_unknown(
        tracker.snapshot(kStart + 10ms),
        "first HOLD sample starts an unknown handback transition");

    tracker.observe(std::uint16_t{1500U}, kStart + 60ms);
    expect_unknown(
        tracker.snapshot(kStart + 60ms),
        "multiple HOLD samples do not bypass elapsed debounce time");

    tracker.observe(std::uint16_t{1500U}, kStart + 110ms);
    const RcOwnershipSnapshot snapshot = tracker.snapshot(kStart + 110ms);
    test_support::check(
        snapshot.fresh
            && snapshot.owner == ControlOwner::pi
            && snapshot.switch_position == OwnershipSwitch::hold,
        "stable HOLD hands control back after time and sample-count debounce");
}

void test_offboard_is_a_debounced_pi_authorization()
{
    RcOwnershipTracker tracker{kConfig};
    tracker.observe(std::uint16_t{1000U}, kStart);
    expect_unknown(
        tracker.snapshot(kStart),
        "one OFFBOARD sample cannot authorize autonomous flight");

    tracker.observe(std::uint16_t{1000U}, kStart + 100ms);
    const RcOwnershipSnapshot snapshot = tracker.snapshot(kStart + 100ms);
    test_support::check(
        snapshot.fresh
            && snapshot.owner == ControlOwner::pi
            && snapshot.switch_position == OwnershipSwitch::offboard,
        "two stable OFFBOARD samples authorize the Pi after debounce");
}

void test_invalid_deadband_and_stale_samples_fail_closed()
{
    RcOwnershipTracker tracker{kConfig};
    tracker.observe(std::uint16_t{1500U}, kStart);
    tracker.observe(std::uint16_t{1500U}, kStart + 100ms);
    test_support::check(
        tracker.snapshot(kStart + 100ms).owner == ControlOwner::pi,
        "test starts from an authorized HOLD");

    tracker.observe(std::uint16_t{1250U}, kStart + 110ms);
    const RcOwnershipSnapshot deadband = tracker.snapshot(kStart + 110ms);
    expect_unknown(deadband, "PWM outside every configured range is unknown");
    test_support::check(
        deadband.raw_pwm == std::optional<std::uint16_t>{1250U},
        "deadband PWM remains available for diagnostics");

    tracker.observe(std::nullopt, kStart + 120ms);
    const RcOwnershipSnapshot invalid = tracker.snapshot(kStart + 120ms);
    expect_unknown(invalid, "an unused or missing RC channel is unknown");
    test_support::check(
        !invalid.raw_pwm.has_value(),
        "an invalid MAVLink channel has no diagnostic PWM");

    tracker.observe(std::uint16_t{1500U}, kStart + 200ms);
    tracker.observe(std::uint16_t{1500U}, kStart + 300ms);
    expect_unknown(
        tracker.snapshot(kStart + 801ms),
        "a sample older than the stale timeout cannot authorize the Pi");
}

void test_first_sample_after_stale_gap_restarts_debounce()
{
    RcOwnershipTracker tracker{kConfig};
    tracker.observe(std::uint16_t{1500U}, kStart);
    tracker.observe(std::uint16_t{1500U}, kStart + 100ms);
    test_support::check(
        tracker.snapshot(kStart + 100ms).owner == ControlOwner::pi,
        "HOLD is initially authorized");

    tracker.observe(std::uint16_t{1500U}, kStart + 700ms);
    expect_unknown(
        tracker.snapshot(kStart + 700ms),
        "the first HOLD after a stale gap cannot revive old authorization");
    tracker.observe(std::uint16_t{1500U}, kStart + 800ms);
    test_support::check(
        tracker.snapshot(kStart + 800ms).owner == ControlOwner::pi,
        "a fresh handback can be established after a stale gap");
}

void test_switch_jitter_restarts_handback_debounce()
{
    RcOwnershipTracker tracker{kConfig};
    tracker.observe(std::uint16_t{1500U}, kStart);
    tracker.observe(std::uint16_t{1000U}, kStart + 90ms);
    tracker.observe(std::uint16_t{1500U}, kStart + 100ms);
    tracker.observe(std::uint16_t{1500U}, kStart + 190ms);
    expect_unknown(
        tracker.snapshot(kStart + 190ms),
        "a different switch range restarts the handback interval");
    tracker.observe(std::uint16_t{1500U}, kStart + 200ms);
    test_support::check(
        tracker.snapshot(kStart + 200ms).owner == ControlOwner::pi,
        "HOLD authorizes only after the restarted debounce completes");
}

void test_bad_configuration_is_rejected()
{
    bool overlap_rejected = false;
    try {
        RcOwnershipTracker tracker{RcOwnershipTrackerConfig{
            RcPwmRange{900U, 1500U},
            RcPwmRange{1400U, 1600U},
            RcPwmRange{1900U, 2100U},
            100ms,
            500ms,
        }};
        static_cast<void>(tracker);
    } catch (const std::invalid_argument&) {
        overlap_rejected = true;
    }
    test_support::check(
        overlap_rejected,
        "overlapping switch ranges are rejected instead of using precedence");

    bool timing_rejected = false;
    try {
        RcOwnershipTracker tracker{RcOwnershipTrackerConfig{
            RcPwmRange{900U, 1100U},
            RcPwmRange{1400U, 1600U},
            RcPwmRange{1900U, 2100U},
            500ms,
            500ms,
        }};
        static_cast<void>(tracker);
    } catch (const std::invalid_argument&) {
        timing_rejected = true;
    }
    test_support::check(
        timing_rejected,
        "stale timeout must leave room for handback debounce");
}

void test_arm_switch_positions_are_separately_debounced()
{
    RcArmSwitchTracker tracker{kArmConfig};
    tracker.observe(std::uint16_t{1000U}, kStart);
    expect_arm_unknown(
        tracker.snapshot(kStart),
        "one DISARM sample cannot prove the physical switch position");
    tracker.observe(std::uint16_t{1000U}, kStart + 100ms);
    const RcArmSwitchSnapshot disarmed = tracker.snapshot(kStart + 100ms);
    test_support::check(
        disarmed.fresh
            && disarmed.position == ArmSwitchPosition::disarm
            && disarmed.raw_pwm == std::optional<std::uint16_t>{1000U},
        "stable CH5 DISARM is reported after debounce");

    tracker.observe(std::uint16_t{2000U}, kStart + 110ms);
    expect_arm_unknown(
        tracker.snapshot(kStart + 110ms),
        "moving CH5 to ARM revokes the previous DISARM proof immediately");
    tracker.observe(std::uint16_t{2000U}, kStart + 210ms);
    test_support::check(
        tracker.snapshot(kStart + 210ms).position == ArmSwitchPosition::arm,
        "stable CH5 ARM is reported after its own debounce");
}

void test_arm_switch_invalid_and_stale_states_fail_closed()
{
    RcArmSwitchTracker tracker{kArmConfig};
    tracker.observe(std::uint16_t{1000U}, kStart);
    tracker.observe(std::uint16_t{1000U}, kStart + 100ms);

    tracker.observe(std::uint16_t{1500U}, kStart + 110ms);
    const RcArmSwitchSnapshot deadband = tracker.snapshot(kStart + 110ms);
    expect_arm_unknown(deadband, "CH5 deadband is not treated as DISARM");
    test_support::check(
        deadband.raw_pwm == std::optional<std::uint16_t>{1500U},
        "CH5 deadband PWM remains available for diagnostics");

    tracker.observe(std::nullopt, kStart + 120ms);
    expect_arm_unknown(
        tracker.snapshot(kStart + 120ms),
        "missing CH5 data invalidates the arm-switch state");

    tracker.observe(std::uint16_t{1000U}, kStart + 200ms);
    tracker.observe(std::uint16_t{1000U}, kStart + 300ms);
    expect_arm_unknown(
        tracker.snapshot(kStart + 801ms),
        "stale CH5 DISARM cannot authorize a later remote start");
}

void test_bad_arm_switch_configuration_is_rejected()
{
    bool overlap_rejected = false;
    try {
        RcArmSwitchTracker tracker{RcArmSwitchTrackerConfig{
            RcPwmRange{900U, 1500U},
            RcPwmRange{1400U, 2100U},
            100ms,
            500ms,
        }};
        static_cast<void>(tracker);
    } catch (const std::invalid_argument&) {
        overlap_rejected = true;
    }
    test_support::check(
        overlap_rejected,
        "overlapping CH5 ARM/DISARM ranges are rejected");
}

void test_position_epoch_is_sticky_across_fast_handback()
{
    RcTakeoverEpochTracker epoch;
    test_support::check(epoch.current() == 0U, "takeover epoch starts at zero");
    epoch.observe(true);
    epoch.observe(true);
    epoch.observe(false);
    test_support::check(
        epoch.current() == 1U,
        "repeated POSITION samples count as one sticky takeover edge");
    epoch.observe(true);
    test_support::check(
        epoch.current() == 2U,
        "POSITION after a handback creates a new takeover epoch");
}

void test_dispatch_epoch_invalidates_the_old_start()
{
    test_support::check(
        dc::flight::dispatch_ownership_epoch_matches(
            std::optional<std::uint64_t>{4U}, 4U),
        "START remains valid while no POSITION edge occurred");
    test_support::check(
        !dc::flight::dispatch_ownership_epoch_matches(
            std::optional<std::uint64_t>{4U}, 5U)
            && !dc::flight::dispatch_ownership_epoch_matches(std::nullopt, 5U),
        "a later POSITION epoch or an unstamped START fails closed");
}

void observe_gate(
    RcCommandAuthorizationGate& gate,
    const std::uint16_t ownership_pwm,
    const std::uint16_t arm_pwm,
    const RcCommandAuthorizationGate::TimePoint observed_at)
{
    gate.observe(ownership_pwm, arm_pwm, observed_at);
}

void test_mission_authorization_requires_joint_hold_and_disarm()
{
    RcCommandAuthorizationGate gate{kConfig, kArmConfig};
    observe_gate(gate, 1500U, 1000U, kStart);
    test_support::check(
        !gate.mission_authorization_epoch(kStart).has_value(),
        "one joint HOLD/DISARM sample cannot authorize START");

    observe_gate(gate, 1500U, 1000U, kStart + 100ms);
    test_support::check(
        gate.mission_authorization_epoch(kStart + 100ms)
            == std::optional<std::uint64_t>{0U},
        "fresh debounced HOLD and DISARM authorize the current epoch");

    observe_gate(gate, 1000U, 1000U, kStart + 110ms);
    observe_gate(gate, 1000U, 1000U, kStart + 210ms);
    test_support::check(
        !gate.mission_authorization_epoch(kStart + 210ms).has_value(),
        "OFFBOARD grants PI ownership but cannot authorize a new START");

    observe_gate(gate, 1500U, 2000U, kStart + 220ms);
    observe_gate(gate, 1500U, 2000U, kStart + 320ms);
    test_support::check(
        !gate.mission_authorization_epoch(kStart + 320ms).has_value(),
        "CH5 ARM cannot authorize remote START even with CH6 HOLD");

    observe_gate(gate, 2000U, 1000U, kStart + 330ms);
    test_support::check(
        !gate.mission_authorization_epoch(kStart + 330ms).has_value()
            && gate.takeover_epoch() == 1U,
        "POSITION atomically revokes authorization and advances its epoch");

    observe_gate(gate, 1500U, 1000U, kStart + 340ms);
    observe_gate(gate, 1500U, 1000U, kStart + 440ms);
    test_support::check(
        gate.mission_authorization_epoch(kStart + 440ms)
            == std::optional<std::uint64_t>{1U},
        "a new debounced HOLD/DISARM handback authorizes only the new epoch");
}

void test_command_initiation_rechecks_epoch_and_fresh_pi_under_gate()
{
    RcCommandAuthorizationGate gate{kConfig, kArmConfig};
    observe_gate(gate, 1500U, 1000U, kStart);
    observe_gate(gate, 1500U, 1000U, kStart + 100ms);

    bool initiated = false;
    test_support::check(
        gate.initiate_authorized_command(
            0U,
            kStart + 100ms,
            [&initiated] { initiated = true; })
            && initiated,
        "matching epoch and fresh PI ownership permit nonblocking initiation");

    initiated = false;
    test_support::check(
        !gate.initiate_authorized_command(
            1U,
            kStart + 100ms,
            [&initiated] { initiated = true; })
            && !initiated,
        "an epoch mismatch rejects without invoking the initiator");

    observe_gate(gate, 2000U, 1000U, kStart + 110ms);
    initiated = false;
    test_support::check(
        !gate.initiate_authorized_command(
            0U,
            kStart + 110ms,
            [&initiated] { initiated = true; })
            && !initiated,
        "a POSITION publication wins before command initiation");

    observe_gate(gate, 1000U, 1000U, kStart + 120ms);
    observe_gate(gate, 1000U, 1000U, kStart + 220ms);
    initiated = false;
    test_support::check(
        !gate.initiate_authorized_command(
            1U,
            kStart + 220ms,
            [&initiated] { initiated = true; })
            && !initiated
            && !gate.mission_authorization_epoch(kStart + 220ms).has_value(),
        "OFFBOARD is PI-owned but cannot authorize a server command");

    observe_gate(gate, 1500U, 2000U, kStart + 230ms);
    observe_gate(gate, 1500U, 2000U, kStart + 330ms);
    initiated = false;
    test_support::check(
        !gate.initiate_authorized_command(
            1U,
            kStart + 330ms,
            [&initiated] { initiated = true; })
            && !initiated,
        "CH5 ARM revokes command initiation even with fresh CH6 HOLD");

    observe_gate(gate, 1500U, 1000U, kStart + 340ms);
    observe_gate(gate, 1500U, 1000U, kStart + 440ms);
    initiated = false;
    test_support::check(
        gate.initiate_authorized_command(
            1U,
            kStart + 440ms,
            [&initiated] { initiated = true; })
            && initiated,
        "a fresh HOLD+DISARM handback restores command initiation");

    initiated = false;
    test_support::check(
        !gate.initiate_authorized_command(
            1U,
            kStart + 941ms,
            [&initiated] { initiated = true; })
            && !initiated,
        "stale PI ownership cannot initiate a command");
}

}  // namespace

int main()
{
    test_names_are_wire_stable();
    test_position_takes_control_immediately();
    test_hold_handback_needs_time_and_multiple_samples();
    test_offboard_is_a_debounced_pi_authorization();
    test_invalid_deadband_and_stale_samples_fail_closed();
    test_first_sample_after_stale_gap_restarts_debounce();
    test_switch_jitter_restarts_handback_debounce();
    test_bad_configuration_is_rejected();
    test_arm_switch_positions_are_separately_debounced();
    test_arm_switch_invalid_and_stale_states_fail_closed();
    test_bad_arm_switch_configuration_is_rejected();
    test_position_epoch_is_sticky_across_fast_handback();
    test_dispatch_epoch_invalidates_the_old_start();
    test_mission_authorization_requires_joint_hold_and_disarm();
    test_command_initiation_rechecks_epoch_and_fresh_pi_under_gate();
    return test_support::report("rc_ownership_tests");
}
