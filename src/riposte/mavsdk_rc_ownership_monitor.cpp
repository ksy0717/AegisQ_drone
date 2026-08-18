#include "mavsdk_rc_ownership_monitor.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace dc::flight {
namespace {

[[nodiscard]] std::optional<std::uint16_t> channel_pwm(
    const mavlink_rc_channels_t& channels,
    const std::uint8_t channel) noexcept
{
    if (channel == 0U || channel > 18U || channels.chancount < channel) {
        return std::nullopt;
    }
    const std::array<std::uint16_t, 18U> values{
        channels.chan1_raw,
        channels.chan2_raw,
        channels.chan3_raw,
        channels.chan4_raw,
        channels.chan5_raw,
        channels.chan6_raw,
        channels.chan7_raw,
        channels.chan8_raw,
        channels.chan9_raw,
        channels.chan10_raw,
        channels.chan11_raw,
        channels.chan12_raw,
        channels.chan13_raw,
        channels.chan14_raw,
        channels.chan15_raw,
        channels.chan16_raw,
        channels.chan17_raw,
        channels.chan18_raw,
    };
    const std::uint16_t pwm = values[static_cast<std::size_t>(channel) - 1U];
    if (pwm == std::numeric_limits<std::uint16_t>::max()) {
        return std::nullopt;
    }
    return pwm;
}

}  // namespace

struct MavsdkRcOwnershipMonitor::SharedState {
    explicit SharedState(const MavsdkRcOwnershipMonitorConfig& config)
        : gate(config.tracker, RcArmSwitchTrackerConfig{
              config.disarm_pwm,
              config.arm_pwm,
              config.tracker.handback_debounce,
              config.tracker.stale_timeout})
    {
    }

    RcCommandAuthorizationGate gate;
};

std::shared_ptr<mavsdk::System> MavsdkRcOwnershipMonitor::require_system(
    const std::shared_ptr<mavsdk::System>& system)
{
    if (!system) {
        throw std::invalid_argument("MAVSDK system is required for RC ownership");
    }
    return system;
}

MavsdkRcOwnershipMonitor::MavsdkRcOwnershipMonitor(
    std::shared_ptr<mavsdk::System> system,
    MavsdkRcOwnershipMonitorConfig config)
    : config_(std::move(config)),
      state_(std::make_shared<SharedState>(config_)),
      passthrough_(require_system(system))
{
    if (config_.channel == 0U || config_.channel > 18U
        || config_.arm_channel == 0U || config_.arm_channel > 18U
        || config_.arm_channel == config_.channel
        || !std::isfinite(config_.message_rate_hz)
        || config_.message_rate_hz < 2.0
        || config_.message_rate_hz > 50.0) {
        throw std::invalid_argument("invalid MAVSDK RC ownership monitor configuration");
    }

    target_system_id_ = passthrough_.get_target_sysid();
    if (target_system_id_ == 0U) {
        throw std::runtime_error("PX4 target system id is unavailable for RC ownership");
    }
    const std::weak_ptr<SharedState> weak_state{state_};
    const std::uint8_t target_system_id = target_system_id_;
    const std::uint8_t ownership_channel = config_.channel;
    const std::uint8_t arm_channel = config_.arm_channel;
    subscription_ = passthrough_.subscribe_message(
        MAVLINK_MSG_ID_RC_CHANNELS,
        [weak_state,
         target_system_id,
         ownership_channel,
         arm_channel](
            const mavlink_message_t& message) {
            if (message.msgid != MAVLINK_MSG_ID_RC_CHANNELS
                || message.sysid != target_system_id
                || message.compid
                    != static_cast<std::uint8_t>(MAV_COMP_ID_AUTOPILOT1)) {
                return;
            }

            const std::shared_ptr<SharedState> state = weak_state.lock();
            if (!state) {
                return;
            }
            mavlink_rc_channels_t channels{};
            mavlink_msg_rc_channels_decode(&message, &channels);
            const std::optional<std::uint16_t> ownership_pwm =
                channel_pwm(channels, ownership_channel);
            state->gate.observe(
                ownership_pwm,
                channel_pwm(channels, arm_channel),
                RcCommandAuthorizationGate::Clock::now());
        });
    if (!subscription_.valid()) {
        throw std::runtime_error("cannot subscribe to PX4 RC_CHANNELS");
    }
}

MavsdkRcOwnershipMonitor::~MavsdkRcOwnershipMonitor()
{
    rate_confirmed_.store(false, std::memory_order_release);
    try {
        if (subscription_.valid()) {
            passthrough_.unsubscribe_message(
                MAVLINK_MSG_ID_RC_CHANNELS, subscription_);
        }
    } catch (...) {
        // A callback captures only weak_state. Even if MAVSDK has already
        // dispatched it, releasing our shared state below cannot expose this
        // object after destruction.
    }
    state_.reset();
}

void MavsdkRcOwnershipMonitor::require_message_rate()
{
    rate_confirmed_.store(false, std::memory_order_release);
    const double interval_us = 1'000'000.0 / config_.message_rate_hz;
    const mavsdk::MavlinkPassthrough::CommandLong command{
        target_system_id_,
        static_cast<std::uint8_t>(MAV_COMP_ID_AUTOPILOT1),
        static_cast<std::uint16_t>(MAV_CMD_SET_MESSAGE_INTERVAL),
        static_cast<float>(MAVLINK_MSG_ID_RC_CHANNELS),
        static_cast<float>(interval_us),
        0.0F,
        0.0F,
        0.0F,
        0.0F,
        0.0F,
    };
    const mavsdk::MavlinkPassthrough::Result result =
        passthrough_.send_command_long(command);
    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        state_->gate.observe(
            std::nullopt,
            std::nullopt,
            RcCommandAuthorizationGate::Clock::now());
        std::ostringstream message;
        message << "PX4 RC_CHANNELS rate request failed: " << result;
        throw std::runtime_error(message.str());
    }
    rate_confirmed_.store(true, std::memory_order_release);
}

RcOwnershipSnapshot MavsdkRcOwnershipMonitor::snapshot() const noexcept
{
    if (!rate_confirmed_.load(std::memory_order_acquire)) {
        return {};
    }
    try {
        const std::shared_ptr<SharedState> state = state_;
        if (!state) {
            return {};
        }
        return state->gate.snapshot(RcCommandAuthorizationGate::Clock::now());
    } catch (...) {
        return {};
    }
}

std::uint64_t MavsdkRcOwnershipMonitor::takeover_epoch() const noexcept
{
    try {
        const std::shared_ptr<SharedState> state = state_;
        return state
            ? state->gate.takeover_epoch()
            : 0U;
    } catch (...) {
        return 0U;
    }
}

std::optional<std::uint64_t>
MavsdkRcOwnershipMonitor::mission_authorization_epoch() const noexcept
{
    if (!rate_confirmed_.load(std::memory_order_acquire)) {
        return std::nullopt;
    }
    try {
        const std::shared_ptr<SharedState> state = state_;
        return state
            ? state->gate.mission_authorization_epoch(
                  RcCommandAuthorizationGate::Clock::now())
            : std::nullopt;
    } catch (...) {
        return std::nullopt;
    }
}

bool MavsdkRcOwnershipMonitor::initiate_authorized_command(
    const std::uint64_t expected_epoch,
    const NonblockingCommandInitiator& initiator) const noexcept
{
    if (!rate_confirmed_.load(std::memory_order_acquire)) {
        return false;
    }
    try {
        const std::shared_ptr<SharedState> state = state_;
        return state
            && state->gate.initiate_authorized_command(
                expected_epoch,
                RcCommandAuthorizationGate::Clock::now(),
                initiator);
    } catch (...) {
        return false;
    }
}

}  // namespace dc::flight
