#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/system.h>

#include "dc/flight/rc_ownership.hpp"

namespace dc::flight {

struct MavsdkRcOwnershipMonitorConfig {
    RcOwnershipTrackerConfig tracker;
    RcPwmRange disarm_pwm{900U, 1100U};
    RcPwmRange arm_pwm{1900U, 2100U};
    std::uint8_t channel{6U};
    std::uint8_t arm_channel{5U};
    double message_rate_hz{};
};

// Receives PX4's RC_CHANNELS stream on a MAVSDK callback thread and exposes a
// small, immutable snapshot to the sole flight-command worker. The callback
// owns no reference to this object, so an already-dispatched callback cannot
// access a monitor whose subscription is being torn down.
class MavsdkRcOwnershipMonitor final : public RcOwnershipProvider {
public:
    MavsdkRcOwnershipMonitor(
        std::shared_ptr<mavsdk::System> system,
        MavsdkRcOwnershipMonitorConfig config);
    ~MavsdkRcOwnershipMonitor() override;

    MavsdkRcOwnershipMonitor(const MavsdkRcOwnershipMonitor&) = delete;
    MavsdkRcOwnershipMonitor& operator=(const MavsdkRcOwnershipMonitor&) = delete;

    // Enables RC_CHANNELS at the configured rate. Until PX4 acknowledges this
    // request, snapshot() remains UNKNOWN even if an incidental default stream
    // is present, so forgetting this startup gate cannot grant Pi ownership.
    void require_message_rate();

    [[nodiscard]] RcOwnershipSnapshot snapshot() const noexcept override;
    [[nodiscard]] std::uint64_t takeover_epoch() const noexcept override;
    [[nodiscard]] std::optional<std::uint64_t>
    mission_authorization_epoch() const noexcept override;
    [[nodiscard]] bool initiate_authorized_command(
        std::uint64_t expected_epoch,
        const NonblockingCommandInitiator& initiator) const noexcept override;

private:
    struct SharedState;

    static std::shared_ptr<mavsdk::System> require_system(
        const std::shared_ptr<mavsdk::System>& system);

    MavsdkRcOwnershipMonitorConfig config_;
    std::shared_ptr<SharedState> state_;
    mavsdk::MavlinkPassthrough passthrough_;
    mavsdk::MavlinkPassthrough::MessageHandle subscription_;
    std::uint8_t target_system_id_{};
    std::atomic_bool rate_confirmed_{false};
};

}  // namespace dc::flight
