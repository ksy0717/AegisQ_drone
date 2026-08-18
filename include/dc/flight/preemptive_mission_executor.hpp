#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "dc/flight/ports.hpp"

namespace dc::flight {

enum class PreemptionRequest : std::uint8_t {
    none,
    stop,
    return_to_launch,
    // A link-loss return yields to a battery return: losing the operator is
    // recoverable, running the pack flat is not.
    link_loss_return_to_launch,
    // Containment failure outranks losing the operator but yields to the
    // battery: leaving the area is recoverable by returning, a flat pack is not.
    geofence_return_to_launch,
    low_battery_return_to_launch,
    emergency_stop,
    shutdown,
};

class PreemptionToken final {
public:
    explicit PreemptionToken(
        const std::atomic<PreemptionRequest>& request) noexcept
        : request_(&request)
    {
    }

    [[nodiscard]] PreemptionRequest requested() const noexcept
    {
        return request_->load();
    }

private:
    const std::atomic<PreemptionRequest>* request_;
};

struct FlightOperationResult {
    ExecutionOutcome outcome{ExecutionOutcome::failed};
    std::string detail;
};

// Implementations run only on PreemptiveMissionExecutor's worker. During a
// route, execute() must poll token at the configured setpoint/watchdog cadence,
// cease route setpoints, and return FAILED when preemption is requested. The
// worker then executes the queued STOP/RTL/E-stop command in order; execute()
// must not perform that next command itself.
class MissionBackend {
public:
    virtual ~MissionBackend() = default;
    virtual CommandResult recover_after_restart(
        const PreemptionToken& token) = 0;
    virtual FlightOperationResult execute(
        const RemoteCommand& command,
        const PreemptionToken& token) = 0;
    virtual FlightOperationResult shutdown_safely(
        const PreemptionToken& token) = 0;
    // The executor calls this short read-only hook while accepting START. It
    // returns an epoch only for a jointly-published fresh HOLD+DISARM sample.
    // A later POSITION epoch invalidates that exact queued command even if CH6
    // returns to HOLD before the worker reaches PX4.
    [[nodiscard]] virtual std::optional<std::uint64_t>
    mission_authorization_epoch() const noexcept
    {
        return std::nullopt;
    }
};

struct PreemptiveExecutorConfig {
    std::size_t mission_queue_capacity{};
    std::size_t safety_queue_capacity{};
    std::size_t terminal_event_capacity{};
};

// Owns the only thread allowed to call MissionBackend/PX4. dispatch() only
// performs bounded memory operations. Emergency commands have a reserved queue
// and can raise the preemption token even when an ordinary mission queue is full.
class PreemptiveMissionExecutor final : public MissionExecutor {
public:
    PreemptiveMissionExecutor(
        MissionBackend& backend, PreemptiveExecutorConfig config);
    ~PreemptiveMissionExecutor() override;

    PreemptiveMissionExecutor(const PreemptiveMissionExecutor&) = delete;
    PreemptiveMissionExecutor& operator=(const PreemptiveMissionExecutor&) = delete;

    CommandResult dispatch(const RemoteCommand& command) override;
    bool wait_event(
        ExecutionEvent& event, std::chrono::milliseconds timeout) override;
    CommandResult recover_after_restart() override;
    // Used by the local battery and link-loss supervisors. The synthetic
    // command has no server commandId, so its terminal event is intentionally
    // not published as a command ACK.
    CommandResult request_low_battery_return_to_launch();
    // The server cannot command a vehicle it cannot reach, so once the broker
    // session has been down long enough the aircraft returns on its own.
    CommandResult request_link_loss_return_to_launch();
    // Temporary software containment: the aircraft returns on its own once it
    // is further than the configured radius from the launch point.
    CommandResult request_geofence_return_to_launch();
    bool wait_stopped(std::chrono::milliseconds timeout) noexcept override;
    void shutdown() noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

[[nodiscard]] const char* preemption_request_name(
    PreemptionRequest request) noexcept;

}  // namespace dc::flight
