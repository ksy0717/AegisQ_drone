#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>

#include "dc/flight/ports.hpp"

namespace dc::flight {

// Serializes transport commands onto one flight-control owner. This class does
// not know MQTT or MAVSDK; those dependencies remain behind the ports.
class RemoteMissionCoordinator {
public:
    using Clock = std::function<std::chrono::system_clock::time_point()>;

    RemoteMissionCoordinator(
        CommandSource& source,
        MissionExecutor& executor,
        CommandLedger& ledger,
        StatusPublisher& publisher,
        std::string expected_vehicle_id,
        RouteLimits limits,
        CommandFreshnessLimits freshness_limits,
        Clock clock = [] { return std::chrono::system_clock::now(); });

    void run();
    void request_stop() noexcept;

    // Exposed for deterministic unit tests and non-blocking application loops.
    // Call from exactly one worker thread.
    [[nodiscard]] bool process_one(const RemoteCommand& command);
    [[nodiscard]] bool process_event(const ExecutionEvent& event);

private:
    [[nodiscard]] MissionStatus publish_result(
        const RemoteCommand& command,
        const std::string& state,
        const std::string& detail,
        StatusRecordMode mode);

    CommandSource& source_;
    MissionExecutor& executor_;
    CommandLedger& ledger_;
    StatusPublisher& publisher_;
    std::string expected_vehicle_id_;
    RouteLimits limits_;
    CommandFreshnessLimits freshness_limits_;
    Clock clock_;
    std::atomic<bool> stop_requested_{false};
};

}  // namespace dc::flight
