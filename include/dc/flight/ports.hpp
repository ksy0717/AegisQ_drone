#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "dc/flight/remote_command.hpp"

namespace dc::flight {

struct CommandResult {
    bool accepted{};
    std::string detail;
};

enum class ExecutionOutcome : std::uint8_t {
    completed,
    failed,
};

struct ExecutionEvent {
    std::string command_id;
    std::string vehicle_id;
    std::string mission_id;
    ExecutionOutcome outcome{ExecutionOutcome::failed};
    std::string detail;
};

struct MissionStatus {
    std::string command_id;
    std::string vehicle_id;
    std::string mission_id;
    std::string state;
    std::string detail;
};

enum class CommandClaimDisposition : std::uint8_t {
    claimed,
    duplicate,
    conflict,
    unavailable,
};

struct CommandClaimResult {
    CommandClaimDisposition disposition{CommandClaimDisposition::unavailable};
    std::optional<MissionStatus> latest_status;
    std::string detail;
};

// The production implementation must make claim() durable before returning
// claimed. A crash after this point may lose progress, but must never allow the
// same command UUID to reach PX4 a second time.
class CommandLedger {
public:
    virtual ~CommandLedger() = default;
    virtual CommandClaimResult claim(const RemoteCommand& command) noexcept = 0;
};

// MQTT callbacks must only decode and enqueue. A single application worker
// drains this port and is the sole caller of MissionExecutor.
class CommandSource {
public:
    virtual ~CommandSource() = default;
    virtual bool wait_next(RemoteCommand& command, std::chrono::milliseconds timeout) = 0;
    virtual void close() noexcept = 0;
};

// dispatch() must return after a bounded queue operation; long-running flight
// work lives on the executor's sole PX4-owner thread. This lets the coordinator
// deliver safety commands while a mission is active. shutdown() only signals
// that owner and may be called by a process-control thread.
class MissionExecutor {
public:
    virtual ~MissionExecutor() = default;
    virtual CommandResult dispatch(const RemoteCommand& command) = 0;
    virtual bool wait_event(
        ExecutionEvent& event, std::chrono::milliseconds timeout) = 0;
    virtual CommandResult recover_after_restart() = 0;
    virtual bool wait_stopped(std::chrono::milliseconds timeout) noexcept = 0;
    virtual void shutdown() noexcept = 0;
};

enum class StatusRecordMode : std::uint8_t {
    update_claim,
    outbox_only,
};

struct StatusPublishResult {
    bool durable{};
    std::string detail;
};

class StatusPublisher {
public:
    virtual ~StatusPublisher() = default;
    // A production implementation persists the exact ACK in an outbox before
    // returning durable=true. Broker errors remain contained and are retried
    // from that outbox; they cannot unwind into flight control.
    virtual StatusPublishResult publish(
        const MissionStatus& status, StatusRecordMode mode) noexcept = 0;
};

}  // namespace dc::flight
