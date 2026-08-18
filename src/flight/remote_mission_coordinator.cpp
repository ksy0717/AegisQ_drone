#include "dc/flight/remote_mission_coordinator.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

namespace dc::flight {

RemoteMissionCoordinator::RemoteMissionCoordinator(
    CommandSource& source,
    MissionExecutor& executor,
    CommandLedger& ledger,
    StatusPublisher& publisher,
    std::string expected_vehicle_id,
    RouteLimits limits,
    CommandFreshnessLimits freshness_limits,
    Clock clock)
    : source_(source),
      executor_(executor),
      ledger_(ledger),
      publisher_(publisher),
      expected_vehicle_id_(std::move(expected_vehicle_id)),
      limits_(limits),
      freshness_limits_(freshness_limits),
      clock_(std::move(clock))
{
    if (expected_vehicle_id_.empty()) {
        throw std::invalid_argument("expected_vehicle_id must not be empty");
    }
    if (!valid_route_limits(limits_)) {
        throw std::invalid_argument("route limits must be configured");
    }
    if (freshness_limits_.max_age <= std::chrono::seconds::zero()
        || freshness_limits_.max_future_skew < std::chrono::seconds::zero()) {
        throw std::invalid_argument("freshness limits must be configured");
    }
    if (!clock_) {
        throw std::invalid_argument("clock must be configured");
    }
}

void RemoteMissionCoordinator::run()
{
    while (!stop_requested_.load()) {
        ExecutionEvent event;
        while (executor_.wait_event(event, std::chrono::milliseconds{0})) {
            static_cast<void>(process_event(event));
        }
        RemoteCommand command;
        if (source_.wait_next(command, std::chrono::milliseconds{50})) {
            static_cast<void>(process_one(command));
        }
    }

    while (!executor_.wait_stopped(std::chrono::milliseconds{50})) {
        ExecutionEvent pending_event;
        while (executor_.wait_event(
            pending_event, std::chrono::milliseconds{0})) {
            static_cast<void>(process_event(pending_event));
        }
    }
    ExecutionEvent event;
    while (executor_.wait_event(event, std::chrono::milliseconds{0})) {
        static_cast<void>(process_event(event));
    }
}

void RemoteMissionCoordinator::request_stop() noexcept
{
    stop_requested_.store(true);
    source_.close();
    executor_.shutdown();
}

bool RemoteMissionCoordinator::process_one(const RemoteCommand& command)
{
    const ValidationResult validation = validate_command(command, limits_);
    if (!validation.ok()) {
        static_cast<void>(
            publish_result(
                command,
                "REJECTED",
                validation_error_name(validation.error),
                StatusRecordMode::outbox_only));
        return false;
    }
    if (command.vehicle_id != expected_vehicle_id_) {
        static_cast<void>(publish_result(
            command,
            "REJECTED",
            "unexpected_vehicle_id",
            StatusRecordMode::outbox_only));
        return false;
    }

    const CommandClaimResult claim = ledger_.claim(command);
    if (claim.disposition == CommandClaimDisposition::unavailable) {
        return false;
    }
    if (claim.disposition == CommandClaimDisposition::conflict) {
        static_cast<void>(publish_result(
            command,
            "REJECTED",
            "command_id_conflict",
            StatusRecordMode::outbox_only));
        return false;
    }
    if (claim.disposition == CommandClaimDisposition::duplicate) {
        if (!claim.latest_status.has_value()) {
            // This is possible only after a crash between durable claim and
            // the first status record. Startup recovery must make the vehicle
            // safe before replacing this with a terminal FAILED status.
            return false;
        }
        const StatusPublishResult replay = publisher_.publish(
            *claim.latest_status, StatusRecordMode::outbox_only);
        return replay.durable && claim.latest_status->state == "ACCEPTED";
    }

    const ValidationResult freshness =
        validate_freshness(command, freshness_limits_, clock_());
    if (!freshness.ok()) {
        static_cast<void>(
            publish_result(
                command,
                "REJECTED",
                validation_error_name(freshness.error),
                StatusRecordMode::update_claim));
        return false;
    }

    MissionStatus status;
    try {
        const CommandResult result = executor_.dispatch(command);
        status = publish_result(
            command,
            result.accepted ? "ACCEPTED" : "REJECTED",
            result.detail,
            StatusRecordMode::update_claim);
    } catch (const std::exception& error) {
        status = publish_result(
            command, "FAILED", error.what(), StatusRecordMode::update_claim);
    } catch (...) {
        status = publish_result(
            command,
            "FAILED",
            "unknown_executor_error",
            StatusRecordMode::update_claim);
    }
    return status.state == "ACCEPTED";
}

bool RemoteMissionCoordinator::process_event(const ExecutionEvent& event)
{
    if (event.command_id.empty()
        || event.vehicle_id != expected_vehicle_id_
        || (event.outcome != ExecutionOutcome::completed
            && event.outcome != ExecutionOutcome::failed)) {
        return false;
    }

    MissionStatus status;
    status.command_id = event.command_id;
    status.vehicle_id = event.vehicle_id;
    status.mission_id = event.mission_id;
    status.state = event.outcome == ExecutionOutcome::completed
        ? "COMPLETED"
        : "FAILED";
    status.detail = event.detail;
    return publisher_.publish(status, StatusRecordMode::update_claim).durable;
}

MissionStatus RemoteMissionCoordinator::publish_result(
    const RemoteCommand& command,
    const std::string& state,
    const std::string& detail,
    const StatusRecordMode mode)
{
    MissionStatus status;
    status.command_id = command.command_id;
    status.vehicle_id = command.vehicle_id;
    status.mission_id = command.mission_id.value_or("");
    status.state = state;
    status.detail = detail;
    const StatusPublishResult result = publisher_.publish(status, mode);
    if (!result.durable) {
        status.state = "PERSISTENCE_FAILED";
        status.detail = result.detail.empty()
            ? "status_outbox_unavailable"
            : result.detail;
    }
    return status;
}

}  // namespace dc::flight
