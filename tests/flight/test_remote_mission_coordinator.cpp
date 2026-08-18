#include "dc/flight/remote_mission_coordinator.hpp"
#include "test_support.hpp"

#include <chrono>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using dc::flight::CommandFreshnessLimits;
using dc::flight::CommandClaimDisposition;
using dc::flight::CommandClaimResult;
using dc::flight::CommandLedger;
using dc::flight::CommandResult;
using dc::flight::CommandSource;
using dc::flight::ExecutionEvent;
using dc::flight::ExecutionOutcome;
using dc::flight::MissionExecutor;
using dc::flight::MissionStatus;
using dc::flight::RemoteCommand;
using dc::flight::RemoteCommandType;
using dc::flight::RemoteMissionCoordinator;
using dc::flight::LaunchSpec;
using dc::flight::RouteLimits;
using dc::flight::RoutePlan;
using dc::flight::StatusPublisher;
using dc::flight::StatusPublishResult;
using dc::flight::StatusRecordMode;
using dc::flight::WaypointAction;
using dc::flight::WaypointPositionMode;

constexpr RouteLimits kLimits{10U, 2.0, 120.0, 60U, 120U, 0.5, 8.0};
constexpr CommandFreshnessLimits kFreshness{
    std::chrono::seconds{300},
    std::chrono::seconds{10},
};
const auto kNow =
    std::chrono::system_clock::time_point{std::chrono::seconds{1'786'000'000}};

class IdleSource final : public CommandSource {
public:
    bool wait_next(RemoteCommand&, std::chrono::milliseconds) override
    {
        return false;
    }

    void close() noexcept override
    {
        closed = true;
    }

    bool closed{};
};

class FakeExecutor final : public MissionExecutor {
public:
    CommandResult dispatch(const RemoteCommand& command) override
    {
        ++calls;
        last_command_id = command.command_id;
        if (throw_error) {
            throw std::runtime_error("executor_error");
        }
        return next_result;
    }

    bool wait_event(ExecutionEvent& event, std::chrono::milliseconds) override
    {
        if (events.empty()) {
            return false;
        }
        event = std::move(events.front());
        events.erase(events.begin());
        return true;
    }

    CommandResult recover_after_restart() override
    {
        ++recovery_calls;
        return recovery_result;
    }

    bool wait_stopped(std::chrono::milliseconds) noexcept override
    {
        return shutdown_called;
    }

    void shutdown() noexcept override
    {
        shutdown_called = true;
    }

    int calls{};
    std::string last_command_id;
    bool throw_error{};
    CommandResult next_result{true, "queued"};
    std::vector<ExecutionEvent> events;
    int recovery_calls{};
    CommandResult recovery_result{true, "vehicle_safe"};
    bool shutdown_called{};
};

class RecordingPublisher final : public CommandLedger, public StatusPublisher {
public:
    CommandClaimResult claim(const RemoteCommand& command) noexcept override
    {
        const auto found = commands.find(command.command_id);
        if (found == commands.end()) {
            commands.emplace(
                command.command_id,
                StoredCommand{command.canonical_payload, std::nullopt});
            return {CommandClaimDisposition::claimed, std::nullopt, {}};
        }
        if (found->second.canonical_payload != command.canonical_payload) {
            return {CommandClaimDisposition::conflict, std::nullopt, {}};
        }
        return {
            CommandClaimDisposition::duplicate,
            found->second.latest_status,
            {},
        };
    }

    StatusPublishResult publish(
        const MissionStatus& status, const StatusRecordMode mode) noexcept override
    {
        statuses.push_back(status);
        if (mode == StatusRecordMode::update_claim) {
            const auto found = commands.find(status.command_id);
            if (found != commands.end()) {
                found->second.latest_status = status;
            }
        }
        return {true, {}};
    }

    struct StoredCommand {
        std::string canonical_payload;
        std::optional<MissionStatus> latest_status;
    };

    std::unordered_map<std::string, StoredCommand> commands;
    std::vector<MissionStatus> statuses;
};

RemoteCommand mission(
    std::string command_id,
    std::string vehicle_id = "DRONE-01")
{
    RemoteCommand command;
    command.command_id = std::move(command_id);
    command.vehicle_id = std::move(vehicle_id);
    command.type = RemoteCommandType::execute_mission;
    command.mission_id = "f7c5c855-6677-4889-aabb-112233445566";
    command.route = RoutePlan{
        {
            {36.6610082, 126.3417854, 20.0, 1U,
             WaypointAction::takeoff, 0U, WaypointPositionMode::current,
             std::nullopt},
            {std::nullopt, std::nullopt, std::nullopt, 2U,
             WaypointAction::land, 0U, WaypointPositionMode::current,
             std::nullopt},
        },
        "ABSOLUTE",
        "HOME_RELATIVE",
        LaunchSpec{"CURRENT_POSITION", 20.0},
        std::nullopt,
    };
    command.issued_by = "ksy";
    command.issued_at = kNow;
    command.expires_at = kNow + std::chrono::seconds{60};
    command.canonical_payload = command.command_id + ":mission-v1";
    return command;
}

RemoteMissionCoordinator coordinator(
    IdleSource& source,
    FakeExecutor& executor,
    RecordingPublisher& publisher)
{
    return {
        source,
        executor,
        publisher,
        publisher,
        "DRONE-01",
        kLimits,
        kFreshness,
        [] { return kNow; },
    };
}

void test_valid_command_is_dispatched_and_published()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto target = coordinator(source, executor, publisher);

    const bool accepted =
        target.process_one(mission("2ac105b7-2233-4455-aabb-66778899aabb"));
    test_support::check(accepted, "accepted executor result is returned");
    test_support::check(executor.calls == 1, "valid command reaches flight executor once");
    test_support::check(publisher.statuses.size() == 1U, "result is published once");
    test_support::check(
        publisher.statuses.back().state == "ACCEPTED", "ACCEPTED state is published");
    test_support::check(
        publisher.statuses.back().command_id
            == "2ac105b7-2233-4455-aabb-66778899aabb",
        "command id is preserved in status");
}

void test_invalid_command_never_reaches_executor()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto target = coordinator(source, executor, publisher);

    RemoteCommand command =
        mission("2ac105b7-2233-4455-aabb-66778899aabc");
    command.route->waypoints[0].relative_altitude_m = 500.0;
    const bool accepted = target.process_one(command);
    test_support::check(!accepted, "invalid route is rejected");
    test_support::check(executor.calls == 0, "invalid route cannot reach PX4 adapter");
    test_support::check(
        publisher.statuses.back().detail == "altitude_out_of_range",
        "validation reason is published");
}

void test_duplicate_uuid_replays_original_ack()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto target = coordinator(source, executor, publisher);
    const std::string id = "2ac105b7-2233-4455-aabb-66778899aabd";

    static_cast<void>(target.process_one(mission(id)));
    static_cast<void>(target.process_one(mission(id)));

    test_support::check(executor.calls == 1, "duplicate UUID is not dispatched twice");
    test_support::check(publisher.statuses.size() == 2U, "duplicate receives an ACK replay");
    test_support::check(
        publisher.statuses[1].state == "ACCEPTED",
        "duplicate replays the original ACK instead of a rejection");
}

void test_delayed_duplicate_replays_and_changed_payload_conflicts()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto current_time = kNow;
    RemoteMissionCoordinator target{
        source,
        executor,
        publisher,
        publisher,
        "DRONE-01",
        kLimits,
        kFreshness,
        [&current_time] { return current_time; },
    };
    const std::string id = "2ac105b7-2233-4455-aabb-66778899aac1";

    RemoteCommand original = mission(id);
    static_cast<void>(target.process_one(original));

    current_time += std::chrono::seconds{301};
    static_cast<void>(target.process_one(original));
    test_support::check(
        executor.calls == 1,
        "stale broker duplicate is not dispatched");
    test_support::check(
        publisher.statuses.back().state == "ACCEPTED",
        "identical stale duplicate replays the original ACK");

    RemoteCommand changed = original;
    *changed.route->waypoints[0].latitude_deg += 0.001;
    changed.canonical_payload += ":changed";
    static_cast<void>(target.process_one(changed));
    test_support::check(
        executor.calls == 1,
        "same UUID with changed payload is not dispatched");
    test_support::check(
        publisher.statuses.back().detail == "command_id_conflict",
        "same UUID with changed payload is rejected as a conflict");
}

void test_distinct_uuid_is_not_ordered()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto target = coordinator(source, executor, publisher);

    static_cast<void>(target.process_one(
        mission("ffffffff-ffff-ffff-ffff-ffffffffffff")));
    static_cast<void>(target.process_one(
        mission("00000000-0000-0000-0000-000000000001")));
    test_support::check(executor.calls == 2, "UUIDs are deduplicated, never numerically ordered");
}

void test_stale_command_is_rejected()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto target = coordinator(source, executor, publisher);
    RemoteCommand command =
        mission("2ac105b7-2233-4455-aabb-66778899aabe");
    command.issued_at = kNow - std::chrono::seconds{301};

    static_cast<void>(target.process_one(command));
    test_support::check(executor.calls == 0, "stale command cannot reach executor");
    test_support::check(
        publisher.statuses.back().detail == "stale_command",
        "stale reason is published");
}

void test_unexpected_vehicle_and_executor_failure()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto target = coordinator(source, executor, publisher);

    static_cast<void>(target.process_one(mission(
        "2ac105b7-2233-4455-aabb-66778899aabf", "WHEEL-01")));
    test_support::check(executor.calls == 0, "wrong vehicle cannot reach PX4");
    test_support::check(
        publisher.statuses.back().detail == "unexpected_vehicle_id",
        "wrong vehicle id is reported");

    executor.throw_error = true;
    static_cast<void>(target.process_one(
        mission("2ac105b7-2233-4455-aabb-66778899aac0")));
    test_support::check(
        publisher.statuses.back().state == "FAILED",
        "executor exception publishes FAILED");
}

void test_request_stop_closes_source()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto target = coordinator(source, executor, publisher);

    target.request_stop();
    test_support::check(source.closed, "stop wakes and closes command source");
    test_support::check(
        executor.shutdown_called,
        "stop signals the sole PX4-owner executor");
}

void test_terminal_event_updates_durable_status()
{
    IdleSource source;
    FakeExecutor executor;
    RecordingPublisher publisher;
    auto target = coordinator(source, executor, publisher);
    const RemoteCommand command =
        mission("2ac105b7-2233-4455-aabb-66778899aac2");

    static_cast<void>(target.process_one(command));
    const ExecutionEvent completed{
        command.command_id,
        command.vehicle_id,
        command.mission_id.value_or(""),
        ExecutionOutcome::completed,
        "landed_and_disarmed",
    };
    test_support::check(
        target.process_event(completed),
        "executor terminal event is durably published");
    test_support::check(
        publisher.statuses.size() == 2U
            && publisher.statuses.back().state == "COMPLETED",
        "ACCEPTED is followed by terminal COMPLETED");

    const auto replay = publisher.claim(command);
    test_support::check(
        replay.latest_status.has_value()
            && replay.latest_status->state == "COMPLETED",
        "terminal result replaces nonterminal state in the ledger");
}

}  // namespace

int main()
{
    test_valid_command_is_dispatched_and_published();
    test_invalid_command_never_reaches_executor();
    test_duplicate_uuid_replays_original_ack();
    test_delayed_duplicate_replays_and_changed_payload_conflicts();
    test_distinct_uuid_is_not_ordered();
    test_stale_command_is_rejected();
    test_unexpected_vehicle_and_executor_failure();
    test_request_stop_closes_source();
    test_terminal_event_updates_durable_status();
    return test_support::report("remote_mission_coordinator_tests");
}
