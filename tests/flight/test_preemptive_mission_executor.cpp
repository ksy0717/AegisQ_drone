#include "dc/flight/preemptive_mission_executor.hpp"
#include "test_support.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using dc::flight::CommandResult;
using dc::flight::ExecutionEvent;
using dc::flight::ExecutionOutcome;
using dc::flight::FlightOperationResult;
using dc::flight::MissionBackend;
using dc::flight::PreemptionRequest;
using dc::flight::PreemptionToken;
using dc::flight::PreemptiveExecutorConfig;
using dc::flight::PreemptiveMissionExecutor;
using dc::flight::RemoteCommand;
using dc::flight::RemoteCommandType;

constexpr PreemptiveExecutorConfig kConfig{1U, 4U, 8U};

RemoteCommand command(
    std::string id, const RemoteCommandType type)
{
    RemoteCommand result;
    result.command_id = std::move(id);
    result.vehicle_id = "DRONE-01";
    result.type = type;
    result.mission_id = "f7c5c855-6677-4889-aabb-112233445566";
    return result;
}

class BlockingBackend final : public MissionBackend {
public:
    CommandResult recover_after_restart(const PreemptionToken&) override
    {
        ++recovery_calls;
        return recovery_result;
    }

    FlightOperationResult execute(
        const RemoteCommand& command,
        const PreemptionToken& token) override
    {
        {
            std::lock_guard<std::mutex> lock{mutex};
            calls.push_back(command.type);
            if (command.type == RemoteCommandType::execute_mission) {
                observed_dispatch_epoch = command.dispatch_ownership_epoch;
                mission_entered = true;
                entered.notify_all();
            }
        }
        if (command.type != RemoteCommandType::execute_mission) {
            return {ExecutionOutcome::completed, "safety_action_complete"};
        }

        while (!complete_mission.load()
               && token.requested() == PreemptionRequest::none) {
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        const PreemptionRequest requested = token.requested();
        if (requested != PreemptionRequest::none) {
            observed_preemption.store(requested);
            return {
                ExecutionOutcome::failed,
                std::string{"preempted_by_"}
                    + dc::flight::preemption_request_name(requested),
            };
        }
        return {ExecutionOutcome::completed, "mission_complete"};
    }

    FlightOperationResult shutdown_safely(const PreemptionToken&) override
    {
        ++shutdown_calls;
        return {ExecutionOutcome::completed, "shutdown_safe"};
    }

    [[nodiscard]] std::optional<std::uint64_t>
    mission_authorization_epoch() const noexcept override
    {
        return mission_authorized.load()
            ? std::optional<std::uint64_t>{ownership_epoch_value.load()}
            : std::nullopt;
    }

    bool wait_until_mission_entered()
    {
        std::unique_lock<std::mutex> lock{mutex};
        return entered.wait_for(lock, std::chrono::seconds{2}, [this] {
            return mission_entered;
        });
    }

    std::mutex mutex;
    std::condition_variable entered;
    std::vector<RemoteCommandType> calls;
    std::atomic<bool> complete_mission{false};
    std::atomic<PreemptionRequest> observed_preemption{PreemptionRequest::none};
    std::atomic<std::uint64_t> ownership_epoch_value{7U};
    std::atomic_bool mission_authorized{true};
    std::optional<std::uint64_t> observed_dispatch_epoch;
    bool mission_entered{};
    int recovery_calls{};
    int shutdown_calls{};
    CommandResult recovery_result{true, "vehicle_safe"};
};

bool next_event(PreemptiveMissionExecutor& executor, ExecutionEvent& event)
{
    return executor.wait_event(event, std::chrono::seconds{2});
}

void test_recovery_gate_and_normal_completion()
{
    BlockingBackend backend;
    PreemptiveMissionExecutor executor{backend, kConfig};
    const RemoteCommand mission = command(
        "00000000-0000-4000-8000-000000000001",
        RemoteCommandType::execute_mission);

    test_support::check(
        !executor.dispatch(mission).accepted,
        "commands are blocked before PX4 recovery");
    test_support::check(
        executor.recover_after_restart().accepted && backend.recovery_calls == 1,
        "backend recovery opens the dispatch gate");
    test_support::check(
        executor.dispatch(mission).accepted,
        "mission dispatch is a bounded queue operation");
    test_support::check(
        backend.wait_until_mission_entered(),
        "sole backend worker starts the mission");
    {
        const std::lock_guard<std::mutex> lock{backend.mutex};
        test_support::check(
            backend.observed_dispatch_epoch
                == std::optional<std::uint64_t>{7U},
            "executor stamps the ownership epoch at START acceptance");
    }
    backend.complete_mission.store(true);

    ExecutionEvent event;
    test_support::check(
        next_event(executor, event)
            && event.command_id == mission.command_id
            && event.outcome == ExecutionOutcome::completed,
        "actual backend completion becomes a terminal event");
    executor.shutdown();
    test_support::check(
        executor.wait_stopped(std::chrono::seconds{2}),
        "safe shutdown finishes on the backend worker");
}

void test_start_without_rc_authorization_is_rejected_before_queueing()
{
    BlockingBackend backend;
    backend.mission_authorized.store(false);
    PreemptiveMissionExecutor executor{backend, kConfig};
    static_cast<void>(executor.recover_after_restart());
    const RemoteCommand mission = command(
        "00000000-0000-4000-8000-00000000000d",
        RemoteCommandType::execute_mission);

    const CommandResult result = executor.dispatch(mission);
    bool worker_untouched = false;
    {
        const std::lock_guard<std::mutex> lock{backend.mutex};
        worker_untouched = backend.calls.empty() && !backend.mission_entered;
    }
    test_support::check(
        !result.accepted
            && result.detail == "mission_start_not_authorized_by_rc"
            && worker_untouched,
        "START without a fresh HOLD+DISARM authorization never reaches the worker");
    executor.shutdown();
    static_cast<void>(executor.wait_stopped(std::chrono::seconds{2}));
}

void test_stop_preempts_active_mission()
{
    BlockingBackend backend;
    PreemptiveMissionExecutor executor{backend, kConfig};
    static_cast<void>(executor.recover_after_restart());
    const RemoteCommand mission = command(
        "00000000-0000-4000-8000-000000000002",
        RemoteCommandType::execute_mission);
    const RemoteCommand stop = command(
        "00000000-0000-4000-8000-000000000003",
        RemoteCommandType::stop_mission);

    static_cast<void>(executor.dispatch(mission));
    test_support::check(backend.wait_until_mission_entered(), "mission is active");
    test_support::check(
        executor.dispatch(stop).accepted,
        "STOP enters its reserved queue during active flight");

    ExecutionEvent mission_event;
    ExecutionEvent stop_event;
    test_support::check(
        next_event(executor, mission_event)
            && mission_event.command_id == mission.command_id
            && mission_event.outcome == ExecutionOutcome::failed,
        "preempted mission gets terminal FAILED");
    test_support::check(
        next_event(executor, stop_event)
            && stop_event.command_id == stop.command_id
            && stop_event.outcome == ExecutionOutcome::completed,
        "STOP itself gets terminal COMPLETED");
    test_support::check(
        backend.observed_preemption.load() == PreemptionRequest::stop,
        "backend observes STOP through the polling token");
    executor.shutdown();
    static_cast<void>(executor.wait_stopped(std::chrono::seconds{2}));
}

void test_emergency_stop_overtakes_queued_safety_work()
{
    BlockingBackend backend;
    PreemptiveMissionExecutor executor{backend, kConfig};
    static_cast<void>(executor.recover_after_restart());
    const RemoteCommand mission = command(
        "00000000-0000-4000-8000-000000000004",
        RemoteCommandType::execute_mission);
    const RemoteCommand stop = command(
        "00000000-0000-4000-8000-000000000005",
        RemoteCommandType::stop_mission);
    const RemoteCommand rtl = command(
        "00000000-0000-4000-8000-000000000006",
        RemoteCommandType::return_to_launch);
    const RemoteCommand emergency = command(
        "00000000-0000-4000-8000-000000000007",
        RemoteCommandType::emergency_stop);

    static_cast<void>(executor.dispatch(mission));
    static_cast<void>(backend.wait_until_mission_entered());
    static_cast<void>(executor.dispatch(stop));
    static_cast<void>(executor.dispatch(rtl));
    static_cast<void>(executor.dispatch(emergency));

    ExecutionEvent event;
    static_cast<void>(next_event(executor, event));
    static_cast<void>(next_event(executor, event));
    test_support::check(
        event.command_id == emergency.command_id,
        "E-stop overtakes queued STOP/RTL after mission preemption");
    test_support::check(
        backend.observed_preemption.load() == PreemptionRequest::emergency_stop,
        "strongest pending safety request reaches the backend token");

    static_cast<void>(next_event(executor, event));
    test_support::check(
        event.command_id == stop.command_id,
        "non-emergency safety commands otherwise retain FIFO order");
    static_cast<void>(next_event(executor, event));
    test_support::check(
        event.command_id == rtl.command_id,
        "server STOP then RTL order is preserved");
    executor.shutdown();
    static_cast<void>(executor.wait_stopped(std::chrono::seconds{2}));
}

void test_low_battery_supervisor_preempts_and_queues_rtl()
{
    BlockingBackend backend;
    PreemptiveMissionExecutor executor{backend, kConfig};
    static_cast<void>(executor.recover_after_restart());
    const RemoteCommand mission = command(
        "00000000-0000-4000-8000-000000000009",
        RemoteCommandType::execute_mission);

    static_cast<void>(executor.dispatch(mission));
    static_cast<void>(backend.wait_until_mission_entered());
    test_support::check(
        executor.request_low_battery_return_to_launch().accepted,
        "local battery supervisor has a reserved RTL path");

    ExecutionEvent mission_event;
    ExecutionEvent internal_event;
    test_support::check(
        next_event(executor, mission_event)
            && mission_event.command_id == mission.command_id
            && mission_event.outcome == ExecutionOutcome::failed,
        "low battery terminalizes the active server mission");
    test_support::check(
        next_event(executor, internal_event)
            && internal_event.command_id.empty()
            && internal_event.outcome == ExecutionOutcome::completed,
        "synthetic low-battery RTL completes without forging a server ACK");
    test_support::check(
        backend.observed_preemption.load()
            == PreemptionRequest::low_battery_return_to_launch,
        "backend observes the local low-battery reason");
    {
        const std::lock_guard<std::mutex> lock{backend.mutex};
        test_support::check(
            backend.calls.size() == 2U
                && backend.calls.back()
                    == RemoteCommandType::return_to_launch,
            "PX4 owner executes RTL after stopping the mission");
    }
    executor.shutdown();
    static_cast<void>(executor.wait_stopped(std::chrono::seconds{2}));
}

void test_link_loss_supervisor_preempts_and_queues_rtl()
{
    BlockingBackend backend;
    PreemptiveMissionExecutor executor{backend, kConfig};
    static_cast<void>(executor.recover_after_restart());
    const RemoteCommand mission = command(
        "00000000-0000-4000-8000-00000000000a",
        RemoteCommandType::execute_mission);

    static_cast<void>(executor.dispatch(mission));
    static_cast<void>(backend.wait_until_mission_entered());
    test_support::check(
        executor.request_link_loss_return_to_launch().accepted,
        "link-loss supervisor has a reserved RTL path");

    ExecutionEvent mission_event;
    ExecutionEvent internal_event;
    test_support::check(
        next_event(executor, mission_event)
            && mission_event.command_id == mission.command_id
            && mission_event.outcome == ExecutionOutcome::failed,
        "link loss terminalizes the active server mission");
    test_support::check(
        next_event(executor, internal_event)
            && internal_event.command_id.empty()
            && internal_event.outcome == ExecutionOutcome::completed,
        "synthetic link-loss RTL completes without forging a server ACK");
    test_support::check(
        backend.observed_preemption.load()
            == PreemptionRequest::link_loss_return_to_launch,
        "backend observes the local link-loss reason");
    {
        const std::lock_guard<std::mutex> lock{backend.mutex};
        test_support::check(
            backend.calls.size() == 2U
                && backend.calls.back()
                    == RemoteCommandType::return_to_launch,
            "PX4 owner executes RTL after stopping the mission");
    }
    executor.shutdown();
    static_cast<void>(executor.wait_stopped(std::chrono::seconds{2}));
}

void test_link_loss_return_is_not_duplicated()
{
    BlockingBackend backend;
    PreemptiveMissionExecutor executor{backend, kConfig};
    static_cast<void>(executor.recover_after_restart());
    static_cast<void>(executor.dispatch(command(
        "00000000-0000-4000-8000-00000000000b",
        RemoteCommandType::execute_mission)));
    static_cast<void>(backend.wait_until_mission_entered());

    test_support::check(
        executor.request_link_loss_return_to_launch().accepted,
        "the first link-loss return is queued");
    // The supervisor re-checks on its cadence while the broker stays down, so
    // a second request during the same outage must not stack another RTL.
    const CommandResult repeated = executor.request_link_loss_return_to_launch();
    test_support::check(
        repeated.accepted && repeated.detail == "safe_return_already_pending",
        "a repeated link-loss request does not queue a second return");
    const CommandResult battery =
        executor.request_low_battery_return_to_launch();
    test_support::check(
        battery.accepted && battery.detail == "safe_return_already_pending",
        "a battery return defers to the pending link-loss return");

    executor.shutdown();
    static_cast<void>(executor.wait_stopped(std::chrono::seconds{2}));
}

void test_geofence_return_outranks_link_loss()
{
    BlockingBackend backend;
    PreemptiveMissionExecutor executor{backend, kConfig};
    static_cast<void>(executor.recover_after_restart());
    static_cast<void>(executor.dispatch(command(
        "00000000-0000-4000-8000-00000000000c",
        RemoteCommandType::execute_mission)));
    static_cast<void>(backend.wait_until_mission_entered());

    test_support::check(
        executor.request_geofence_return_to_launch().accepted,
        "the geofence supervisor has a reserved RTL path");

    ExecutionEvent mission_event;
    ExecutionEvent internal_event;
    test_support::check(
        next_event(executor, mission_event)
            && mission_event.outcome == ExecutionOutcome::failed,
        "a geofence breach terminalizes the active server mission");
    test_support::check(
        next_event(executor, internal_event)
            && internal_event.command_id.empty()
            && internal_event.outcome == ExecutionOutcome::completed,
        "synthetic geofence RTL completes without forging a server ACK");
    // Read only after the terminal events land: the backend observes the
    // token on its own thread, so checking straight after the request races.
    test_support::check(
        backend.observed_preemption.load()
            == PreemptionRequest::geofence_return_to_launch,
        "backend observes the geofence reason");

    executor.shutdown();
    static_cast<void>(executor.wait_stopped(std::chrono::seconds{2}));
}

void test_shutdown_preempts_and_terminalizes_active_work()
{
    BlockingBackend backend;
    PreemptiveMissionExecutor executor{backend, kConfig};
    static_cast<void>(executor.recover_after_restart());
    const RemoteCommand mission = command(
        "00000000-0000-4000-8000-000000000008",
        RemoteCommandType::execute_mission);
    static_cast<void>(executor.dispatch(mission));
    static_cast<void>(backend.wait_until_mission_entered());

    executor.shutdown();
    test_support::check(
        executor.wait_stopped(std::chrono::seconds{2}),
        "shutdown waits for the backend safe action");
    ExecutionEvent event;
    test_support::check(
        next_event(executor, event)
            && event.outcome == ExecutionOutcome::failed
            && backend.shutdown_calls == 1,
        "active command is terminalized before safe backend shutdown");
}

void test_invalid_bounds_fail_closed()
{
    BlockingBackend backend;
    bool rejected = false;
    try {
        PreemptiveMissionExecutor invalid{backend, {1U, 2U, 2U}};
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    test_support::check(
        rejected,
        "terminal queue must reserve space for active and safety commands");
}

}  // namespace

int main()
{
    test_recovery_gate_and_normal_completion();
    test_start_without_rc_authorization_is_rejected_before_queueing();
    test_stop_preempts_active_mission();
    test_emergency_stop_overtakes_queued_safety_work();
    test_low_battery_supervisor_preempts_and_queues_rtl();
    test_link_loss_supervisor_preempts_and_queues_rtl();
    test_link_loss_return_is_not_duplicated();
    test_geofence_return_outranks_link_loss();
    test_shutdown_preempts_and_terminalizes_active_work();
    test_invalid_bounds_fail_closed();
    return test_support::report("preemptive_mission_executor_tests");
}
