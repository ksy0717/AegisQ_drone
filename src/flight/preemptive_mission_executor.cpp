#include "dc/flight/preemptive_mission_executor.hpp"

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace dc::flight {
namespace {

PreemptionRequest request_for(const RemoteCommandType type) noexcept
{
    switch (type) {
        case RemoteCommandType::stop_mission:
            return PreemptionRequest::stop;
        case RemoteCommandType::return_to_launch:
            return PreemptionRequest::return_to_launch;
        case RemoteCommandType::emergency_stop:
            return PreemptionRequest::emergency_stop;
        case RemoteCommandType::execute_mission:
            return PreemptionRequest::none;
    }
    return PreemptionRequest::none;
}

std::uint8_t priority(const PreemptionRequest request) noexcept
{
    switch (request) {
        case PreemptionRequest::none: return 0U;
        case PreemptionRequest::stop: return 1U;
        case PreemptionRequest::return_to_launch: return 2U;
        case PreemptionRequest::link_loss_return_to_launch: return 3U;
        case PreemptionRequest::geofence_return_to_launch: return 4U;
        case PreemptionRequest::low_battery_return_to_launch: return 5U;
        case PreemptionRequest::emergency_stop: return 6U;
        case PreemptionRequest::shutdown: return 7U;
    }
    return 0U;
}

void raise_request(
    std::atomic<PreemptionRequest>& current,
    const PreemptionRequest requested) noexcept
{
    PreemptionRequest observed = current.load();
    while (priority(requested) > priority(observed)
           && !current.compare_exchange_weak(observed, requested)) {
    }
}

ExecutionEvent terminal_event(
    const RemoteCommand& command, FlightOperationResult result)
{
    ExecutionEvent event;
    event.command_id = command.command_id;
    event.vehicle_id = command.vehicle_id;
    event.mission_id = command.mission_id.value_or("");
    event.outcome = result.outcome;
    event.detail = std::move(result.detail);
    return event;
}

}  // namespace

class PreemptiveMissionExecutor::Impl {
public:
    Impl(MissionBackend& backend, PreemptiveExecutorConfig config);
    ~Impl();

    CommandResult dispatch(const RemoteCommand& command);
    bool wait_event(
        ExecutionEvent& event, std::chrono::milliseconds timeout);
    CommandResult recover_after_restart();
    CommandResult request_low_battery_return_to_launch();
    CommandResult request_link_loss_return_to_launch();
    CommandResult request_geofence_return_to_launch();
    bool wait_stopped(std::chrono::milliseconds timeout) noexcept;
    void shutdown() noexcept;

private:
    CommandResult request_safe_return(
        const char* reason,
        PreemptionRequest request,
        const char* backpressure_detail,
        const char* queued_detail);
    [[nodiscard]] FlightOperationResult execute_backend(
        const RemoteCommand& command) noexcept;
    void push_event_locked(ExecutionEvent event);
    void fail_queued_locked(const std::string& detail);
    void worker_loop() noexcept;

    MissionBackend& backend_;
    PreemptiveExecutorConfig config_;
    std::atomic<PreemptionRequest> preemption_{PreemptionRequest::none};
    PreemptionToken token_;

    std::mutex mutex_;
    std::condition_variable work_ready_;
    std::condition_variable event_ready_;
    std::condition_variable event_space_;
    std::condition_variable recovery_ready_;
    std::condition_variable stopped_ready_;
    std::deque<RemoteCommand> mission_queue_;
    std::deque<RemoteCommand> safety_queue_;
    std::deque<ExecutionEvent> terminal_events_;
    std::optional<RemoteCommand> active_;
    CommandResult recovery_result_{false, "vehicle_recovery_required"};
    bool recovery_requested_{};
    bool recovery_finished_{};
    bool recovered_{};
    bool stopping_{};
    bool stopped_{};
    std::thread worker_;
};

PreemptiveMissionExecutor::Impl::Impl(
    MissionBackend& backend, const PreemptiveExecutorConfig config)
    : backend_(backend),
      config_(config),
      token_(preemption_)
{
    if (config_.mission_queue_capacity == 0U
        || config_.safety_queue_capacity == 0U
        || config_.terminal_event_capacity
            < config_.mission_queue_capacity
                + config_.safety_queue_capacity + 1U) {
        throw std::invalid_argument("invalid preemptive executor bounds");
    }
    worker_ = std::thread{[this] { worker_loop(); }};
}

PreemptiveMissionExecutor::Impl::~Impl()
{
    shutdown();
    if (worker_.joinable()) {
        worker_.join();
    }
}

CommandResult PreemptiveMissionExecutor::Impl::dispatch(
    const RemoteCommand& command)
{
    std::optional<std::uint64_t> mission_authorization;
    if (command.type == RemoteCommandType::execute_mission) {
        mission_authorization = backend_.mission_authorization_epoch();
        if (!mission_authorization.has_value()) {
            return {false, "mission_start_not_authorized_by_rc"};
        }
    }

    std::lock_guard<std::mutex> lock{mutex_};
    if (stopping_) {
        return {false, "executor_stopping"};
    }
    if (!recovered_) {
        return {false, "vehicle_recovery_required"};
    }

    const std::size_t outstanding = terminal_events_.size()
        + mission_queue_.size() + safety_queue_.size()
        + (active_.has_value() ? 1U : 0U);
    if (is_safety_command(command.type)) {
        if (safety_queue_.size() >= config_.safety_queue_capacity
            || outstanding >= config_.terminal_event_capacity) {
            return {false, "safety_queue_backpressure"};
        }
        if (command.type == RemoteCommandType::emergency_stop) {
            safety_queue_.push_front(command);
        } else {
            safety_queue_.push_back(command);
        }
        if (active_.has_value()) {
            const PreemptionRequest requested = request_for(command.type);
            const PreemptionRequest active_request = request_for(active_->type);
            if (active_->type == RemoteCommandType::execute_mission
                || priority(requested) > priority(active_request)) {
                raise_request(preemption_, requested);
            }
        }
    } else {
        const std::size_t normal_limit = config_.terminal_event_capacity
            - config_.safety_queue_capacity;
        if (outstanding >= normal_limit
            || mission_queue_.size() >= config_.mission_queue_capacity
            || active_.has_value()
            || !safety_queue_.empty()) {
            return {false, "mission_executor_busy"};
        }
        RemoteCommand stamped_command = command;
        stamped_command.dispatch_ownership_epoch = mission_authorization;
        mission_queue_.push_back(std::move(stamped_command));
    }
    work_ready_.notify_one();
    return {true, "queued_for_px4_owner"};
}

bool PreemptiveMissionExecutor::Impl::wait_event(
    ExecutionEvent& event, const std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock{mutex_};
    event_ready_.wait_for(lock, timeout, [this] {
        return !terminal_events_.empty() || stopped_;
    });
    if (terminal_events_.empty()) {
        return false;
    }
    event = std::move(terminal_events_.front());
    terminal_events_.pop_front();
    event_space_.notify_one();
    return true;
}

CommandResult PreemptiveMissionExecutor::Impl::recover_after_restart()
{
    std::unique_lock<std::mutex> lock{mutex_};
    if (stopping_) {
        return {false, "executor_stopping"};
    }
    if (recovered_) {
        return recovery_result_;
    }
    if (active_.has_value()
        || !mission_queue_.empty()
        || !safety_queue_.empty()) {
        return {false, "cannot_recover_with_pending_commands"};
    }
    recovery_requested_ = true;
    recovery_finished_ = false;
    work_ready_.notify_one();
    recovery_ready_.wait(lock, [this] {
        return recovery_finished_ || stopped_;
    });
    return recovery_finished_
        ? recovery_result_
        : CommandResult{false, "executor_stopped_during_recovery"};
}

// Shared by every locally originated safe return. The caller owns the policy
// decision; this only queues it ahead of ordinary missions and preempts an
// active route. reason travels to PX4 and into the terminal event, so the
// operator can tell a battery return from a link-loss one after the fact.
CommandResult PreemptiveMissionExecutor::Impl::request_safe_return(
    const char* reason,
    const PreemptionRequest request,
    const char* backpressure_detail,
    const char* queued_detail)
{
    std::lock_guard<std::mutex> lock{mutex_};
    if (stopping_) {
        return {false, "executor_stopping"};
    }
    if (!recovered_) {
        return {false, "vehicle_recovery_required"};
    }
    const auto already_rtl = [](const RemoteCommand& command) {
        return command.type == RemoteCommandType::return_to_launch
            || command.type == RemoteCommandType::emergency_stop;
    };
    if ((active_.has_value() && already_rtl(*active_))
        || std::any_of(
            safety_queue_.begin(), safety_queue_.end(), already_rtl)) {
        return {true, "safe_return_already_pending"};
    }

    const std::size_t outstanding = terminal_events_.size()
        + mission_queue_.size() + safety_queue_.size()
        + (active_.has_value() ? 1U : 0U);
    if (safety_queue_.size() >= config_.safety_queue_capacity
        || outstanding >= config_.terminal_event_capacity) {
        return {false, backpressure_detail};
    }

    RemoteCommand command;
    command.type = RemoteCommandType::return_to_launch;
    command.reason = reason;
    const auto first_non_emergency = std::find_if(
        safety_queue_.begin(),
        safety_queue_.end(),
        [](const RemoteCommand& queued) {
            return queued.type != RemoteCommandType::emergency_stop;
        });
    safety_queue_.insert(first_non_emergency, std::move(command));
    if (active_.has_value()) {
        raise_request(preemption_, request);
    }
    work_ready_.notify_one();
    return {true, queued_detail};
}

CommandResult
PreemptiveMissionExecutor::Impl::request_low_battery_return_to_launch()
{
    return request_safe_return(
        "LOW_BATTERY",
        PreemptionRequest::low_battery_return_to_launch,
        "low_battery_safety_queue_backpressure",
        "low_battery_rtl_queued");
}

CommandResult
PreemptiveMissionExecutor::Impl::request_link_loss_return_to_launch()
{
    return request_safe_return(
        "LINK_LOSS",
        PreemptionRequest::link_loss_return_to_launch,
        "link_loss_safety_queue_backpressure",
        "link_loss_rtl_queued");
}

CommandResult
PreemptiveMissionExecutor::Impl::request_geofence_return_to_launch()
{
    return request_safe_return(
        "GEOFENCE",
        PreemptionRequest::geofence_return_to_launch,
        "geofence_safety_queue_backpressure",
        "geofence_rtl_queued");
}

bool PreemptiveMissionExecutor::Impl::wait_stopped(
    const std::chrono::milliseconds timeout) noexcept
{
    try {
        std::unique_lock<std::mutex> lock{mutex_};
        return stopped_ready_.wait_for(lock, timeout, [this] {
            return stopped_;
        });
    } catch (...) {
        return false;
    }
}

void PreemptiveMissionExecutor::Impl::shutdown() noexcept
{
    try {
        {
            std::lock_guard<std::mutex> lock{mutex_};
            if (stopping_) {
                return;
            }
            stopping_ = true;
            raise_request(preemption_, PreemptionRequest::shutdown);
        }
        work_ready_.notify_all();
        recovery_ready_.notify_all();
        event_space_.notify_all();
    } catch (...) {
    }
}

FlightOperationResult PreemptiveMissionExecutor::Impl::execute_backend(
    const RemoteCommand& command) noexcept
{
    try {
        return backend_.execute(command, token_);
    } catch (const std::exception& error) {
        return {ExecutionOutcome::failed, error.what()};
    } catch (...) {
        return {ExecutionOutcome::failed, "unknown_backend_error"};
    }
}

void PreemptiveMissionExecutor::Impl::push_event_locked(ExecutionEvent event)
{
    terminal_events_.push_back(std::move(event));
    event_ready_.notify_one();
}

void PreemptiveMissionExecutor::Impl::fail_queued_locked(
    const std::string& detail)
{
    while (!safety_queue_.empty()) {
        RemoteCommand command = std::move(safety_queue_.front());
        safety_queue_.pop_front();
        push_event_locked(terminal_event(
            command, {ExecutionOutcome::failed, detail}));
    }
    while (!mission_queue_.empty()) {
        RemoteCommand command = std::move(mission_queue_.front());
        mission_queue_.pop_front();
        push_event_locked(terminal_event(
            command, {ExecutionOutcome::failed, detail}));
    }
}

void PreemptiveMissionExecutor::Impl::worker_loop() noexcept
{
    while (true) {
        std::optional<RemoteCommand> command;
        {
            std::unique_lock<std::mutex> lock{mutex_};
            work_ready_.wait(lock, [this] {
                return stopping_ || recovery_requested_
                    || !safety_queue_.empty() || !mission_queue_.empty();
            });

            if (recovery_requested_ && !stopping_) {
                recovery_requested_ = false;
                lock.unlock();
                CommandResult result;
                try {
                    result = backend_.recover_after_restart(token_);
                } catch (const std::exception& error) {
                    result = {false, error.what()};
                } catch (...) {
                    result = {false, "unknown_recovery_error"};
                }
                lock.lock();
                recovery_result_ = std::move(result);
                recovered_ = recovery_result_.accepted;
                recovery_finished_ = true;
                recovery_ready_.notify_all();
                continue;
            }

            if (stopping_) {
                fail_queued_locked("executor_shutdown_before_dispatch");
                lock.unlock();
                try {
                    static_cast<void>(backend_.shutdown_safely(token_));
                } catch (...) {
                }
                lock.lock();
                stopped_ = true;
                recovery_finished_ = true;
                recovery_ready_.notify_all();
                stopped_ready_.notify_all();
                event_ready_.notify_all();
                return;
            }

            if (!safety_queue_.empty()) {
                command = std::move(safety_queue_.front());
                safety_queue_.pop_front();
            } else if (!mission_queue_.empty()) {
                command = std::move(mission_queue_.front());
                mission_queue_.pop_front();
            }
            if (!command.has_value()) {
                continue;
            }
            active_ = *command;
            preemption_.store(PreemptionRequest::none);
        }

        FlightOperationResult result = execute_backend(*command);
        {
            std::unique_lock<std::mutex> lock{mutex_};
            event_space_.wait(lock, [this] {
                return terminal_events_.size()
                    < config_.terminal_event_capacity;
            });
            push_event_locked(terminal_event(*command, std::move(result)));
            active_.reset();
        }
    }
}

PreemptiveMissionExecutor::PreemptiveMissionExecutor(
    MissionBackend& backend, const PreemptiveExecutorConfig config)
    : impl_(std::make_unique<Impl>(backend, config))
{
}

PreemptiveMissionExecutor::~PreemptiveMissionExecutor() = default;

CommandResult PreemptiveMissionExecutor::dispatch(
    const RemoteCommand& command)
{
    return impl_->dispatch(command);
}

bool PreemptiveMissionExecutor::wait_event(
    ExecutionEvent& event, const std::chrono::milliseconds timeout)
{
    return impl_->wait_event(event, timeout);
}

CommandResult PreemptiveMissionExecutor::recover_after_restart()
{
    return impl_->recover_after_restart();
}

CommandResult
PreemptiveMissionExecutor::request_low_battery_return_to_launch()
{
    return impl_->request_low_battery_return_to_launch();
}

CommandResult
PreemptiveMissionExecutor::request_link_loss_return_to_launch()
{
    return impl_->request_link_loss_return_to_launch();
}

CommandResult
PreemptiveMissionExecutor::request_geofence_return_to_launch()
{
    return impl_->request_geofence_return_to_launch();
}

bool PreemptiveMissionExecutor::wait_stopped(
    const std::chrono::milliseconds timeout) noexcept
{
    return impl_->wait_stopped(timeout);
}

void PreemptiveMissionExecutor::shutdown() noexcept
{
    impl_->shutdown();
}

const char* preemption_request_name(const PreemptionRequest request) noexcept
{
    switch (request) {
        case PreemptionRequest::none: return "none";
        case PreemptionRequest::stop: return "stop";
        case PreemptionRequest::return_to_launch: return "return_to_launch";
        case PreemptionRequest::link_loss_return_to_launch:
            return "link_loss_return_to_launch";
        case PreemptionRequest::geofence_return_to_launch:
            return "geofence_return_to_launch";
        case PreemptionRequest::low_battery_return_to_launch:
            return "low_battery_return_to_launch";
        case PreemptionRequest::emergency_stop: return "emergency_stop";
        case PreemptionRequest::shutdown: return "shutdown";
    }
    return "unknown";
}

}  // namespace dc::flight
