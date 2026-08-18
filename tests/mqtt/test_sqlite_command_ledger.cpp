#include "dc/mqtt/sqlite_command_ledger.hpp"
#include "fixtures.hpp"
#include "test_support.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "dc/mqtt/mission_codec.hpp"

namespace {

using dc::flight::CommandClaimDisposition;
using dc::flight::MissionStatus;
using dc::flight::RemoteCommand;
using dc::flight::RouteLimits;
using dc::flight::StatusRecordMode;
using dc::mqtt::MissionCodecLimits;
using dc::mqtt::SqliteCommandLedger;
using dc::mqtt::SqliteCommandLedgerConfig;

const auto kNow =
    std::chrono::system_clock::time_point{std::chrono::seconds{1'786'000'000}};
constexpr MissionCodecLimits kCodecLimits{
    RouteLimits{20U, 2.0, 120.0, 60U, 180U, 0.5, 8.0},
    128U,
    4U,
};

class TemporaryLedgerFile final {
public:
    TemporaryLedgerFile()
    {
        static std::atomic<unsigned long long> next{0U};
        const auto stamp = std::chrono::steady_clock::now()
            .time_since_epoch()
            .count();
        path_ = std::filesystem::temp_directory_path()
            / ("dc-command-ledger-" + std::to_string(stamp) + "-"
               + std::to_string(next.fetch_add(1U)) + ".sqlite3");
    }

    ~TemporaryLedgerFile()
    {
        std::error_code error;
        static_cast<void>(std::filesystem::remove(path_, error));
        static_cast<void>(std::filesystem::remove(path_.string() + "-wal", error));
        static_cast<void>(std::filesystem::remove(path_.string() + "-shm", error));
    }

    [[nodiscard]] std::string string() const
    {
        return path_.string();
    }

private:
    std::filesystem::path path_;
};

SqliteCommandLedgerConfig config(
    const TemporaryLedgerFile& file,
    const std::size_t max_outbox = 16U,
    const std::size_t reserve = 2U)
{
    return {
        file.string(),
        "DRONE-01",
        max_outbox,
        reserve,
        512U,
        std::chrono::milliseconds{250},
    };
}

RemoteCommand mission()
{
    const auto decoded = dc::mqtt::decode_command(
        mqtt_fixtures::execute_mission_payload(), "DRONE-01", kCodecLimits);
    if (!decoded.ok()) {
        throw std::runtime_error("fixture decode failed: " + decoded.error);
    }
    return *decoded.command;
}

MissionStatus status_for(
    const RemoteCommand& command,
    std::string state,
    std::string detail = {})
{
    MissionStatus status;
    status.command_id = command.command_id;
    status.vehicle_id = command.vehicle_id;
    status.mission_id = command.mission_id.value_or("");
    status.state = std::move(state);
    status.detail = std::move(detail);
    return status;
}

void test_claim_and_ack_survive_reopen()
{
    TemporaryLedgerFile file;
    const RemoteCommand command = mission();

    {
        SqliteCommandLedger ledger{config(file), [] { return kNow; }};
        test_support::check(
            ledger.claim(command).disposition == CommandClaimDisposition::claimed,
            "first UUID is durably claimed");
        const auto stored = ledger.record_status(
            status_for(command, "ACCEPTED", "px4_queue_claimed"),
            StatusRecordMode::update_claim);
        test_support::check(stored.durable, "ACCEPTED and outbox are atomic");
    }

    {
        SqliteCommandLedger ledger{config(file), [] { return kNow; }};
        const auto duplicate = ledger.claim(command);
        test_support::check(
            duplicate.disposition == CommandClaimDisposition::duplicate,
            "UUID remains claimed after reopen");
        test_support::check(
            duplicate.latest_status.has_value()
                && duplicate.latest_status->state == "ACCEPTED",
            "latest ACK state survives reopen");

        std::string error;
        const auto pending = ledger.pending_outbox(8U, error);
        test_support::check(error.empty() && pending.size() == 1U, "ACK remains pending");
        if (!pending.empty()) {
            const auto payload = nlohmann::json::parse(pending.front().payload);
            test_support::check(
                pending.front().topic == "aegisq/cmd-ack/DRONE-01"
                    && pending.front().qos == 1
                    && !pending.front().retained,
                "outbox preserves exact QoS 1 non-retained destination");
            test_support::check(
                payload.at("status") == "ACCEPTED",
                "outbox preserves exact ACK JSON");
            test_support::check(
                ledger.mark_delivered(pending.front().id, error),
                "PUBACK deletes the durable outbox row");
        }
        test_support::check(
            ledger.pending_outbox_count(error) == 0U && error.empty(),
            "delivered ACK is no longer replayed");
    }
}

void test_conflicting_uuid_cannot_replace_original_state()
{
    TemporaryLedgerFile file;
    SqliteCommandLedger ledger{config(file), [] { return kNow; }};
    const RemoteCommand original = mission();
    test_support::check(
        ledger.claim(original).disposition == CommandClaimDisposition::claimed,
        "original is claimed");
    test_support::check(
        ledger.record_status(
            status_for(original, "ACCEPTED"), StatusRecordMode::update_claim)
            .durable,
        "original status is stored");

    RemoteCommand changed = original;
    changed.canonical_payload += " ";
    test_support::check(
        ledger.claim(changed).disposition == CommandClaimDisposition::conflict,
        "same UUID with different canonical JSON conflicts");
    test_support::check(
        ledger.record_status(
            status_for(changed, "REJECTED", "command_id_conflict"),
            StatusRecordMode::outbox_only)
            .durable,
        "conflict ACK is durable without changing the claimed command");

    const auto replay = ledger.claim(original);
    test_support::check(
        replay.latest_status.has_value()
            && replay.latest_status->state == "ACCEPTED",
        "conflict ACK cannot overwrite original execution state");
}

void test_unresolved_claim_requires_explicit_vehicle_recovery()
{
    TemporaryLedgerFile file;
    const RemoteCommand command = mission();
    {
        SqliteCommandLedger ledger{config(file), [] { return kNow; }};
        static_cast<void>(ledger.claim(command));
    }

    SqliteCommandLedger ledger{config(file), [] { return kNow; }};
    std::string error;
    const auto unresolved = ledger.unresolved_commands(error);
    test_support::check(
        error.empty() && unresolved.size() == 1U,
        "crash between claim and ACK is detected on restart");
    test_support::check(
        ledger.fail_unresolved_after_vehicle_recovery(
            "restart_vehicle_recovered", error),
        "confirmed recovery terminalizes unresolved commands");

    const auto duplicate = ledger.claim(command);
    test_support::check(
        duplicate.latest_status.has_value()
            && duplicate.latest_status->state == "FAILED"
            && duplicate.latest_status->detail == "restart_vehicle_recovered",
        "restarted command cannot execute again");
    const auto pending = ledger.pending_outbox(8U, error);
    test_support::check(
        pending.size() == 1U
            && nlohmann::json::parse(pending.front().payload).at("status") == "FAILED",
        "restart failure is queued for broker replay");
}

void test_outbox_backpressure_blocks_new_claims()
{
    TemporaryLedgerFile file;
    SqliteCommandLedger ledger{config(file, 4U, 2U), [] { return kNow; }};
    const RemoteCommand command = mission();

    MissionStatus first = status_for(command, "REJECTED", "schema_error");
    first.command_id = "00000000-0000-4000-8000-000000000001";
    MissionStatus second = first;
    second.command_id = "00000000-0000-4000-8000-000000000002";
    test_support::check(
        ledger.record_status(first, StatusRecordMode::outbox_only).durable
            && ledger.record_status(second, StatusRecordMode::outbox_only).durable,
        "negative ACKs occupy bounded outbox slots");
    const auto blocked = ledger.claim(command);
    test_support::check(
        blocked.disposition == CommandClaimDisposition::unavailable
            && blocked.detail == "status_outbox_backpressure",
        "new flight is blocked while terminal-status reserve is all that remains");
}

void test_status_update_requires_prior_claim()
{
    TemporaryLedgerFile file;
    SqliteCommandLedger ledger{config(file), [] { return kNow; }};
    const RemoteCommand command = mission();
    const auto result = ledger.record_status(
        status_for(command, "ACCEPTED"), StatusRecordMode::update_claim);
    test_support::check(
        !result.durable && result.detail == "status_has_no_durable_claim",
        "ACCEPTED cannot exist without an atomic prior claim");
    std::string error;
    test_support::check(
        ledger.pending_outbox_count(error) == 0U,
        "failed state update cannot leave an orphan ACK");
}

void test_terminal_state_is_sticky()
{
    TemporaryLedgerFile file;
    SqliteCommandLedger ledger{config(file), [] { return kNow; }};
    const RemoteCommand command = mission();
    static_cast<void>(ledger.claim(command));
    static_cast<void>(ledger.record_status(
        status_for(command, "ACCEPTED"), StatusRecordMode::update_claim));
    test_support::check(
        ledger.record_status(
            status_for(command, "COMPLETED", "landed_and_disarmed"),
            StatusRecordMode::update_claim)
            .durable,
        "ACCEPTED may transition to COMPLETED");
    const auto invalid = ledger.record_status(
        status_for(command, "FAILED", "late_failure"),
        StatusRecordMode::update_claim);
    test_support::check(
        !invalid.durable
            && invalid.detail == "invalid_command_state_transition",
        "terminal COMPLETED cannot be overwritten by a late FAILED event");
    const auto duplicate = ledger.claim(command);
    test_support::check(
        duplicate.latest_status.has_value()
            && duplicate.latest_status->state == "COMPLETED",
        "duplicate replay keeps the first terminal result");
}

void test_configuration_is_fail_closed()
{
    bool relative_rejected = false;
    try {
        SqliteCommandLedger invalid{
            {"relative.sqlite3", "DRONE-01", 16U, 2U, 512U,
             std::chrono::milliseconds{250}}};
    } catch (const std::invalid_argument&) {
        relative_rejected = true;
    }
    test_support::check(relative_rejected, "ledger path must be absolute");
}

// The broker adapter answers a decode failure with a negative ACK for a
// command that was never claimed, and must truncate the codec's error to this
// bound first or record_status() drops the whole record.
void test_negative_ack_for_unclaimed_command()
{
    TemporaryLedgerFile file;
    SqliteCommandLedger ledger{config(file), [] { return kNow; }};

    test_support::check(
        ledger.max_status_detail_bytes() == 512U,
        "the status detail bound is readable by the broker adapter");

    MissionStatus rejection;
    rejection.command_id = "00000000-0000-4000-8000-0000000000ab";
    rejection.vehicle_id = "DRONE-01";
    rejection.state = "REJECTED";
    rejection.detail = "altitude_out_of_range:waypoint=2";
    test_support::check(
        ledger.record_status(rejection, StatusRecordMode::outbox_only).durable,
        "a decode failure is answerable without a durable claim");

    std::string error;
    const auto pending = ledger.pending_outbox(8U, error);
    test_support::check(
        error.empty() && pending.size() == 1U
            && pending.front().topic == ledger.ack_topic()
            && pending.front().qos == 1,
        "the rejection ACK is queued on the command ack topic at QoS 1");

    rejection.detail =
        std::string(ledger.max_status_detail_bytes() + 1U, 'x');
    test_support::check(
        !ledger.record_status(rejection, StatusRecordMode::outbox_only).durable,
        "an over-long detail is refused, so the caller must truncate");
}

}  // namespace

int main()
{
    test_claim_and_ack_survive_reopen();
    test_negative_ack_for_unclaimed_command();
    test_conflicting_uuid_cannot_replace_original_state();
    test_unresolved_claim_requires_explicit_vehicle_recovery();
    test_outbox_backpressure_blocks_new_claims();
    test_status_update_requires_prior_claim();
    test_terminal_state_is_sticky();
    test_configuration_is_fail_closed();
    return test_support::report("mqtt_command_ledger_tests");
}
