#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "dc/flight/ports.hpp"

namespace dc::mqtt {

struct SqliteCommandLedgerConfig {
    std::string database_path;
    std::string vehicle_id;
    std::size_t max_pending_outbox{};
    std::size_t outbox_claim_reserve{};
    std::size_t max_status_detail_bytes{};
    std::chrono::milliseconds busy_timeout{};
};

struct OutboxRecord {
    std::int64_t id{};
    std::string topic;
    std::string payload;
    int qos{};
    bool retained{};
};

struct UnresolvedCommand {
    std::string command_id;
    std::string vehicle_id;
    std::string mission_id;
    std::string state;
};

// One SQLite transaction owns both command state and the exact MQTT ACK
// payload. WAL + synchronous=FULL make a successful claim survive power loss;
// the broker PUBACK deletes the corresponding outbox row later.
class SqliteCommandLedger final : public flight::CommandLedger {
public:
    using Clock = std::function<std::chrono::system_clock::time_point()>;

    SqliteCommandLedger(
        SqliteCommandLedgerConfig config,
        Clock clock = [] { return std::chrono::system_clock::now(); });
    ~SqliteCommandLedger() override;

    SqliteCommandLedger(const SqliteCommandLedger&) = delete;
    SqliteCommandLedger& operator=(const SqliteCommandLedger&) = delete;

    flight::CommandClaimResult claim(
        const flight::RemoteCommand& command) noexcept override;

    [[nodiscard]] flight::StatusPublishResult record_status(
        const flight::MissionStatus& status,
        flight::StatusRecordMode mode) noexcept;

    [[nodiscard]] std::vector<OutboxRecord> pending_outbox(
        std::size_t limit, std::string& error) noexcept;
    [[nodiscard]] bool mark_delivered(
        std::int64_t outbox_id, std::string& error) noexcept;

    [[nodiscard]] std::vector<UnresolvedCommand> unresolved_commands(
        std::string& error) noexcept;

    // Call only after the PX4 adapter has positively confirmed a safe recovery
    // action (RTL/land/disarm as applicable). It terminalizes every command
    // that was CLAIMED or ACCEPTED when the previous process died.
    [[nodiscard]] bool fail_unresolved_after_vehicle_recovery(
        const std::string& detail, std::string& error) noexcept;

    [[nodiscard]] std::size_t pending_outbox_count(std::string& error) noexcept;
    [[nodiscard]] const std::string& ack_topic() const noexcept;
    // Callers that build a status detail from an untrusted source need this to
    // truncate before record_status() rejects the whole record.
    [[nodiscard]] std::size_t max_status_detail_bytes() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dc::mqtt
