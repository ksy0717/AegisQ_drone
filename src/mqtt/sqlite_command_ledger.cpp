#include "dc/mqtt/sqlite_command_ledger.hpp"

#include <sqlite3.h>

#include <climits>
#include <filesystem>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "dc/mqtt/mission_codec.hpp"
#include "dc/mqtt/mqtt_command_source.hpp"

namespace dc::mqtt {
namespace {

constexpr int kSchemaVersion = 1;

void require_sqlite(
    const int result, sqlite3* const database, const std::string_view operation)
{
    if (result == SQLITE_OK || result == SQLITE_ROW || result == SQLITE_DONE) {
        return;
    }
    const char* const message = database == nullptr
        ? sqlite3_errstr(result)
        : sqlite3_errmsg(database);
    throw std::runtime_error(
        std::string{operation} + ": " + (message == nullptr ? "sqlite_error" : message));
}

void execute_sql(sqlite3* const database, const char* const sql)
{
    char* raw_error = nullptr;
    const int result = sqlite3_exec(database, sql, nullptr, nullptr, &raw_error);
    if (result == SQLITE_OK) {
        return;
    }
    const std::string detail = raw_error == nullptr
        ? std::string{sqlite3_errmsg(database)}
        : std::string{raw_error};
    sqlite3_free(raw_error);
    throw std::runtime_error(detail);
}

class Statement final {
public:
    Statement(sqlite3* const database, const char* const sql)
        : database_(database)
    {
        require_sqlite(
            sqlite3_prepare_v2(database_, sql, -1, &statement_, nullptr),
            database_,
            "prepare");
    }

    ~Statement()
    {
        if (statement_ != nullptr) {
            static_cast<void>(sqlite3_finalize(statement_));
        }
    }

    Statement(const Statement&) = delete;
    Statement& operator=(const Statement&) = delete;

    void bind(const int index, const std::string& value)
    {
        if (value.size() > static_cast<std::size_t>(INT_MAX)) {
            throw std::runtime_error("sqlite_text_too_large");
        }
        require_sqlite(
            sqlite3_bind_text(
                statement_,
                index,
                value.data(),
                static_cast<int>(value.size()),
                SQLITE_TRANSIENT),
            database_,
            "bind_text");
    }

    void bind(const int index, const int value)
    {
        require_sqlite(
            sqlite3_bind_int(statement_, index, value), database_, "bind_int");
    }

    void bind(const int index, const std::int64_t value)
    {
        require_sqlite(
            sqlite3_bind_int64(
                statement_, index, static_cast<sqlite3_int64>(value)),
            database_,
            "bind_int64");
    }

    void bind_optional(const int index, const std::optional<std::string>& value)
    {
        if (value.has_value()) {
            bind(index, *value);
            return;
        }
        require_sqlite(
            sqlite3_bind_null(statement_, index), database_, "bind_null");
    }

    [[nodiscard]] int step()
    {
        const int result = sqlite3_step(statement_);
        require_sqlite(result, database_, "step");
        return result;
    }

    [[nodiscard]] sqlite3_stmt* get() noexcept
    {
        return statement_;
    }

private:
    sqlite3* database_{};
    sqlite3_stmt* statement_{};
};

class Transaction final {
public:
    explicit Transaction(sqlite3* const database) : database_(database)
    {
        execute_sql(database_, "BEGIN IMMEDIATE");
    }

    ~Transaction()
    {
        if (!committed_) {
            try {
                execute_sql(database_, "ROLLBACK");
            } catch (...) {
                // The original failure is more useful than rollback failure.
            }
        }
    }

    Transaction(const Transaction&) = delete;
    Transaction& operator=(const Transaction&) = delete;

    void commit()
    {
        execute_sql(database_, "COMMIT");
        committed_ = true;
    }

private:
    sqlite3* database_{};
    bool committed_{};
};

std::string column_text(sqlite3_stmt* const statement, const int column)
{
    const unsigned char* const value = sqlite3_column_text(statement, column);
    if (value == nullptr) {
        return {};
    }
    return std::string{reinterpret_cast<const char*>(value)};
}

std::optional<std::string> column_optional_text(
    sqlite3_stmt* const statement, const int column)
{
    if (sqlite3_column_type(statement, column) == SQLITE_NULL) {
        return std::nullopt;
    }
    return column_text(statement, column);
}

std::int64_t scalar_count(sqlite3* const database, const char* const sql)
{
    Statement statement{database, sql};
    if (statement.step() != SQLITE_ROW) {
        throw std::runtime_error("sqlite_count_missing");
    }
    return static_cast<std::int64_t>(sqlite3_column_int64(statement.get(), 0));
}

bool is_supported_status(const std::string_view state) noexcept
{
    return state == "ACCEPTED"
        || state == "REJECTED"
        || state == "COMPLETED"
        || state == "FAILED";
}

bool is_unresolved_state(const std::string_view state) noexcept
{
    return state.empty() || state == "ACCEPTED";
}

bool terminal_state(const std::string_view state) noexcept
{
    return state == "REJECTED"
        || state == "COMPLETED"
        || state == "FAILED";
}

}  // namespace

class SqliteCommandLedger::Impl {
public:
    Impl(SqliteCommandLedgerConfig config, Clock clock)
        : config_(std::move(config)),
          clock_(std::move(clock)),
          ack_topic_(command_ack_topic_for(config_.vehicle_id))
    {
        validate_config();
        open_database();
    }

    ~Impl()
    {
        if (database_ != nullptr) {
            static_cast<void>(sqlite3_close_v2(database_));
        }
    }

    flight::CommandClaimResult claim(
        const flight::RemoteCommand& command) noexcept
    {
        try {
            if (command.vehicle_id != config_.vehicle_id) {
                return {
                    flight::CommandClaimDisposition::unavailable,
                    std::nullopt,
                    "unexpected_vehicle_id",
                };
            }
            if (!is_canonical_uuid(command.command_id)
                || command.canonical_payload.empty()) {
                return {
                    flight::CommandClaimDisposition::unavailable,
                    std::nullopt,
                    "command_not_canonical",
                };
            }

            std::lock_guard<std::mutex> lock{mutex_};
            Transaction transaction{database_};
            const std::int64_t pending = scalar_count(
                database_, "SELECT COUNT(*) FROM command_outbox");
            const std::size_t claim_limit =
                config_.max_pending_outbox - config_.outbox_claim_reserve;
            if (pending < 0
                || static_cast<std::uint64_t>(pending)
                    >= static_cast<std::uint64_t>(claim_limit)) {
                return {
                    flight::CommandClaimDisposition::unavailable,
                    std::nullopt,
                    "status_outbox_backpressure",
                };
            }

            Statement existing{
                database_,
                "SELECT canonical_payload, vehicle_id, mission_id, state, detail "
                "FROM command_ledger WHERE command_id = ?1"};
            existing.bind(1, command.command_id);
            if (existing.step() == SQLITE_ROW) {
                const std::string stored_payload = column_text(existing.get(), 0);
                if (stored_payload != command.canonical_payload) {
                    return {
                        flight::CommandClaimDisposition::conflict,
                        std::nullopt,
                        "command_id_conflict",
                    };
                }

                const std::optional<std::string> stored_state =
                    column_optional_text(existing.get(), 3);
                std::optional<flight::MissionStatus> latest;
                if (stored_state.has_value()) {
                    flight::MissionStatus status;
                    status.command_id = command.command_id;
                    status.vehicle_id = column_text(existing.get(), 1);
                    status.mission_id = column_text(existing.get(), 2);
                    status.state = *stored_state;
                    status.detail = column_text(existing.get(), 4);
                    latest = std::move(status);
                }
                return {
                    flight::CommandClaimDisposition::duplicate,
                    std::move(latest),
                    {},
                };
            }

            const std::string now = format_rfc3339(clock_());
            Statement insert{
                database_,
                "INSERT INTO command_ledger("
                "command_id, canonical_payload, vehicle_id, mission_id, "
                "command_type, claimed_at, updated_at, state, detail) "
                "VALUES(?1, ?2, ?3, ?4, ?5, ?6, ?6, NULL, NULL)"};
            insert.bind(1, command.command_id);
            insert.bind(2, command.canonical_payload);
            insert.bind(3, command.vehicle_id);
            insert.bind_optional(4, command.mission_id);
            insert.bind(5, static_cast<int>(command.type));
            insert.bind(6, now);
            if (insert.step() != SQLITE_DONE) {
                throw std::runtime_error("command_claim_not_written");
            }
            transaction.commit();
            return {
                flight::CommandClaimDisposition::claimed,
                std::nullopt,
                {},
            };
        } catch (const std::exception& error) {
            return {
                flight::CommandClaimDisposition::unavailable,
                std::nullopt,
                error.what(),
            };
        } catch (...) {
            return {
                flight::CommandClaimDisposition::unavailable,
                std::nullopt,
                "unknown_ledger_error",
            };
        }
    }

    flight::StatusPublishResult record_status(
        const flight::MissionStatus& status,
        const flight::StatusRecordMode mode) noexcept
    {
        try {
            if (status.vehicle_id != config_.vehicle_id
                || !is_canonical_uuid(status.command_id)
                || !is_supported_status(status.state)
                || status.detail.size() > config_.max_status_detail_bytes) {
                return {false, "invalid_status_record"};
            }
            const auto acked_at = clock_();
            const std::string timestamp = format_rfc3339(acked_at);
            const std::string payload = encode_command_ack(status, acked_at);

            std::lock_guard<std::mutex> lock{mutex_};
            Transaction transaction{database_};
            const std::int64_t pending = scalar_count(
                database_, "SELECT COUNT(*) FROM command_outbox");
            if (pending < 0
                || static_cast<std::uint64_t>(pending)
                    >= static_cast<std::uint64_t>(config_.max_pending_outbox)) {
                return {false, "status_outbox_full"};
            }

            if (mode == flight::StatusRecordMode::update_claim) {
                Statement current{
                    database_,
                    "SELECT state, detail FROM command_ledger "
                    "WHERE command_id = ?1"};
                current.bind(1, status.command_id);
                if (current.step() != SQLITE_ROW) {
                    throw std::runtime_error("status_has_no_durable_claim");
                }
                const std::optional<std::string> previous_state =
                    column_optional_text(current.get(), 0);
                const std::string previous_detail = column_text(current.get(), 1);
                const bool replay_terminal = previous_state.has_value()
                    && terminal_state(*previous_state)
                    && *previous_state == status.state
                    && previous_detail == status.detail;
                const bool transition_allowed = !previous_state.has_value()
                    || (*previous_state == "ACCEPTED"
                        && (status.state == "ACCEPTED"
                            || status.state == "COMPLETED"
                            || status.state == "FAILED"))
                    || replay_terminal;
                if (!transition_allowed) {
                    throw std::runtime_error("invalid_command_state_transition");
                }

                if (!replay_terminal) {
                Statement update{
                    database_,
                    "UPDATE command_ledger SET state = ?1, detail = ?2, "
                    "updated_at = ?3 WHERE command_id = ?4"};
                update.bind(1, status.state);
                update.bind(2, status.detail);
                update.bind(3, timestamp);
                update.bind(4, status.command_id);
                if (update.step() != SQLITE_DONE
                    || sqlite3_changes(database_) != 1) {
                    throw std::runtime_error("status_has_no_durable_claim");
                }
                }
            }

            insert_outbox(status.command_id, payload, timestamp);
            transaction.commit();
            return {true, {}};
        } catch (const std::exception& error) {
            return {false, error.what()};
        } catch (...) {
            return {false, "unknown_status_store_error"};
        }
    }

    std::vector<OutboxRecord> pending_outbox(
        const std::size_t limit, std::string& error) noexcept
    {
        try {
            error.clear();
            if (limit == 0U
                || limit > static_cast<std::size_t>(
                    std::numeric_limits<std::int64_t>::max())) {
                throw std::invalid_argument("invalid_outbox_read_limit");
            }
            std::lock_guard<std::mutex> lock{mutex_};
            Statement select{
                database_,
                "SELECT id, topic, payload, qos, retained "
                "FROM command_outbox ORDER BY id LIMIT ?1"};
            select.bind(1, static_cast<std::int64_t>(limit));

            std::vector<OutboxRecord> records;
            while (select.step() == SQLITE_ROW) {
                OutboxRecord record;
                record.id = static_cast<std::int64_t>(
                    sqlite3_column_int64(select.get(), 0));
                record.topic = column_text(select.get(), 1);
                record.payload = column_text(select.get(), 2);
                record.qos = sqlite3_column_int(select.get(), 3);
                record.retained = sqlite3_column_int(select.get(), 4) != 0;
                if (record.id <= 0
                    || record.topic != ack_topic_
                    || record.payload.empty()
                    || record.qos != 1
                    || record.retained) {
                    throw std::runtime_error("corrupt_outbox_record");
                }
                records.push_back(std::move(record));
            }
            return records;
        } catch (const std::exception& exception) {
            error = exception.what();
            return {};
        } catch (...) {
            error = "unknown_outbox_read_error";
            return {};
        }
    }

    bool mark_delivered(
        const std::int64_t outbox_id, std::string& error) noexcept
    {
        try {
            error.clear();
            if (outbox_id <= 0) {
                throw std::invalid_argument("invalid_outbox_id");
            }
            std::lock_guard<std::mutex> lock{mutex_};
            Transaction transaction{database_};
            Statement remove{
                database_, "DELETE FROM command_outbox WHERE id = ?1"};
            remove.bind(1, outbox_id);
            if (remove.step() != SQLITE_DONE) {
                throw std::runtime_error("outbox_delete_failed");
            }
            transaction.commit();
            return true;
        } catch (const std::exception& exception) {
            error = exception.what();
            return false;
        } catch (...) {
            error = "unknown_outbox_delete_error";
            return false;
        }
    }

    std::vector<UnresolvedCommand> unresolved_commands(
        std::string& error) noexcept
    {
        try {
            error.clear();
            std::lock_guard<std::mutex> lock{mutex_};
            return unresolved_commands_locked();
        } catch (const std::exception& exception) {
            error = exception.what();
            return {};
        } catch (...) {
            error = "unknown_recovery_read_error";
            return {};
        }
    }

    bool fail_unresolved_after_vehicle_recovery(
        const std::string& detail, std::string& error) noexcept
    {
        try {
            error.clear();
            if (detail.empty()) {
                throw std::invalid_argument("recovery_detail_required");
            }
            if (detail.size() > config_.max_status_detail_bytes) {
                throw std::invalid_argument("recovery_detail_too_large");
            }
            const auto acked_at = clock_();
            const std::string timestamp = format_rfc3339(acked_at);

            std::lock_guard<std::mutex> lock{mutex_};
            Transaction transaction{database_};
            const std::vector<UnresolvedCommand> unresolved =
                unresolved_commands_locked();
            const std::int64_t pending = scalar_count(
                database_, "SELECT COUNT(*) FROM command_outbox");
            if (pending < 0
                || static_cast<std::uint64_t>(pending)
                    > static_cast<std::uint64_t>(config_.max_pending_outbox)
                || unresolved.size()
                    > config_.max_pending_outbox
                        - static_cast<std::size_t>(pending)) {
                throw std::runtime_error("status_outbox_full_during_recovery");
            }

            for (const UnresolvedCommand& command : unresolved) {
                flight::MissionStatus status;
                status.command_id = command.command_id;
                status.vehicle_id = command.vehicle_id;
                status.mission_id = command.mission_id;
                status.state = "FAILED";
                status.detail = detail;

                Statement update{
                    database_,
                    "UPDATE command_ledger SET state = 'FAILED', detail = ?1, "
                    "updated_at = ?2 WHERE command_id = ?3 "
                    "AND (state IS NULL OR state = 'ACCEPTED')"};
                update.bind(1, detail);
                update.bind(2, timestamp);
                update.bind(3, command.command_id);
                if (update.step() != SQLITE_DONE
                    || sqlite3_changes(database_) != 1) {
                    throw std::runtime_error("recovery_state_changed_concurrently");
                }
                insert_outbox(
                    command.command_id,
                    encode_command_ack(status, acked_at),
                    timestamp);
            }
            transaction.commit();
            return true;
        } catch (const std::exception& exception) {
            error = exception.what();
            return false;
        } catch (...) {
            error = "unknown_recovery_write_error";
            return false;
        }
    }

    std::size_t pending_outbox_count(std::string& error) noexcept
    {
        try {
            error.clear();
            std::lock_guard<std::mutex> lock{mutex_};
            const std::int64_t count = scalar_count(
                database_, "SELECT COUNT(*) FROM command_outbox");
            if (count < 0
                || static_cast<std::uint64_t>(count)
                    > static_cast<std::uint64_t>(
                        std::numeric_limits<std::size_t>::max())) {
                throw std::runtime_error("invalid_outbox_count");
            }
            return static_cast<std::size_t>(count);
        } catch (const std::exception& exception) {
            error = exception.what();
            return 0U;
        } catch (...) {
            error = "unknown_outbox_count_error";
            return 0U;
        }
    }

    [[nodiscard]] const std::string& ack_topic() const noexcept
    {
        return ack_topic_;
    }

    [[nodiscard]] std::size_t max_status_detail_bytes() const noexcept
    {
        return config_.max_status_detail_bytes;
    }

private:
    void validate_config()
    {
        if (!clock_) {
            throw std::invalid_argument("ledger clock is required");
        }
        if (!valid_vehicle_id(config_.vehicle_id)) {
            throw std::invalid_argument("invalid ledger vehicle id");
        }
        if (config_.database_path.empty()) {
            throw std::invalid_argument("ledger database path is required");
        }
        const std::filesystem::path path{config_.database_path};
        if (!path.is_absolute()) {
            throw std::invalid_argument("ledger database path must be absolute");
        }
        const std::filesystem::path parent = path.parent_path();
        std::error_code filesystem_error;
        if (parent.empty()
            || !std::filesystem::is_directory(parent, filesystem_error)
            || filesystem_error) {
            throw std::invalid_argument("ledger database parent must exist");
        }
        if (config_.max_pending_outbox < 4U
            || config_.outbox_claim_reserve < 2U
            || config_.outbox_claim_reserve >= config_.max_pending_outbox
            || config_.max_status_detail_bytes == 0U) {
            throw std::invalid_argument("invalid outbox bounds");
        }
        if (config_.max_pending_outbox
            > static_cast<std::size_t>(
                std::numeric_limits<std::int64_t>::max())) {
            throw std::invalid_argument("outbox bound is too large");
        }
        if (config_.busy_timeout <= std::chrono::milliseconds::zero()
            || config_.busy_timeout.count() > INT_MAX) {
            throw std::invalid_argument("invalid SQLite busy timeout");
        }
    }

    void open_database()
    {
        sqlite3* database = nullptr;
        const int result = sqlite3_open_v2(
            config_.database_path.c_str(),
            &database,
            SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX,
            nullptr);
        if (result != SQLITE_OK) {
            const std::string detail = database == nullptr
                ? std::string{sqlite3_errstr(result)}
                : std::string{sqlite3_errmsg(database)};
            if (database != nullptr) {
                static_cast<void>(sqlite3_close_v2(database));
            }
            throw std::runtime_error("cannot open command ledger: " + detail);
        }
        database_ = database;
        require_sqlite(
            sqlite3_extended_result_codes(database_, 1),
            database_,
            "extended_result_codes");
        require_sqlite(
            sqlite3_busy_timeout(
                database_, static_cast<int>(config_.busy_timeout.count())),
            database_,
            "busy_timeout");

        Statement journal_mode{database_, "PRAGMA journal_mode=WAL"};
        if (journal_mode.step() != SQLITE_ROW
            || column_text(journal_mode.get(), 0) != "wal") {
            throw std::runtime_error("SQLite WAL mode unavailable");
        }
        execute_sql(database_, "PRAGMA synchronous=FULL");
        execute_sql(database_, "PRAGMA foreign_keys=ON");
        execute_sql(database_, "PRAGMA trusted_schema=OFF");

        const std::int64_t schema_version = scalar_count(
            database_, "PRAGMA user_version");
        if (schema_version != 0 && schema_version != kSchemaVersion) {
            throw std::runtime_error("unsupported command ledger schema");
        }
        execute_sql(
            database_,
            "CREATE TABLE IF NOT EXISTS command_ledger("
            "command_id TEXT PRIMARY KEY NOT NULL,"
            "canonical_payload TEXT NOT NULL,"
            "vehicle_id TEXT NOT NULL,"
            "mission_id TEXT,"
            "command_type INTEGER NOT NULL,"
            "claimed_at TEXT NOT NULL,"
            "updated_at TEXT NOT NULL,"
            "state TEXT,"
            "detail TEXT) WITHOUT ROWID;"
            "CREATE TABLE IF NOT EXISTS command_outbox("
            "id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "command_id TEXT NOT NULL,"
            "topic TEXT NOT NULL,"
            "payload TEXT NOT NULL,"
            "qos INTEGER NOT NULL CHECK(qos = 1),"
            "retained INTEGER NOT NULL CHECK(retained = 0),"
            "created_at TEXT NOT NULL);"
            "CREATE INDEX IF NOT EXISTS command_outbox_created "
            "ON command_outbox(id);"
            "PRAGMA user_version=1;");

        Statement integrity{database_, "PRAGMA quick_check(1)"};
        if (integrity.step() != SQLITE_ROW
            || column_text(integrity.get(), 0) != "ok") {
            throw std::runtime_error("command ledger integrity check failed");
        }
    }

    void insert_outbox(
        const std::string& command_id,
        const std::string& payload,
        const std::string& created_at)
    {
        Statement insert{
            database_,
            "INSERT INTO command_outbox("
            "command_id, topic, payload, qos, retained, created_at) "
            "VALUES(?1, ?2, ?3, 1, 0, ?4)"};
        insert.bind(1, command_id);
        insert.bind(2, ack_topic_);
        insert.bind(3, payload);
        insert.bind(4, created_at);
        if (insert.step() != SQLITE_DONE) {
            throw std::runtime_error("outbox_insert_failed");
        }
    }

    std::vector<UnresolvedCommand> unresolved_commands_locked()
    {
        Statement select{
            database_,
            "SELECT command_id, vehicle_id, mission_id, COALESCE(state, '') "
            "FROM command_ledger WHERE state IS NULL OR state = 'ACCEPTED' "
            "ORDER BY claimed_at"};
        std::vector<UnresolvedCommand> records;
        while (select.step() == SQLITE_ROW) {
            UnresolvedCommand record;
            record.command_id = column_text(select.get(), 0);
            record.vehicle_id = column_text(select.get(), 1);
            record.mission_id = column_text(select.get(), 2);
            record.state = column_text(select.get(), 3);
            if (!is_unresolved_state(record.state)) {
                throw std::runtime_error("invalid_unresolved_state");
            }
            records.push_back(std::move(record));
        }
        return records;
    }

    SqliteCommandLedgerConfig config_;
    Clock clock_;
    std::string ack_topic_;
    sqlite3* database_{};
    std::mutex mutex_;
};

SqliteCommandLedger::SqliteCommandLedger(
    SqliteCommandLedgerConfig config, Clock clock)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(clock)))
{
}

SqliteCommandLedger::~SqliteCommandLedger() = default;

flight::CommandClaimResult SqliteCommandLedger::claim(
    const flight::RemoteCommand& command) noexcept
{
    return impl_->claim(command);
}

flight::StatusPublishResult SqliteCommandLedger::record_status(
    const flight::MissionStatus& status,
    const flight::StatusRecordMode mode) noexcept
{
    return impl_->record_status(status, mode);
}

std::vector<OutboxRecord> SqliteCommandLedger::pending_outbox(
    const std::size_t limit, std::string& error) noexcept
{
    return impl_->pending_outbox(limit, error);
}

bool SqliteCommandLedger::mark_delivered(
    const std::int64_t outbox_id, std::string& error) noexcept
{
    return impl_->mark_delivered(outbox_id, error);
}

std::vector<UnresolvedCommand> SqliteCommandLedger::unresolved_commands(
    std::string& error) noexcept
{
    return impl_->unresolved_commands(error);
}

bool SqliteCommandLedger::fail_unresolved_after_vehicle_recovery(
    const std::string& detail, std::string& error) noexcept
{
    return impl_->fail_unresolved_after_vehicle_recovery(detail, error);
}

std::size_t SqliteCommandLedger::pending_outbox_count(
    std::string& error) noexcept
{
    return impl_->pending_outbox_count(error);
}

const std::string& SqliteCommandLedger::ack_topic() const noexcept
{
    return impl_->ack_topic();
}

std::size_t SqliteCommandLedger::max_status_detail_bytes() const noexcept
{
    return impl_->max_status_detail_bytes();
}

}  // namespace dc::mqtt
