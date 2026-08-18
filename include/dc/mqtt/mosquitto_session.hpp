#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "dc/flight/ports.hpp"
#include "dc/flight/telemetry.hpp"
#include "dc/mqtt/aegisq_state_codec.hpp"
#include "dc/mqtt/mqtt_command_source.hpp"
#include "dc/mqtt/sqlite_command_ledger.hpp"

namespace dc::mqtt {

struct MosquittoSessionConfig {
    std::string broker_host;
    std::uint16_t broker_port{};
    std::string wireguard_server_ip;
    std::string wireguard_interface;
    std::string client_id;
    std::string ca_file;
    std::string client_certificate_file;
    std::string client_key_file;
    int qos{};
    int keepalive_seconds{};
    unsigned int reconnect_min_seconds{};
    unsigned int reconnect_max_seconds{};
    std::size_t outbox_batch_size{};
    std::chrono::milliseconds graceful_offline_timeout{};
    VehicleRegistration registration;
};

// Linux/libmosquitto transport only. MqttCommandSource remains independently
// testable and has no broker dependency.
class MosquittoSession final : public flight::StatusPublisher {
public:
    MosquittoSession(
        MqttCommandSource& source,
        SqliteCommandLedger& ledger,
        MosquittoSessionConfig config);
    ~MosquittoSession() override;

    MosquittoSession(const MosquittoSession&) = delete;
    MosquittoSession& operator=(const MosquittoSession&) = delete;

    [[nodiscard]] bool start(std::string& error) noexcept;
    void stop() noexcept;
    [[nodiscard]] bool connected() const noexcept;
    [[nodiscard]] bool publish_telemetry(
        const flight::VehicleTelemetrySnapshot& snapshot,
        std::string& error) noexcept;

    // The exact ACK is committed to SQLite before any network attempt. A
    // successful broker PUBACK removes it; disconnects leave it for replay.
    flight::StatusPublishResult publish(
        const flight::MissionStatus& status,
        flight::StatusRecordMode mode) noexcept override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dc::mqtt
