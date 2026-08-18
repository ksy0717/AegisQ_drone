#include "dc/mqtt/mosquitto_session.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <mosquitto.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "dc/mqtt/mission_codec.hpp"

namespace dc::mqtt {
namespace {

constexpr int kMqttKeepaliveMinimumSeconds = 5;
constexpr int kMqttKeepaliveMaximumSeconds = 65535;
constexpr long kRouteQueryTimeoutSeconds = 1L;

void log_mqtt_error(
    const std::string_view message, const std::string_view detail = {}) noexcept
{
    try {
        std::cerr << "[mqtt] " << message;
        if (!detail.empty()) {
            std::cerr << ": " << detail;
        }
        std::cerr << '\n';
    } catch (...) {
        // Logging must never let an exception escape a libmosquitto callback.
    }
}

std::mutex g_library_mutex;
std::size_t g_library_users = 0U;

bool acquire_mosquitto(std::string& error)
{
    std::lock_guard<std::mutex> lock{g_library_mutex};
    if (g_library_users == 0U) {
        const int result = mosquitto_lib_init();
        if (result != MOSQ_ERR_SUCCESS) {
            error = mosquitto_strerror(result);
            return false;
        }
    }
    ++g_library_users;
    return true;
}

void release_mosquitto() noexcept
{
    std::lock_guard<std::mutex> lock{g_library_mutex};
    if (g_library_users == 0U) {
        return;
    }
    --g_library_users;
    if (g_library_users == 0U) {
        static_cast<void>(mosquitto_lib_cleanup());
    }
}

class SocketHandle {
public:
    explicit SocketHandle(const int value) noexcept : value_(value) {}
    ~SocketHandle()
    {
        if (value_ >= 0) {
            static_cast<void>(::close(value_));
        }
    }

    SocketHandle(const SocketHandle&) = delete;
    SocketHandle& operator=(const SocketHandle&) = delete;

    [[nodiscard]] int get() const noexcept
    {
        return value_;
    }

private:
    int value_;
};

std::optional<unsigned int> route_interface_index(
    const std::string& server_ip, std::string& error)
{
    in_addr destination{};
    if (inet_pton(AF_INET, server_ip.c_str(), &destination) != 1) {
        error = "WIREGUARD_SERVER_IP must be an IPv4 address";
        return std::nullopt;
    }

    const SocketHandle netlink{::socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE)};
    if (netlink.get() < 0) {
        error = "cannot open route socket: " + std::string{std::strerror(errno)};
        return std::nullopt;
    }

    timeval receive_timeout{};
    receive_timeout.tv_sec = kRouteQueryTimeoutSeconds;
    if (setsockopt(
            netlink.get(),
            SOL_SOCKET,
            SO_RCVTIMEO,
            &receive_timeout,
            static_cast<socklen_t>(sizeof(receive_timeout))) != 0) {
        error = "cannot bound route query: " + std::string{std::strerror(errno)};
        return std::nullopt;
    }

    alignas(nlmsghdr)
        std::array<char, NLMSG_SPACE(sizeof(rtmsg)) + RTA_SPACE(sizeof(in_addr))> request{};
    auto* header = reinterpret_cast<nlmsghdr*>(request.data());
    header->nlmsg_len = NLMSG_LENGTH(sizeof(rtmsg));
    header->nlmsg_type = RTM_GETROUTE;
    header->nlmsg_flags = NLM_F_REQUEST;
    header->nlmsg_seq = 1U;

    auto* route = reinterpret_cast<rtmsg*>(NLMSG_DATA(header));
    route->rtm_family = AF_INET;
    route->rtm_dst_len = 32U;

    auto* attribute = reinterpret_cast<rtattr*>(
        reinterpret_cast<char*>(header) + NLMSG_ALIGN(header->nlmsg_len));
    attribute->rta_type = RTA_DST;
    attribute->rta_len = RTA_LENGTH(sizeof(destination));
    std::memcpy(RTA_DATA(attribute), &destination, sizeof(destination));
    header->nlmsg_len =
        NLMSG_ALIGN(header->nlmsg_len) + RTA_LENGTH(sizeof(destination));

    sockaddr_nl kernel{};
    kernel.nl_family = AF_NETLINK;
    const ssize_t sent = sendto(
        netlink.get(),
        request.data(),
        header->nlmsg_len,
        0,
        reinterpret_cast<const sockaddr*>(&kernel),
        static_cast<socklen_t>(sizeof(kernel)));
    if (sent < 0 || static_cast<std::size_t>(sent) != header->nlmsg_len) {
        error = "cannot query server route: " + std::string{std::strerror(errno)};
        return std::nullopt;
    }

    alignas(nlmsghdr) std::array<char, 8192> response{};
    const ssize_t received = recv(netlink.get(), response.data(), response.size(), 0);
    if (received <= 0) {
        if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            error = "server route query timed out";
        } else if (received < 0) {
            error = "cannot read server route: " + std::string{std::strerror(errno)};
        } else {
            error = "server route query returned no data";
        }
        return std::nullopt;
    }

    std::size_t remaining = static_cast<std::size_t>(received);
    char* cursor = response.data();
    while (remaining >= sizeof(nlmsghdr)) {
        auto* response_header = reinterpret_cast<nlmsghdr*>(cursor);
        const std::size_t message_size =
            static_cast<std::size_t>(response_header->nlmsg_len);
        if (message_size < sizeof(nlmsghdr) || message_size > remaining) {
            break;
        }
        if (response_header->nlmsg_type == NLMSG_ERROR) {
            error = "kernel rejected server route query";
            return std::nullopt;
        }
        if (response_header->nlmsg_type == RTM_NEWROUTE
            && message_size >= NLMSG_LENGTH(sizeof(rtmsg))) {
            auto* response_route =
                reinterpret_cast<rtmsg*>(NLMSG_DATA(response_header));
            char* attribute_cursor =
                reinterpret_cast<char*>(response_route) + NLMSG_ALIGN(sizeof(rtmsg));
            const std::size_t header_bytes =
                static_cast<std::size_t>(attribute_cursor - cursor);
            std::size_t attributes_remaining =
                header_bytes <= message_size ? message_size - header_bytes : 0U;

            while (attributes_remaining >= sizeof(rtattr)) {
                auto* response_attribute =
                    reinterpret_cast<rtattr*>(attribute_cursor);
                const std::size_t attribute_size =
                    static_cast<std::size_t>(response_attribute->rta_len);
                if (attribute_size < sizeof(rtattr)
                    || attribute_size > attributes_remaining) {
                    break;
                }
                if (response_attribute->rta_type == RTA_OIF
                    && attribute_size - sizeof(rtattr) >= sizeof(int)) {
                    int interface_index = 0;
                    std::memcpy(
                        &interface_index,
                        attribute_cursor + sizeof(rtattr),
                        sizeof(interface_index));
                    if (interface_index > 0) {
                        return static_cast<unsigned int>(interface_index);
                    }
                }
                const std::size_t aligned_attribute =
                    RTA_ALIGN(attribute_size);
                if (aligned_attribute > attributes_remaining) {
                    break;
                }
                attribute_cursor += aligned_attribute;
                attributes_remaining -= aligned_attribute;
            }
        }

        const std::size_t aligned_message = NLMSG_ALIGN(message_size);
        if (aligned_message > remaining) {
            break;
        }
        cursor += aligned_message;
        remaining -= aligned_message;
    }
    error = "server route has no output interface";
    return std::nullopt;
}

std::optional<std::string> interface_ipv4(
    const std::string& interface_name, std::string& error)
{
    const unsigned int index = if_nametoindex(interface_name.c_str());
    if (index == 0U) {
        error = "WireGuard interface does not exist";
        return std::nullopt;
    }

    ifaddrs* addresses = nullptr;
    if (getifaddrs(&addresses) != 0) {
        error = "cannot enumerate interfaces: " + std::string{std::strerror(errno)};
        return std::nullopt;
    }

    std::optional<std::string> result;
    for (const ifaddrs* address = addresses; address != nullptr; address = address->ifa_next) {
        if (address->ifa_addr == nullptr
            || address->ifa_addr->sa_family != AF_INET
            || address->ifa_name == nullptr
            || interface_name != address->ifa_name
            || (address->ifa_flags & IFF_UP) == 0U) {
            continue;
        }
        const auto* ipv4 = reinterpret_cast<const sockaddr_in*>(address->ifa_addr);
        std::array<char, INET_ADDRSTRLEN> text{};
        if (inet_ntop(
                AF_INET,
                &ipv4->sin_addr,
                text.data(),
                static_cast<socklen_t>(text.size())) != nullptr) {
            result = std::string{text.data()};
            break;
        }
    }
    freeifaddrs(addresses);
    if (!result.has_value()) {
        error = "WireGuard interface has no active IPv4 address";
    }
    return result;
}

bool validate_wireguard_route(
    const MosquittoSessionConfig& config,
    std::string& bind_address,
    std::string& error)
{
    // Defense in depth only: a route snapshot plus source-address bind is not
    // a kernel-enforced output-interface policy. Keep the flight agent OFF
    // until deployment also provides a verified wg0-only egress rule.
    if (config.broker_host != config.wireguard_server_ip) {
        error = "broker host must equal WIREGUARD_SERVER_IP";
        return false;
    }
    const unsigned int expected_index =
        if_nametoindex(config.wireguard_interface.c_str());
    if (expected_index == 0U) {
        error = "WireGuard interface does not exist";
        return false;
    }
    const auto route_index =
        route_interface_index(config.wireguard_server_ip, error);
    if (!route_index.has_value() || *route_index != expected_index) {
        if (error.empty()) {
            error = "broker route does not use the configured WireGuard interface";
        }
        return false;
    }
    const auto address = interface_ipv4(config.wireguard_interface, error);
    if (!address.has_value()) {
        return false;
    }
    bind_address = *address;
    return true;
}

bool validate_config(
    const MqttCommandSource& source,
    const MosquittoSessionConfig& config,
    std::string& error)
{
    in_addr parsed{};
    if (config.broker_host.empty()
        || inet_pton(AF_INET, config.broker_host.c_str(), &parsed) != 1) {
        error = "MQTT broker must be a literal IPv4 address covered by its TLS SAN";
        return false;
    }
    if (config.broker_port != 8883U) {
        error = "MQTT control requires TLS port 8883";
        return false;
    }
    if (config.client_id != source.vehicle_id()) {
        error = "MQTT client id must match the configured vehicle id";
        return false;
    }
    if (config.wireguard_interface.empty()
        || config.wireguard_server_ip.empty()
        || config.qos != 1
        || config.keepalive_seconds < kMqttKeepaliveMinimumSeconds
        || config.keepalive_seconds > kMqttKeepaliveMaximumSeconds
        || config.reconnect_min_seconds == 0U
        || config.reconnect_max_seconds < config.reconnect_min_seconds
        || config.outbox_batch_size == 0U
        || config.outbox_batch_size > 4096U
        || config.graceful_offline_timeout
            <= std::chrono::milliseconds::zero()
        || config.graceful_offline_timeout > std::chrono::seconds{5}
        || config.registration.robot_id != source.vehicle_id()) {
        error = "invalid MQTT session configuration";
        return false;
    }
    try {
        static_cast<void>(encode_vehicle_registration(config.registration));
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
    for (const std::string* path :
         {&config.ca_file, &config.client_certificate_file, &config.client_key_file}) {
        if (path->empty()) {
            error = "MQTT TLS credential path is empty";
            return false;
        }
        const std::filesystem::path credential_path{*path};
        if (!credential_path.is_absolute()) {
            error = "MQTT TLS credential path must be absolute: " + *path;
            return false;
        }
        std::error_code filesystem_error;
        const bool regular_file =
            std::filesystem::is_regular_file(credential_path, filesystem_error);
        if (filesystem_error) {
            error = "cannot inspect MQTT TLS credential: " + *path;
            return false;
        }
        if (!regular_file) {
            error = "MQTT TLS credential is not a regular file: " + *path;
            return false;
        }
    }
    return true;
}

}  // namespace

class MosquittoSession::Impl {
public:
    Impl(
        MqttCommandSource& source,
        SqliteCommandLedger& ledger,
        MosquittoSessionConfig config)
        : source_(source),
          ledger_(ledger),
          config_(std::move(config))
    {
        if (ledger_.ack_topic() != command_ack_topic_for(source_.vehicle_id())) {
            throw std::invalid_argument("ledger ACK topic does not match command source");
        }
        registration_topic_ = registration_topic_for(source_.vehicle_id());
        telemetry_topic_ = telemetry_topic_for(source_.vehicle_id());
        lwt_topic_ = lwt_topic_for(source_.vehicle_id());
        registration_payload_ =
            encode_vehicle_registration(config_.registration);
        caps_topic_ = "aegisq/caps/" + std::string{source_.vehicle_id()};
        caps_payload_ = encode_capabilities(config_.registration.firmware.empty()
            ? "unknown" : config_.registration.firmware);
        abnormal_lwt_payload_ = encode_vehicle_lwt(
            source_.vehicle_id(),
            "abnormal-disconnect",
            std::chrono::system_clock::now());
    }

    ~Impl()
    {
        stop();
    }

    bool start(std::string& error) noexcept
    {
        std::lock_guard<std::mutex> lifecycle_lock{lifecycle_mutex_};
        try {
            error.clear();
            if (stopped_) {
                error = "MQTT session has been stopped";
                return false;
            }
            if (started_) {
                error = "MQTT session already started";
                return false;
            }
            accepting_callbacks_.store(false);
            connected_.store(false);
            const std::vector<UnresolvedCommand> unresolved =
                ledger_.unresolved_commands(error);
            if (!error.empty()) {
                return false;
            }
            if (!unresolved.empty()) {
                error = "unresolved commands require confirmed PX4 recovery";
                return false;
            }
            if (!validate_config(source_, config_, error)
                || !validate_wireguard_route(config_, bind_address_, error)
                || !acquire_mosquitto(error)) {
                return false;
            }
            library_acquired_ = true;

            // The stable vehicle client id owns a broker-side MQTT session.
            mosquitto* const client =
                mosquitto_new(config_.client_id.c_str(), false, this);
            if (client == nullptr) {
                error = "mosquitto_new failed";
                cleanup_transport_locked();
                return false;
            }
            {
                std::lock_guard<std::mutex> client_lock{client_mutex_};
                client_ = client;
            }
            mosquitto_connect_callback_set(client, &Impl::handle_connect);
            mosquitto_disconnect_callback_set(client, &Impl::handle_disconnect);
            mosquitto_subscribe_callback_set(client, &Impl::handle_subscribe);
            mosquitto_message_callback_set(client, &Impl::handle_message);
            mosquitto_publish_callback_set(client, &Impl::handle_publish);

            int result = mosquitto_will_set(
                client,
                lwt_topic_.c_str(),
                static_cast<int>(abnormal_lwt_payload_.size()),
                abnormal_lwt_payload_.data(),
                1,
                true);
            if (result == MOSQ_ERR_SUCCESS) {
                result = mosquitto_tls_set(
                client,
                config_.ca_file.c_str(),
                nullptr,
                config_.client_certificate_file.c_str(),
                config_.client_key_file.c_str(),
                nullptr);
            }
            if (result == MOSQ_ERR_SUCCESS) {
                result = mosquitto_tls_opts_set(client, 1, "tlsv1.2", nullptr);
            }
            if (result == MOSQ_ERR_SUCCESS) {
                result = mosquitto_tls_insecure_set(client, false);
            }
            if (result == MOSQ_ERR_SUCCESS) {
                result = mosquitto_reconnect_delay_set(
                    client,
                    config_.reconnect_min_seconds,
                    config_.reconnect_max_seconds,
                    true);
            }
            if (result == MOSQ_ERR_SUCCESS) {
                result = mosquitto_connect_bind_async(
                    client,
                    config_.broker_host.c_str(),
                    static_cast<int>(config_.broker_port),
                    config_.keepalive_seconds,
                    bind_address_.c_str());
            }
            if (result == MOSQ_ERR_SUCCESS) {
                accepting_callbacks_.store(true);
                result = mosquitto_loop_start(client);
                loop_started_ = result == MOSQ_ERR_SUCCESS;
            }
            if (result != MOSQ_ERR_SUCCESS) {
                error = mosquitto_strerror(result);
                cleanup_transport_locked();
                return false;
            }
            started_ = true;
            return true;
        } catch (const std::exception& exception) {
            error = exception.what();
            cleanup_transport_locked();
            return false;
        } catch (...) {
            error = "unknown MQTT startup error";
            cleanup_transport_locked();
            return false;
        }
    }

    void stop() noexcept
    {
        std::lock_guard<std::mutex> lifecycle_lock{lifecycle_mutex_};
        if (stopped_) {
            return;
        }
        stopped_ = true;
        publish_graceful_offline_locked();
        accepting_callbacks_.store(false);
        connected_.store(false);

        // Close first: a callback already decoding will observe closed_ before
        // it can enqueue, and queued commands are discarded before shutdown.
        source_.close();
        cleanup_transport_locked();
        connected_.store(false);
    }

    [[nodiscard]] bool connected() const noexcept
    {
        return connected_.load();
    }

    bool publish_telemetry(
        const flight::VehicleTelemetrySnapshot& snapshot,
        std::string& error) noexcept
    {
        try {
            if (snapshot.vehicle_id != source_.vehicle_id()) {
                error = "telemetry vehicle id mismatch";
                return false;
            }
            const std::string payload = encode_vehicle_telemetry(snapshot);
            return publish_raw(
                telemetry_topic_, payload, 0, false, nullptr, error);
        } catch (const std::exception& exception) {
            error = exception.what();
            return false;
        } catch (...) {
            error = "unknown telemetry publish error";
            return false;
        }
    }

    flight::StatusPublishResult publish(
        const flight::MissionStatus& status,
        const flight::StatusRecordMode mode) noexcept
    {
        const flight::StatusPublishResult stored =
            ledger_.record_status(status, mode);
        if (!stored.durable) {
            return stored;
        }
        std::string error;
        if (!drain_outbox(error) && !error.empty()) {
            log_mqtt_error("ACK outbox drain deferred", error);
        }
        return stored;
    }

private:
    bool publish_raw(
        const std::string& topic,
        const std::string& payload,
        const int qos,
        const bool retained,
        int* const message_id,
        std::string& error) noexcept
    {
        try {
            error.clear();
            if (payload.size()
                > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                error = "MQTT payload exceeds libmosquitto limit";
                return false;
            }
            std::lock_guard<std::mutex> client_lock{client_mutex_};
            if (!accepting_callbacks_.load()
                || !connected_.load()
                || client_ == nullptr) {
                error = "MQTT session is not connected";
                return false;
            }
            int local_message_id = 0;
            const int result = mosquitto_publish(
                client_,
                &local_message_id,
                topic.c_str(),
                static_cast<int>(payload.size()),
                payload.data(),
                qos,
                retained);
            if (result != MOSQ_ERR_SUCCESS) {
                error = mosquitto_strerror(result);
                return false;
            }
            if (message_id != nullptr) {
                *message_id = local_message_id;
            }
            return true;
        } catch (const std::exception& exception) {
            error = exception.what();
            return false;
        } catch (...) {
            error = "unknown MQTT publish error";
            return false;
        }
    }

    void publish_graceful_offline_locked() noexcept
    {
        try {
            if (!connected_.load()) {
                return;
            }
            const std::string payload = encode_vehicle_lwt(
                source_.vehicle_id(),
                "graceful-shutdown",
                std::chrono::system_clock::now());
            std::unique_lock<std::mutex> offline_lock{offline_mutex_};
            std::string error;
            int message_id = 0;
            if (!publish_raw(
                    lwt_topic_, payload, 1, true, &message_id, error)
                || message_id <= 0) {
                log_mqtt_error("graceful offline publication failed", error);
                return;
            }
            offline_message_id_ = message_id;
            offline_puback_ = false;
            offline_disconnected_ = false;
            offline_ready_.wait_for(
                offline_lock,
                config_.graceful_offline_timeout,
                [this] { return offline_puback_ || offline_disconnected_; });
            offline_message_id_ = 0;
        } catch (const std::exception& exception) {
            log_mqtt_error(
                "graceful offline publication failed", exception.what());
        } catch (...) {
            log_mqtt_error(
                "graceful offline publication failed", "unknown error");
        }
    }

    // lifecycle_mutex_ is held by every caller. client_mutex_ only protects
    // the raw libmosquitto pointer against concurrent StatusPublisher calls.
    void cleanup_transport_locked() noexcept
    {
        accepting_callbacks_.store(false);
        connected_.store(false);

        mosquitto* client = nullptr;
        {
            std::lock_guard<std::mutex> client_lock{client_mutex_};
            std::swap(client, client_);
        }
        if (client != nullptr) {
            const int disconnect_result = mosquitto_disconnect(client);
            if (disconnect_result != MOSQ_ERR_SUCCESS
                && disconnect_result != MOSQ_ERR_NO_CONN) {
                log_mqtt_error(
                    "disconnect failed", mosquitto_strerror(disconnect_result));
            }
            if (loop_started_) {
                // The route callback has a bounded receive timeout, so a
                // graceful join cannot be held forever by its netlink query.
                const int loop_result = mosquitto_loop_stop(client, false);
                if (loop_result != MOSQ_ERR_SUCCESS
                    && loop_result != MOSQ_ERR_NO_CONN) {
                    log_mqtt_error(
                        "network loop stop failed", mosquitto_strerror(loop_result));
                }
            }
            mosquitto_destroy(client);
        }

        loop_started_ = false;
        started_ = false;
        {
            std::lock_guard<std::mutex> outbox_lock{outbox_mutex_};
            inflight_by_message_id_.clear();
            inflight_outbox_ids_.clear();
        }
        if (library_acquired_) {
            release_mosquitto();
            library_acquired_ = false;
        }
    }

    bool drain_outbox(std::string& error) noexcept
    {
        try {
            error.clear();
            if (!accepting_callbacks_.load() || !connected_.load()) {
                return true;
            }
            std::lock_guard<std::mutex> drain_lock{drain_mutex_};
            const std::vector<OutboxRecord> pending =
                ledger_.pending_outbox(config_.outbox_batch_size, error);
            if (!error.empty()) {
                return false;
            }

            for (const OutboxRecord& record : pending) {
                if (record.payload.size()
                    > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                    error = "outbox payload exceeds libmosquitto limit";
                    return false;
                }

                std::lock_guard<std::mutex> client_lock{client_mutex_};
                std::lock_guard<std::mutex> outbox_lock{outbox_mutex_};
                if (!accepting_callbacks_.load()
                    || !connected_.load()
                    || client_ == nullptr) {
                    return true;
                }
                if (inflight_outbox_ids_.find(record.id)
                    != inflight_outbox_ids_.end()) {
                    continue;
                }

                int message_id = 0;
                const int result = mosquitto_publish(
                    client_,
                    &message_id,
                    record.topic.c_str(),
                    static_cast<int>(record.payload.size()),
                    record.payload.data(),
                    record.qos,
                    record.retained);
                if (result != MOSQ_ERR_SUCCESS) {
                    error = mosquitto_strerror(result);
                    return false;
                }
                if (message_id <= 0) {
                    error = "libmosquitto returned invalid message id";
                    return false;
                }
                inflight_by_message_id_.emplace(message_id, record.id);
                inflight_outbox_ids_.emplace(record.id);
            }
            return true;
        } catch (const std::exception& exception) {
            error = exception.what();
            return false;
        } catch (...) {
            error = "unknown outbox drain error";
            return false;
        }
    }

    // A command the codec could name but not accept never reaches the
    // coordinator, so no other layer can answer it. Without this the server
    // waits forever for an ACK that nobody is going to send - a mission whose
    // altitude exceeds vehicle policy is an ordinary operator mistake, not an
    // attack, and it deserves a reason rather than silence.
    //
    // Only errors that recovered a canonical commandId can be answered; a
    // payload too broken to name itself has no addressee. A closing source is
    // shutting the process down and must not queue new work.
    void publish_ingest_rejection(const IngestResult& result) noexcept
    {
        try {
            if (result.command_id.empty()
                || result.error == IngestError::closed) {
                return;
            }

            flight::MissionStatus status;
            status.command_id = result.command_id;
            status.vehicle_id = source_.vehicle_id();
            status.state = "REJECTED";
            status.detail = result.detail;

            const std::size_t limit = ledger_.max_status_detail_bytes();
            if (status.detail.size() > limit) {
                status.detail.resize(limit);
                // Cutting inside a multi-byte sequence would make the ACK
                // unencodable, which would drop the very message this exists
                // to deliver.
                while (!status.detail.empty()
                       && (static_cast<unsigned char>(status.detail.back())
                           & 0xC0U) == 0x80U) {
                    status.detail.pop_back();
                }
                if (!status.detail.empty()
                    && (static_cast<unsigned char>(status.detail.back())
                        & 0x80U) != 0U) {
                    status.detail.pop_back();
                }
            }

            // outbox_only: this command was never claimed, so there is no
            // ledger row to update and none may be created for it.
            const flight::StatusPublishResult stored = ledger_.record_status(
                status, flight::StatusRecordMode::outbox_only);
            if (!stored.durable) {
                log_mqtt_error("cannot store rejection ACK", stored.detail);
                return;
            }
            std::string error;
            if (!drain_outbox(error) && !error.empty()) {
                log_mqtt_error("rejection ACK drain deferred", error);
            }
        } catch (const std::exception& exception) {
            log_mqtt_error("rejection ACK failed", exception.what());
        } catch (...) {
            log_mqtt_error("rejection ACK failed", "unknown error");
        }
    }

    static void handle_connect(
        mosquitto* client, void* context, const int result) noexcept
    {
        auto* self = static_cast<Impl*>(context);
        if (self == nullptr) {
            return;
        }
        try {
            self->connected_.store(false);
            if (!self->accepting_callbacks_.load() || result != 0) {
                return;
            }
            std::string current_address;
            std::string route_error;
            if (!validate_wireguard_route(
                    self->config_, current_address, route_error)
                || current_address != self->bind_address_) {
                log_mqtt_error(
                    "refusing subscription",
                    route_error.empty() ? "WireGuard address changed" : route_error);
                // This deliberate disconnect is fail-closed. A future route
                // supervisor must recreate the session after wg0 recovers.
                static_cast<void>(mosquitto_disconnect(client));
                return;
            }
            int subscription_id = 0;
            const int subscribe_result = mosquitto_subscribe(
                client,
                &subscription_id,
                self->source_.command_topic().c_str(),
                self->config_.qos);
            if (subscribe_result != MOSQ_ERR_SUCCESS) {
                log_mqtt_error(
                    "subscribe failed", mosquitto_strerror(subscribe_result));
                static_cast<void>(mosquitto_disconnect(client));
            }
        } catch (const std::exception& exception) {
            self->connected_.store(false);
            log_mqtt_error("connect callback failed", exception.what());
            static_cast<void>(mosquitto_disconnect(client));
        } catch (...) {
            self->connected_.store(false);
            log_mqtt_error("connect callback failed", "unknown error");
            static_cast<void>(mosquitto_disconnect(client));
        }
    }

    static void handle_disconnect(
        mosquitto*, void* context, const int) noexcept
    {
        auto* self = static_cast<Impl*>(context);
        if (self != nullptr) {
            self->connected_.store(false);
            {
                std::lock_guard<std::mutex> offline_lock{self->offline_mutex_};
                self->offline_disconnected_ = true;
            }
            self->offline_ready_.notify_all();
            std::lock_guard<std::mutex> outbox_lock{self->outbox_mutex_};
            self->inflight_by_message_id_.clear();
            self->inflight_outbox_ids_.clear();
        }
    }

    static void handle_subscribe(
        mosquitto*,
        void* context,
        const int,
        const int granted_count,
        const int* granted_qos) noexcept
    {
        auto* self = static_cast<Impl*>(context);
        if (self == nullptr) {
            return;
        }
        const bool granted =
            self->accepting_callbacks_.load()
            && granted_count == 1
            && granted_qos != nullptr
            && granted_qos[0] == 1;
        self->connected_.store(granted);
        if (granted) {
            std::string error;
            if (!self->publish_raw(
                    self->registration_topic_,
                    self->registration_payload_,
                    1,
                    false,
                    nullptr,
                    error)) {
                log_mqtt_error("vehicle registration failed", error);
            }
            // Capability declaration (§5.8) — retained, so the server re-learns
            // it after its own restart without waiting for our reconnect. The
            // payload is generated next to the command parser; publishing a
            // stale one silently re-opens the "console shows success, aircraft
            // ignores it" hole, so failures are logged loudly, not swallowed.
            if (!self->publish_raw(
                    self->caps_topic_,
                    self->caps_payload_,
                    1,
                    true,
                    nullptr,
                    error)) {
                log_mqtt_error("capability declaration failed", error);
            }
            if (!self->drain_outbox(error)) {
                log_mqtt_error("ACK outbox replay failed", error);
            }
        }
    }

    static void handle_publish(
        mosquitto*, void* context, const int message_id) noexcept
    {
        auto* self = static_cast<Impl*>(context);
        if (self == nullptr || message_id <= 0) {
            return;
        }
        try {
            {
                std::lock_guard<std::mutex> offline_lock{self->offline_mutex_};
                if (message_id == self->offline_message_id_) {
                    self->offline_puback_ = true;
                    self->offline_ready_.notify_all();
                }
            }
            std::int64_t outbox_id = 0;
            {
                std::lock_guard<std::mutex> outbox_lock{self->outbox_mutex_};
                const auto found =
                    self->inflight_by_message_id_.find(message_id);
                if (found == self->inflight_by_message_id_.end()) {
                    return;
                }
                outbox_id = found->second;
                self->inflight_by_message_id_.erase(found);
                self->inflight_outbox_ids_.erase(outbox_id);
            }

            std::string error;
            if (!self->ledger_.mark_delivered(outbox_id, error)) {
                log_mqtt_error("cannot retire PUBACKed outbox row", error);
                return;
            }
            if (!self->drain_outbox(error)) {
                log_mqtt_error("ACK outbox continuation failed", error);
            }
        } catch (const std::exception& exception) {
            log_mqtt_error("publish callback failed", exception.what());
        } catch (...) {
            log_mqtt_error("publish callback failed", "unknown error");
        }
    }

    static void handle_message(
        mosquitto*, void* context, const mosquitto_message* message) noexcept
    {
        auto* self = static_cast<Impl*>(context);
        if (self == nullptr
            || !self->accepting_callbacks_.load()
            || message == nullptr
            || message->topic == nullptr
            || message->payloadlen < 0
            || (message->payloadlen > 0 && message->payload == nullptr)) {
            return;
        }
        try {
            const auto payload_size = static_cast<std::size_t>(message->payloadlen);
            const auto* payload_data = static_cast<const char*>(message->payload);
            const std::string_view payload{
                payload_data == nullptr ? "" : payload_data,
                payload_size,
            };
            const IngestResult result = self->source_.ingest(
                message->topic, payload, message->qos, message->retain);
            if (!result.ok()) {
                // Never log the payload or certificate material.
                log_mqtt_error("command rejected", result.detail);
                self->publish_ingest_rejection(result);
            }
        } catch (const std::exception& exception) {
            log_mqtt_error("message callback failed", exception.what());
        } catch (...) {
            log_mqtt_error("message callback failed", "unknown error");
        }
    }

    MqttCommandSource& source_;
    SqliteCommandLedger& ledger_;
    MosquittoSessionConfig config_;
    std::string bind_address_;
    std::string registration_topic_;
    std::string caps_topic_;
    std::string caps_payload_;
    std::string telemetry_topic_;
    std::string lwt_topic_;
    std::string registration_payload_;
    std::string abnormal_lwt_payload_;
    std::mutex lifecycle_mutex_;
    std::mutex client_mutex_;
    std::mutex drain_mutex_;
    std::mutex outbox_mutex_;
    std::mutex offline_mutex_;
    std::condition_variable offline_ready_;
    std::unordered_map<int, std::int64_t> inflight_by_message_id_;
    std::unordered_set<std::int64_t> inflight_outbox_ids_;
    mosquitto* client_{};
    std::atomic<bool> connected_{false};
    std::atomic<bool> accepting_callbacks_{false};
    int offline_message_id_{};
    bool offline_puback_{};
    bool offline_disconnected_{};
    bool library_acquired_{};
    bool loop_started_{};
    bool started_{};
    bool stopped_{};
};

MosquittoSession::MosquittoSession(
    MqttCommandSource& source,
    SqliteCommandLedger& ledger,
    MosquittoSessionConfig config)
    : impl_(std::make_unique<Impl>(source, ledger, std::move(config)))
{
}

MosquittoSession::~MosquittoSession() = default;

bool MosquittoSession::start(std::string& error) noexcept
{
    return impl_->start(error);
}

void MosquittoSession::stop() noexcept
{
    impl_->stop();
}

bool MosquittoSession::connected() const noexcept
{
    return impl_->connected();
}

bool MosquittoSession::publish_telemetry(
    const flight::VehicleTelemetrySnapshot& snapshot,
    std::string& error) noexcept
{
    return impl_->publish_telemetry(snapshot, error);
}

flight::StatusPublishResult MosquittoSession::publish(
    const flight::MissionStatus& status,
    const flight::StatusRecordMode mode) noexcept
{
    return impl_->publish(status, mode);
}

}  // namespace dc::mqtt
