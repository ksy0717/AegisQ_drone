#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

#include "dc/flight/ports.hpp"

namespace dc::mqtt {

/**
 * Capability declaration for this agent (§5.8) -- the exact verb strings the
 * decoder below parses, with the ack statuses each can produce. Lives in the
 * same translation unit as the parser so a verb added there without updating
 * this string fails code review in one diff, not two files apart. Published
 * retained on `aegisq/caps/<vehicleId>` right after registration; the server
 * uses it as the source of truth for command filtering, so an out-of-date
 * declaration silently re-opens the "console shows success, aircraft ignores
 * it" hole this exists to close.
 */
[[nodiscard]] std::string encode_capabilities(std::string_view agent_version);

struct MissionCodecLimits {
    flight::RouteLimits route;
    std::size_t max_issued_by_bytes{};
    std::size_t max_json_depth{};
};

struct DecodeResult {
    std::optional<flight::RemoteCommand> command;
    std::string error;
    // Present only when a canonical commandId could be recovered safely. The
    // callback may use it for a bounded negative ACK on a later schema error.
    std::string command_id;

    [[nodiscard]] bool ok() const noexcept
    {
        return command.has_value();
    }
};

// Decodes the schemaVersion-less AegisQ prototype-v2 drone envelope. Unknown
// and duplicate fields are rejected. Supported verbs are EXECUTE_MISSION,
// STOP_MISSION, RTL, and EMERGENCY_STOP.
// The vehicle id is bound to the exact subscription topic and injected here
// because the server payload intentionally does not repeat it. Authentication
// and authorization remain the mTLS transport and broker ACL's responsibility.
[[nodiscard]] DecodeResult decode_command(
    std::string_view payload,
    std::string_view vehicle_id,
    const MissionCodecLimits& limits) noexcept;

// Compatibility entry point for callers that intentionally accept missions
// only. Safety-command consumers should call decode_command().
[[nodiscard]] DecodeResult decode_execute_mission(
    std::string_view payload,
    std::string_view vehicle_id,
    const MissionCodecLimits& limits) noexcept;

[[nodiscard]] bool is_canonical_uuid(std::string_view value) noexcept;

[[nodiscard]] std::optional<std::chrono::system_clock::time_point>
parse_rfc3339(std::string_view value) noexcept;

[[nodiscard]] std::string format_rfc3339(
    std::chrono::system_clock::time_point value);

// AegisQ consumes commandId/status/ackedAt and ignores the optional missionId
// and error detail. This codec never publishes; the broker adapter owns I/O.
[[nodiscard]] std::string encode_command_ack(
    const flight::MissionStatus& status,
    std::chrono::system_clock::time_point acked_at);

}  // namespace dc::mqtt
