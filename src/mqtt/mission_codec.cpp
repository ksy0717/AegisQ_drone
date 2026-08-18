#include "dc/mqtt/mission_codec.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace dc::mqtt {
namespace {

using Json = nlohmann::json;

class StrictJsonSax final : public nlohmann::json_sax<Json> {
public:
    explicit StrictJsonSax(const std::size_t max_depth) : max_depth_(max_depth) {}

    bool null() override { return true; }
    bool boolean(bool) override { return true; }
    bool number_integer(number_integer_t) override { return true; }
    bool number_unsigned(number_unsigned_t) override { return true; }
    bool number_float(number_float_t, const string_t&) override { return true; }
    bool string(string_t&) override { return true; }
    bool binary(binary_t&) override { return true; }

    bool start_object(std::size_t) override
    {
        if (!start_container()) return false;
        object_keys_.emplace_back();
        return true;
    }

    bool key(string_t& value) override
    {
        if (object_keys_.empty() || !object_keys_.back().insert(value).second) {
            duplicate_key = true;
            return false;
        }
        return true;
    }

    bool end_object() override
    {
        if (object_keys_.empty() || container_depth_ == 0U) return false;
        object_keys_.pop_back();
        --container_depth_;
        return true;
    }

    bool start_array(std::size_t) override { return start_container(); }

    bool end_array() override
    {
        if (container_depth_ == 0U) return false;
        --container_depth_;
        return true;
    }

    bool parse_error(
        std::size_t, const std::string&, const nlohmann::detail::exception&) override
    {
        syntax_error = true;
        return false;
    }

    bool duplicate_key{};
    bool too_deep{};
    bool syntax_error{};

private:
    bool start_container()
    {
        ++container_depth_;
        if (container_depth_ > max_depth_) {
            too_deep = true;
            return false;
        }
        return true;
    }

    std::size_t max_depth_;
    std::size_t container_depth_{};
    std::vector<std::unordered_set<std::string>> object_keys_;
};

bool listed(
    const std::string& key,
    const std::initializer_list<std::string_view> fields) noexcept
{
    return std::any_of(fields.begin(), fields.end(), [&key](const std::string_view field) {
        return key == field;
    });
}

bool has_schema(
    const Json& object,
    const std::initializer_list<std::string_view> required,
    const std::initializer_list<std::string_view> optional = {})
{
    if (!object.is_object()) return false;
    for (const std::string_view field : required) {
        if (!object.contains(std::string{field})) return false;
    }
    for (auto iterator = object.cbegin(); iterator != object.cend(); ++iterator) {
        if (!listed(iterator.key(), required) && !listed(iterator.key(), optional)) {
            return false;
        }
    }
    return true;
}

// Waypoints deliberately tolerate unknown keys. The server preserves any field
// an edge device has started writing ("알 수 없는 키는 보존한다"), so a whitelist
// here rejects an otherwise valid route every time either side adds one. The
// root stays strict: a field there carries command-level meaning, and silently
// ignoring one is how a vehicle ends up obeying a command it did not read.
bool has_waypoint_schema(
    const Json& object, const std::initializer_list<std::string_view> required)
{
    if (!object.is_object()) return false;
    for (const std::string_view field : required) {
        if (!object.contains(std::string{field})) return false;
    }
    return true;
}

bool has_non_whitespace(const std::string_view value)
{
    return std::any_of(value.begin(), value.end(), [](const char character) {
        return std::isspace(static_cast<unsigned char>(character)) == 0;
    });
}

bool valid_text_field(const Json& root, const char* name, const std::size_t max_bytes)
{
    if (!root.at(name).is_string()) return false;
    const std::string value = root.at(name).get<std::string>();
    return !value.empty() && value.size() <= max_bytes && has_non_whitespace(value);
}

std::optional<std::uint32_t> exact_uint32(const Json& value)
{
    if (value.is_number_unsigned()) {
        const std::uint64_t decoded = value.get<std::uint64_t>();
        if (decoded <= std::numeric_limits<std::uint32_t>::max()) {
            return static_cast<std::uint32_t>(decoded);
        }
    } else if (value.is_number_integer()) {
        const std::int64_t decoded = value.get<std::int64_t>();
        if (decoded >= 0
            && static_cast<std::uint64_t>(decoded)
                <= std::numeric_limits<std::uint32_t>::max()) {
            return static_cast<std::uint32_t>(decoded);
        }
    }
    return std::nullopt;
}

std::optional<double> finite_number(const Json& value)
{
    if (!value.is_number()) return std::nullopt;
    const double decoded = value.get<double>();
    return std::isfinite(decoded) ? std::optional<double>{decoded} : std::nullopt;
}

std::optional<flight::RemoteCommandType> decode_command_type(const std::string& value)
{
    if (value == "EXECUTE_MISSION") return flight::RemoteCommandType::execute_mission;
    if (value == "STOP_MISSION") return flight::RemoteCommandType::stop_mission;
    if (value == "RTL") return flight::RemoteCommandType::return_to_launch;
    if (value == "EMERGENCY_STOP") return flight::RemoteCommandType::emergency_stop;
    return std::nullopt;
}

std::optional<flight::WaypointAction> decode_action(const std::string& value)
{
    if (value == "TAKEOFF") return flight::WaypointAction::takeoff;
    if (value == "MOVE") return flight::WaypointAction::move;
    if (value == "WAIT") return flight::WaypointAction::wait;
    if (value == "HOVER") return flight::WaypointAction::hover;
    if (value == "LPR") return flight::WaypointAction::lpr;
    if (value == "WATCH") return flight::WaypointAction::watch;
    if (value == "LAND") return flight::WaypointAction::land;
    if (value == "RTL") return flight::WaypointAction::return_to_launch;
    return std::nullopt;
}

std::optional<flight::WaypointPositionMode> decode_position_mode(
    const std::string& value)
{
    if (value == "WAYPOINT") return flight::WaypointPositionMode::waypoint;
    if (value == "CURRENT") return flight::WaypointPositionMode::current;
    if (value == "HOME") return flight::WaypointPositionMode::home;
    return std::nullopt;
}

int days_in_month(const int year, const int month) noexcept
{
    static constexpr std::array<int, 12> days{
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12) return 0;
    int result = days[static_cast<std::size_t>(month - 1)];
    const bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    if (month == 2 && leap) ++result;
    return result;
}

std::optional<int> parse_digits(
    const std::string_view value,
    const std::size_t offset,
    const std::size_t count) noexcept
{
    if (offset > value.size() || count > value.size() - offset) return std::nullopt;
    int result = 0;
    for (std::size_t index = 0U; index < count; ++index) {
        const char character = value[offset + index];
        if (character < '0' || character > '9') return std::nullopt;
        result = result * 10 + (character - '0');
    }
    return result;
}

std::int64_t days_from_civil(int year, const unsigned month, const unsigned day) noexcept
{
    year -= month <= 2U ? 1 : 0;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned year_of_era = static_cast<unsigned>(year - era * 400);
    const unsigned day_of_year =
        (153U * (month > 2U ? month - 3U : month + 9U) + 2U) / 5U + day - 1U;
    const unsigned day_of_era =
        year_of_era * 365U + year_of_era / 4U - year_of_era / 100U + day_of_year;
    return static_cast<std::int64_t>(era) * 146097
        + static_cast<std::int64_t>(day_of_era) - 719468;
}

DecodeResult failure(std::string error, std::string command_id = {})
{
    DecodeResult result;
    result.error = std::move(error);
    result.command_id = std::move(command_id);
    return result;
}

std::string validation_detail(const flight::ValidationResult& result)
{
    std::string detail = flight::validation_error_name(result.error);
    if (result.waypoint_index != flight::ValidationResult::no_waypoint) {
        detail += ":waypoint=" + std::to_string(result.waypoint_index + 1U);
    }
    return detail;
}

}  // namespace

std::string encode_capabilities(const std::string_view agent_version)
{
    // Kept adjacent to command_type_from() below: the verbs here must be the
    // strings that function parses, nothing more and nothing less. The ack
    // status lists mirror what the executor actually publishes on cmd-ack
    // (ACCEPTED / FAILED / REJECTED / COMPLETED).
    // The version string is config-provided; a stray quote or backslash in it
    // would break the whole declaration and silently drop the server back to
    // its stale fallback list -- escape the two JSON-breaking characters.
    std::string version;
    version.reserve(agent_version.size());
    for (const char c : agent_version) {
        if (c == '"' || c == '\\') version.push_back('\\');
        if (c >= 0x20) version.push_back(c);   // control chars dropped
    }
    std::string out = R"({"agent":"aegisq_drone_companion","version":")";
    out += version;
    out += R"(","verbs":{)";
    out += R"("EXECUTE_MISSION":{"params":["waypoints"],"acks":["ACCEPTED","FAILED","REJECTED","COMPLETED"]},)";
    out += R"("STOP_MISSION":{"acks":["ACCEPTED","FAILED"]},)";
    out += R"("RTL":{"acks":["ACCEPTED","FAILED"]},)";
    out += R"("EMERGENCY_STOP":{"acks":["ACCEPTED","FAILED"]}}})";
    return out;
}


bool is_canonical_uuid(const std::string_view value) noexcept
{
    if (value.size() != 36U) return false;
    bool has_nonzero_digit = false;
    for (std::size_t index = 0U; index < value.size(); ++index) {
        const bool dash = index == 8U || index == 13U || index == 18U || index == 23U;
        if (dash) {
            if (value[index] != '-') return false;
            continue;
        }
        const char character = value[index];
        const bool hexadecimal =
            (character >= '0' && character <= '9')
            || (character >= 'a' && character <= 'f');
        if (!hexadecimal) return false;
        has_nonzero_digit = has_nonzero_digit || character != '0';
    }
    return has_nonzero_digit;
}

std::optional<std::chrono::system_clock::time_point>
parse_rfc3339(const std::string_view value) noexcept
{
    if (value.size() < 20U
        || value[4] != '-' || value[7] != '-' || value[10] != 'T'
        || value[13] != ':' || value[16] != ':') {
        return std::nullopt;
    }

    const auto year = parse_digits(value, 0U, 4U);
    const auto month = parse_digits(value, 5U, 2U);
    const auto day = parse_digits(value, 8U, 2U);
    const auto hour = parse_digits(value, 11U, 2U);
    const auto minute = parse_digits(value, 14U, 2U);
    const auto second = parse_digits(value, 17U, 2U);
    if (!year || !month || !day || !hour || !minute || !second
        || *year < 1970 || *month < 1 || *month > 12
        || *day < 1 || *day > days_in_month(*year, *month)
        || *hour > 23 || *minute > 59 || *second > 59) {
        return std::nullopt;
    }

    std::size_t position = 19U;
    std::int64_t nanoseconds = 0;
    if (position < value.size() && value[position] == '.') {
        ++position;
        const std::size_t fraction_start = position;
        while (position < value.size()
            && value[position] >= '0' && value[position] <= '9') {
            if (position - fraction_start >= 9U) return std::nullopt;
            nanoseconds = nanoseconds * 10 + (value[position] - '0');
            ++position;
        }
        const std::size_t fraction_digits = position - fraction_start;
        if (fraction_digits == 0U) return std::nullopt;
        for (std::size_t index = fraction_digits; index < 9U; ++index) {
            nanoseconds *= 10;
        }
    }

    int offset_seconds = 0;
    if (position < value.size() && value[position] == 'Z') {
        ++position;
    } else {
        if (position + 6U != value.size()
            || (value[position] != '+' && value[position] != '-')
            || value[position + 3U] != ':') {
            return std::nullopt;
        }
        const auto offset_hour = parse_digits(value, position + 1U, 2U);
        const auto offset_minute = parse_digits(value, position + 4U, 2U);
        if (!offset_hour || !offset_minute
            || *offset_hour > 14 || *offset_minute > 59
            || (*offset_hour == 14 && *offset_minute != 0)) {
            return std::nullopt;
        }
        offset_seconds = (*offset_hour * 60 + *offset_minute) * 60;
        if (value[position] == '-') offset_seconds = -offset_seconds;
        position += 6U;
    }
    if (position != value.size()) return std::nullopt;

    const std::int64_t civil_seconds =
        days_from_civil(*year, static_cast<unsigned>(*month), static_cast<unsigned>(*day))
            * 86400
        + static_cast<std::int64_t>(*hour * 3600 + *minute * 60 + *second);
    using FloatingSeconds = std::chrono::duration<long double>;
    using ClockDuration = std::chrono::system_clock::duration;
    const long double total_seconds =
        static_cast<long double>(civil_seconds - offset_seconds)
        + static_cast<long double>(nanoseconds) / 1'000'000'000.0L;
    const long double minimum_seconds =
        std::chrono::duration_cast<FloatingSeconds>(ClockDuration::min()).count();
    const long double maximum_seconds =
        std::chrono::duration_cast<FloatingSeconds>(ClockDuration::max()).count();
    if (total_seconds < minimum_seconds || total_seconds > maximum_seconds) {
        return std::nullopt;
    }
    return std::chrono::system_clock::time_point{
        std::chrono::duration_cast<ClockDuration>(FloatingSeconds{total_seconds})};
}

DecodeResult decode_command(
    const std::string_view payload,
    const std::string_view vehicle_id,
    const MissionCodecLimits& limits) noexcept
{
    if (vehicle_id.empty()) return failure("empty_vehicle_id");
    if (!flight::valid_route_limits(limits.route)
        || limits.max_issued_by_bytes == 0U
        || limits.max_json_depth < 3U) {
        return failure("invalid_codec_limits");
    }

    try {
        StrictJsonSax strict_sax{limits.max_json_depth};
        const bool structurally_valid = Json::sax_parse(
            payload.begin(), payload.end(), &strict_sax,
            Json::input_format_t::json, true, false);
        if (strict_sax.duplicate_key) return failure("duplicate_json_key");
        if (strict_sax.too_deep) return failure("json_too_deep");
        if (!structurally_valid || strict_sax.syntax_error) {
            return failure("invalid_json");
        }

        const Json root = Json::parse(payload.begin(), payload.end());
        if (!root.is_object()) return failure("invalid_root_schema");

        std::string command_id;
        if (root.contains("commandId") && root.at("commandId").is_string()) {
            const std::string candidate = root.at("commandId").get<std::string>();
            if (is_canonical_uuid(candidate)) command_id = candidate;
        }
        if (command_id.empty()) return failure("invalid_command_id");
        if (!root.contains("command") || !root.at("command").is_string()) {
            return failure("invalid_type:command", command_id);
        }
        const auto command_type =
            decode_command_type(root.at("command").get<std::string>());
        if (!command_type.has_value()) {
            return failure("unsupported_command", command_id);
        }

        bool schema_valid = false;
        switch (*command_type) {
            case flight::RemoteCommandType::execute_mission:
                // cruiseSpeedMps is optional so a server that predates the
                // field still decodes; the current one always sends it.
                schema_valid = has_schema(
                    root,
                    {"commandId", "command", "missionId", "waypoints",
                     "coordinateFrame", "altitudeReference", "launch", "expiresAt",
                     "issuedBy", "issuedAt"},
                    {"cruiseSpeedMps"});
                break;
            case flight::RemoteCommandType::stop_mission:
                schema_valid = has_schema(
                    root, {"commandId", "command", "issuedBy", "issuedAt"},
                    {"missionId"});
                break;
            case flight::RemoteCommandType::return_to_launch:
            case flight::RemoteCommandType::emergency_stop:
                schema_valid = has_schema(
                    root, {"commandId", "command", "issuedBy", "issuedAt"},
                    {"missionId", "reason"});
                break;
        }
        if (!schema_valid) return failure("invalid_root_schema", command_id);
        if (!valid_text_field(root, "issuedBy", limits.max_issued_by_bytes)) {
            return failure("invalid_issued_by", command_id);
        }
        if (!root.at("issuedAt").is_string()) {
            return failure("invalid_type:issuedAt", command_id);
        }
        const auto issued_at =
            parse_rfc3339(root.at("issuedAt").get<std::string>());
        if (!issued_at.has_value()) return failure("invalid_issued_at", command_id);

        flight::RemoteCommand command;
        command.command_id = command_id;
        command.vehicle_id = std::string{vehicle_id};
        command.type = *command_type;
        command.issued_by = root.at("issuedBy").get<std::string>();
        command.issued_at = issued_at;
        command.canonical_payload = root.dump();

        if (root.contains("missionId")) {
            if (!root.at("missionId").is_string()) {
                return failure("invalid_type:missionId", command_id);
            }
            const std::string mission_id = root.at("missionId").get<std::string>();
            if (!is_canonical_uuid(mission_id)) {
                return failure("invalid_mission_id", command_id);
            }
            command.mission_id = mission_id;
        }
        if (root.contains("reason")) {
            if (!valid_text_field(root, "reason", limits.max_issued_by_bytes)) {
                return failure("invalid_reason", command_id);
            }
            command.reason = root.at("reason").get<std::string>();
        }

        if (*command_type == flight::RemoteCommandType::execute_mission) {
            for (const char* field :
                 {"coordinateFrame", "altitudeReference", "expiresAt"}) {
                if (!root.at(field).is_string()) {
                    return failure("invalid_type:" + std::string{field}, command_id);
                }
            }
            const auto expires_at =
                parse_rfc3339(root.at("expiresAt").get<std::string>());
            if (!expires_at.has_value()) {
                return failure("invalid_expires_at", command_id);
            }
            command.expires_at = expires_at;

            const Json& launch_json = root.at("launch");
            if (!has_schema(launch_json, {"mode"}, {"takeoffAltitudeM"})
                || !launch_json.at("mode").is_string()) {
                return failure("invalid_launch_schema", command_id);
            }
            flight::RoutePlan route;
            route.coordinate_frame = root.at("coordinateFrame").get<std::string>();
            route.altitude_reference = root.at("altitudeReference").get<std::string>();
            route.launch.mode = launch_json.at("mode").get<std::string>();
            if (launch_json.contains("takeoffAltitudeM")) {
                const auto altitude = finite_number(launch_json.at("takeoffAltitudeM"));
                if (!altitude.has_value()) {
                    return failure("invalid_launch_altitude", command_id);
                }
                route.launch.takeoff_altitude_m = *altitude;
            }
            if (root.contains("cruiseSpeedMps")) {
                const auto cruise = finite_number(root.at("cruiseSpeedMps"));
                if (!cruise.has_value()) {
                    return failure("invalid_cruise_speed", command_id);
                }
                route.cruise_speed_m_s = *cruise;
            }

            const Json& waypoint_array = root.at("waypoints");
            if (!waypoint_array.is_array()) {
                return failure("invalid_type:waypoints", command_id);
            }
            if (waypoint_array.empty()) return failure("empty_waypoints", command_id);
            if (waypoint_array.size() > limits.route.max_waypoints) {
                return failure("too_many_waypoints", command_id);
            }
            route.waypoints.reserve(waypoint_array.size());

            for (std::size_t index = 0U; index < waypoint_array.size(); ++index) {
                const Json& encoded = waypoint_array[index];
                if (!has_waypoint_schema(
                        encoded, {"seq", "action", "dwellSec", "positionMode"})) {
                    return failure(
                        "invalid_waypoint_schema:" + std::to_string(index + 1U),
                        command_id);
                }
                if (!encoded.at("action").is_string()
                    || !encoded.at("positionMode").is_string()) {
                    return failure(
                        "invalid_waypoint_value:" + std::to_string(index + 1U),
                        command_id);
                }

                const auto sequence = exact_uint32(encoded.at("seq"));
                const auto dwell = exact_uint32(encoded.at("dwellSec"));
                const auto action = decode_action(encoded.at("action").get<std::string>());
                const auto position_mode = decode_position_mode(
                    encoded.at("positionMode").get<std::string>());
                if (!sequence || !dwell || !action || !position_mode) {
                    return failure(
                        "invalid_waypoint_value:" + std::to_string(index + 1U),
                        command_id);
                }

                flight::GeoWaypoint point;
                point.sequence = *sequence;
                point.action = *action;
                point.dwell_seconds = *dwell;
                point.position_mode = *position_mode;
                if (encoded.contains("lat")) {
                    const auto latitude = finite_number(encoded.at("lat"));
                    if (!latitude.has_value()) {
                        return failure(
                            "invalid_latitude:" + std::to_string(index + 1U),
                            command_id);
                    }
                    point.latitude_deg = *latitude;
                }
                if (encoded.contains("lng")) {
                    const auto longitude = finite_number(encoded.at("lng"));
                    if (!longitude.has_value()) {
                        return failure(
                            "invalid_longitude:" + std::to_string(index + 1U),
                            command_id);
                    }
                    point.longitude_deg = *longitude;
                }
                if (encoded.contains("altitude")) {
                    const auto altitude = finite_number(encoded.at("altitude"));
                    if (!altitude.has_value()) {
                        return failure(
                            "invalid_altitude:" + std::to_string(index + 1U),
                            command_id);
                    }
                    point.relative_altitude_m = *altitude;
                }
                if (encoded.contains("speed")) {
                    const auto speed = finite_number(encoded.at("speed"));
                    if (!speed.has_value()) {
                        return failure(
                            "invalid_waypoint_speed:" + std::to_string(index + 1U),
                            command_id);
                    }
                    point.speed_m_s = *speed;
                }
                route.waypoints.push_back(std::move(point));
            }
            command.route = std::move(route);
        }

        const flight::ValidationResult validation =
            flight::validate_command(command, limits.route);
        if (!validation.ok()) {
            return failure(validation_detail(validation), command_id);
        }

        DecodeResult result;
        result.command_id = command_id;
        result.command = std::move(command);
        return result;
    } catch (const std::exception&) {
        return failure("invalid_json");
    } catch (...) {
        return failure("invalid_json");
    }
}

DecodeResult decode_execute_mission(
    const std::string_view payload,
    const std::string_view vehicle_id,
    const MissionCodecLimits& limits) noexcept
{
    DecodeResult result = decode_command(payload, vehicle_id, limits);
    if (result.ok()
        && result.command->type != flight::RemoteCommandType::execute_mission) {
        return failure("unsupported_command", result.command_id);
    }
    return result;
}

std::string format_rfc3339(const std::chrono::system_clock::time_point value)
{
    const auto since_epoch = value.time_since_epoch();
    auto whole_seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
    auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch - whole_seconds);
    if (milliseconds < std::chrono::milliseconds::zero()) {
        whole_seconds -= std::chrono::seconds{1};
        milliseconds += std::chrono::milliseconds{1000};
    }
    const std::time_t encoded_time =
        std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::time_point{whole_seconds});
    std::tm utc{};
    {
        static std::mutex utc_mutex;
        const std::lock_guard<std::mutex> lock{utc_mutex};
        const std::tm* decoded = std::gmtime(&encoded_time);
        if (decoded == nullptr) throw std::runtime_error("utc_format_failed");
        utc = *decoded;
    }
    std::ostringstream encoded;
    encoded << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S");
    encoded << '.' << std::setw(3) << std::setfill('0')
            << milliseconds.count() << 'Z';
    return encoded.str();
}

std::string encode_command_ack(
    const flight::MissionStatus& status,
    const std::chrono::system_clock::time_point acked_at)
{
    Json payload{
        {"commandId", status.command_id},
        {"status", status.state},
        {"ackedAt", format_rfc3339(acked_at)},
    };
    if (!status.mission_id.empty()) payload["missionId"] = status.mission_id;
    if (!status.detail.empty() && status.state != "ACCEPTED") {
        payload["error"] = status.detail;
    }
    return payload.dump();
}

}  // namespace dc::mqtt
