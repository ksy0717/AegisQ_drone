#pragma once

#include <string>

namespace mqtt_fixtures {

inline std::string execute_mission_payload()
{
    return R"json({
  "commandId": "2ac105b7-2233-4455-aabb-66778899aabb",
  "command": "EXECUTE_MISSION",
  "missionId": "f7c5c855-6677-4889-aabb-112233445566",
  "waypoints": [
    {
      "seq": 1,
      "action": "TAKEOFF",
      "dwellSec": 0,
      "altitude": 10.0,
      "positionMode": "CURRENT"
    },
    {
      "seq": 2,
      "lat": 35.338500,
      "lng": 129.030500,
      "action": "MOVE",
      "dwellSec": 0,
      "altitude": 15.0,
      "positionMode": "WAYPOINT"
    },
    {
      "seq": 3,
      "lat": 35.338800,
      "lng": 129.030800,
      "action": "HOVER",
      "dwellSec": 15,
      "altitude": 15.0,
      "positionMode": "WAYPOINT"
    },
    {
      "seq": 4,
      "action": "RTL",
      "dwellSec": 0,
      "positionMode": "HOME"
    }
  ],
  "coordinateFrame": "ABSOLUTE",
  "altitudeReference": "HOME_RELATIVE",
  "launch": {
    "mode": "CURRENT_POSITION",
    "takeoffAltitudeM": 10.0
  },
  "expiresAt": "2026-08-06T03:04:15.123Z",
  "issuedBy": "ksy",
  "issuedAt": "2026-08-06T03:04:05.123Z"
})json";
}

// The shape aegisq-plus actually publishes as of prototype-v2 ea67dbb
// (2026-08-12): mission-level cruiseSpeedMps on every EXECUTE_MISSION, an
// optional per-leg speed, and whatever extra waypoint keys the console stored
// (MissionWaypointCodec copies the map before normalising: "알 수 없는 키는
// 보존한다"). A coordinate-carrying LAND is expanded server-side into
// MOVE(WAYPOINT) + LAND(CURRENT), which is why that pair appears here.
inline std::string server_execute_mission_payload()
{
    return R"json({
  "commandId": "7ac105b7-2233-4455-aabb-66778899aabb",
  "command": "EXECUTE_MISSION",
  "missionId": "a7c5c855-6677-4889-aabb-112233445566",
  "waypoints": [
    {
      "seq": 1,
      "action": "TAKEOFF",
      "dwellSec": 0,
      "altitude": 5.0,
      "positionMode": "CURRENT"
    },
    {
      "seq": 2,
      "lat": 35.338700,
      "lng": 129.030700,
      "action": "MOVE",
      "dwellSec": 0,
      "altitude": 5.0,
      "positionMode": "WAYPOINT",
      "speed": 1.5,
      "operatorLabel": "gate-A"
    },
    {
      "seq": 3,
      "lat": 35.338800,
      "lng": 129.030800,
      "action": "MOVE",
      "dwellSec": 0,
      "altitude": 5.0,
      "positionMode": "WAYPOINT"
    },
    {
      "seq": 4,
      "action": "LAND",
      "dwellSec": 0,
      "positionMode": "CURRENT"
    }
  ],
  "cruiseSpeedMps": 2.0,
  "coordinateFrame": "ABSOLUTE",
  "altitudeReference": "HOME_RELATIVE",
  "launch": {
    "mode": "CURRENT_POSITION",
    "takeoffAltitudeM": 5.0
  },
  "expiresAt": "2026-08-12T03:04:15.123Z",
  "issuedBy": "ksy",
  "issuedAt": "2026-08-12T03:04:05.123Z"
})json";
}

inline std::string stop_mission_payload()
{
    return R"json({
  "commandId": "3ac105b7-2233-4455-aabb-66778899aabb",
  "command": "STOP_MISSION",
  "missionId": "f7c5c855-6677-4889-aabb-112233445566",
  "issuedBy": "ksy",
  "issuedAt": "2026-08-06T03:04:06.123Z"
})json";
}

inline std::string rtl_payload()
{
    return R"json({
  "commandId": "4ac105b7-2233-4455-aabb-66778899aabb",
  "command": "RTL",
  "missionId": "f7c5c855-6677-4889-aabb-112233445566",
  "reason": "OPERATOR_STOP",
  "issuedBy": "ksy",
  "issuedAt": "2026-08-06T03:04:07.123Z"
})json";
}

inline std::string emergency_stop_payload()
{
    return R"json({
  "commandId": "5ac105b7-2233-4455-aabb-66778899aabb",
  "command": "EMERGENCY_STOP",
  "issuedBy": "ksy",
  "issuedAt": "2026-08-06T03:04:08.123Z"
})json";
}

inline std::string replace_once(
    std::string value, const std::string& from, const std::string& to)
{
    const std::size_t position = value.find(from);
    if (position != std::string::npos) value.replace(position, from.size(), to);
    return value;
}

}  // namespace mqtt_fixtures
