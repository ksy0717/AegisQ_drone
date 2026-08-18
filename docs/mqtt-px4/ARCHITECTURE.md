# MQTT/PX4 project profile

This branch is a long-lived product variant based on main. Shared fixes flow
from main into this branch. Product-only MQTT code stays behind ports so it
does not leak into the RTSP process or the verified Riposte flight math.

## Product and process boundaries

    AegisQ backend (10.100.0.1)
              |
         WireGuard wg0
              |
        MQTT mTLS :8883
              |
       MosquittoSession
          |         |
          |     StatusPublisher
          v         |
    MqttCommandSource
              |
      bounded FIFO queue
              |
    RemoteMissionCoordinator
              |
       MissionExecutor port
              |
       future MAVSDK / PX4

    camera -> drone_companion -> MediaMTX -> RTSP client

The video and flight-control processes are independent. The camera remains
owned by drone_companion, and a future MQTT agent must not open the camera or
depend on GStreamer/Hailo.

The connector, codec, transport library, and tests exist. The PX4 executor and
the mqtt_px4_control executable do not. DC_BUILD_MQTT_PX4_AGENT is OFF, and the
service has not been installed, enabled, deployed, or run.

## Build profile

All default build switches live in config/project.cmake. Do not duplicate
branch defaults in CMakeLists.txt or scripts/install.sh.

The current profile selects:

- DC_BUILD_VIDEO=ON, so camera and RTSP stay enabled;
- DC_HAILORT_MODE=OFF at compile time, with Hailo inference OFF at runtime;
- DC_BUILD_MQTT_CONNECTOR=ON and DC_BUILD_MQTT_TRANSPORT=ON;
- shared flight, remote-command, and MQTT connector tests=ON;
- Riposte vehicle agent, GCS, and Riposte-only protocol tests=OFF;
- DC_BUILD_MQTT_PX4_AGENT=OFF.

Building the connector or transport static libraries does not make the product
flight-ready. Command-line cache overrides are for diagnostic builds only.

## AegisQ topics and vehicle identity

The backend contract uses one topic shape for every registered robot:

- command subscription: aegisq/cmd/{vehicleId}
- command ACK publication: aegisq/cmd-ack/{vehicleId}
- QoS: 1 in both directions
- retained commands: forbidden

For DRONE-01 these resolve exactly to aegisq/cmd/DRONE-01 and
aegisq/cmd-ack/DRONE-01. The connector does not use wildcard subscriptions.
MQTT_CLIENT_ID and MQTT_VEHICLE_ID must be identical. The command payload has no
vehicleId field; the connector injects the identity from its exact subscription
topic instead of trusting payload identity.

AegisQ has K9, WHEEL, DRONE, and PHONE mobilityClass values, but the currently
inspected backend mission dispatch and web mission selector do not enforce
mobilityClass == DRONE. They publish through the common robot topic shape. The
strict drone payload reduces accidental execution, but it cannot prove that the
server-selected id belongs to a drone. Until backend and frontend validation is
added, operational controls and broker ACLs must restrict mission targets to
known drone ids. This repository change does not modify server or frontend code.

## Strict EXECUTE_MISSION contract

The implemented decoder supports only the current unversioned, implicit-v1
EXECUTE_MISSION envelope. A valid example is:

    {
      "commandId": "2ac105b7-2233-4455-aabb-66778899aabb",
      "command": "EXECUTE_MISSION",
      "missionId": "f7c5c855-6677-4889-aabb-112233445566",
      "waypoints": [
        {
          "seq": 1,
          "lat": 35.338000,
          "lng": 129.030100,
          "action": "TAKEOFF",
          "dwellSec": 0,
          "altitude": 10.0
        },
        {
          "seq": 2,
          "lat": 35.338500,
          "lng": 129.030500,
          "action": "MOVE",
          "dwellSec": 0,
          "altitude": 15.0
        },
        {
          "seq": 3,
          "lat": 35.338800,
          "lng": 129.030800,
          "action": "HOVER",
          "dwellSec": 15,
          "altitude": 15.0
        },
        {
          "seq": 4,
          "lat": 35.338000,
          "lng": 129.030100,
          "action": "LAND",
          "dwellSec": 0,
          "altitude": 10.0
        }
      ],
      "issuedBy": "ksy",
      "issuedAt": "2026-08-06T03:04:05.123Z"
    }

The root must contain exactly commandId, command, missionId, waypoints,
issuedBy, and issuedAt. Each waypoint must contain exactly seq, lat, lng,
action, dwellSec, and altitude. Unknown or missing fields, duplicate JSON keys,
invalid types, excessive nesting, and malformed JSON are rejected. A
schemaVersion field is also rejected until AegisQ publishes and coordinates a
versioned contract.

commandId and missionId are full canonical UUID strings. Truncated UUIDs and
numeric monotonic ids are invalid. issuedBy must contain non-whitespace text and
fit the configured byte limit. issuedAt must be RFC 3339 and pass configured
maximum-age and future-skew checks; the Companion therefore needs a trustworthy
system clock.

Waypoint rules are drone-specific:

- lat/lng are finite WGS84 degrees in their legal ranges;
- altitude is finite height in metres relative to mission home, not AMSL;
- points are normalized by seq and then must form the exact sequence 1..N;
- the first action is TAKEOFF and the last action is LAND;
- intermediate actions are MOVE or HOVER;
- HOVER requires dwellSec > 0; TAKEOFF, MOVE, and LAND require dwellSec == 0;
- waypoint count, relative altitude, per-point dwell, and total dwell must pass
  explicit per-vehicle runtime limits.

There are no compiled safety defaults for route or freshness limits. Empty or
invalid limits must prevent agent startup. The source also enforces exact topic,
QoS 1, non-retained delivery, payload size, queue capacity, issuer length, and
JSON depth.

## Idempotency and ACK semantics

The coordinator currently remembers UUIDs only in process memory. A redelivery
of the same UUID and equivalent decoded command content replays the original
ACK without executing again. Reuse of that UUID with different content is
rejected as a command_id_conflict. This protection is lost on restart.

The transport requests a persistent broker session for the stable vehicle
client id, so an existing QoS 1 subscription can receive messages published
while the client is offline. This does not replace the durable local command
ledger or ACK outbox required before flight activation.

The ACK encoding is:

    {
      "commandId": "<same canonical UUID>",
      "status": "ACCEPTED | REJECTED | FAILED",
      "ackedAt": "<RFC3339 UTC>",
      "missionId": "<optional mission UUID>",
      "error": "<optional non-ACCEPTED detail>"
    }

The inspected AegisQ backend treats ACCEPTED and REJECTED as non-terminal
acknowledgements. COMPLETED, SUCCESS, or OK marks a mission complete, while
FAILED or ERROR marks it failed. The present connector can report synchronous
acceptance, rejection, or an executor exception, but it has no flight-progress
publisher that emits a final COMPLETED/FAILED result. Decode failures, a full
ingress queue, or an unavailable ACK connection also do not yet have a durable
negative-ACK/outbox path.

Before activation, use a persistent command ledger/inbox that atomically claims
a UUID before execution, stores canonical command content and the result, and survives
process or power loss. Pair it with a persistent ACK outbox and replay so QoS 1
redelivery and broker outages converge without executing a mission twice.

## mTLS and broker authorization

The application endpoint is mqtts://10.100.0.1:8883. Plain MQTT 1883 is not an
application fallback. AegisQ-MQTT-CA and the DRONE-01 client identity are read
in place:

- CA: /etc/aegisq/certs/ca.crt
- client certificate: /etc/aegisq/certs/DRONE-01.crt
- client key: /etc/aegisq/certs/DRONE-01.key

Never disable certificate or broker hostname/IP verification, and never copy
the private key into this repository. File names and MQTT client-id equality
are configuration checks, not cryptographic proof that the certificate subject
is DRONE-01. The broker must map the authenticated certificate principal to
exactly these permissions:

- subscribe aegisq/cmd/DRONE-01;
- publish aegisq/cmd-ack/DRONE-01;
- deny wildcard and every other vehicle topic.

Apply the same one-principal/one-vehicle rule for each drone identity. Validate
the broker ACL before any flight-capable service is enabled.

## WireGuard fail-closed requirement

The control connection is intended to reach 10.100.0.1 only through wg0. The
transport checks the selected route and WireGuard source address before
connecting/subscribing. That check is useful defense-in-depth, but it is not a
kernel-enforced kill switch and cannot alone guarantee that a later routing
change never uses WLAN.

Provision an external firewall or policy-routing kill switch that prevents
10.100.0.1:8883 from leaving any interface except wg0. Verify fail-closed
behaviour by dropping wg0 while connected and while reconnecting, and confirm
that no MQTT packets appear on WLAN. Do not enable remote flight until this
test passes. The systemd Wants/After dependency on wg-quick@wg0.service is
ordering only; it is not a security boundary.

## Flight activation status (2026-08-13)

Items 1 through 4 and 7 are done. `DC_BUILD_MQTT_PX4_AGENT` is ON, the service
template is installed and running on the aircraft, and `REMOTE_FLIGHT_EXECUTE`
is the single remaining gate -- currently 1 for external testing.

Closed:

1. persistent inbox, ACK outbox, UUID deduplication, atomic command claiming,
   and crash/restart replay -- `SqliteCommandLedger`, verified by a real power
   cut (a stranded ACCEPTED row was terminalized and five replayed QoS 1
   commands were rejected `stale_command`);
2. STOP_MISSION and EMERGENCY_STOP with priority over ordinary missions,
   preemption of an in-progress mission, and ACKs -- SITL measured preemption
   at 11 ms for STOP and 56 ms for EMERGENCY_STOP;
3. a MAVSDK backend adapted from the verified Riposte path
   (`MavsdkMissionBackend` over `MavsdkOffboardControl`), with
   `PreemptiveMissionExecutor` owning the only thread that touches PX4;
4. terminal COMPLETED/FAILED publication tied to the actual PX4 outcome, made
   durable through the ledger outbox;
7. SITL coverage for the full mission matrix, plus link loss, geofence breach,
   and pilot override.

RETURN_TO_LAUNCH is deliberately **not** a server command. It exists as a
terminal waypoint action and as a synthetic command the local supervisors
raise; a server sending that name is answered `unsupported_command`.

Still open:

5. broker certificate-to-vehicle ACL enforcement and an externally verified
   WireGuard kill switch;
6. backend/frontend enforcement that the selected mission target has
   mobilityClass == DRONE, or an equivalent server-side authorization gate;
8. real-aircraft validation beyond the single takeoff/land flight already
   completed -- sustained operation, route flight, battery gate, and preemption
   have only been shown in SITL.

## Local autonomous safety actions

Four supervisors can originate a return without a server command. All are gated
on `REMOTE_FLIGHT_EXECUTE=1`; in observe-only mode they record and do nothing.
None of them forge a server ACK, because the ledger requires a canonical
`commandId` and a synthetic command has none -- the server learns from the
telemetry `flight_mode`.

| Trigger | Condition | Latency |
|---|---|---|
| Low battery | fraction at or below the configured floor | supervisor cadence |
| Link loss | broker session down past `MQTT_LINK_LOSS_TIMEOUT_S`, airborne, home valid | measured 20.3 s end to end |
| Geofence | measured horizontal distance from home exceeds `GEOFENCE_RADIUS_M` | supervisor cadence, independent of broker connectivity |
| Pilot override | fresh raw CH6 enters configured POSITION range | next `RC_CHANNELS` sample / `check_abort()` |

Priority, from `PreemptiveMissionExecutor::priority()`, which is the single
source: stop < return_to_launch < link_loss < geofence < low_battery <
emergency_stop < shutdown. A return already pending absorbs later requests as
`safe_return_already_pending` rather than queueing a second one.

Link-loss latency is dominated by MQTT keepalive, not by the timeout that names
it: a silently broken path is only noticed after roughly `1.5 x
MQTT_KEEPALIVE_SEC`, and the timeout starts after that. Measured on the Pi:
keepalive 30 gives a 45.2 s share, keepalive 10 gives 15.3 s.

After the takeover is latched, pilot override takes **no new** action: no Hold,
Land, RTL, or Disarm, because the aircraft belongs to whoever is holding the sticks. The
authoritative evidence is the configured raw `RC_CHANNELS` CH6 POSITION range;
PX4 mode alone cannot distinguish a real switch movement from Offboard loss.
Missing/stale RC while armed is also fail-closed and permits no new PX4
command. The takeover latches, destroys the MAVSDK Offboard sender without a
mode command, and terminalizes the old mission. There is no airborne resume.
The current MAVSDK calls are synchronous, so an already-entered call cannot be
cancelled and a final-check-to-call race remains. A bounded asynchronous
dispatcher is required before claiming command-initiation linearizability with
the RC callback; propeller-off traces must be used until that work is approved
and completed.

The latch becomes eligible to clear only after PX4 positively reports
OnGround+DISARMED and fresh CH6 HOLD has remained stable through the configured
debounce. A later START is a brand-new mission: it recreates the Offboard
plugin, primes the current position, and only then asks PX4 for Offboard.
Selecting physical CH6 OFFBOARD before START is not the supported handback
path. Whether the aircraft actually stays flying under the pilot is also a PX4
parameter question (`COM_OBL_RC_ACT`, `COM_RC_OVERRIDE`, `COM_RC_IN_MODE`), and
the complete handover cycle still requires propeller-off vehicle validation.

If CH6 still authorizes the Pi but PX4 leaves Offboard after the bounded
post-start heartbeat confirmation window, the event is an Offboard-loss fault,
not a pilot override. That path retains the controlled emergency action.

The codec accepts no geofence field from the server, so the local radius above
is the only spatial containment the aircraft has.
