# MQTT/PX4 WIP handoff (2026-08-10)

This is a temporary handoff note. The working tree is intentionally not being
presented as flight-ready.

## Requested outcome

- Receive prototype-v2 AegisQ commands over MQTT and execute them through
  MAVSDK/PX4.
- Support EXECUTE_MISSION, STOP_MISSION, RTL, and EMERGENCY_STOP.
- Publish durable command ACKs, terminal status, vehicle registration,
  telemetry, PX4 Home, and LWT.
- If battery is at or below 30% while airborne, preempt the current operation,
  return to PX4 Home, and wait for landing.
- Keep WireGuard wg0, mTLS, exact-topic identity, broker ACL, and durable
  command guarantees fail-closed.

The server sources used for the contract are:

- backend: https://github.com/sizzlingbun/aegisq-plus.git,
  branch prototype-v2
- frontend: https://github.com/sizzlingbun/aegisq-web.git,
  branch prototype-v2

These repositories and branches are also recorded near the top of AGENTS.md.

## Implemented in the current working tree

- The strict command codec now matches the additive prototype-v2 envelope:
  coordinateFrame, altitudeReference, launch, expiresAt, and positionMode,
  plus coordinate/altitude omission rules for control actions.
- EXECUTE_MISSION, STOP_MISSION, RTL, and EMERGENCY_STOP are decoded.
- MqttCommandSource has bounded normal and reserved safety queues.
- SQLite WAL/FULL persistence atomically claims UUIDs, stores status, and
  maintains a QoS 1 ACK outbox. Duplicate/conflicting UUID and restart
  recovery paths have tests.
- RemoteMissionCoordinator publishes ACCEPTED, REJECTED, COMPLETED, and FAILED
  based on asynchronous executor events.
- PreemptiveMissionExecutor owns one backend/PX4 worker and supports priority
  safety preemption.
- A shared MAVSDK Offboard/Action controller was extracted from the Riposte
  test runner so the MQTT backend reuses takeoff, setpoint watchdog, landing,
  RTL, and restart-recovery mechanics.
- MavsdkMissionBackend translates absolute WGS84 / HOME_RELATIVE routes into
  local NED and checks PX4 Home/current-launch agreement, health, position,
  route radius, and preflight battery.
- During an active route, battery at or below low_battery_rtl_fraction causes
  RTL and landing confirmation. Lost battery telemetry also causes RTL after
  the configured timeout.
- A process-wide low-battery RTL request was added to
  PreemptiveMissionExecutor so an airborne vehicle can still RTL after a
  mission has stopped and is holding.
- The AegisQ state codec emits:
  - aegisq/telemetry/{id} payloads with battery, current position, PX4 Home,
    homeValid, globalPositionValid, armed, inAir, and occurredAt;
  - aegisq/robot-register/{id} with mobilityClass DRONE;
  - retained aegisq/lwt/{id} offline payloads.
- MosquittoSession has initial registration, telemetry publication, abnormal
  LWT, and best-effort graceful offline publication wired in.

## Incomplete; do not enable or deploy as an active flight service

1. src/mqtt/mqtt_px4_control_main.cpp does not exist yet. It must load and
   strictly validate every runtime parameter, connect/discover PX4, perform
   restart recovery before MQTT subscription, start the coordinator, publish
   telemetry periodically, and supervise low battery while no route is active.
2. CMakeLists.txt still deliberately raises FATAL_ERROR when
   DC_BUILD_MQTT_PX4_AGENT=ON. Add the shared MAVSDK library/backend and
   mqtt_px4_control target, but keep the product default OFF until the required
   validation is complete.
3. The new dc_mqtt_state_codec source/test is not yet added to CMake.
4. The last executor edit uses std::any_of and std::find_if; add
   #include <algorithm> to src/flight/preemptive_mission_executor.cpp before
   compiling.
5. config/mqtt-px4-control.env.example needs all required bounded queue,
   SQLite, telemetry, PX4 tuning, route, freshness, Home-distance, and battery
   variables. The intended deployed value is LOW_BATTERY_RTL_PERCENT=30; keep
   REMOTE_FLIGHT_EXECUTE=0.
6. scripts/install.sh must install SQLite development headers when selected
   and install the MQTT environment/service only when the current build
   actually produced mqtt_px4_control. It must not enable or start that service
   automatically.
7. docs/mqtt-px4/ARCHITECTURE.md and the lower MQTT section of AGENTS.md still
   contain pre-implementation statements (old six-field schema,
   in-memory-only ledger, no STOP/RTL, no executor). Rewrite them to distinguish
   implemented code from unverified activation gates.
8. Invalid decoded commands with a recoverable canonical commandId are
   currently logged but do not yet enter a bounded rejection queue for durable
   REJECTED ACK publication. MQTT callbacks must remain decode/enqueue-only.
9. Review the new graceful-LWT PUBACK wait and registration publication under
   Linux/libmosquitto with -Wconversion -Werror.
10. Review and test MAVSDK v3 calls in the new backend on Linux, especially
    Telemetry::heading(), duration conversions, and all Action/Offboard
    results.

## Test state

- Before the newest MAVSDK backend, state codec, session-publication, and
  process-wide low-battery edits, an Android NDK strict diagnostic build
  compiled the hardware-independent targets and reported 20 passing test
  targets.
- That result is stale for the current working tree. No full build/test has
  been run after the latest edits.
- The present Windows host has Android SDK CMake/Ninja/NDK, but no native
  Windows C++ compiler, no installed WSL distribution, and no Docker. Android
  cross-builds can prove compilation but cannot execute ctest binaries on
  Windows.
- The Raspberry Pi was physically disconnected. No Pi-native build,
  WireGuard/broker round trip, PX4 SITL, propellers-off bench, or real-aircraft
  test has been performed for this WIP.

## Resume checklist

1. Read AGENTS.md, this file, and git status --short before editing.
2. Preserve unrelated user files CLAUDE.md and docs/TODO.md; do not include
   them in product commits.
3. Fix the known algorithm include, finish the executable/configuration, and
   connect all new targets in CMake.
4. Run a clean strict host build and every ctest suite. Then run a Linux/Pi
   build with libmosquitto, SQLite, nlohmann-json, and MAVSDK 3.17.1+.
5. Add broker-contract tests for QoS, retained rejection, exact topics,
   registration, telemetry, LWT, ACK replay, reconnect, and broker outage.
6. Run PX4 SITL cases for mission completion, STOP then RTL, direct RTL,
   emergency Land, stale/expired commands, duplicate/conflicting UUIDs,
   restart recovery, low battery at 30%, battery telemetry loss, and link loss.
7. When the Pi is reconnected, deploy to a new staging directory so its dirty
   existing checkout is not overwritten. Keep the service disabled and
   REMOTE_FLIGHT_EXECUTE=0 until broker ACL, external wg0 kill switch, SITL,
   bench, and approved vehicle checks pass.

## Safety notes

- EMERGENCY_STOP intentionally requests immediate Land; it does not terminate
  motors in air.
- libmosquitto queues inbound QoS 1 PUBACK before invoking the message callback.
  Durable local claiming prevents duplicate PX4 execution after the callback,
  but a small callback/crash ingress-loss window remains unless the MQTT client
  design is changed. Treat this as an activation decision, not as solved by the
  SQLite ledger.
- Never run riposte-agent.service and mqtt-px4-control.service together; only
  one process may own the PX4 command path.
