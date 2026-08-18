# Ground-only RC handover contract (2026-08-18)

This is the only supported ownership cycle for `mqtt-px4-control`:

```text
OnGround + DISARMED + CH6 HOLD
        -> server EXECUTE_MISSION
        -> Pi arms, takes off, primes and starts Offboard
        -> CH6 POSITION
        -> pilot owns the aircraft; the command ends FAILED/pilot_override_*
        -> pilot lands and uses CH5 DISARM
        -> CH6 HOLD remains stable through the debounce interval
        -> readyForMission=true; the next START begins a new mission from zero
```

There is no airborne resume and no saved waypoint checkpoint. Moving CH6 back
to HOLD while still armed or airborne never releases the pilot latch.
Any POSITION edge also increments a sticky local ownership epoch. The exact
START accepted before that edge remains invalid even if CH6 returns to HOLD
during a long preflight wait; only a later command can carry the new epoch.

## Why START requires HOLD, not OFFBOARD

The operator must not select OFFBOARD on the transmitter before START. The Pi
has to publish a fresh current-position setpoint stream before it asks PX4 to
enter Offboard; selecting the mode first can invoke PX4's Offboard-loss path
and on the field airframe prevents the navigation state needed by preflight.
Leave CH6 at HOLD. `readyForMission` is false in the physical OFFBOARD switch
position even though telemetry reports `controlOwner=PI`.

## Ownership evidence

`flightMode == POSCTL` is not ownership evidence: it can be either the RC mode
switch or PX4's Offboard-loss action. The agent instead consumes MAVLink
`RC_CHANNELS`, requests it at `PX4_RATE_RC_CHANNELS_HZ`, and combines it with
PX4 `RC_STATUS`. The same frame also proves that the configured CH5 arm switch
is physically in DISARM before a remote start.

- one fresh POSITION sample yields immediately to the pilot;
- HOLD/OFFBOARD needs at least two samples and the configured debounce time;
- stale RC, receiver unavailable, a dead-band PWM, or an unused channel is
  `UNKNOWN` and can never authorize START;
- PWM windows are inclusive, configurable, and must not overlap.
- CH5 DISARM/ARM uses a separate pair of debounced PWM windows; CH5 is only
  physical-switch evidence, while PX4 `armed=false` remains authoritative.

Here, "immediately" means that the RC callback atomically invalidates queued
START commands and that the mission loop yields at its next abort check. The
current MAVSDK layer still uses blocking Action/Offboard calls: a call already
in progress cannot be cancelled, and there remains a narrow interval between
the final ownership check and entering that call. A bounded asynchronous
command dispatcher is required before the stronger guarantee "no command is
initiated after the POSITION sample" can be claimed. Until then this contract
is source-level fail-closed intent, not propeller-on flight approval.

Before flight, read the `[RC]` journal lines with the propellers removed and
set the CH6 three-position and CH5 two-position PWM windows around the measured
values. Do not widen ranges until their edges touch.

## Session cleanup

Once a pilot takeover reaches the mission loop, the agent does not newly call
`Offboard::stop()`, `Action::hold()`, `land()`, `return_to_launch()`, or
`disarm()`. MAVSDK 3.17's Offboard plugin is destroyed after the watchdog
generation is invalidated and joined. Destruction removes MAVSDK's cached
20 Hz setpoint sender without sending the HOLD mode command that
`Offboard::stop()` would send. A blocking MAVSDK call which began just before
the RC sample remains the limitation described above.

The first prime setpoint is tracked separately from confirmed Offboard mode.
Every terminal, external-preemption, pilot-yield, and watchdog path destroys
that cached sender, including failure before `Offboard::start()` succeeds.
ARM, TAKEOFF, and Offboard-start are recorded as "may have applied" immediately
before their blocking calls so a lost ACK cannot select the no-action cleanup
path merely because the success response was absent.

The next ground START rechecks fresh CH6=HOLD and PX4
`connected && !armed && !in_air && OnGround`, then creates an empty Offboard
plugin and primes it from the new current position. No setpoint or watchdog
latch from the prior flight is reused.

The watchdog is generation-scoped. A timeout is first exposed to the ordinary
flight owner; only if that owner is still stalled after
`FLIGHT_SETPOINT_WATCHDOG_OWNER_GRACE_S` may the watchdog issue its serialized
Offboard-stop and Land fallback. A pilot/unknown owner destroys only the cached
sender and receives no PX4 mode/action command.

The takeover latch is not durable across a process restart. Consequently an
already armed or airborne vehicle is always owner-ambiguous at startup, even
when the current switch sample is HOLD. Restart recovery emits no Disarm, RTL,
Land, Hold, or Offboard-stop command in that state; PX4's configured
Offboard-loss action is the independent fallback.

## Telemetry/server contract

Every telemetry frame now includes:

```json
{
  "controlOwner": "PI | PILOT | UNKNOWN",
  "readyForMission": false,
  "hardware": {
    "ownershipSwitch": "HOLD | OFFBOARD | POSITION | UNKNOWN",
    "ownershipPwm": 1500,
    "armSwitch": "DISARM | ARM | UNKNOWN",
    "armSwitchPwm": 1000,
    "ownershipEpoch": 3
  }
}
```

`readyForMission=true` means PX4 is connected, the receiver and both switch
samples are fresh, CH6 is HOLD, CH5 is DISARM, the vehicle is explicitly
OnGround and disarmed, and no current watchdog fault exists.

The current AegisQ server and web client do not yet preserve or gate on these
fields. Until both are updated, the button state is not an authority signal;
the edge agent still rejects an unsafe START with
`mission_start_requires_fresh_rc_hold_and_disarm` or
`new_mission_requires_landed_disarmed_vehicle`.

The server must preserve `controlOwner` and `readyForMission` from every frame
in `robots.flight_state`, copy them to WebSocket frames, and require explicit
`armed=false`, `inAir=false`, `controlOwner=PI`, and `readyForMission=true` in
one reusable server-side flight-start gate. Missing values are UNKNOWN, not
false. That gate must cover scheduled/event/DroneOps/direct EXECUTE entry
points as well as the mission page.

The web client must consume the same fields, use tri-state armed/inAir values,
and disable every takeoff/mission/GOTO entry point when the proof is missing.
When ownership becomes PILOT or UNKNOWN, the server must discard or explicitly
cancel old queued automatic EXECUTE commands. The companion cannot distinguish
an old delayed command from the operator's new click after handback; the new
click must create a new commandId. STOP/RTL must not use the ground-start gate.

## Verification order

Keep `REMOTE_FLIGHT_EXECUTE=0` for ordinary observation. After installing the
newly built binary, perform these gates in order:

1. Unit/strict build: all CTest suites pass with `-Wconversion -Werror`.
2. SITL with an RC input source: run two complete START cycles and inspect mode,
   command ACK, and ULog transitions.
3. Propellers-off bench: calibrate CH5/CH6 PWM, then prove POSITION causes exactly
   one pilot terminal event and no later `offboard.stop`, Hold, Land, RTL, or
   Disarm from component 191.
4. Still on the bench, manually reach OnGround+DISARMED, switch to HOLD, wait
   for `readyForMission=true`, and execute a second fresh START.
5. Only after repeated passes should a propeller-on test be considered.

Capture the full journal without filters:

```bash
sudo journalctl -u mqtt-px4-control -f -o short-precise \
  | tee ~/mqtt-px4-full.log
```

Also retain the matching PX4 ULog topics: `input_rc`, `vehicle_status`,
`vehicle_control_mode`, `vehicle_land_detected`, `failsafe_flags`,
`vehicle_command`, and `vehicle_command_ack`.

## Plugin lifecycle invariant (fix 2026-08-18)

Every site that calls `offboard_.reset()` immediately follows it with
`offboard_ = std::make_unique<mavsdk::Offboard>(system_)`. This preserves
the invariant that `offboard_` is never null after construction, which
allows `riposte_test --repeat >1` and repeated AegisQ START cycles to use
the same `MavsdkOffboardControl` object without dereferencing a null pointer.

`prepare_for_new_ground_mission()` also recreates the plugin unconditionally.
When it runs after one of the yield/watchdog paths the double-creation is
harmless: the empty plugin from the yield path has no cached sender and is
simply replaced by another empty plugin.

Sites covered (as of this fix):
- `stop_offboard()` — prime-only and confirmed-started branches
- `cancel_for_external_preemption()`
- `yield_to_pilot()`
- `latch_watchdog_pilot_takeover()`
- watchdog timeout + land (both early-return and nominal paths)
