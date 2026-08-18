# 조종권 소유 문제 — 인계 문서

> **상태: 2026-08-18에 대체됨.** 이 파일은 2026-08-13 당시 결함과 조사 기록을
> 보존하는 역사 문서다. 아래 §7의 공중 resume 제안과 §8 운용 규칙을 현재 구현으로
> 사용하지 말 것. 현재 계약은
> [`GROUND-RC-HANDOVER-2026-08-18.md`](GROUND-RC-HANDOVER-2026-08-18.md)이며,
> 수동 인계 뒤에는 반드시 수동 착륙+PX4 DISARMED+CH5 DISARM+CH6 HOLD를 확인한
> 다음 **새 commandId의 새 미션**만 처음부터 시작한다.

**작성 2026-08-13.** 이 문서는 진행 중인 작업을 다른 에이전트에게 넘기기 위한 것이다.
결론부터: **`REMOTE_FLIGHT_EXECUTE=1` 로 프로펠러를 달고 날리면 안 된다.** 아래 §4 의
구멍들이 닫히기 전까지는 벤치(프로펠러 제거)에서만 시험한다.

---

## 1. 무엇을 하려는 것인가

조종사가 조종기로 기체를 가져갔을 때, Pi 가 **완전히 물러나야** 한다.

- 착륙·RTL·시동정지·Offboard 재진입·setpoint 전송을 **하지 않는다**
- 서버 명령(미션·STOP·RTL·E-STOP)을 전부 거부한다
- 조종사가 착륙·시동정지한 뒤에만 자율로 돌아온다

2026-08-13 오후 실기체에서 **이것이 실패했다.** 조종사가 POSCTL 로 날리는 중 미션이
도착했고, Pi 가 Offboard 를 뺏으려다 실패했고, setpoint 워치독이 만료됐고, 일반 오류
경로가 **`emergency_land_and_wait()` 로 기체를 착륙시켰다.** 프로펠러가 없어 사고는
아니었다.

---

## 2. 시스템 구조 (읽을 순서)

```
MQTT 브로커
   ↓
mqtt_px4_control_main.cpp        감독자 루프 (100 ms). 배터리·링크손실·지오펜스 감독
   ↓
PreemptiveMissionExecutor        PX4 를 만지는 유일한 스레드를 소유. 선점·안전큐
   ↓
MavsdkMissionBackend             미션 실행, 예외→안전동작, 래치 보유
   ↓
MavsdkOffboardControl            setpoint 생성, 워치독, PX4 직접 호출
   ↓
PX4 (MAVSDK 3.17.1, serial:///dev/ttyAMA0:921600)
```

**중요:** "MAVSDK 는 executor 워커 스레드만 호출한다" 는 불변식이 **실제로는 깨져 있다.**
setpoint 워치독 스레드가 `offboard_.stop()` 과 `action_.land()` 를 직접 부른다.

---

## 3. 지금까지 한 것 (전부 배포됨)

| 커밋 | 내용 |
|---|---|
| `bd9fc07` | `PilotOverrideDetected` 별도 예외 타입. 일반 예외는 전부 비상착륙으로 흐르므로 그보다 앞에서 잡아 **아무 동작도 하지 않는다**. `pilot_override_latched_` 래치. 재시작 복구도 조종사 모드면 RTL 안 함 |
| `ba4f436` | `offboard_intended_` 도입. 워치독이 `offboard_started_` 를 인라인으로 지워서 인계가 워치독 만료로 보고되던 것을 수정 |
| `4d55097` | `is_pilot_mode()` 로 한정. 그 전에는 **Offboard 아닌 모든 모드**를 인계로 판정해 정상 이륙 중 HOLD 구간에서 `pilot_override_Hold` 로 기체가 영구 래치됐다 |
| `e886cca` | 워치독의 `action_.land()` 와 `shutdown_safely()` 의 RTL 을 조종사 모드일 때 억제 |

**두 번의 수정이 현장에서만 잡혔다. 시험은 통과했다.** 이유는 §6.

---

## 4. Codex 적대적 검토 결과 (2026-08-13, gpt-5.6-sol max)

> **결론: 조종사가 기체를 가져간 뒤에도 Offboard·Hold·RTL·Land·Disarm 이 여전히 나갈 수
> 있다. `REMOTE_FLIGHT_EXECUTE=1` 을 비행에 안전하다고 볼 수 없다.**

### 4.1 치명 — 인계를 감지해도 워치독이 잠들지 않는다

1. 인계 감지 → 래치만 걸고 `return`. **워치독은 계속 돈다**
2. setpoint 생성이 멈추니 워치독 만료
3. 워치독이 **먼저 `offboard_.stop()` 을 보낸 뒤** 비행 모드를 검사한다
4. `Offboard::stop()` 자체가 Hold 로 바꾸는 명령이다
5. 다음 샘플이 Hold 로 읽히면 `is_pilot_mode(Hold)=false` → **`action_.land()` 발사**

즉 `e886cca` 의 가드가 **검사 순서 때문에 무력화된다.** 설령 캐시가 아직 Position 이라
Land 를 막더라도, 이미 조종사에게 `Offboard::stop()` 을 보낸 뒤다.

`shutdown_safely()` 의 조종사 분기도 **워치독을 멈추지 않고** 반환한다.

### 4.2 치명 — Offboard 진입 *전* 인계는 보이지 않는다

`offboard_intended_` 는 `offboard_.start()` **성공 후에** 선다. 준비·이륙·프라이밍 중에
조종사가 가져가면 `check_pilot_override()` 가 조기 반환한다. `start()` 가 실패하면 일반
예외 → 비상 착륙. **오후 사고와 같은 부류이고 창만 좁아졌다.**

### 4.3 원리적 — 비행 모드만으로는 소유권을 알 수 없다

- **Hold 가 목록에 없다.** 이 기체 조종기 SC 중간 자리가 Hold 다
- `COM_OBL_RC_ACT=Position` 이면 **PX4 의 Offboard 상실 failsafe 도 Position 으로 간다** —
  조종사가 가져간 것과 텔레메트리상 **구별 불가능**
- `COM_RC_OVERRIDE=3` 의 스틱 override 도 Position(또는 Altitude)으로 간다
- 항법 상실 failsafe 는 Altitude/Manual 로 갈 수 있다
- MAVSDK 가 모델링하지 않는 PX4 서브모드(Precision Land, VTOL Takeoff, External 1–8)는
  `FlightMode::Unknown` 으로 떨어지고, 현재 코드는 Unknown 을 비-조종사로 취급한다
- Rattitude 는 PX4 1.15 의 레거시/예약값이라 실질적으로 무의미

**따라서 `Telemetry::FlightMode` 기반의 올바른 `is_pilot_mode()` 집합은 존재하지 않는다.**

### 4.4 래치가 영영 안 풀리는 경우

해제 조건은 `!in_air && !armed` 인데, **프로펠러가 없으면 PX4 착륙판정기가 `in_air` 를
계속 참으로 보고할 수 있다.** 그러면 E-STOP 을 포함한 모든 명령이 무기한 거부된다.
재시작도 확실한 해결이 아니다(§4.5).

### 4.5 그 밖

- **재시작 시 래치 소실.** 부팅 복구가 항상 돈다. 안정된 Position/Manual 이면 다시 래치되지만,
  **Hold 나 Unknown 이면 복구 RTL 로 간다.** 지상+무장이면 복구 Disarm 으로 간다
- **공중 DISARM** 후 `in_air` 가 거짓/낡음이면 래치가 풀린다
- **수동 착륙 직후 auto-dispatch** 가 오면 그 명령이 래치를 풀고 그대로 실행한다.
  스위치 엣지·확인·유예·명시적 소유권 이전이 전혀 없다
- `pilot_in_control()` 이 **텔레메트리 예외 시 false 를 반환한다** — 증거가 없으면 권한을 준다
- **배터리 이상 경로가 `execute()` 를 다시 거치지 않고** RTL/비상착륙을 직접 부른다.
  래치 검사를 안 받는다 (`mavsdk_mission_backend.cpp` 의 telemetry 콜백 계열)
- `watchdog_triggered_` 는 **워치독 재시작 때만** 초기화된다. 다음 미션의 첫 프라이밍
  setpoint 가 낡은 트리거를 보고 일반 비상 처리로 갈 수 있다
- 지오펜스·링크손실 one-shot 플래그는 **큐잉 시점에** 서고 PX4 수락 시점이 아니다.
  래치로 거부되면 조건이 남아 있는데 재시도가 억제된다
- `arm()`, `takeoff()`, `hold()`, `disarm()`, 모든 RTL 경로에 같은 **검사-실행 사이 경합**이 있다
- `pilot_override_latched_` 는 평범한 bool 이다. 현재는 executor 워커 전용이라 데이터
  레이스는 아니지만, **워치독 스레드가 래치를 볼 수 없다는 구조적 문제**가 남는다
- `armed()` / `in_air()` / `flight_mode()` 는 **독립된 캐시 샘플**이지 일관된 스냅샷이 아니다

Codex 원문: `scratchpad/codex-fallback-out.md`

---

## 5. 별개의 미해결 증상 — Offboard 가 6초 만에 빠진다

미션을 시작하면 Offboard 에 들어갔다가 곧 튕긴다. **스틱을 중앙에 둬도 재현된다.**

```
18:15:05  HOLD → OFFBOARD      우리가 진입
18:15:11  OFFBOARD → POSCTL    6초 뒤
18:15:13  OFFBOARD → HOLD
```

우리 로그에 **오류 문구가 없다.** 즉 우리가 실패를 낸 게 아니라 PX4 가 걷어냈다.

**유력 가설:** 우리 setpoint 스트림에 구멍이 생겨 PX4 가 Offboard 신호 상실로 판정하고
`COM_OBL_RC_ACT=Position` 을 적용한다. 스틱과 무관하다.

**확인해야 할 것:**
- QGC 에서 `COM_OF_LOSS_T` 값 (기본 0.5~1초)
- 해당 시도가 AegisQ START 로 시작한 미션이었는지, 스위치만 돌린 것인지
  (미션 없이 스위치만 Offboard 로 올리면 setpoint 가 없으니 PX4 가 거부하는 게 정상)
- `FLIGHT_SETPOINT_RATE_HZ=20`, `FLIGHT_SETPOINT_WATCHDOG_TIMEOUT_S=0.30` 인데
  20 Hz 루프가 300 ms 를 놓치는 이유. Pi 는 영상 파이프라인도 돌린다(load 1.3)

이것을 먼저 풀지 않으면 미션 자체가 못 돌아 인계 시험을 할 수 없다.

---

## 6. 시험이 왜 못 잡았는가 — 반복하지 말 것

**MAVSDK 계층에는 단위시험이 없다.** 모의 객체가 없어 `MavsdkMissionBackend` /
`MavsdkOffboardControl` 은 SITL 로만 검증된다. ctest 12/12 통과는 이 영역에 대해
아무것도 증명하지 않는다.

**SITL 이 버그를 확인해 준 사례가 있다.** RC 없는 SITL 에서 Position 은 PX4 failsafe
착륙을 부르므로, 인계를 흉내 내려고 `auto:loiter` 를 썼다. `auto:loiter` 가 곧 HOLD 이고,
그게 **절대 인계로 세면 안 되는 값**이었다. 코드가 틀린 방향과 시험이 틀린 방향이 같아
통과했다.

**제대로 된 SITL 시험에는 모의 조종기가 필요하다.** MAVSDK `ManualControl` 플러그인으로
manual control 입력을 넣으면 PX4 가 Position 을 유지하고 RC 상실 failsafe 가 돌지 않는다.
그것 없이 만든 인계 시나리오는 신뢰할 수 없다.

---

## 7. 제안하는 해결 방향

**비행 모드로 소유권을 추론하는 것을 그만둔다.** §4.3 때문에 원리적으로 불가능하다.

대신 **MAVLink `RC_CHANNELS` 를 구독해 CH6(SC 스위치) 원시값을 읽는다.** 스위치 위치는
PX4 failsafe 와 절대 혼동되지 않는다.

목표 상태:

```
SC=Offboard  →  Pi 주행
   ↓ SC→Position
Pi 즉시 정지 + 현재 waypoint 저장 + 워치독 정지 + Land/RTL 억제
   ↓ SC→Offboard
현재 위치 고정 setpoint 를 20 Hz 로 1초 이상 선행 전송
   ↓
offboard.start() → 실제 모드가 OFFBOARD 인지 확인
   ↓
현재 위치에서 남은 경로 재생성 후 재개
```

필요한 것:

1. **RC 채널 구독** — CH6 실제 위치 읽기. MAVSDK 에 직접 API 가 없으면 MAVLink passthrough
2. **소유권 상태머신** — `PI` / `PILOT` 을 스위치 엣지로 전이. 모드는 보조 증거로만
3. **워치독 정지를 인계 처리에 포함** — §4.1. 래치를 거는 순간 워치독도 멈춘다.
   그리고 워치독의 모드 검사를 `offboard_.stop()` **앞으로** 옮긴다
4. **진입 전 구간 보호** — §4.2. `offboard_intended_` 를 `start()` 시도 **전에** 세운다
5. **텔레메트리 예외 시 보수적으로** — §4.6. 증거가 없으면 권한을 주지 않는다
6. **배터리 직접 경로도 래치를 보게** — §4.5
7. **서버에 `controlOwner` 보고** — `PILOT` / `PI`, 그리고 `missionState=PAUSED_MANUAL`.
   서버가 디스패치를 보류할 근거가 된다 (서버팀과 계약 합의 필요)
8. **모의 조종기 SITL 시나리오** — §6

---

## 8. 현재 기체 상태 (2026-08-13 18:20 기준)

```
Pi           192.168.0.6 (동글, SkyAP_5G), wg0 10.100.0.13
서비스        mqtt-px4-control active / aegisq-agent inactive / riposte-agent inactive
바이너리      /usr/local/bin/mqtt_px4_control  (17:53 배포, e886cca 상당)
게이트        REMOTE_FLIGHT_EXECUTE=1        ← 벤치 시험용. 비행 전 재검토 필요
안전값        GEOFENCE_RADIUS_M=50 · FLIGHT_MAX_ROUTE_RADIUS_M=60
             MQTT_KEEPALIVE_SEC=10 · MQTT_LINK_LOSS_TIMEOUT_S=5 (실측 20.3초)
setpoint     FLIGHT_SETPOINT_RATE_HZ=20 · WATCHDOG_TIMEOUT_S=0.30
PX4          COM_RC_OVERRIDE=3 · COM_RC_STICK_OV=30% · COM_OBL_RC_ACT=Position
조종기        SC 3단 — Offboard / Hold / Position. ARM 은 별도 스위치(CH5)
프로펠러      제거됨
```

**운용 규칙(현재 코드 기준):**

```
지상·DISARM·SC=Hold → AegisQ START → Pi 자동 ARM/Takeoff/OFFBOARD
  → SC=Position 으로 수동 개입 → 조종자 착륙 → DISARM → 다음 START
```

- **ARM 스위치를 조종권 전환에 쓰지 말 것.** Manual/Stabilized 계열에서 비행 중 DISARM 은
  모터 정지다
- **공중에서 SC 를 Offboard 로 되돌려 미션을 재개하는 시험은 하지 말 것.** 래치가 지상+
  DISARM 에서만 풀리므로 재개되지 않고, 그 상태의 거동은 검증된 바 없다
- **수동조종 중 Pi 서비스를 재시작·종료하지 말 것** (§4.5). 코드로 막는 것이 목표이지
  사람이 기억할 규칙으로 두면 안 된다

---

## 9. 서버 쪽 미해결

- **`auto-dispatch` 봉투가 전부 거부된다.** 루트 키에 `eventId`·`location` 이 있고
  `waypoints` 가 없다. `location` 은 객체가 아니라 JSON 문자열. 몇 분 간격으로 재시도 중
- 서버는 `hardware.flightMode` 를 1 Hz 로 이미 받고 있다. 디스패치 게이트에 쓸 수 있다
- **수동 인계 후 재이륙 위험** — 조종사 착륙·DISARM 으로 래치가 풀린 직후 대기 중이던
  auto-dispatch 가 도착하면 기체가 스스로 무장·이륙한다. 서버 보류 또는 명시적 재허용
  둘 중 하나를 정해야 한다

서버팀에 보낼 초안: `scratchpad/reply-autodispatch-ko.md`

---

## 10. 관련 파일

```
src/riposte/mavsdk_offboard_control.{hpp,cpp}   setpoint·워치독·PX4 직접 호출
src/riposte/mavsdk_mission_backend.{hpp,cpp}    미션 실행·래치·예외 처리
src/flight/preemptive_mission_executor.cpp      선점·안전큐·우선순위
src/mqtt/mqtt_px4_control_main.cpp              감독자 루프·지오펜스·링크손실
include/dc/flight/telemetry.hpp                 스냅샷·horizontal_distance_m
config/mqtt-px4-control.env.example             전 설정값과 근거
```

로그 확인:

```bash
sudo journalctl -u mqtt-px4-control -b --no-pager | \
  grep -E "flight mode:|STATE|override|geofence|link lost|safety fault"
```

`[STATE]` 는 무장 또는 공중일 때만 1 Hz 로 찍히고 지상에서는 조용하다.
journald 는 영속이므로 전원을 내려도 남는다.
