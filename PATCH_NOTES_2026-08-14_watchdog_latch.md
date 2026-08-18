# 패치 노트 — 워치독 래치 누수 + 조종사 양보 수정 (2026-08-14/15)

> 2026-08-15 갱신: 수정 2(아래) 추가. 벤치 시험(props-off, 15:18 KST) 로그 분석 반영.

기준본: `aegisq_drone_companion_handback_20260814.zip` (2026-08-14 handback).
이 트리는 기준본에 아래 수정 하나만 더한 것이다.

## 증상 (2026-08-14 현장)

미션이 한 번 실행된 뒤 두 번째 EXECUTE_MISSION 부터는 ACCEPTED 직후 **7~55ms 만에**
`setpoint_generation_watchdog_expired` 로 FAILED 가 반복됐다. 브라우저 새로고침·새
commandId 로도 재발했고, 에이전트 프로세스 재시작으로만 풀렸다.

서버 로그 근거 (sky-black, aegisq-backend journal):

```
13:57:40.288 Command ACK: DRONE-01 ... status=ACCEPTED
13:57:40.295 Command ACK: DRONE-01 ... status=FAILED error=setpoint_generation_watchdog_expired
(13:57:50 / 13:57:58 / 13:58:08 동일 반복)
```

진짜 워치독 만료는 setpoint 스트림이 수 초 끊겨야 발생한다. 수락 후 수 ms 만의 만료는
**이전 비행이 남긴 래치를 새 미션이 읽고 즉사**한 것이다.

## 원인

- `watchdog_triggered_` 는 워치독 발화 시 true 로 래치된다
  (`mavsdk_offboard_control.cpp`, 워치독 스레드).
- 이 플래그를 false 로 되돌리는 곳은 `start_setpoint_watchdog()` 안 단 한 곳인데,
  이 함수는 `prime_and_start_offboard()` 에서 offboard 시작이 **성공한 뒤**에야 불린다.
- 새 미션의 `takeoff_to()` 는 가장 먼저 `check_abort()` 를 부르고, `check_abort()` 는
  래치가 남아 있으면 즉시 `setpoint_generation_watchdog_expired` 를 던진다.
  → 리셋 지점에 도달하기 전에 죽는다. 한 번 발화하면 프로세스 재시작 전까지 전 미션 실패.

## 수정 (파일 3개)

1. `src/riposte/mavsdk_offboard_control.hpp`
   — public `void reset_watchdog_latch() noexcept;` 선언 추가.
2. `src/riposte/mavsdk_offboard_control.cpp`
   — 구현 추가: `watchdog_running_` 이 true 면(=작전 진행 중, 래치가 살아있는 중단
   신호일 수 있음) **아무것도 하지 않고**, 비가동일 때만 `watchdog_triggered_` 를 클리어.
3. `src/riposte/mavsdk_mission_backend.cpp`
   — `execute()` 의 명령별 플래그 리셋 자리(`internal_failsafe_` 등 초기화하는 곳)에서
   `flight_control_.reset_watchdog_latch()` 호출. "새 명령 = 새 작전" 시점에만 이전
   실행의 잔재를 청소한다. 조종사 우선 게이트(래치·현재 조종 판정) **뒤**에서 호출되므로
   조종사 관련 안전 판정에는 영향이 없다.

비행 중 안전성: 워치독 스레드가 살아 있는 동안에는 리셋이 거부되므로, 진행 중인 작전이
자기 중단 신호를 잃는 경로는 구조적으로 없다.

## 검증 상태

- MAVSDK 3.17.2 링크, `-Wall -Wextra -Wpedantic -Wconversion -Werror` 로 clean 빌드
  (`dc_mavsdk_flight` + `riposte_test` + 전체 활성 타깃).
- 단위시험 6종 전부 통과 (MAVSDK 무관 시험들 — 이 수정 경로의 직접 시험은 아님).
- **실기/SITL 검증은 미완.** 권장 재현 시험: 프로펠러 제거 벤치에서
  ① 미션 1회 실행 중 워치독을 의도적으로 발화(setpoint 스트림 차단 등)
  ② 착륙·disarm 후 두 번째 미션이 정상 수락·진행되는지 확인.

## 수정 2 — 워치독의 조종사 양보가 래치·모드 탈취를 남기던 문제 (2026-08-15)

### 증상 (2026-08-14 벤치, props-off)

조종사가 POSITION 으로 개입한 직후:

```
15:18:15.795 setpoint_generation_watchdog_expired   ← 개입인데 결함으로 래치됨
15:18:15.806 watchdog_yielded_to_pilot
15:18:16     OFFBOARD -> POSCTL                     ← 조종사 모드
15:18:17     POSCTL -> HOLD                         ← 워치독의 offboard.stop() 이 모드를 빼앗음
(PX4: "Received ack for not-existing command: 176" — DO_SET_MODE 경합)
```

이후 STOP 두 건이 남은 래치에 걸려 즉시 실패하고 비상착륙 경로로 흘렀다
(landing_confirmation_timed_out — props-off 라 in_air 가 안 풀리는 벤치 고유 현상 포함).

### 원인

워치독 스레드가 만료 시 (1) `watchdog_triggered_` 래치 → (2) `offboard_.stop()` →
(3) 그제서야 조종사 모드 확인 순서로 동작했다. 조종사 개입으로 setpoint 가 끊긴 경우
(의도된 결과)에도 결함이 래치되고, `offboard_.stop()` 이 PX4 를 Hold 로 강제 전환해
**조종사가 잡은 모드를 1초 만에 빼앗았다.**

### 수정 (`mavsdk_offboard_control.cpp` 워치독 스레드)

만료 판정 직후 **가장 먼저** 조종사 모드를 확인한다:
- 조종사 모드면: 래치하지 않고, PX4 에 아무 명령도 보내지 않고(`offboard.stop()` 포함),
  `watchdog_yielded_to_pilot` 만 보고하고 물러난다.
- 조종사 모드가 아니면: 기존과 동일(래치 → offboard 중단 → 착륙) — 이 스레드의 존재 이유인
  "미션 스레드 사망" 시나리오는 그대로 방어된다.
- 모드를 읽지 못하면 결함 경로로 간다 — 모름을 "조종사"로 읽으면 진짜 고장에서 착륙을 못 한다.

### 서버 측 동반 수정 (AegisQ_Backend, 같은 날)

- `PilotOverrideGuard`: **운영자가 직접 누른** 명령(미션 START·ICD·이벤트 출동·직접 명령)은
  기체가 `armed=false ∧ inAir=false` 를 **명시 보고** 중이면 300초 유예를 적용하지 않는다.
  자동 출동(AutoDispatchService)은 종전대로 300초 유지. 엣지의 landed_and_disarmed 해제
  조건과 정합.
- 미션 preflight: `armed=true`/`inAir=true` 명시 보고 시 409(`VEHICLE_STILL_ARMED`/
  `VEHICLE_AIRBORNE`) — 엣지의 `new_mission_requires_landed_disarmed_vehicle` 을 서버가
  미리 비춘다.

## 알려진 한계 / 후속 제안

- 워치독 스레드가 `offboard.stop()`/`land()` 를 직접 호출하는 구조와 세션 구분은
  그대로다. 미션/Offboard 세션마다 generation 카운터를 부여하면 스레드 잔존 경합까지
  구조적으로 차단할 수 있다(이번 수정과 목적 동일, 더 견고한 형태).
- 제어권 소유(controlOwner)·readyForMission 텔레메트리, 지상 명시 권한 반환 신호는
  별도 과제다 — 서버(PILOT_OVERRIDE_HOLD 300s)와 함께 설계해야 한다.
