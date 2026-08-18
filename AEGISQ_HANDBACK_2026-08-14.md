# 조종사 지휘권 이양/반환 수정 (2026-08-14)

## 보고된 증상

- aegisq 미션 **시작 전에** 조종기로 권한이 넘어가 START 버튼이 비활성
- 조종기로 권한이 넘어간 뒤, **미션 도중이나 미션 후에** 다시 aegisq/Pi로 권한이 돌아오지 않음
- 조종기 CH6에 `OFFBOARD` / `HOLD` / `POSITION` 3단 설정

## 원인

권한 반환 판정이 **서버와 엣지 양쪽에서 착륙을 요구**하고 있었습니다.

| 계층 | 기존 해제 조건 | 결과 |
|---|---|---|
| 엣지(companion) | 지상 + disarm **에서만** latch 해제 | 공중에서 OFFBOARD로 돌려도 재개 불가 |
| 서버 | 마지막 조종 프레임으로부터 **300초 경과** | OFFBOARD로 돌려놔도 5분간 START 차단 |

그래서 다음 흐름이 성립하지 않았습니다:

```
조종기로 수동 이륙(POSCTL, armed)  →  CH6를 OFFBOARD로 전환  →  aegisq 미션 START
                                        ↑ 여기서 이양한 셈인데            ↑ 양쪽이 모두 거부
```

`POSCTL`로 arm하는 순간 조종사 우선으로 잡히는 것 자체는 **정상 동작**입니다(2026-08-13 사고 방지).
문제는 그 뒤에 **되돌릴 방법이 시간 경과와 착륙뿐**이었다는 점입니다.

## 수정

**스위치를 자율 모드로 되돌리는 것을 명시적 이양 신호로 인정**합니다. 스틱 중앙 복귀와는 다릅니다 —
스틱은 POSCTL 안에서도 중앙일 수 있지만, 모드 전환은 "내 입력을 제어원으로 쓰지 말라"고 PX4에
직접 지시하는 행위입니다.

### 엣지 — `src/riposte/mavsdk_mission_backend.cpp`

```cpp
const bool landed_and_disarmed = !flight_control_.in_air() && !flight_control_.armed();
// 공중일 때만 이양으로 인정한다 — 지상에서 스위치가 자율 위치인 것은 이양이 아니라
// 그냥 스위치 위치이고, 그때 풀면 조종사 옆에서 자동 이륙하는 사고가 그대로 돌아온다.
const bool handed_back = flight_control_.in_air() && !flight_control_.pilot_in_control();
if (!landed_and_disarmed && !handed_back) {
    return {ExecutionOutcome::failed, "pilot_override_active"};
}
pilot_override_latched_ = false;
```

`pilot_in_control()`은 이미 **"조종사 모드 AND (armed 또는 공중)"** 을 뜻하므로, 사람이 실제로
조종 중이면 여전히 false가 되지 않습니다 — 기존 방어가 약해지지 않습니다.

### 서버 — `PilotOverrideGuard` (배포 완료, 커밋 `52d7eb6`)

보고된 `hardware.flightMode`가 자율/보조 모드면 300초 유예를 **즉시 종료**합니다.
`flightMode`를 싣지 않는 프레임은 기존대로 시간 유예를 지킵니다(모르는 것을 "반환됐다"로
읽으면 게이트가 조용히 무력해지므로).

**양쪽이 같은 규칙을 써야 합니다.** 한쪽만 해제하면 한 계층은 명령을 받고 다른 계층은 거부해
증상이 더 헷갈립니다.

## 수정 후 기대 동작

| 상황 | 결과 |
|---|---|
| 지상 대기, CH6=POSITION, disarm | 미션 START **가능** (기존과 동일) |
| 조종기로 수동 비행 중(POSCTL, armed) | 자율 명령 **거부** (기존과 동일 — 의도된 안전장치) |
| **공중에서** CH6를 OFFBOARD로 전환 | **즉시** aegisq 지휘권 복귀 ← **이번 수정** |
| 조종사가 착륙·disarm | 즉시 복귀 (기존과 동일) |
| 착륙 후 CH6가 POSITION이든 OFFBOARD든 | 300초 유예 유지 — 조종사가 기체 옆에 있다 |

## 확인 부탁드립니다

1. **`flightMode` 문자열 형식** — 서버는 `MANUAL`/`POSCTL`/`ALTCTL`/`ACRO`/`STABILIZED`/`RATTITUDE`
   와 별칭 `POSITION`/`ALTITUDE`/`STABILIZE`를 조종사 모드로 봅니다. 그 외(`OFFBOARD`, `HOLD`,
   `AUTO.*`)는 자율로 판정합니다. 기체가 이와 다른 표기를 보내면 알려주십시오 — 서버 목록에
   추가해야 합니다. (현재 DRONE-01은 `POSCTL`로 올라오고 있어 정상입니다.)

2. **CH6 `HOLD` 위치** — `HOLD`는 자율 모드로 분류됩니다. 즉 HOLD로 두면 aegisq가 지휘권을
   가져갈 수 있습니다. 의도와 맞는지 확인 부탁드립니다. 조종사가 HOLD를 "내가 잡고 있는 상태"로
   쓰신다면 서버·엣지 양쪽 목록을 함께 바꿔야 합니다.

## 검증 상태

- **전체 빌드·테스트 통과** — MAVSDK 3.17.1 + cairo/mosquitto를 개발 PC에 설치해
  `-DDC_BUILD_MQTT_PX4_AGENT=ON` 으로 구성, 실행 파일 `mqtt_px4_control` 까지 링크 성공(rc=0).
  `dc_mavsdk_flight` 에 `mavsdk_mission_backend.cpp.o` 가 생성되고 `MavsdkMissionBackend::execute`
  심볼이 들어간 것을 확인했습니다. ctest **12/12 통과**.
- **안전 재리뷰 반영(2차)** — 첫 수정은 모드만 보고 유예를 풀어 **지상에서도** 반환을 인정했는데,
  그러면 조종사가 내려놓은 기체 옆에서 자동 이륙하는 경로가 열립니다. 공중 조건을 추가했습니다.
- **실기 검증 미완** — 조종기 CH6를 실제로 전환하며 다음 두 가지를 확인해 주십시오:
  ① 수동 비행 중 OFFBOARD 전환 → 미션 재개 가능,
  ② 수동 비행 중(POSCTL, armed) → 자율 명령이 여전히 거부되는지(안전장치 유지).

---

# 추가 발견 — 엣지가 해석하지 않는 명령 (같은 날 코드 리뷰)

## 무엇을 찾았나

companion이 실제로 파싱하는 로봇-링크 동사는 **네 개뿐**입니다
(`RemoteCommandType` = execute_mission / stop_mission / return_to_launch / emergency_stop):

```
EXECUTE_MISSION · STOP_MISSION · RTL · EMERGENCY_STOP
```

그런데 서버는 이보다 넓은 동사 집합을 발행하고 있었습니다. 특히 콘솔의
**ARM · DISARM · HOLD · START** 는 모두 `STANDBY` 로 매핑되는데, companion 에는
`STANDBY` 를 명령으로 받는 코드가 없습니다(문자열은 텔레메트리 `status` 값으로만 쓰입니다).

## 왜 위험한가

서버는 매핑 후 **MQTT 발행에 성공하면 곧바로 ACK** 를 돌려줍니다. 그래서 이 명령들은
콘솔에 **성공으로 표시**됐습니다. 운영자가 DISARM 을 눌러 "모터를 껐다"고 믿는데
기체는 무장 상태 그대로인 것이 이 결함의 실제 위험입니다.

## 서버 측 조치 (배포 완료)

- 지원 목록에 없는 동사는 **발행하지 않고** `EDGE_UNSUPPORTED` 로 거절합니다.
  보내지 않으므로 기체가 거부할 것도 없고, 콘솔은 "아무 일도 일어나지 않았다"를 정확히 압니다.
- 목록은 설정값 `aegisq.flight.edge-verbs` 입니다(기본값이 위 네 개).
  **companion 이 `STANDBY` 를 구현하면 이 설정에 추가하면 열립니다** — 서버 재배포가 필요 없습니다.
- 콘솔은 미지원 명령 버튼을 취소선 + 비활성으로 표시합니다.

실서버 검증: `ARM/DISARM/HOLD/START → NACK EDGE_UNSUPPORTED`,
`TAKEOFF/RTL/STOP_MISSION/EMERGENCY_STOP → ACK`.

## 드론팀에 요청드리는 판단

`STANDBY`(= arm / disarm / hold)를 companion 에 구현하실 계획인지 알려주십시오.

- **구현하신다면**: 서버 설정에 `STANDBY` 를 추가해 열겠습니다. 다만 그 시점부터
  **비행 중 DISARM 이 실제로 작동**하게 되므로, 서버에 이미 넣어 둔
  `AIRBORNE_DISARM_BLOCKED`(공중에서 DISARM 거절)가 그때부터 실효를 갖습니다.
- **구현하지 않으신다면**: 현행 유지가 맞습니다. 콘솔에서 해당 버튼은 계속 비활성입니다.

또한 `RESUME` · `REBOOT` · `GO_HOME` · `SIREN` · `LIGHTS` 도 같은 상태입니다(서버 매핑에는
있으나 엣지 미구현). 이 중 실제로 쓰실 것이 있으면 함께 알려주시면 우선순위를 맞추겠습니다.

---

# 추가 — 능력 선언(caps) 발행 구현 (§5.8, 같은 날 2차)

서버가 이제 `aegisq/caps/<robotId>` (retained) 선언을 명령 필터의 1순위 근거로 씁니다.
이 companion 에 선언 발행을 구현했습니다:

- `mission_codec.cpp` 의 `encode_capabilities()` — **명령 파서 바로 옆**에 있습니다.
  파서에 동사를 추가하면 같은 파일의 이 함수도 함께 고쳐야 하며, 그것이 이 배치의 목적입니다.
- `mosquitto_session.cpp` — 구독 성사 시 registration 직후 retained(QoS 1)로 발행합니다.
- 선언 내용: EXECUTE_MISSION(waypoints) / STOP_MISSION / RTL / EMERGENCY_STOP,
  ack status = ACCEPTED·FAILED·REJECTED·COMPLETED. version 은 registration.firmware 를 씁니다.

서버 검증 완료(실페이로드): 저장 → `supportedVerbs` 노출 → 선언 기반 필터(TAKEOFF 통과,
ARM 거절) → 빈 retained 로 삭제 시 DB NULL 복귀.

**앞으로 STANDBY(arm/disarm/hold)를 구현하실 때**: 파서에 동사를 추가하고
`encode_capabilities()` 에 같은 동사를 추가하면 끝입니다 — 서버·콘솔은 재배포 없이
선언을 따라갑니다.


---

# 최종 정리 — 서버측 완비 사항과 남은 확인 (같은 날 3차)

## 서버에 배포 완료된 것 (드론팀 작업 불필요)

- **ACL**: `aegisq/caps/%u` 쓰기 권한이 mTLS ACL 에 추가·리로드됐습니다. **이전에는 이 권한이
  없어 caps 발행이 조용히 거부될 상태였습니다** — 이 zip 의 caps 발행 코드는 이제 실기체
  mTLS 경로에서도 동작합니다.
- **CapabilityGate**: 선언에 없는 동사는 서버가 모든 경로(콘솔 ICD·직접 명령·이벤트 출동·
  자동 출동)에서 발행 전에 거절합니다. 단 **STOP_MISSION / RTL / EMERGENCY_STOP 은 선언과
  무관하게 항상 발행**됩니다 — 선언이 잘못돼도 복구 수단은 유지됩니다.
- **콘솔 ACK 분류**: COMPLETED 를 성공으로 인식합니다(이전엔 "거부"로 표시될 결함이 있었음).

## 이 zip 에 담긴 companion 변경 요약

1. **지휘권 반환**: latch 해제 = (지상+disarm) **또는** (공중+비조종사모드) — `mavsdk_mission_backend.cpp`
2. **caps 발행**: 구독 성사 시 `aegisq/caps/<id>` retained — `mosquitto_session.cpp`
3. **선언 인코더**: 파서 옆 `encode_capabilities()` + version 문자열 JSON 이스케이프 — `mission_codec.cpp`

빌드·테스트: MAVSDK 3.17.1 로 전체 빌드(rc=0), ctest 12/12, 선언 JSON 유효성 확인.

## 실기 확인 요청 (우선순위 순)

1. 기체 기동 후 서버 `robots.capabilities` 가 채워지는지 (mTLS 경로 첫 실검증)
2. 수동 비행 중 CH6 → OFFBOARD 전환 시 미션 재개
3. 수동 비행 중(POSCTL, armed) 자율 명령 거부 유지
4. CH6 의 HOLD 위치 의미 회신 (자율로 분류 중 — 조종사 보류용이면 알려주십시오)
