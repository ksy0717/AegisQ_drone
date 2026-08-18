# Ground Station 원격 시험 실행 (riposte_gcs)

Ground Station(지상국) PC 에서 **텔레메트리(MAVLink) 링크로** companion 컴퓨터의
시험 앱에 Test 1~23 실행을 요청하고, 진행 상황과 최종 PASS/FAIL 을 받아 보는
방법을 설명합니다.

```
┌─────────────┐  무전기/UDP   ┌──────────────┐  TELEM2/UDP  ┌───────────────────────┐
│ riposte_gcs │◄────────────►│  PX4 (기체)  │◄───────────►│ riposte_test --serve  │
│  (지상국)   │   MAVLink    │ MAVLink 라우터│   MAVLink   │ (companion, RPi 등)   │
└─────────────┘              └──────────────┘              └───────────────────────┘
     요청 →        COMMAND_LONG (MAV_CMD_USER_1, 컴포넌트 191 지정)        → 수신
     수신 ←        COMMAND_ACK / STATUSTEXT "RT|..." (진행·결과)           ← 송신
```

- **비행 제어는 전부 companion 쪽**에서 합니다. 지상국은 요청과 수신만 합니다.
  기존 앱의 안전 계층(setpoint watchdog, 소프트웨어 반경 제한, 배터리 3단
  게이트, 비상 착륙)은 원격 실행에서도 그대로 동작하며, **텔레메트리 링크가
  끊겨도** companion-PX4 링크가 살아 있는 한 시험은 안전하게 계속됩니다.
- 표준 MAVLink 메시지만 사용하므로 QGroundControl 과 같은 링크를 공유해도
  충돌하지 않습니다.

---

## 1. 구성 요소

| 프로그램 | 실행 위치 | 역할 |
|---------|----------|------|
| `riposte_test --serve --execute` | companion (RPi) | 요청 대기 → 시험 실행 → 결과 송신. "에이전트" |
| `riposte_gcs` | 지상국 PC | 시험 요청, 진행 모니터, 중단 명령 |

두 바이너리는 같은 CMake 빌드에서 함께 생성됩니다.

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
# -> build/riposte_test, build/riposte_gcs
```

MAVSDK 가 별도 위치에 있으면 `-DCMAKE_PREFIX_PATH=/opt/mavsdk` 를 추가하십시오.

---

## 2. 프로토콜 명세

프로토콜 정의는 `include/gcs_protocol.hpp` 한 곳에 있고
`tests/test_gcs_protocol.cpp` 가 고정합니다.

### 2.1 시험 요청 (GCS → 에이전트)

`COMMAND_LONG`, `command = MAV_CMD_USER_1 (31010)`,
`target_component = 191 (MAV_COMP_ID_ONBOARD_COMPUTER)`

| 필드 | 의미 |
|------|------|
| param1 | 시험 번호 (1~23) |
| param2 | 반복 횟수 덮어쓰기 (0 = 카탈로그 값) |
| param3 | 안전 매직 코드 `7305` |

param3 매직 코드는 **다른 GCS 소프트웨어가 MAV_CMD_USER_1 을 다른 용도로 쓸 때
시험이 오발진하는 것을 막는 장치**입니다. 보안 수단이 아니므로 링크 자체의
보안(무전기 페어링, 넷 아이디)은 별도로 관리하십시오.

### 2.2 응답 (에이전트 → GCS)

`COMMAND_ACK` (컴포넌트 191 발신):

| result | 의미 | GCS 동작 |
|--------|------|---------|
| `ACCEPTED` | 요청 수락, 시험 시작 | 결과 대기 |
| `TEMPORARILY_REJECTED` | 다른 시험 실행 중 (busy) | 종료 코드 1 |
| `DENIED` | 잘못된 요청 (번호/매직/반복 범위) | 종료 코드 1 |

### 2.3 진행/결과 (에이전트 → GCS, STATUSTEXT)

모든 라인은 `RT|` 접두사와 49자 제한을 지킵니다.

| 라인 | 의미 |
|------|------|
| `RT\|READY` | 요청 대기 중 |
| `RT\|START\|13\|x10` | Test 13 을 10회 반복으로 시작 |
| `RT\|IT\|13\|2/10\|PASS` | 2회차 사이클 종료, PASS |
| `RT\|RESULT\|13\|PASS\|10/10` | 시험 전체 종료 (GCS 는 이 라인으로 종료 코드 결정) |
| `RT\|ABORTED\|13` | GCS 중단 요청으로 착륙 완료, 계속 대기 (`RT\|ABORTED\|0` = 유휴 중이라 중단할 것 없음) |
| `RT\|STOPPED` | companion 쪽 운영자가 에이전트를 중지 (오류 아님, 시험은 미완) |
| `RT\|ERROR\|<사유>` | 오류로 에이전트 종료. **사람 점검 전 재요청 금지** |

### 2.4 중단 (GCS → 에이전트)

`COMMAND_LONG`, `command = MAV_CMD_USER_2 (31011)`, `param1 = 7305`.
진행 중 시험만 착륙시키고 에이전트는 계속 대기합니다. `riposte_gcs` 에서
**Ctrl+C 를 눌러도 자동으로 이 중단 명령을 보냅니다.**

`riposte_gcs` 는 ACK 수신 후 **착륙 완료 확인(`RT|ABORTED`)을 최대 30초**
기다립니다. 확인이 오지 않아도 착륙 자체는 companion 이 링크와 무관하게
수행하므로, RC 조종사 대기 상태에서 `--monitor` 로 계속 관찰하십시오.

---

## 3. PX4 설정 (실기체)

GCS ↔ 에이전트 메시지는 PX4 를 **경유**하므로 MAVLink 포워딩이 필요합니다.
QGroundControl 파라미터 화면에서 확인하십시오.

| 파라미터 | 값 | 의미 |
|----------|----|------|
| `MAV_0_CONFIG` | TELEM1 | GCS 무전기 링크 (보통 기본값) |
| `MAV_0_FORWARD` | 1 | GCS 링크로 들어온 메시지를 다른 링크로 전달 |
| `MAV_1_CONFIG` | TELEM2 | companion 링크 |
| `MAV_1_MODE` | Onboard | companion 용 스트림 구성 |
| `MAV_1_FORWARD` | 1 | companion 링크 수신 메시지를 다른 링크로 전달 |
| `SER_TEL2_BAUD` | 921600 | companion UART 속도 (기존 구성 그대로) |

파라미터 변경 후 재부팅해야 적용됩니다.

> 포워딩이 꺼져 있으면: 요청 ACK 가 오지 않고, `riposte_gcs` 시작 시
> "component 191 is not visible" 경고가 나옵니다.

---

## 4. SITL 로 먼저 검증 (필수)

터미널 3개로 전체 경로를 실기체 없이 검증합니다.

```bash
# 터미널 1 — PX4 SITL
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# 터미널 2 — 에이전트 (companion 역할, SITL 의 offboard 포트 14540)
./build/riposte_test --serve --execute --connection udpin://0.0.0.0:14540
#  기대 출력: [SERVE] Waiting for GCS test requests as MAVLink component 191 ...

# 터미널 3 — 지상국 (SITL 의 GCS 포트 14550)
./build/riposte_gcs --test 1 --execute
```

터미널 3 기대 출력 흐름:

```
Waiting for the vehicle on udpin://0.0.0.0:14550...
Test agent (component 191) is visible on this link.
Requesting test 1 from the agent on vehicle sysid 1...
Request ACCEPTED
[AGENT] RT|START|1|x1
[AGENT] RT|IT|1|1/1|PASS
[AGENT] RT|RESULT|1|PASS|1/1

================ Remote test 1 ================
iterations: 1/1
OVERALL: PASS
```

중간에 중단을 시험하려면 터미널 3에서 Ctrl+C (자동으로 중단 명령 송신) 또는
별도 셸에서 `./build/riposte_gcs --abort` 를 실행하십시오. 에이전트가 착륙 후
`RT|ABORTED|<id>` 를 보내고 다음 요청을 계속 기다리는지 확인하십시오.

> SITL 에서 ACK 가 오지 않으면 PX4 콘솔(pxh>)에서 `mavlink status` 로 두 인스턴스가
> 떠 있는지 보고, `param set MAV_0_FORWARD 1`, `param set MAV_1_FORWARD 1` 후
> 재시작하십시오.

---

## 5. 실기체 절차

1. **README §9 의 "첫 실기체 시험 전 확인" 을 모두 완료**하십시오 (SITL 10회
   반복 PASS, RC 조종사 대기, geofence 등). 원격 실행이라고 생략되는 항목은
   없습니다.
2. companion (RPi) 에서 에이전트를 켭니다.
   ```bash
   ./build/riposte_test --serve --execute --connection serial:///dev/ttyAMA0:921600
   ```
3. 지상국 PC 에 텔레메트리 무전기를 꽂고 요청합니다.
   ```bash
   ./build/riposte_gcs --test 1 --execute --connection serial:///dev/ttyUSB0:57600
   ./build/riposte_gcs --test 13 --repeat 2 --execute --connection serial:///dev/ttyUSB0:57600
   ```
4. 시험 사이에는 에이전트를 끌 필요가 없습니다. `RT|READY` 가 보이면 다음
   요청을 보낼 수 있습니다.
5. 중단이 필요하면 **RC 조종사의 모드 전환(Position/Land)이 최우선**입니다.
   `riposte_gcs --abort` (또는 Ctrl+C) 는 보조 수단입니다.

### CSV 로그 위치

상세/요약 CSV 는 **companion 컴퓨터의 `logs/`** 에 남습니다. 지상국에는 진행
라인과 최종 판정만 옵니다 (텔레메트리 대역폭 때문입니다). 시험 후 회수하십시오.

---

## 6. riposte_gcs 명령 정리

| 명령 | 동작 |
|------|------|
| `riposte_gcs --list` | 시험 카탈로그 출력 |
| `riposte_gcs --test <id> --execute` | 시험 요청 후 결과까지 대기 |
| `riposte_gcs --test <id> --repeat <n> --execute` | 반복 횟수를 덮어써서 요청 |
| `riposte_gcs --abort` | 진행 중 시험 중단(착륙) 요청 |
| `riposte_gcs --monitor` | 요청 없이 STATUSTEXT 수신만 표시 |
| `--timeout <s>` | 결과 대기 제한 (기본 무제한) |
| `--connection <url>` | 링크 지정 (기본 `udpin://0.0.0.0:14550` = SITL) |

### 종료 코드

| 코드 | 의미 |
|------|------|
| 0 | 시험 PASS (--abort/--monitor 는 정상 완료) |
| 1 | FAIL / 요청 거부(busy·denied) / ACK 없음 / 타임아웃 / 사용법 오류 |
| 130 | 운영자 Ctrl+C (중단 명령을 보낸 뒤 종료) |

스크립트에서 `$?` 로 원격 시험의 PASS/FAIL 을 그대로 판정할 수 있습니다.

---

## 7. 안전 설계 요약

- **이중 --execute 게이트**: 에이전트(`--serve --execute`)와 지상국(`--test
  --execute`) 모두 명시적 허용이 필요합니다. 어느 한쪽이라도 빠지면 Arm 이
  나가지 않습니다.
- **매직 코드**: param3 이 7305 가 아니면 DENIED. 같은 링크의 다른 GCS
  소프트웨어가 MAV_CMD_USER_1 을 보내도 시험이 시작되지 않습니다.
- **busy 거절**: 시험 중 새 요청은 TEMPORARILY_REJECTED. 큐에 쌓지 않습니다.
  (쌓아 두면 현장에서 "몇 개가 예약돼 있는지" 파악하기 어렵습니다.)
- **오류 시 서비스 중단**: watchdog/health/반경 초과 등 예상 밖 오류가 나면
  에이전트는 비상 착륙 후 **종료**합니다. `RT|ERROR` 수신 후에는 companion 을
  직접 점검하기 전까지 원격 재시도가 불가능합니다.
- **GCS 링크 상실**: 요청이 이미 수락됐다면 시험은 companion 단독으로 완주하고
  결과 CSV 를 남깁니다. 기존 안전 계층이 그대로 동작합니다.
- **RC 우선**: 어떤 원격 중단 수단보다 RC 조종사의 모드 전환이 우선입니다.

---

## 8. 문제 해결

| 증상 | 원인 후보 | 조치 |
|------|----------|------|
| `component 191 is not visible` 경고 | 에이전트 미실행 / 포워딩 꺼짐 | 에이전트 콘솔 확인 → §3 파라미터 확인 |
| `did not acknowledge the request` | 포워딩 꺼짐, 무전기 페어링/대역폭 | §3, 무전기 LED·baud 확인 |
| `Request BUSY` | 이전 시험 진행 중 | `--monitor` 로 진행 확인 후 재요청 |
| `Request DENIED (invalid request)` | 번호/반복 범위 밖, 매직 불일치 | 에이전트 콘솔의 `[SERVE] request denied:` 사유 확인 |
| `RT\|ERROR\|...` 수신 | 에이전트 쪽 오류로 종료 | companion 로그/ULog 점검 후 에이전트 재시작 |
| 진행 라인이 안 보임 (ACK 는 옴) | STATUSTEXT 포워딩 대역폭 부족 | 무전기 속도 확인, `--monitor` 로 재확인 |
| SITL 에서 vehicle 미발견 | 포트 충돌 (QGC 가 14550 점유) | QGC 종료 또는 `--connection udpin://0.0.0.0:14551` 등 조정 |
| 반복 시험 도중 `arm failed ... Command Denied` (PX4: `Arming denied: Resolve system health failures`) | GCS 링크 단절 — PX4 기본 arming 체크가 GCS 연결을 요구 | 반복 시험 동안 `riposte_gcs` 를 결과 대기 상태로 유지 (`--timeout` 으로 끊지 말 것). 필요 시 현장 정책에 따라 `COM_ARM_WO_GCS` 검토 |

---

## 9. 검증 현황

| 항목 | 상태 |
|------|------|
| 프로토콜 단위시험 (`gcs_protocol_tests`, ASan/UBSan 포함) | 통과 |
| 두 바이너리 MAVSDK 3.17.1 링크 빌드 (`-Wconversion -Werror`) | 통과 |
| CLI 안전 게이트 (--execute 없이 요청/서브 거부 등) | 통과 |
| PX4 SITL end-to-end (§4 절차) | **통과** — 2026-07-17, PX4 v1.17.0 SITL(gz_x500, headless) |
| 실기체 (무전기 경유) | 미수행 |

### 9.1 SITL 에서 검증한 시나리오 (2026-07-17)

| 시나리오 | 결과 |
|---------|------|
| Test 1 원격 요청 → Arm → 이륙(Offboard 직접 상승) → 호버 → 착륙 → PASS 수신 | 통과 (GCS 종료 코드 0) |
| 비행 중 GCS Ctrl+C → 자동 중단 명령 → 착륙 → `RT\|ABORTED` 확인 → 에이전트 대기 계속 | 통과 (종료 코드 130) |
| 유휴 상태 `--abort` → `RT\|ABORTED\|0` 즉시 확인 | 통과 (종료 코드 0) |
| 비행 중 에이전트 SIGINT → 착륙 → `RT\|STOPPED` → GCS 안내 후 종료 코드 1 | 통과 |
| `--monitor` 모드 + 로컬 직접 실행(`--test 1 --execute`) 병행 | 통과 |

### 9.2 SITL 검증에서 발견되어 수정된 사항

같은 증상이 재발하면 이 목록을 먼저 확인하십시오.

1. **MAVSDK v3.17 의 CompanionComputer 기본 컴포넌트 ID 는 195** — 프로토콜
   대상(191)과 달라 요청이 도달하지 않았음. 에이전트가 191 을 명시 설정하도록
   수정 (`set_component_id`).
2. **MAVSDK 는 수신 메시지를 발신자 sysid 의 System 으로 분배** — autopilot
   System 에만 구독하면 지상국(sysid 245)의 COMMAND_LONG 이 조용히 사라짐.
   발견되는 모든 System 에 수신 구독을 걸도록 수정.
3. **MAVSDK v3 의 `remaining_percent` 는 0~100 퍼센트** (v1/v2 는 0~1 비율) —
   0~1 가정이던 기존 코드가 100 을 무효로 버려 배터리 게이트 전체가 무력화됨.
   `battery_remaining_fraction()` 한 곳에서 정규화하도록 수정.
   **기존 미션 앱(v1.4.2-ChatGPT)에도 같은 문제가 있을 가능성이 높습니다.**
4. stdout 리다이렉트 시 진행 로그가 버퍼에 갇히는 문제 — 줄 단위 버퍼링 적용.
5. **반복 시험에서 `offboard.start` 가 간헐적으로 'No Setpoint Set'** —
   MAVSDK 는 "Offboard 모드가 아니고 마지막 start() 후 3초 경과"인 heartbeat 를
   보면 내부 setpoint 상태를 되돌립니다. 프라임 마지막 전송과 start() 사이
   50 ms 틈에 1 Hz heartbeat 가 끼면 실패하며, 프라임 길이(1.0 s)와 위상이
   고착되어 재시도도 같은 지점에서 실패했음. 마지막 setpoint 직후 무지연
   start() + 시도별 프라임 길이 가변화로 수정 (Test 3/6 10회 반복 재현 → 수정
   후 10/10 PASS).
6. **Test 21/22 시작 직후 'Global position is not valid'** — MAVSDK 의
   position() 캐시는 첫 GLOBAL_POSITION_INT 수신 전 NaN. 유효값 대기 루프
   추가 (`wait_for_valid_global_position`).
7. **gz SITL 의 홈 좌표는 `PX4_HOME_LAT/LON` 이 아니라 월드 SDF 의
   `spherical_coordinates` 가 결정** — 기본 월드는 취리히라 Test 21/22 의
   L>d 검증이 d≈8,972 km 로 거부됨(검증 로직이 의도대로 오배치를 차단한 사례).
   태안 좌표 월드를 만들어 `PX4_GZ_WORLD=<world>` 로 실행해야 함
   (docs/TEST_USAGE.md §4.2 참조).
