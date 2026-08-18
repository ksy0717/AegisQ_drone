# Drone Companion Project TODO List

본 문서는 `drone_companion` 프로젝트(`project/mqtt-px4-control` 브랜치)의
코드베이스 분석 결과와 프로덕션 비행 제어기 활성화를 위한 TODO를 정리한다.

마지막 검증: 2026-08-07 — 소스 코드 전수 분석 기반.

---

## 📌 1. 현재 코드베이스 상태 요약

### 1.1. 빌드 타깃

| 타깃 (CMake) | 스위치 | 상태 | 주요 파일 |
| :--- | :--- | :--- | :--- |
| `drone_companion` (영상) | `DC_BUILD_VIDEO` | **ON** | `src/main.cpp`, `src/pipeline.cpp` |
| `dc_remote_mission` (lib) | `DC_BUILD_FLIGHT_CORE` | **ON** | `src/flight/remote_mission_coordinator.cpp` |
| `dc_mqtt_codec` (lib) | `DC_BUILD_MQTT_CONNECTOR` | **ON** | `src/mqtt/mission_codec.cpp` |
| `dc_mqtt_command_source` (lib) | `DC_BUILD_MQTT_CONNECTOR` | **ON** | `src/mqtt/mqtt_command_source.cpp`, `aegisq_state_codec.cpp` |
| `dc_mqtt_persistence` (lib) | `DC_BUILD_MQTT_TRANSPORT` | **ON** | `src/mqtt/sqlite_command_ledger.cpp` |
| `dc_mqtt_transport` (lib) | `DC_BUILD_MQTT_TRANSPORT` | **ON** | `src/mqtt/mosquitto_session.cpp`, `agent_config.cpp` |
| `dc_mavsdk_flight` (lib) | `DC_BUILD_MQTT_PX4_AGENT` | **ON** | `src/riposte/mavsdk_mission_backend.cpp`, `mavsdk_offboard_control.cpp` |
| `mqtt_px4_control` (exe) | `DC_BUILD_MQTT_PX4_AGENT` | **ON — 기체 배포됨** | `src/mqtt/mqtt_px4_control_main.cpp` |
| `riposte_test` (exe) | `DC_BUILD_RIPOSTE_AGENT` | **OFF** | `src/riposte/test_main.cpp` (1,831줄, 시험 경로 전용) |
| `riposte_gcs` (exe) | `DC_BUILD_RIPOSTE_GCS` | **OFF** | `src/riposte/gcs_main.cpp` |

`DC_BUILD_MQTT_PX4_AGENT=ON` 의 `FATAL_ERROR` 가드는 제거됐다. 이제 의존성 검사만 한다.

### 1.2. 테스트

ctest 타깃 12개, 전부 Pi 에서 통과한다(2026-08-13):

`bullet_path_tests`, `geo_tests`, `test_trajectory_tests`, `video_pipeline_tests`,
`remote_command_tests`, `remote_mission_coordinator_tests`,
`preemptive_mission_executor_tests`, `mqtt_mission_codec_tests`,
`mqtt_command_source_tests`, `mqtt_state_codec_tests`, `mqtt_agent_config_tests`,
`mqtt_command_ledger_tests`.

**MAVSDK 계층은 단위시험이 없다** — 모의 객체가 없어 `MavsdkMissionBackend` /
`MavsdkOffboardControl` 은 SITL 로만 검증된다. 그쪽을 고쳤으면 ctest 통과는 아무것도
증명하지 않는다.

### 1.3. 명령 데이터 모델 현황

`RemoteCommandType` 은 4개 값이며 **전부 구현·검증됐다**:

| enum 값 | 서버 `command` | 코덱 | 실행 경로 |
| :--- | :--- | :--- | :--- |
| `execute_mission` | `EXECUTE_MISSION` | ✅ | ✅ 이륙·항로·복귀 |
| `stop_mission` | `STOP_MISSION` | ✅ | ✅ 선점 후 hold |
| `return_to_launch` | *(독립 명령 없음)* | — | ✅ 로컬 감독자·웨이포인트 `RTL` 로만 |
| `emergency_stop` | `EMERGENCY_STOP` | ✅ | ✅ 선점 후 즉시 착륙 |

`RETURN_TO_LAUNCH` 는 **서버 명령으로 받지 않는다**. 항로의 종단 웨이포인트
(`action: "RTL"`) 이거나 로컬 안전 감독자가 만드는 합성 명령이다. 서버가 그 이름으로
보내면 `unsupported_command` 로 거부된다.

### 1.4. ACK 상태 체계

4가지가 모두 발행된다: `ACCEPTED`(비-terminal), `REJECTED`(비-terminal),
`COMPLETED`, `FAILED`. 원장이 outbox 에 영속화한 뒤 발행하므로 재접속 후 재생된다.

로컬 감독자가 만든 복귀(배터리·링크손실·지오펜스)는 **서버 `commandId` 가 없어 ACK 를
만들지 않는다**. 원장이 정규 UUID 를 요구하기 때문이며, 서버는 텔레메트리의
`flight_mode` 로 알게 된다.

---

## ✅ 2. 원격 비행 활성화 차단 과제 — 2.1~2.4 해소 (2026-08-13)

2.1 영속성, 2.2 선점, 2.3 MAVSDK 어댑터, 2.4 terminal 발행은 모두 구현되고 SITL 및
실기체(이륙·착륙 1회)에서 검증됐다. 아래 체크박스는 당시 기록으로 남긴다.
**2.5 보안·ACL·kill-switch 는 여전히 열려 있다.**

### 2026-08-13 에 추가된, 원래 이 목록에 없던 것

- **링크 손실 자율 복귀** — 브로커 세션이 끊긴 채 공중이면 스스로 RTL.
  PX4 는 Pi↔FC MAVLink 가 멀쩡하므로 datalink loss 로 보지 않아 자체 failsafe 가 안 돈다.
  실측 감지 20.3초 = keepalive 몫 15.3초(= 1.5 × `MQTT_KEEPALIVE_SEC` 10) +
  `MQTT_LINK_LOSS_TIMEOUT_S` 5초. SITL 3종(블랙홀·RST·브로커 재시작) 통과.
- **지오펜스 복귀** — `GEOFENCE_RADIUS_M` 50 m, **실측 위치** 기준.
  기존 `FLIGHT_MAX_ROUTE_RADIUS_M` 60 m 는 *명령* 게이트라 바람·표류·수동 조종으로
  실제 이탈한 경우를 못 잡는다. 연결 상태와 무관하게 2초 주기로 검사한다.
- **수동 인계 감지·latch** — raw `RC_CHANNELS`의 CH6 POSITION을 권한 인계의
  근거로 사용한다. PX4 모드만으로는 실제 스위치 전환과 Offboard-loss를 구분할 수
  없다. 일반 예외와 **다른 타입**이어야 하고, 인계 시 MAVSDK의 cached setpoint
  sender만 제거하며 Hold/Land/RTL/Disarm을 보내지 않는다. 이전 미션은 종료되고,
  OnGround+DISARMED+debounced CH6 HOLD 뒤 새 미션만 허용한다. 이 새 경로는 컴파일·
  단위시험만 완료되었고 SITL/프로펠러 제거 실기체 시험은 아직이다.

### 남은 것

- **2.5 보안·ACL·kill-switch** (아래)
- **PX4 파라미터 확인** — `COM_OBL_RC_ACT`(offboard 상실 시 조종기로 넘길 것),
  `COM_RC_OVERRIDE`, `COM_RC_IN_MODE`, `MAV_2_CONFIG` 가 TELEM3 를 점유하지 않을 것.
  코드가 아니라 기체 설정 문제이고, 프로펠러 제거 벤치 시험 전에 끝나야 한다.
- **MAVSDK 계층 단위시험** — 모의 객체가 없어 SITL 에만 의존한다.
- **RC/PX4 명령 선형화** — START queue stamp와 RC 상태 publication은 한 gate로
  묶였지만 Action/Offboard 호출 자체는 아직 동기식이다. 마지막 RC 검사 직후
  POSITION이 관측되거나 MAVSDK 호출이 멈추면 그 호출을 취소할 수 없다. RC gate
  안에서는 비동기 요청 시작만 하고, ACK는 gate 밖에서 bounded timeout으로 기다리는
  단일 command dispatcher로 전환한 뒤 SITL/ULog에서 검증해야 한다.
- **AegisQ owner/ready gate** — 서버와 웹이 `controlOwner`,
  `readyForMission`, CH5/CH6를 보존·검사하고, PILOT/UNKNOWN 동안 과거
  scheduler/auto-dispatch 명령을 폐기해야 한다. Companion만으로는 수동 착륙 후
  오래된 자동 명령과 사용자의 새 START를 구별할 수 없다.
- **서버 geofence 수신** — 코덱 루트 화이트리스트에 geofence 필드가 없다.
  서버가 보내면 미션 전체가 `invalid_root_schema` 로 거부된다. 현재 기체의 유일한
  공간 제약은 위의 로컬 지오펜스다.

### (당시 기록) 원격 비행 활성화 차단 과제

`DC_BUILD_MQTT_PX4_AGENT`를 `ON`으로 전환하려면 **`CMakeLists.txt:160-165`의
`FATAL_ERROR` 가드를 해제**해야 하며, 그 전에 아래 항목이 모두 완료되어야 한다.
`ARCHITECTURE.md`의 blocker #1~#8에 대응한다.

### 2.1. 영속성 계층 (Persistent Ingress/Outbox Ledger)

_ARCHITECTURE.md blocker #1_

- [ ] **원자적 Command Claim 및 Inbox 구현**
  - `commandId` (UUID) 및 미션 명령 원본을 재부팅/전원 차단에도 유지되는 영속
    저장소에 기록. 현재 `RemoteMissionCoordinator`의 `completed_commands_`는
    `std::unordered_map`으로 **프로세스 메모리 범위**다.
  - QoS 1 메시지 재전송 또는 프로세스 재시작 시 동일 UUID 미션이 중복 실행되지
    않도록 실행 **전** 원자적 claim 트랜잭션 보장.
- [ ] **Persistent ACK Outbox & Replay**
  - ACK 송신 실패 또는 Broker 재연결 시 저장된 ACK를 재송신할 수 있는 Outbox
    Queue 구축. 현재 ACK는 `mosquitto_publish()` 즉시 호출이며, 실패 시 재시도
    경로가 없다.
- [ ] **Negative-ACK Durable Outbox**
  - Decode 실패, ingress queue 가득 참, ACK 연결 불가 시에도 REJECTED/FAILED
    ACK를 영속적으로 보관 후 재전송하는 경로 구축.
    (`ARCHITECTURE.md` L181-183에 명시된 누락 경로)

### 2.2. 우선순위 제어 및 긴급 선점 경로 (Priority & Preemption Commands)

_ARCHITECTURE.md blocker #2_

- [ ] **긴급 명령 프로토콜 및 디코더 확장**
  - `STOP_MISSION`, `RTL`, `EMERGENCY_STOP` 디코딩 및 검증 로직 구현.
  - 기존 `RemoteCommandType` enum에 `cancel`, `return_to_launch`이 이미 정의되어
    있으나 AegisQ 계약의 명령 이름과 대응 관계를 확정해야 함.
- [ ] **`MissionExecutor` 포트에 선점(Cancellation) API 추가**
  - 현재 `MissionExecutor` 인터페이스(`ports.hpp`)에는 `execute()`만 있고
    `cancel()` 메서드가 **없다**. 진행 중인 미션을 중단할 방법이 포트 수준에서
    존재하지 않으므로, 긴급 명령 수신 시 실행 중인 미션을 선점할 수 없다.
- [ ] **진행 중인 미션 선점 (Preemption)**
  - 일반 `EXECUTE_MISSION` 수행 중 긴급 명령 수신 시 즉시 미션 중단 및 안전 모드
    전환 경로 구축. 한계 응답 시간(Bounded Latency) 내 긴급 ACK 발행.

### 2.3. MAVSDK MissionExecutor Adapter 구현

_ARCHITECTURE.md blocker #3_

- [ ] **Riposte 안전 비행 로직 어댑트**
  - 검증된 안전 로직(exception→비상착륙, setpoint watchdog, software radius limit,
    battery gate)은 **`src/riposte/test_main.cpp`** (1,949줄)에 인라인 내장되어
    있다. `include/riposte/`는 기하·궤적·판정 헤더 온리 라이브러리이며 MAVSDK 호출
    코드를 포함하지 않는다.
  - 이 안전 로직을 `MissionExecutor` 포트 뒤의 독립 adapter로 **추출**해야 한다.
    같은 이륙/Offboard/watchdog/착륙 코드를 MQTT용으로 복사하지 않는다(AGENTS.md).
- [ ] **PX4 MAVLink/MAVSDK 링크 단일 소유권 보장**
  - `riposte-agent.service`와 `mqtt-px4-control.service`의 `Conflicts=` 지시어가
    이미 systemd 템플릿에 있으나, 프로세스 수준에서도 PX4 링크가 하나의 소유자만
    갖도록 보장.

### 2.4. Terminal Status Publication

_ARCHITECTURE.md blocker #4_

- [ ] **비행 진행 상태 추적 상태 머신 구현**
  - 현재 `AckStatus` enum에 `COMPLETED`가 **없으며**, 비행 진행 결과를 추적하는
    상태 머신 자체가 미구현이다.
  - 실제 PX4 비행 결과에 따라 AegisQ 백엔드가 terminal로 인식하는
    `COMPLETED`/`FAILED`를 발행하는 경로를 `StatusPublisher` 포트를 통해 구현.
  - `encode_command_ack()`는 문자열 상태를 직렬화할 수 있으므로 enum 확장 또는
    별도 인코더 경로를 결정해야 한다.

### 2.5. 보안, Broker ACL 및 Network Fail-Closed 검증

_ARCHITECTURE.md blocker #5_

- [ ] **Broker ACL 검증**
  - AegisQ Broker ACL이 인증된 Client Principal(예: `DRONE-01`)을 정확한 토픽
    (`aegisq/cmd/DRONE-01` 구독, `aegisq/cmd-ack/DRONE-01` 발행)에만 제한하고
    와일드카드를 차단하는지 확인.
- [ ] **WireGuard 외부 Fail-Closed Kill-Switch 시험**
  - `mosquitto_session.cpp`의 netlink 라우팅 검사는 defense-in-depth일 뿐
    kernel-enforced output-interface policy가 아니다(코드 주석 L286-289).
  - 방화벽(nftables/iptables) 또는 policy routing으로 `10.100.0.1:8883` 트래픽이
    `wg0` 외 인터페이스(예: WLAN)로 유출되지 않음을 터널 강제 차단 시험으로 검증.

### 2.6. 서버 / 프론트엔드 `mobilityClass` 검증 연동

_ARCHITECTURE.md blocker #6_

- [ ] **서버 측 대상 검증 확인**
  - AegisQ 백엔드 및 프론트엔드 웹에서 mission 대상 로봇의
    `mobilityClass == DRONE` 검증이 반영되었는지 연동 확인 (오배송 방지).

### 2.7. SITL (Software-in-the-Loop) 시뮬레이션 검증

_ARCHITECTURE.md blocker #7_

- [ ] **SITL 통합 시나리오 테스트 작성 및 통과**
  - Mission Upload → Arm/Takeoff → Waypoint Move → Hover/Dwell → Land 전 과정.
  - Stale/Future timestamp, Duplicate UUID, Conflicting UUID, Reconnect,
    Broker outage, Process restart, Telemetry loss, PX4 rejection 시나리오.
  - Preemption, `STOP_MISSION`, `RTL`, `EMERGENCY_STOP` 시뮬레이션.

### 2.8. 라즈베리 파이 5 & PX4 실기체 검증

_ARCHITECTURE.md blocker #8_

- [ ] **하드웨어 및 네트워크 라운드트립 검증**
  - Raspberry Pi 5 + Ubuntu 24.04 실환경에서 WireGuard, Broker, PX4 하드웨어
    연동 시험 수행.

---

## 🛠️ 3. 코드 품질, 리팩토링 및 기능 개선 TODO

### 3.1. 테스트

- [ ] **MQTT connector 테스트 플랫폼 확대**
  - 현재 MQTT connector 테스트는 `CMAKE_SYSTEM_NAME STREQUAL "Linux"` 가드로
    **Linux에서만 빌드/실행**된다. Windows 호스트 개발 환경에서 ctest에 포함되지
    않으므로, 교차 빌드 검증 시 주의.
- [ ] **테스트 커버리지 확대**
  - `tests/mqtt/` 및 `tests/flight/`에 경계 조건 추가: 네트워크 단절/재연결,
    concurrent 접근(현재 single-worker 가정), persistent deduplication 시나리오.

### 3.2. 인프라

- [ ] **WireGuard Route Recovery Supervisor**
  - `wg0` 인터페이스가 drop되거나 IP가 변경되면 `handle_connect`가 fail-closed로
    연결을 끊지만, 복구 후 세션을 재생성하는 감시 로직이 없다
    (`mosquitto_session.cpp` L603-605 주석: "A future route supervisor must
    recreate the session after wg0 recovers").
- [ ] **시간 동기화 (NTP / System Clock) 모니터링**
  - `issuedAt` RFC3339 freshness 검증을 위해 라즈베리 파이의 시스템 시계 동기화
    상태 모니터링 강화.
- [ ] **Telemetry 발행 경로**
  - 미션 진행률 및 드론 위치/상태 telemetry 주기적 발행.
  - AegisQ 백엔드 계약에 telemetry 토픽이 정의되어 있는지 확인 필요.

### 3.3. 영상 / 운영

- [ ] **영상 파이프라인 성능 모니터링 준수**
  - 영상 코드 수정 시 FPS(캡처 ≈ 추론 ≈ 29~30 fps, 드롭 0) 및 CPU 점유율 측정.
- [ ] **미션 실행을 영상 프로세스와 통합**
  - 현재 비행(`riposte_test`)과 영상(`drone_companion`)은 별도 바이너리.
    안전 로직을 서브시스템으로 분리한 뒤 프로세스 통합.
- [ ] **검출 결과 별도 채널 내보내기**
  - 현재 검출 박스는 영상에 burn-in만 된다. JSON/MAVLink 별도 채널로도 내보내면
    태블릿에서 오버레이 on/off 가능.
- [ ] **`mediamtx.service` 비특권 사용자 전환**
  - 카메라를 열지 않으므로 root에서 비특권 사용자로 낮출 수 있다.
- [ ] **RTSP 인증 추가**
  - 현재 익명 접근 허용. 인증 메커니즘 도입 필요.

### 3.4. 문서

- [ ] **Riposte 문서 고도 설명 정정**
  - `docs/riposte/RIPOSTE_README.md`의 시험 고도 설명(50cm/100cm/300cm)이
    실제 코드의 시험 카탈로그(5m/6m/7m, 23종)와 다르다.

---

## 🔗 4. 관련 문서 (References)

| 문서 | 내용 |
| :--- | :--- |
| `AGENTS.md` | 저장소 가이드라인, 빌드 프로필, 모듈 경계, 코드 스타일 |
| `docs/mqtt-px4/ARCHITECTURE.md` | MQTT/PX4 아키텍처, AegisQ 계약, activation blocker #1~#8 |
| `docs/setup_notes.md` | Raspberry Pi 5 셋업 유의사항 |
| `config/project.cmake` | 프로젝트 빌드 프로필 원본 (단일 진실 원천) |
| `config/mqtt-px4-control.env.example` | MQTT PX4 Agent 런타임 설정 템플릿 |
| `systemd/mqtt-px4-control.service` | MQTT PX4 서비스 템플릿 (미설치/미활성) |
| `README.md` §9 | 영상·운영 관련 남은 과제 |
