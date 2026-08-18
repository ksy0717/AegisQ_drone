# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 이 파일의 범위

**규범 문서는 [AGENTS.md](AGENTS.md) 다.** 빌드 프로필 정책, MQTT/PX4 모듈 경계, 비행 코드 규칙,
코드 스타일, 커밋 규칙은 전부 거기 있고 여기서 반복하지 않는다. 두 문서가 어긋나면 AGENTS.md 가
이긴다. 이 파일은 Claude Code 세션에서만 의미가 있는 것 — **어디서 무엇을 실행하는지**와
**이 저장소에서 에이전트가 반복해서 틀리는 지점** — 만 다룬다.

읽는 순서: `AGENTS.md` → `docs/mqtt-px4/ARCHITECTURE.md`(계약과 활성화 차단 조건) →
`docs/TODO.md`(남은 작업) → `README.md`(영상·Pi 운용·실측치).

## 명령

정본은 Linux(Pi 5 또는 Linux 호스트)다.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4
ctest --test-dir build --output-on-failure

ctest --test-dir build -R remote_command_tests --output-on-failure   # 단일 스위트
./build/remote_command_tests                                        # 실행 파일 직접 (출력이 더 직관적)

cmake -DDC_PRINT_PROFILE=ON -P config/project.cmake   # 프로필이 요구하는 의존성 조회
sudo ./scripts/install.sh                             # Pi 부트스트랩 (SKIP_LIBCAMERA=1 / SKIP_MAVSDK=1)
```

현재 프로필(`config/project.cmake`)이 만드는 ctest 타깃 12개:
`bullet_path_tests`, `geo_tests`, `test_trajectory_tests`, `video_pipeline_tests`,
`remote_command_tests`, `remote_mission_coordinator_tests`,
`preemptive_mission_executor_tests`, `mqtt_mission_codec_tests`,
`mqtt_command_source_tests`, `mqtt_state_codec_tests`, `mqtt_agent_config_tests`,
`mqtt_command_ledger_tests`. `gcs_protocol_tests`·`verdict_tests` 는 이 브랜치에서 OFF다.

- **CMake cache 가 `-D` 값을 유지한다.** 진단용 오버라이드를 쓴 뒤 프로필을 검증할 때는 깨끗한
  빌드 디렉터리를 쓰거나 `install.sh` 처럼 `cmake -U 'DC_*'` 로 지운다.
- 시험은 외부 프레임워크가 없다. `tests/riposte/test_support.hpp` 의 `check` / `check_close` /
  `report` 를 쓰고 `assert` 는 쓰지 않는다(Release 의 `-DNDEBUG` 에서 사라진다).
  MQTT 픽스처는 `tests/mqtt/fixtures.hpp`.
- **시험 파일만 추가하면 실행되지 않는다.** `CMakeLists.txt` 의 해당 블록(`FLIGHT_TEST_SUITES`
  목록 또는 `DC_BUILD_*_TESTS` 블록)에 `add_executable`/`add_test` 를 같이 넣어야 한다.

## Windows 개발 호스트에서 할 수 있는 것

이 PC 에는 **cmake/ninja/gcc 가 PATH 에 없고 WSL 배포판도 설치돼 있지 않다.** 있는 것은
Android SDK 가 딸려 온 CMake 3.22.1 + Ninja 와 NDK 28.2 의 clang 뿐이다. 그래서 호스트에서
가능한 검증은 **NDK 교차 컴파일뿐이며, 이는 컴파일 검증일 뿐 실행 검증이 아니다**(AGENTS.md).

**이 프로젝트는 안드로이드 앱이 아니다.** 타깃은 Pi 5(aarch64 Linux)이고, NDK 는 이 PC 에서
구할 수 있는 유일한 aarch64 C++ 컴파일러라서 쓰는 것뿐이다. 산출물은 Bionic libc 대상이라
Pi 에 올릴 수 있는 바이너리가 아니며 그대로 버린다. 잡히는 것은 문법·타입 오류 수준이고,
Pi 의 gcc 가 낼 `-Wconversion` 경고까지 같다고 보장하지 않는다.

```bash
SDK="C:/Users/USER/AppData/Local/Android/Sdk"
"$SDK/cmake/3.22.1/bin/cmake.exe" -G Ninja \
  -DCMAKE_MAKE_PROGRAM="$SDK/cmake/3.22.1/bin/ninja.exe" \
  -DCMAKE_TOOLCHAIN_FILE="$SDK/ndk/28.2.13676358/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-24 \
  -DDC_BUILD_VIDEO=OFF -DDC_BUILD_MQTT_CONNECTOR=OFF \
  -DDC_BUILD_MQTT_TRANSPORT=OFF -DDC_BUILD_MQTT_CONNECTOR_TESTS=OFF \
  -S . -B <스크래치패드>/hostcheck
```

끄는 이유 — 프로필 기본값 그대로는 호스트에서 configure 가 실패한다:

| 스위치 | 호스트에서 실패하는 이유 |
|---|---|
| `DC_BUILD_VIDEO` | pkg-config + GStreamer/cairo 필요 |
| `DC_BUILD_MQTT_CONNECTOR(_TESTS)` | `nlohmann_json` 필요 |
| `DC_BUILD_MQTT_TRANSPORT` | `CMakeLists.txt` 가 Linux 아니면 `FATAL_ERROR` (libmosquitto/netlink) |

즉 **호스트에서 컴파일되는 범위는 flight core 와 remote command/coordinator 뿐이다.** MQTT 코덱·
소스·전송 계층을 고쳤으면 호스트 빌드가 통과해도 아무것도 검증되지 않은 것이다 — Linux 에서
`ctest` 를 돌리기 전에는 그렇게 보고한다.

저장소의 기존 `build/` 는 이 NDK 툴체인 캐시이고 `DC_*` 선택이 낡았다. 재사용하지 말고
스크래치패드에 새 디렉터리를 만든다. `build/matrix-off/` 는 전 스위치 OFF 검증용 캐시다.

## 아키텍처 지도

여러 파일을 읽어야 보이는 것만 적는다. 프로세스 경계와 계약 상세는 `docs/mqtt-px4/ARCHITECTURE.md`.

**두 계통은 별도 프로세스다.** 영상(`drone_companion`)이 카메라를 단독 소유하고 RTSP 로 낸다.
비행 프로세스는 GStreamer/Hailo 에 의존하지 않는다.

**MQTT 명령 경로 — 방향이 곧 계층 규칙이다.**

```
MosquittoSession(Linux 전용 transport)  ── ingest ─▶  MqttCommandSource (bounded FIFO, decode만)
        ▲ StatusPublisher                                        │ CommandSource 포트
        └──────── publish ── RemoteMissionCoordinator (단일 워커) ─┘
                                     │ MissionExecutor 포트
                                     ▼  PreemptiveMissionExecutor (PX4 를 만지는 유일한 스레드)
                                     ▼  MavsdkMissionBackend → MavsdkOffboardControl
```

CMake 라이브러리 의존이 이 방향을 강제한다:
`dc_flight_core`(INTERFACE, 헤더 온리) ← `dc_remote_mission`,
`dc_flight_core` ← `dc_mqtt_codec` ← `dc_mqtt_command_source` ← `dc_mqtt_transport`.
**역방향 의존을 만들지 말 것** — 코덱이나 CommandSource 가 libmosquitto/MAVSDK 를 알게 되면
호스트에서 단위시험을 돌릴 수 없게 되고, 이 구조를 만든 이유가 사라진다.

**2026-08-13 부로 MQTT→PX4 경로는 전부 구현·배포됐다.** `mqtt_px4_control` 실행 파일,
MAVSDK 백엔드, SQLite 원장(claim/outbox), terminal ACK 발행이 모두 동작하고 기체에서 돌고 있다.
`DC_BUILD_MQTT_PX4_AGENT=ON` 은 이제 `FATAL_ERROR` 가 아니라 의존성 검사다. 예전 문서가
"구현되어 있지 않다"고 말하면 그 문서가 낡은 것이다.

**단 하나 남은 가드는 `REMOTE_FLIGHT_EXECUTE` 다.** 0 이면 MAVSDK 를 아예 열지 않고 명령마다
정직하게 `ACCEPTED → FAILED("remote_flight_execute_disabled")` 로 답한다. 1 이면 **서비스가 뜨는
즉시 PX4 에 실제 명령이 나간다**(첫 동작이 재시작 복구라 지상이면 무장 해제, 공중이면 RTL).
현재 기체 배포값은 0 이다. 이 값을 바꾸는 것은 코드 변경이 아니라 비행 결정이다.

**안전 비행 로직은 `MissionExecutor` 포트 뒤로 추출이 끝났다** — `MavsdkOffboardControl`
(setpoint watchdog, 반경 제한, 배터리 게이트, 수동 인계 감지)과 `MavsdkMissionBackend`
(예외→비상착륙, 선점, 복구). `src/riposte/test_main.cpp` 에도 여전히 인라인 사본이 있으나
그쪽은 riposte 시험 경로 전용이고, **MQTT 경로는 추출된 클래스를 쓴다. 둘을 다시 합치거나
한쪽을 다른 쪽에 복사하지 말 것.** `include/riposte/` 는 기하·궤적·판정 헤더 온리이며 MAVSDK 를
모른다.

**로컬 자율 안전 동작 네 가지가 감독자에 있다**(전부 `REMOTE_FLIGHT_EXECUTE=1` 에서만 실제 명령):
배터리 복귀, 링크 손실 복귀(`MQTT_LINK_LOSS_TIMEOUT_S`), 지오펜스 복귀(`GEOFENCE_RADIUS_M`,
실측 위치 기준), 수동 인계 latch. 우선순위는 STOP < RTL < 링크손실 < 지오펜스 < 배터리 <
E-STOP < shutdown 이고 `PreemptiveMissionExecutor` 의 `priority()` 가 단일 원본이다.

**빌드 스위치의 단일 원본은 `config/project.cmake`** 하나다. `CMakeLists.txt` 나
`scripts/install.sh` 에 제품 기본값을 복제하지 않는다.

## 이 저장소에서 자주 틀리는 지점

- **문서가 코드보다 뒤처져 있을 수 있다.** 예로 `docs/TODO.md` §1.3 의 명령 enum 표는 현재
  `include/dc/flight/remote_command.hpp` 와 다르고, 코덱이 받는 명령 종류 서술도 헤더 주석과
  어긋난 시점이 있다. **문서를 인용하기 전에 헤더와 시험을 먼저 읽고**, 어긋나면 고쳐 쓰지 말고
  사용자에게 알린다.
- **줄바꿈은 LF 다**(`.gitattributes`). Windows 에서 파일을 새로 만들 때 CRLF 가 섞이면 bash
  스크립트는 `\r: command not found`, systemd unit 은 파싱 실패로 죽는다.
- **주석·문서 언어는 영역을 따른다.** `include/dc/flight/`, `include/dc/mqtt/`, `src/mqtt/`,
  `docs/mqtt-px4/` 는 영어. 영상·Riposte 코드와 `README.md`·`docs/setup_notes.md`·`docs/riposte/`
  는 한국어. 새 파일은 그 영역의 관습을 따른다.
- **컴파일은 검증이 아니다.** Pi 에서 돌려보지 않았으면 동작한다고 보고하지 않는다. 영상
  파이프라인을 건드렸으면 fps 를 재기 전까지 개선을 주장하지 않는다(AGENTS.md).
- 비행 타깃은 `-Wconversion -Werror` 로 빌드한다. 경고가 나면 경고 정책을 완화하지 말고 코드를
  고친다.
