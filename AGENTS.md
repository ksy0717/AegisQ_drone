# Repository Guidelines

## AegisQ 서버 계약 기준

이 브랜치의 MQTT 명령·ACK·텔레메트리·기체 등록 계약은 다음 두 저장소의
`prototype-v2` 브랜치를 기준으로 구현하고 검증한다.

- 백엔드 서버: https://github.com/sizzlingbun/aegisq-plus/tree/prototype-v2
- 웹 콘솔: https://github.com/sizzlingbun/aegisq-web/tree/prototype-v2

다른 브랜치, 과거 API 문서, 로컬 사본과 동작이 다르면 위 두 `prototype-v2`
브랜치의 실제 발행·파싱 코드와 테스트를 우선한다. 계약을 바꿀 때는 두 저장소의
해당 브랜치와 이 저장소의 코덱·통합 테스트를 함께 대조한다.

## project/mqtt-px4-control 브랜치

이 브랜치는 main에서 분기한 장기 운영 제품 브랜치다. 공용 수정은 main에서 이
브랜치로 병합하고, MQTT 제품 전용 코드는 포트 뒤에 둔다. dev나 Riposte 제품
브랜치를 이 브랜치의 upstream으로 사용하지 않는다.

### 빌드 프로필

- 빌드 기본값의 단일 원본은 config/project.cmake다. CMakeLists.txt나
  scripts/install.sh에 제품별 기본값을 중복해서 넣지 않는다.
- 현재 프로필은 영상/RTSP/선택적 WHIP, 공용 비행 코어, 엄격한 AegisQ JSON 코덱,
  bounded CommandSource, Linux libmosquitto mTLS/WireGuard 전송 계층과 해당
  테스트를 빌드한다. riposte_test, riposte_gcs, Riposte 전용 프로토콜 테스트는
  끈다. MQTT 라이브러리가 빌드된다는 사실은 비행 에이전트가 배포되거나 실행
  중이라는 뜻이 아니다.
- HailoRT 컴파일 지원과 systemd 런타임 기본값은 모두 OFF다. RTSP는 ON이며
  Hailo 없이 통과 모드로 계속 송출한다.
- DC_BUILD_MQTT_PX4_AGENT는 아직 OFF다. 영속 inbox/outbox/UUID ledger,
  우선순위·선점 가능한 안전 명령, 실제 PX4 실행기, SITL/실기 검증과 터미널
  상태 보고가 끝나기 전에는 실행 파일인 것처럼 보이는 placeholder를 만들거나
  옵션을 켜지 않는다.
- install.sh는 CMake script mode로 project.cmake를 조회한다. 의존성 설치 여부나
  설치 타깃 목록을 셸에 다시 하드코딩하지 않는다.
- 명령행 -D 값은 진단 빌드에만 임시로 사용할 수 있다. 제품 기본값 변경은
  project.cmake에서 하고, 기존 CMake cache가 값을 유지할 수 있으므로 프로필
  검증은 깨끗한 build 디렉터리에서도 수행한다.

### MQTT/PX4 모듈 경계

명령 경로는 CommandSource -> RemoteMissionCoordinator -> MissionExecutor 순서다.
상태는 StatusPublisher로만 나간다.

- MQTT 콜백은 payload를 decode하고 bounded queue에 넣기만 한다. Arm, mode 변경,
  setpoint 전송을 직접 수행하지 않는다. QoS 1의 non-retained 메시지만 받으며,
  configured vehicle id에서 유도한 정확한 aegisq/cmd/{vehicleId} 토픽만 구독한다.
  ACK는 aegisq/cmd-ack/{vehicleId}로 QoS 1, non-retained로 보낸다.
- payload에는 vehicle id가 없다. MQTT_VEHICLE_ID와 MQTT_CLIENT_ID는 같아야 하고,
  vehicle id는 정확한 구독 토픽에서 주입한다. 와일드카드 구독이나 payload가
  주장하는 identity를 신뢰하는 방식으로 바꾸지 않는다.
- 서버 제어 경로는 WireGuard wg0 전용이다. 현재 제품 서버는 10.100.0.1이고
  MQTT는 TLS 8883을 사용한다. 프로세스의 route/source-address 검사는
  defense-in-depth일 뿐 완전한 kill switch가 아니다. 방화벽 또는 policy routing의
  외부 kill switch로 10.100.0.1:8883 트래픽이 wg0 밖으로 나가지 못하게 하고,
  터널 차단 시험을 통과하기 전에는 원격 비행을 활성화하지 않는다.
- 서버 인증서는 /etc/aegisq/certs/ca.crt의 AegisQ-MQTT-CA로 검증한다.
  DRONE-01은 같은 디렉터리의 DRONE-01.crt와 DRONE-01.key로 mTLS 인증한다.
  인증서 검증을 끄거나 client key를 저장소로 복사하지 않는다. Companion은
  인증서 subject와 vehicle id의 결속을 증명하지 못하므로 broker ACL이 인증된
  DRONE-01 principal을 aegisq/cmd/DRONE-01 구독과
  aegisq/cmd-ack/DRONE-01 발행에만 묶어야 한다. 다른 vehicle 토픽과 wildcard
  권한을 허용하지 않는다.
- RemoteMissionCoordinator는 단일 worker에서만 호출한다. PX4 command path의
  소유자는 하나여야 한다.
- MAVSDK MissionExecutor는 검증된 Riposte 실행 로직을 어댑트한다. 같은 이륙,
  Offboard, watchdog, 착륙 코드를 MQTT용으로 복사하지 않는다.
- 현재 코덱은 schemaVersion이 없는 AegisQ implicit-v1 EXECUTE_MISSION만 엄격히
  허용한다. root 필드는 commandId, command, missionId, waypoints, issuedBy,
  issuedAt이고 waypoint 필드는 seq, lat, lng, action, dwellSec, altitude다.
  알 수 없거나 빠진 필드, 중복 JSON key, retained/QoS 0 메시지를 거부한다.
  commandId와 missionId는 canonical UUID 문자열이며 숫자 카운터가 아니다.
- altitude는 mission home 기준 상대고도다. seq는 정렬 후 1부터 연속이어야 하며,
  첫 action은 TAKEOFF, 마지막은 LAND, 중간은 MOVE 또는 HOVER다. HOVER만 0보다
  큰 dwellSec를 가질 수 있고 나머지는 0이어야 한다. waypoint 수, WGS84 범위,
  상대고도, 개별/총 dwell, payload 크기, JSON 깊이, issuedBy 길이와 issuedAt
  freshness 제한은 런타임 설정으로 검증한다.
- 현재 AegisQ 데이터 모델은 K9/WHEEL/DRONE/PHONE mobilityClass를 구분하지만,
  서버 mission dispatch와 웹 mission 대상 선택은 아직 DRONE을 강제하지 않고
  공통 aegisq/cmd/{robotId} 형태를 쓴다. 따라서 exact topic과 드론 전용 schema는
  오배송 피해를 줄일 뿐 서버가 선택한 대상이 실제 DRONE임을 증명하지 않는다.
  서버/프론트엔드에 mobilityClass == DRONE 검증이 들어가기 전에는 운영 절차와
  broker ACL로 드론 id를 제한한다.
- 현재 코디네이터의 UUID 중복 방지는 프로세스 메모리 범위다. 같은 UUID와 같은
  decoded command 내용은 저장된 ACK를 재전송하고, 같은 UUID의 다른 내용은 거부하지만
  재부팅 뒤 QoS 1 재전송을 막지 못한다. 실행 전 명령 claim, inbox/outbox,
  결과와 ACK replay를 원자적으로 영속화하기 전에는 원격 비행을 활성화하지 않는다.
- ACK JSON은 commandId, status, ackedAt과 선택적인 missionId/error를 가진다.
  AegisQ backend에서 ACCEPTED/REJECTED는 mission을 terminal 상태로 만들지 않는다.
  COMPLETED/SUCCESS/OK만 완료, FAILED/ERROR만 실패로 반영된다. 현재 connector에는
  실제 비행 진행을 따라 COMPLETED/FAILED를 발행하는 terminal-status 경로가 없다.
- 현재 decoder는 EXECUTE_MISSION만 받는다. STOP_MISSION, RTL,
  EMERGENCY_STOP의 우선순위·선점·진행 중 mission 중단 경로를 구현하지 않은 채
  장시간 mission 실행을 켜지 않는다.
- 영상은 별도 drone_companion 프로세스가 카메라를 단독 소유하고 RTSP 및 선택적
  WHIP으로 보낸다. H.264 인코더는 하나만 사용하되 카메라/RTSP GstPipeline과 WHIP
  GstPipeline은 bounded appsink/appsrc bridge로 격리한다. WHIP의 signalling 상태
  전이나 오류가 RTSP를 멈추게 하는 단일 파이프라인으로 되돌리지 않는다.
  ENABLE_WHIP 기본값은 0이고, 정확한 /w/<server-issued-stream_key> endpoint와 선택
  token은 root 전용 /etc/default/drone-companion에만 둔다. Pi는 Ubuntu의 안정
  GStreamer 1.24 계열을 유지하며 API/ABI 불안정 1.29.x로 교체하지 않는다.
  MQTT 비행 프로세스는 카메라/GStreamer/Hailo에 의존하지 않는다.

런타임 템플릿은 config/mqtt-px4-control.env.example, 서비스 템플릿은
systemd/mqtt-px4-control.service에 있다. REMOTE_FLIGHT_EXECUTE 기본값은 0이며,
MQTT 서비스는 riposte-agent.service와 동시에 PX4 링크를 소유할 수 없다.
현재 mqtt_px4_control 실행 타깃은 OFF이며 서비스는 설치·enable·배포·실행한
상태가 아니다. 자세한 구성과 활성화 차단 조건은
docs/mqtt-px4/ARCHITECTURE.md를 본다.

공용 계층 변경은 최소한 flight core 및 remote command 테스트를 컴파일하고,
호스트 실행 파일을 만들 수 있는 환경에서는 ctest까지 실행한다. Android NDK
교차 빌드는 컴파일 검증일 뿐 실행 검증이 아니다. Pi/PX4에서 실행하지 않았다면
실기 검증을 주장하지 않는다.

드론 온보드 컴패니언 소프트웨어(C++17). 구성·실측치·제약은 [README.md](README.md),
Pi 셋업의 함정은 [docs/setup_notes.md](docs/setup_notes.md) 에 있다 — 이 파일은 작업 방식만 다룬다.

## 실행 환경

**Raspberry Pi 5 / Ubuntu 24.04 / kernel 6.8.0-1060-raspi** 에서만 돈다. 개발은 보통 Windows PC
에서 하고 SSH 로 Pi 에 올려 검증한다. 기체마다 구성이 다르다 — Hailo 유무, 디스플레이 유무,
카메라 센서(imx219 / ov5647 …)가 제각각이니 **특정 기체의 구성을 코드에 못 박지 말 것.**

**Pi 에서 실제로 돌려보지 않은 것을 동작한다고 보고하지 말 것.** 이 저장소의 설계 결정은 거의 전부
"돌려보니 안 되더라" 에서 나왔다 — 문서만 보고 맞을 것 같다는 판단이 여러 번 틀렸다.

## 변경 시 반드시 확인할 것

파이프라인을 건드렸으면 **처리량을 재라.** 이 시스템은 조용히 느려진다:

```bash
journalctl -u drone-companion -f      # "캡처 N(xx.xfps) 추론 N(xx.xfps) ... 드롭 N"
```

정상은 캡처 ≈ 추론 ≈ **29~30 fps, 드롭 0**. 한 자리 수로 떨어졌다면 caps 협상을 먼저 의심한다
(`tee` 분기의 framerate 고정, 다중 스트림 분기의 `queue` 누락 — README 참조).

**CPU 만 보고 개선을 판단하지 말 것.** 프레임이 안 흐르면 CPU 도 낮다. 반드시 fps 와 함께 본다.

화면 출력을 바꿨으면 눈으로 확인한다(SSH 에서도 가능):

```bash
./scripts/grab_fb.sh && scp pi:/tmp/fb.png .
```

RTSP 를 바꿨으면 스트림에서 되돌려 받아 확인한다:

```bash
gst-launch-1.0 rtspsrc location=rtsp://<pi>:8554/cam0 latency=200 protocols=tcp \
  ! rtph264depay ! h264parse ! avdec_h264 ! jpegenc ! multifilesink location=f_%02d.jpg
```

## 선택 의존성이 셋이다 — 조합을 항상 함께 고려한다

**Hailo 유무 · 디스플레이 유무 · MAVSDK 유무**가 각각 독립적으로 갈린다. 기체마다 구성이 다르므로
새 기능을 넣을 때 없는 조합에서도 빌드·실행되는지 보라.

- HailoRT 는 선택 의존성이다. `#include <hailo/...>` 는 `src/hailo_inference.cpp` 안에서만 쓴다 —
  다른 파일에 새면 미장착 기체에서 빌드가 깨진다.
- MAVSDK 도 선택 의존성이다. 없으면 비행 앱만 빠지고 나머지는 빌드된다.
- 검출 결과에 의존하는 코드는 **통과 모드에서 빈 결과**를 받는다는 전제로 작성한다.

## 비행 코드(Riposte)를 건드릴 때

`include/riposte/`, `src/riposte/` 는 검증된 안전 로직을 담고 있다. 다음을 지킬 것:

- **경고 정책을 완화하지 말 것.** 비행 타깃은 `-Wconversion -Werror` 로 빌드한다. 암시적 형 변환이
  궤적·판정 계산에서 실제로 위험하기 때문이며, 영상 쪽보다 의도적으로 엄격하다.
- **비행 앱에 GStreamer/Hailo 의존을 들이지 말 것.** 원본이 EO/IR 을 일부러 뺀 이유가 SITL 장비와
  실기체에서 같은 바이너리를 쓰기 위함이다. 이 성질을 깨면 시험 결과에 영상 부하가 섞인다.
- **로직은 헤더 온리·MAVSDK 무의존을 유지할 것.** 그래야 `tests/riposte/` 가 하드웨어 없이 돈다.
  MAVSDK 코드는 실행 파일 또는 `MissionExecutor` 포트 뒤의 전용 adapter에만 두고
  MQTT codec/source나 공용 비행 헤더로 새지 않게 한다.
- 안전 관련 변경은 `ctest` 를 반드시 돌린다. 실기 시험 전 회귀를 잡는 유일한 수단이다.
- **MAVSDK 계층에는 단위시험이 없다.** 모의 객체가 없어 `MavsdkMissionBackend` 와
  `MavsdkOffboardControl` 은 SITL 로만 검증된다. 그쪽을 고쳤으면 ctest 통과는 아무것도
  증명하지 않으므로 그렇게 보고한다.
- **로컬 안전 동작에 일반 예외를 재사용하지 말 것.** 일반 예외는 전부 비상 착륙으로 흐른다.
  착륙이 정답이 아닌 상황(수동 인계가 그 예다)은 `PilotOverrideDetected` 처럼 **별도 타입**을
  만들어 그보다 앞에서 잡아야 한다. 타입을 재사용하면 조용히 착륙 경로로 떨어진다.
- **감독자 우선순위의 단일 원본은 `PreemptiveMissionExecutor::priority()` 다.** 새 안전 동작을
  넣으면 그 switch 에 값을 추가한다. 빠뜨리면 우선순위 0 으로 떨어져 선점이 조용히 죽는다.
- **워치독이 상태 플래그를 먼저 지울 수 있다.** setpoint 워치독은 발동할 때 `offboard_started_`
  를 인라인으로 내린다. 그 플래그에 의존하는 검사를 새로 넣으면 워치독과 경합해서 조용히
  건너뛰어진다 — `offboard_intended_` 처럼 의도적 종료에서만 내려가는 별도 플래그를 쓸 것.

## 카메라는 단일 소유자다

libcamera 는 카메라를 한 프로세스만 연다. 새 소비자가 필요하면 **반드시 기존 파이프라인의 `tee` 에
분기를 붙여라.** 두 번째 프로세스를 띄우면 조용히 실패하거나 기존 스트림을 죽인다.

## 코드 스타일

- C++17, 4-space 들여쓰기, `.cpp`/`.hpp`.
- 네임스페이스 `dc`. 공개 헤더는 `include/dc/`, 구현은 `src/`.
- 주석은 **왜** 를 적는다. 무엇을 하는지는 코드가 말한다. 이 저장소의 주석 대부분은
  "이렇게 안 하면 이런 식으로 깨진다" 를 기록한 것이다 — 그 형식을 유지할 것.
- 동작 파라미터는 전부 환경변수(`Config`)로 뺀다. 하드코딩하지 말 것.

## 커밋

Conventional Commits (`feat:`, `fix:`, `perf:`, `docs:`, `build:`). 성능에 영향 있는 변경은
커밋 메시지에 **변경 전/후 fps 와 CPU 를 적는다** — 이 저장소에서 가장 자주 잃어버리는 정보다.
