# drone_companion

드론 온보드 컴패니언 컴퓨터 소프트웨어 (Raspberry Pi 5 / Ubuntu 24.04, C++17).

세 계통을 담당한다.

| 계통 | 실행 파일 | 의존성 | 하는 일 |
|---|---|---|---|
| **영상** | `drone_companion` | GStreamer, cairo, *(HailoRT)* | 카메라 → 객체검출 → DSI 패널 + RTSP/WHIP 송출 |
| **비행** | `riposte_test`, `riposte_gcs` | *(MAVSDK ≥ 3.17.1)* | PX4 시험비행·미션 실행, 지상국 연동 |
| **원격 미션** | `mqtt_px4_control` | *(MAVSDK, libmosquitto, SQLite3, nlohmann_json)* | AegisQ 서버의 MQTT 명령 → PX4 미션 실행 (§3.5) |

기울인 의존성은 **선택**이다. 없으면 CMake 가 경고를 내고 해당 기능을 뺀 채 빌드한다 — 기체마다
Hailo 유무, 비행 제어 유무가 다르기 때문이다. 기하·판정 단위시험은 MAVSDK 없이도 항상 빌드·실행된다.

태블릿 GCS(`SkyHunter-Android-GCS`)는 RTSP 로 영상을 받는다. 검출 박스는 영상에 burn-in 되어
전송되므로 앱 쪽에 별도 처리가 필요 없다.

```
[검출 모드 — Hailo 있음]
libcamerasrc ┬ cam.src   1280x720 NV12 ─ BGRA ─ cairooverlay ◀─┐
             │                                 │               │ NMS 결과
             │                                 ├─ 800x480 ─ fbdevsink   (DSI 패널)
             │                                 └─ I420 ─ x264enc ─ RTSP (태블릿)
             └ cam.src_1  640x360 RGB ─ 패딩 ─ 640x640 ─────────┘ libhailort (인프로세스)
                          └ ISP 가 하드웨어로 스케일·색변환

[통과 모드 — Hailo 없음/불가]
libcamerasrc ─┬─ 800x480 ─ fbdevsink
              └─ x264enc ─ RTSP            BGRA 변환·cairooverlay 생략
```

---

# 1. Raspberry Pi 5 처음부터 설정하기

검증 기준: **Raspberry Pi 5 / Ubuntu 24.04.4 LTS / kernel 6.8.0-1060-raspi / arm64**.

전부 `scripts/install.sh` 가 자동으로 하지만, 각 단계가 왜 필요한지와 실패했을 때 수동으로
무엇을 하면 되는지를 아래에 적는다. 더 깊은 진단 절차는 [docs/setup_notes.md](docs/setup_notes.md).

```bash
git clone <repo> && cd drone_companion
sudo ./scripts/install.sh            # 1~7 단계를 전부 수행 (libcamera 빌드 포함, 20~30분)
sudo SKIP_LIBCAMERA=1 ./scripts/install.sh   # libcamera 가 이미 있으면
sudo SKIP_MAVSDK=1   ./scripts/install.sh   # 비행 기능이 필요 없으면
```

## 1.1 apt 소스에 `noble-updates` 가 있어야 한다

없으면 **의존성이 통째로 깨진다.** 증상이 엉뚱하게 나타나므로 먼저 확인한다:

```
bzip2 : Depends: libbz2-1.0 (= 1.0.8-5.1) but 1.0.8-5.1build0.1 is to be installed
```

설치된 라이브러리는 updates 버전인데 apt 목록에는 릴리스 버전만 있어 짝이 맞는 패키지를 찾지
못하는 상황이다. 그 뒤로 `dpkg-dev` → `build-essential` → Hailo 드라이버가 연쇄로 실패한다.
`apt update` 를 다시 돌려도 소용없다 — 포켓 자체가 없어서다.

```bash
sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates/' /etc/apt/sources.list.d/ubuntu.sources
sudo apt-get update
```

## 1.2 패키지

```bash
sudo apt-get install -y --no-install-recommends \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
  gstreamer1.0-rtsp gstreamer1.0-x \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libcairo2-dev libglib2.0-dev \
  build-essential cmake pkg-config v4l-utils curl bc
```

`gstreamer1.0-x` 가 필요한 이유: Ubuntu 는 `textoverlay`/`timeoverlay`(pango 플러그인)를 이 패키지에
넣어 두었다. 이름만 보면 X11 전용 같지만 아니다.

## 1.3 Hailo-8 드라이버 (검출을 쓸 때만)

`hailort` deb 는 Hailo Developer Zone 에서 받는다. **`dkms` 를 먼저 설치할 것** — 없으면
드라이버가 DKMS 트리 대신 직접 빌드로 떨어져(`Failed to install PCIe driver to the DKMS tree`)
**커널 업데이트 때 모듈이 사라진다.**

```bash
sudo apt-get install -y build-essential dkms linux-headers-"$(uname -r)"
sudo dpkg -i hailort_*.deb hailort-pcie-driver_*.deb
dkms status                       # hailo_pci/4.24.0, ...: installed
hailortcli fw-control identify    # Device Architecture: HAILO8
```

## 1.4 libcamera — Pi 5 는 소스 빌드가 필수다

**Ubuntu 24.04 의 libcamera 패키지로는 Pi 5 카메라가 동작하지 않는다.** 커널은 센서를 정상
인식하는데 `cam -l` 이 비어 있고, 디버그를 켜면 이렇게 나온다:

```
Found registered pipeline handler 'PipelineHandlerVc4'
RPI vc4.cpp:179  Unable to acquire a Unicam instance
```

Ubuntu 패키지에는 Pi 4 이하용 `ipa_rpi_vc4.so` 만 있고 Pi 5 의 PiSP 용 `ipa_rpi_pisp.so` 와
`/usr/share/libcamera/ipa/rpi/pisp/` 가 통째로 없다. `libpisp` 도 저장소에 없다.
**설정 문제가 아니라 구성요소 부재**라 config.txt 나 오버레이로는 해결되지 않는다.

```bash
sudo ./scripts/build_libcamera.sh     # libpisp + rpi 포크 libcamera 소스 빌드 (15~25분)
```

버전은 커널과 시기를 맞춘다. 커널 6.8 계열에는 **libcamera `v0.3.2+rpt20241119` + libpisp `v1.0.7`**.
최신 태그(2026년)는 커널보다 앞서 ABI 가 어긋날 위험이 있다.

빌드 전에 Ubuntu 의 libcamera 패키지를 제거한다(구버전 GStreamer 플러그인과 충돌). 스크립트가 처리한다.

## 1.5 카메라 연결 — 커넥터 · 오버레이 · i2c 버스

**CSI 카메라는 전원을 끄고 연결한다.** 동작 중 핫플러그는 커넥터·센서 손상 위험이 있고,
`camera_auto_detect` 는 부팅 시점에만 동작하므로 어차피 재부팅이 필요하다.

Pi 5 에는 커넥터가 둘이고, 오버레이에 **어느 커넥터인지까지** 적어야 한다:

| 커넥터 | 오버레이 | 센서가 뜨는 i2c 버스 | CSI 블록 |
|---|---|---|---|
| CAM/DISP 0 | `dtoverlay=<센서>,cam0` | `i2c-10` | `1f00110000.csi` |
| CAM/DISP 1 | `dtoverlay=<센서>,cam1` | `i2c-11` | `1f00128000.csi` |

`/boot/firmware/config.txt`:

```
camera_auto_detect=1          # 공식 모듈이면 대개 이것으로 잡힌다
# 또는 명시 지정 (자동 감지가 놓칠 때)
camera_auto_detect=0
dtoverlay=imx219,cam0
```

**imx219(Camera Module v2)는 자동 감지가 놓치는 사례를 확인했다.** 같은 모듈이 `camera_auto_detect=1`
로는 전혀 안 잡히다가 `dtoverlay=imx219,cam0` 을 넣으니 바로 30 fps 로 동작했다. imx219 를 쓸 때는
명시 지정을 권한다.

확인:

```bash
sudo ./scripts/camera_setup.sh --snapshot     # 센서·IPA·권한·부팅설정·캡처·ISP 2nd 스트림 일괄 점검
```

## 1.6 사용자 권한

```bash
sudo usermod -aG video,render,dialout sky      # 재로그인 필요
```

- `video` : `/dev/media*`, `/dev/video*`, `/dev/fb0` — 없으면 libcamera 가 카메라를 열거하지 못한다
- `render`: DMA 버퍼
- `dialout`: Pixhawk 시리얼 (`/dev/ttyAMA0`)

## 1.7 MAVSDK ≥ 3.17.1 (비행을 쓸 때만)

```bash
sudo ./scripts/install_mavsdk.sh                    # 기본: 3.17.1 deb 를 받아 설치
sudo MAVSDK_VERSION=3.19.0 ./scripts/install_mavsdk.sh   # 더 최신을 쓰고 싶으면
```

**3.17.1 은 고정 버전이 아니라 최소 버전이다.** `find_package(MAVSDK 3.17.1)` 은 "이 버전 이상"을
요구하므로 더 높은 3.x 가 깔려 있으면 그대로 쓰면 된다. 배포판 패키지로 설치했든 소스로 빌드했든
상관없다.

이 값이 바닥인 이유는 코드가 **v3 에서 의미가 바뀐 것들**에 의존하기 때문이다 —
`remaining_percent` 가 0~1 비율이 아니라 0~100 퍼센트, `CompanionComputer` 기본 컴포넌트 ID 195
등(§9, `docs/riposte/GCS_USAGE.md`). 그 아래 버전에서는 빌드는 되더라도 **조용히 오동작한다.**
메이저가 올라가면(4.x) 이 의미들을 다시 확인할 것.

설치 스크립트가 3.17.1 을 받는 것은 **검증된 조합이라서**이지 다른 버전이 안 되기 때문이 아니다.
`MAVSDK_VERSION` 으로 바꿀 수 있고, 이미 설치돼 있으면 건드리지 않고 그대로 쓴다.

**Ubuntu 24.04 arm64 용 공식 deb 가 없다.** 릴리스에 arm64 는 debian11/12 만, ubuntu24.04 는
amd64 만 있다. 그래서 **debian12 arm64 빌드**를 쓴다 — 더 오래된 glibc 로 빌드된 바이너리는 최신
시스템에서 동작하며, Pi 5 / Ubuntu 24.04 에서 빌드·단위시험 통과를 확인했다.
(Raspberry Pi OS Bookworm 은 Debian 12 자체이므로 이 deb 가 네이티브 타깃이다.)

설치 확인:

```bash
find /usr -name MAVSDKConfig.cmake        # /usr/lib/cmake/MAVSDK/MAVSDKConfig.cmake
```

> `MAVSDK::mavsdk_action` 같은 플러그인별 타깃은 없는 게 정상이다. MAVSDK 3.x 는 단일
> 라이브러리로 통합됐고, CMake 쪽은 `if(TARGET ...)` 로 방어돼 있다.

## 1.8 Pixhawk UART

Pi 5 는 `enable_uart=1` 만으로는 부족하고 **`dtoverlay=uart0-pi5` 가 있어야 `/dev/ttyAMA0` 이 노출된다.**
(`/dev/ttyAMA10` 은 디버그 콘솔용이라 FC 연결에 쓰지 않는다.)

```
enable_uart=1
dtoverlay=uart0-pi5
```

`install.sh` 가 없으면 추가한다. 반영에 재부팅이 필요하다.

### 기체 배선

```
                    ┌─ TELEM1 ── KOA telemetry ─────────> 지상국 / 태블릿
Pixhawk (PX4) ──────┼─ TELEM2 ── GPIO 14/15 ────────────> Raspberry Pi 5  /dev/ttyAMA0
                    └─ TELEM3 ── RC 수신기
```

**Pi 는 TELEM2 에만 물린다.** 지상국은 TELEM1(KOA telemetry)로 붙고, 태블릿↔컴패니언 통신은
**PX4 가 두 링크 사이를 중계**한다(§3.4). 따라서 **Pi 에 MAVLink 라우터를 둘 필요가 없다** —
`riposte_test` 가 `/dev/ttyAMA0` 을 단독으로 쓰는 현재 구성이 의도된 형태다.

TELEM3 의 RC 수신기는 `RC_PORT_CONFIG` 계열로 잡힌다. `MAV_2_CONFIG` 를 TELEM3 에 할당하지 말 것.

## 1.9 디스플레이

DSI 패널이 있으면 `/dev/fb0` 로 잡히고 `fbdevsink` 로 출력한다. **`kmssink` 는 쓰지 않는다** —
커넥터가 없는 vc4(HDMI) 카드를 잡아 `allowed caps is empty` 로 죽는다.

디스플레이가 없는 기체는 `ENABLE_DISPLAY=0` 이어야 파이프라인이 뜬다. `install.sh` 가 `/dev/fb0`
부재를 감지해 자동으로 설정한다.

---

# 2. 빌드

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4
ctest --test-dir build --output-on-failure        # 기하·판정 회귀 (MAVSDK 불필요)
```

원격 미션 에이전트는 프로필 기본값이 OFF 라 따로 켜야 한다. MAVSDK 가 필요하고
`DC_BUILD_MQTT_TRANSPORT=ON`(Linux 전용)을 전제한다:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DDC_BUILD_MQTT_PX4_AGENT=ON
cmake --build build -j4                           # build/mqtt_px4_control
```

configure 로그에서 어떤 기능이 포함됐는지 확인한다:

```
-- HailoRT 발견: /usr/lib/libhailort.so — 검출 기능 포함
-- MAVSDK 3.17.1 발견 — 비행 앱 포함
```

없으면 경고가 뜨고 해당 기능만 빠진다.

---

# 3. 서비스와 운용

| 서비스 | 내용 | 자동시작 |
|---|---|---|
| `mediamtx` | RTSP 서버 (:8554) | enable |
| `drone-companion` | 카메라 → 검출 → 패널 + RTSP publish | enable |
| `riposte-agent` | 비행 에이전트, 지상국 요청 대기 | enable |
| `mqtt-px4-control` | AegisQ MQTT 명령 → PX4 미션 실행 (§3.5) | 수동 |

`riposte-agent`·`mqtt-px4-control`·`aegisq_agent` 는 서로 `Conflicts=` 로 묶여 있다. PX4 링크의
소유자는 하나여야 하고, 뒤의 둘은 MQTT 클라이언트 id 까지 같아서 동시에 붙으면 브로커가 번갈아
끊는다. 하나를 start 하면 systemd 가 나머지를 자동으로 stop 한다.

```bash
sudo systemctl start drone-companion
journalctl -u drone-companion -f      # "캡처 N(xx.xfps) 추론 N(xx.xfps) ... 드롭 N"
./scripts/grab_fb.sh                  # 현재 패널 화면을 /tmp/fb.png 로 (SSH 확인용)
```

태블릿 수신 주소: **`rtsp://<드론IP>:8554/cam0`** (카메라 없이 앱 연동만 볼 때는 `/test`).
경로명 `cam0` 은 GCS 앱의 `UnitNetworkConfig.DEFAULT_CAM_PATH` 와 맞춘 것이라 바꾸면 404 가 난다.

## 3.1 비행 에이전트의 안전 게이트 — 반드시 읽을 것

`riposte-agent` 는 부팅 시 자동으로 뜨며 지상국의 요청을 대기한다. **이중 `--execute` 게이트**가
있어 에이전트만으로는 기체가 움직이지 않는다 — 지상국(`riposte_gcs`)도 자기 쪽 `--execute` 로
요청해야 비행이 시작된다.

그래도 **정비·지상 점검 중이라 기체가 절대 움직이면 안 되는 상황**에서는 에이전트 쪽 게이트를 닫는다:

```bash
sudo systemctl edit riposte-agent      # [Service] Environment=RIPOSTE_EXECUTE=0
sudo systemctl restart riposte-agent
systemctl show riposte-agent -p Environment | grep RIPOSTE_EXECUTE   # 확인
```

`RIPOSTE_EXECUTE=0` 이면 연결·대기·보고만 하고 기체에 어떤 명령도 보내지 않는다.

링크는 `RIPOSTE_CONNECTION` 으로 바꾼다 — Pixhawk UART `serial:///dev/ttyAMA0:921600`,
PX4 SITL `udpin://0.0.0.0:14540`.

FC 가 Pi 보다 늦게 부팅하거나 아직 연결되지 않았으면 `open failed: No such file or directory` 를
내며 5 초마다 재시도한다. 정상 동작이다.

## 3.2 설정 (환경변수)

기본값은 [systemd/drone-companion.service](systemd/drone-companion.service) 와
[systemd/riposte-agent.service](systemd/riposte-agent.service) 에 있다. 하드코딩하지 말 것.

| 변수 | 뜻 |
|---|---|
| `SRC_W`/`SRC_H` | 카메라 해상도 (기본 1280x720) |
| `SCORE_TH`, `CLASS_NAMES`, `NUM_CLASSES`, `HEF` | 검출 |
| `RTSP_URL`, `RTSP_BITRATE`, `RTSP_FPS` | RTSP 송출 |
| `ENABLE_DISPLAY`, `ENABLE_RTSP`, `ENABLE_WHIP` | 출력 on/off |
| `WHIP_URL`, `WHIP_TOKEN` | 서버 발급 WHIP endpoint와 선택 bearer token. [운영 가이드](docs/streaming/WHIP.md) |
| `WHIP_RETRY_SECS` | RTSP와 분리된 WHIP publisher 재시작 주기(1~300초, 기본 15초) |
| `DUAL_STREAM` | ISP 2nd 스트림 사용 (0 이면 tee 폴백) |
| `HAILO`, `HAILO_WAIT_SECS` | 검출 강제 해제 / 부팅 시 장치 대기 |
| `RIPOSTE_CONNECTION`, `RIPOSTE_EXECUTE` | 비행 링크 / 비행 명령 게이트 |

## 3.3 PX4 파라미터 — 이게 없으면 태블릿이 컴패니언을 못 본다

태블릿(지상국)과 컴패니언은 **PX4 를 경유**해 대화한다. PX4 가 TELEM1↔TELEM2 를 중계하도록
포워딩을 켜야 한다. QGroundControl 파라미터 화면에서 설정한다.

| 파라미터 | 값 | 의미 |
|---|---|---|
| `MAV_0_CONFIG` | TELEM1 | 지상국 링크 (KOA telemetry) |
| `MAV_0_FORWARD` | 1 | 지상국 링크 수신 메시지를 다른 링크로 전달 |
| `MAV_1_CONFIG` | TELEM2 | 컴패니언 링크 (Pi) |
| `MAV_1_MODE` | Onboard | 컴패니언용 스트림 구성 |
| `MAV_1_FORWARD` | 1 | 컴패니언 링크 수신 메시지를 다른 링크로 전달 |
| `SER_TEL2_BAUD` | 921600 | 컴패니언 UART 속도 (`riposte-agent` 기본값과 일치해야 한다) |

**변경 후 재부팅해야 적용된다.**

포워딩이 꺼져 있으면 증상이 특징적이다 — `riposte_gcs` 가 **`component 191 is not visible`** 경고를
내고 요청 ACK 가 오지 않는다. 태블릿에서는 시험 요청이 응답 없이 타임아웃된다.

**PX4 기본 arming 체크는 GCS 연결을 요구한다.** 태블릿이나 QGC 가 붙어 있지 않으면 `Arming denied`
가 난다. 반복 시험(예: x10)은 사이클마다 재-Arm 하므로 **중간에 지상국 링크가 끊기면 그 시점부터
재이륙이 실패**한다.

## 3.4 비행 시험 실행

```bash
riposte_test --list                               # 카탈로그 23종
riposte_test --test 21 --plan-only --plan-home 36.6606 126.3417   # 기체 없이 경로만 산출
riposte_test --test 1 --execute --connection serial:///dev/ttyAMA0:921600
```

`--execute` 가 없으면 기체에 어떤 명령도 보내지 않는다. 지상국 사용법은
[docs/riposte/GCS_USAGE.md](docs/riposte/GCS_USAGE.md), 시험 절차는
[docs/riposte/TEST_USAGE.md](docs/riposte/TEST_USAGE.md).

## 3.5 원격 미션 에이전트 (`mqtt-px4-control`)

AegisQ 서버가 MQTT 로 보낸 미션을 받아 PX4 로 실행한다. 계약과 활성화 조건은
[docs/mqtt-px4/ARCHITECTURE.md](docs/mqtt-px4/ARCHITECTURE.md).

### 설치

프로필 기본값이 OFF 이므로 `install.sh` 는 이것을 설치하지 않는다. 수동으로 넣는다:

```bash
sudo install -m755 build/mqtt_px4_control            /usr/local/bin/
sudo install -m644 systemd/mqtt-px4-control.service  /etc/systemd/system/
sudo install -d -m755 /etc/drone-companion
sudo install -m640 -g sky config/mqtt-px4-control.env.example \
     /etc/drone-companion/mqtt-px4-control.env
sudo systemctl daemon-reload
```

원장(SQLite)이 사는 `/var/lib/drone-companion` 은 unit 의 `StateDirectory=` 가 만든다.
에이전트는 디렉터리를 만들지 않으므로 부모가 없으면 기동에 실패한다.

### 설정

`/etc/drone-companion/mqtt-px4-control.env` 를 채운다. **모든 변수가 필수다** — 비었거나 없으면
변수 이름을 찍고 종료한다(exit 2). 컴파일된 기본값이 없는 이유는 기체 한계값의 기본값이란
아무도 검토하지 않은 정책이기 때문이다. mTLS 인증서, 브로커, 항로 한계, 원장 경계, 그리고
`REMOTE_FLIGHT_EXECUTE=1` 일 때만 읽는 `PX4_*`·`FLIGHT_*` 튜닝이 들어 있다.

주요 검사(전부 기동 전에 걸린다): 브로커는 `mqtts://<IPv4>:8883` 이어야 하고 host 가
`WIREGUARD_SERVER_IP` 와 같아야 한다. `MQTT_CLIENT_ID` 는 `MQTT_VEHICLE_ID` 와 같아야 한다.
`MQTT_COMMAND_TOPIC`/`MQTT_COMMAND_ACK_TOPIC` 은 vehicle id 에서 파생된 값과 일치해야 한다 —
템플릿에 적혀 있는 것은 읽으라고 둔 것이지 설정하라고 둔 것이 아니다.

### 안전 게이트 — 반드시 읽을 것

`REMOTE_FLIGHT_EXECUTE` 가 유일한 게이트다.

| 값 | 동작 |
|---|---|
| `0` | **MAVSDK 를 아예 열지 않는다.** 시리얼도 잡지 않고 MAVLink 도 내보내지 않는다. 명령은 받아서 `ACCEPTED` → `FAILED("remote_flight_execute_disabled")` 로 정직하게 답한다 |
| `1` | **기체가 실제로 난다.** |

게이트가 `1` 이면 첫 동작이 재시작 복구다. 이미 지상·DISARMED이면 명령을 내지 않는다.
그 밖의 ARMED/공중 상태에서는 재시작 전 pilot latch가 메모리에서 사라졌기 때문에 CH6=HOLD조차
Pi 소유의 증거가 아니다. 전부 owner UNKNOWN으로 간주해 DISARM/RTL/LAND를 보내지 않고 latch를
건다. 독립 안전층은 PX4의 Offboard-loss 설정이다. 게이트를 열어두면 부팅만으로 이 판정이
수행되지만, 부팅 자체가 공중 기체에 새 명령을 만들지는 않는다.

**링크 손실 시 자율 복귀 (`MQTT_LINK_LOSS_TIMEOUT_S`).** 브로커 세션이 이 시간 이상 끊기면
공중에 있는 기체가 스스로 RTL 한다. 이게 필요한 이유는 **PX4 가 이 상황을 datalink loss 로
보지 않기 때문**이다 — Pi↔PX4 MAVLink 는 멀쩡하므로 PX4 의 failsafe 는 발동하지 않고, 서버는
STOP 도 RTL 도 보낼 수 없는데 기체는 남은 항로를 계속 난다.

| 상태 | 동작 |
|---|---|
| 공중 + home 유효 | 안전 큐에 RTL 삽입, 진행 중 항로 선점. 복귀 지점은 PX4 가 무장 시 잡은 이륙 지점이다 |
| 공중 + home 무효 | RTL 을 걸지 않고 `FATAL` 로그. 목적지가 없는 복귀는 더 위험하다 |
| 지상 / 미이륙 | 기록만 한다 |
| `REMOTE_FLIGHT_EXECUTE=0` | 기록만 한다 (PX4 에 명령할 수 없다) |

우선순위는 배터리 복귀보다 **낮다** — 조종 링크를 잃은 것은 회복 가능하지만 배터리를 바닥내는
것은 아니다. 재접속하면 래치가 풀리고 `broker session restored` 가 남는다.

**반응 시간은 `MQTT_KEEPALIVE_SEC` 이 지배한다.** 조용히 끊긴 경로는 TCP 가 오래 못 알아채므로
MQTT 자체 하트비트가 감지를 만든다. 실측(2026-08-13, Pi):

| `MQTT_KEEPALIVE_SEC` | keepalive 몫 | `MQTT_LINK_LOSS_TIMEOUT_S` | 전체 |
|---|---|---|---|
| 30 | 45.2초 | 10 | 55.2초 |
| 10 | 15.3초 | 10 | 25.3초 |
| **10** | **15.3초** | **5** | **20.3초** ← 현재 배포값 |

즉 keepalive 몫이 `1.5 × MQTT_KEEPALIVE_SEC` 다. 타이머만 줄여도 전체는 별로 안 줄어든다.

**지오펜스 복귀 (`GEOFENCE_RADIUS_M`).** 이륙 지점에서 **실측 수평거리**가 이 값을 넘으면
스스로 RTL 한다. 기존 `FLIGHT_MAX_ROUTE_RADIUS_M`(60 m)와 층위가 다르다 — 그쪽은 *우리가
명령할* 범위를 막는 게이트라 항로 웨이포인트와 생성된 setpoint 만 검사하고, 바람·표류·GPS
오차·수동 조종으로 **실제로** 밖에 나간 경우는 못 잡는다. 위반 시 동작도 다르다: 기존 게이트는
`throw` 라 그 자리 비상 착륙이고, 이쪽은 RTL 이다.

**연결 상태와 무관하게 2초(`BATTERY_SUPERVISOR_INTERVAL_MS`) 주기로 검사한다.** 브로커가
끊긴 중에 50 m 를 넘으면 위의 20.3초를 기다리지 않고 최대 2초 안에 복귀가 걸린다. 먼저 건 쪽이
안전 큐를 차지하고 나중 것은 `safe_return_already_pending` 로 흡수된다.

**수동 인계 (pilot override).** `flightMode=POSCTL` 하나로는 판정하지 않는다. POSCTL은 CH6
POSITION일 수도 있고 Offboard-loss의 결과일 수도 있기 때문이다. 에이전트는 PX4 `RC_CHANNELS`의
CH6 PWM과 `RC_STATUS`를 직접 확인한다. 지상 START는 같은 `RC_CHANNELS` 프레임의 CH5가
debounced DISARM이고 PX4도 실제 `armed=false`일 때만 허용한다. fresh POSITION 한 샘플은
큐에 있던 START를 즉시 무효화하고 다음 비행-loop 검사에서 인계를 래치한다. 그 뒤에는 착륙,
RTL, 시동 정지 명령을 새로 만들지 않고 MAVSDK Offboard 플러그인을 파괴해 마지막 setpoint
재전송만 끊으며, **미션·STOP·RTL·E-STOP을 전부 `pilot_override_active`로 거부한다.** 다만 현재
동기 MAVSDK 호출이 이미 시작됐거나 마지막 검사 직후 진입한 경우는 취소할 수 없다. 이 짧은
경쟁 구간까지 없애려면 bounded 비동기 command dispatcher가 필요하며, 그 전에는 프로펠러 장착
시험을 승인하지 않는다.

| 해제 조건 | |
|---|---|
| 지상 + 시동 꺼짐 + fresh CH6 HOLD | 래치 해제 가능, 새 미션을 처음부터 시작 |
| 공중에서 CH6를 HOLD/OFFBOARD로 복귀 | **해제되지 않음** — 공중 resume 없음 |
| 스틱을 중앙으로 되돌림 | **해제되지 않음** — 반납이 아니다 |
| 서비스 재시작 | ARMED/공중이면 현재 CH6와 무관하게 owner UNKNOWN으로 래치. RTL/Disarm 하지 않는다 |

이 동작이 실제로 성립하려면 **PX4 쪽 설정이 맞아야 한다.** 코드가 아무리 맞아도 여기가
어긋나면 기체 거동이 달라진다. 2026-08-13 기체 실측값:

| 파라미터 | 값 | 뜻 |
|---|---|---|
| `COM_RC_OVERRIDE` | **3** | Auto(bit 0)·Offboard(bit 1) 양쪽에서 스틱 override 허용 |
| `COM_RC_STICK_OV` | **30%** | override 임계값. 낮으면 트림 오차로 미션이 중단되고, 높으면 급할 때 늦다 |
| `COM_OBL_RC_ACT` | **Position** | **Offboard 상실 시 조종사에게 넘긴다.** 착륙/RTL 이 아니다 |

`COM_OBL_RC_ACT=Position` 이 특히 중요하다. SITL 에서는 RC 가 없어 offboard 상실이 곧
착륙이었는데, 실기체에는 대체 조종원이 있으므로 그 경로가 발생하지 않는다. **대신 조종기를
아무도 잡고 있지 않으면 기체는 Position 으로 떠 있는 채 대기한다** — 에이전트가 죽거나 Pi
전원이 나가도 자율 복귀는 없다는 뜻이다.

`COM_RC_IN_MODE=4` 면 스틱이 비활성이라 쓰면 안 되고, `MAV_2_CONFIG` 가 RC 포트(TELEM3)를
점유해서도 안 된다. 이 기체 조종기에는 OFFBOARD/HOLD/POSITION 3단 CH6가 있다. 다만 지상에서
새 START를 준비할 때는 **HOLD에 둔다.** OFFBOARD는 Pi가 fresh current-position setpoint를
prime한 뒤 소프트웨어로 진입해야 한다. 구체 상태 전이·PWM 보정·시험 순서는
[`docs/mqtt-px4/GROUND-RC-HANDOVER-2026-08-18.md`](docs/mqtt-px4/GROUND-RC-HANDOVER-2026-08-18.md)를 따른다.

이 복귀는 서버 명령이 아니라서 **command ACK 를 만들지 않는다**(배터리 복귀와 같은 규칙 —
원장은 정규 UUID 인 `commandId` 를 요구한다). 서버는 텔레메트리의 `controlOwner`,
`readyForMission`, `hardware.ownershipSwitch`, `hardware.armSwitch`로 알게 된다. 현재 서버·웹은 이 필드를 아직
보존·게이트하지 않으므로 양쪽 수정 전에는 버튼 상태를 권한 증거로 쓰면 안 된다.

```bash
sudo sed -i 's|^REMOTE_FLIGHT_EXECUTE=.*|REMOTE_FLIGHT_EXECUTE=0|' \
     /etc/drone-companion/mqtt-px4-control.env
sudo systemctl restart mqtt-px4-control
```

### 실행과 확인

```bash
sudo systemctl start mqtt-px4-control        # enable 은 게이트를 정한 뒤에
journalctl -u mqtt-px4-control -f
```

정상 기동 로그:

```
[mqtt_px4_control] vehicle=DRONE-01 broker=10.100.0.1:8883 flight_execution=DISABLED (observe only)
[INFO] requesting vehicle recovery before serving commands
[INFO] vehicle recovery accepted: observe_only_no_px4_recovery
```

확인 요령:

```bash
sudo fuser -v /dev/ttyAMA0        # 게이트 0 이면 비어 있어야 정상
sudo ss -tnp | grep 8883          # 브로커 세션
```

**브로커 구독 성립은 로그에 따로 찍히지 않는다.** `MQTT_INITIAL_CONNECT_TIMEOUT_S` 를 넘겨
살아 있으면 QoS 1 SUBACK 을 받은 것이다 — 못 받으면 그 시점에 FATAL 로 죽는다. 브로커가
QoS 0 만 내주면 TCP 는 붙은 채 아무 오류 없이 명령이 오지 않으므로, 이 침묵을 실패로 취급한다.

원장 상태:

```bash
sudo sqlite3 /var/lib/drone-companion/commands.sqlite3 \
  'select command_id,state,detail,updated_at from command_ledger;'
```

검증 실패는 claim 이전에 걸러지므로 원장에 행을 남기지 않고 `outbox_only` 로 ACK 만 나간다.
원장이 비어 있는데 서버가 `REJECTED` 를 받았다면 정상이다.

### 전원이 끊긴 뒤

`ACCEPTED` 로 남은 행이 있으면 **PX4 복구가 확인되기 전까지 명령 서비스를 시작하지 않는다.**
전원이 끊겼을 때 기체가 무장 상태였거나 공중이었을 수 있기 때문이다. 게이트 `1` 로 FC 를 연결해
기동하면 지상·DISARMED 확인 시 그 행을 `FAILED("vehicle_safe_after_agent_restart")` 로 종료한다.
ARMED/공중이면 PX4 명령 없이 `FAILED("restart_owner_ambiguous_no_px4_command")` 로 종료하고
pilot latch를 유지한다. 게이트 `0` 으로는 풀 수 없다 — 확인하지 않은 기체 상태를 원장에
쓰지 않기 위해서다.

---

# 4. Hailo 유무 판정과 폴백

**파일이 있는지 보는 것만으로는 부족하다.** 장치 노드가 있어도 드라이버 버전 불일치, HEF 손상,
다른 프로세스의 장치 점유로 실패할 수 있다. 그래서 판정은 실제로 열어보는 것까지다:

1. 빌드에 HailoRT 가 포함되었는가
2. `HAILO` 환경변수, HEF 파일, `/dev/hailo*` 노드
3. **VDevice·HEF·vstream 을 실제로 생성** — 여기까지 성공해야 검출 모드

어느 단계에서 걸리든 통과 모드로 내려가고 사유가 로그 한 줄로 남는다:

```
[hailo] 검출 모드 — 장치 /dev/hailo0 | in ... 1228800B | out ... 4008B | classes 2 max/class 100
[hailo] 통과 모드 — Hailo 카드가 PCIe 에 없음 (미장착) — 빌드에도 HailoRT 미포함
[hailo] 통과 모드 — Hailo 카드는 PCIe 에 있으나 이 빌드에 HailoRT 지원이 없음
[hailo] 통과 모드 — HEF 모델이 없음: /home/sky/models/y5s_0223_6.hef
```

운용 중 추론이 실패해도 영상은 계속 나간다. 낡은 박스를 지우고 HUD 가
`INFERENCE DOWN — video only` 로 바뀐다.

## 부팅 경쟁 조건

부팅 직후에는 서비스가 udev 보다 먼저 뜰 수 있다(실측 마진 2 초). 검출 여부는 기동 시 한 번만
정하므로 순서가 뒤집히면 재부팅 내내 통과 모드로 남는다. 그래서 노드가 없으면 잠깐 기다린다
(`HAILO_WAIT_SECS`, 기본 15 s).

단순히 기다리기만 하면 **미장착 기체가 매 부팅마다 그만큼 늦게 영상을 내보낸다.** 그래서 기다리기
전에 PCIe 버스에 카드(vendor `0x1e60`)가 실제로 있는지 먼저 본다 — 없으면 대기 없이 즉시 통과 모드.

| 상황 | 동작 | 실측 판정 시간 |
|---|---|---|
| 카드 O · 노드 O | 바로 검출 모드 | 0.3 s |
| 카드 O · 노드 X | 노드가 뜰 때까지 대기 후 판정 | 대기시간 + 0.1 s |
| **카드 X (미장착)** | **대기 없이 즉시 통과 모드** | **0.3 s** |

---

# 5. 검증된 실측치

Pi 5 (BCM2712, A76×4 @2.4GHz) / Ubuntu 24.04.4 / kernel 6.8.0-1060-raspi / Hailo-8 / imx219

| 항목 | 값 |
|---|---|
| 캡처 = 추론 처리량 | **29~30 fps** (전 프레임 추론, 드롭 0~1) |
| CPU — 검출 모드 (표시+RTSP 포함) | **1.58 코어 / 4 = 40%**, RSS 115 MB |
| CPU — 통과 모드 (Hailo 없음) | **0.77 코어 / 4 = 19%** |
| 온도 | 53~57°C (스로틀 없음) |
| RTSP 출력 | H.264 Baseline 1280×720, 3 Mbps |
| Hailo 추론 단독 | 315.8 fps (`hailortcli run`) |
| 모델 | yolov5s, 입력 640×640×3 UINT8, **온칩 NMS**, 2 클래스 |

x264 소프트웨어 인코딩 (Pi 5 에는 **하드웨어 H.264 인코더가 없다** — Pi 4 의 인코딩 블록이
BCM2712 에서 제거됨):

| 해상도 | 최대 처리량 | 30fps 시 CPU | 판정 |
|---|---|---|---|
| 720p30 | 68~85 fps | 0.58~0.74 코어 | 채택 |
| 1080p30 | 31~38 fps | 1.12~1.38 코어 | 최악 장면에서 마진 없음 |

WHIP 분기 비용 (같은 바이너리·같은 조건에서 `ENABLE_WHIP` 만 바꿔 측정, 2026-08-13):

| | fps | CPU | RSS |
|---|---|---|---|
| WHIP on | 29.9 | 0.85 코어 | 88,436 KiB |
| WHIP off | 29.3 | 0.77 코어 | 68,076 KiB |

즉 분기 하나가 약 8%p·20 MiB 다. RTSP 처리량에는 영향이 없다.

WHIP 종단 송출 검증 (2026-08-13 12:09 KST, LiveKit ingress `IN_PJuGJLgqdXdG`):

| 항목 | 값 |
|---|---|
| 송신 | `10.100.0.13:50068 → 10.100.0.1:40077` UDP, **3,468 kbit/s** (20초 지속) |
| 패킷 크기 | 1210 B — `webrtcsink` 가 페이로더에 거는 `mtu=1200` 과 일치 |
| 서버 응답 | RTCP 46/34/128 B 수신 |
| 안정성 | 파이프라인 재시작 0회, RTSP 동시 정상 |

2026-08-13 12:09 은 자체 시험용 방(`whip-selftest-01`)이었고, 14:18 에 영구 카메라
`DRONE-01`(방 `stream-DRONE-01`, ingress `IN_A8uLoKpvySNi`)로 전환해 **서버 브라우저에서 재생까지
확인**했다. 이때부터 워치독이 매 주기 전달 여부를 직접 보고한다:

```
[whip] delivering to 1 session(s), 4321440 bytes sent
```

세션이 0이거나 바이트가 안 늘면 그것도 같은 자리에 찍힌다 — 죽은 엔드포인트와 정상 엔드포인트가
기체에서 구분되지 않던 문제가 이걸로 닫힌다.

붙기까지 세 가지가 모두 필요했다 — byte-stream 대신 AVC caps, 브리지의 타임스탬프 재작성,
그리고 **STUN 이름 해석**. 앞의 둘을 고친 뒤에도 08-13 09:44 기동은 실패했는데 DNS 가
`stun.l.google.com` 을 못 풀었기 때문이고, 네트워크 경로를 되돌린 뒤 재시작하니 다른 변경 없이
바로 흘렀다.

원격 미션 에이전트 (2026-08-12 검증):

| 항목 | 결과 |
|---|---|
| PX4 SITL (v1.15.4, SIH quadx) | 서버 봉투 그대로 `ACCEPTED` → 이륙 → 항로 → 착륙 → `COMPLETED` |
| STOP_MISSION 선점 | 수신 **11 ms** 후 `FAILED(preempted_by_stop)`, 기체는 hold 유지 |
| EMERGENCY_STOP 선점 | **56 ms** 후 선점, 즉시 착륙 → `COMPLETED(emergency_land_completed)` |
| 중복 UUID / 변조 페이로드 | 재실행 없이 마지막 상태 재발행 / `REJECTED(command_id_conflict)` |
| 구간 속도 반영 | 동일 33 m 항로에서 `cruiseSpeedMps` 1.0 → 76.5초, 4.0 → 22.5초 |
| 실기체 | 과거 버전에서 이륙·착륙 미션 1회 완주. 이번 RC epoch/지상 handback 경로는 아직 실기체 미검증 |

전원 차단 시 브로커에 쌓여 있던 QoS 1 명령 5건이 재접속과 함께 배달됐고, 전부
`REJECTED(stale_command)` 로 걸렸다 — `MQTT_COMMAND_MAX_AGE_SEC` 가 존재하는 이유다.

추론 입력 생성 비용 (카메라만 돌릴 때 4.2% 기준):

| 방식 | 전체 | 변환 순비용 |
|---|---|---|
| tee + CPU 변환 (720p NV12 → RGB 640×640) | 47.1% | **42.9%** |
| ISP 2nd 스트림 (640×360 RGB → 패딩만) | 7.2% | **3.0%** |

### ISP 스케일로 바꿔도 검출이 나빠지지 않는가 — 검증함

CPU 를 아끼려고 화질을 깎았다면 의미가 없으므로 실제로 재봤다. **한 카메라 요청에서 나온 두
스트림을 동시에 받아 같은 촬영 순간의 프레임끼리** 추론해 비교했다(장면 변화로 인한 오차 없음).
재현: `scripts/ab_detect.py`.

```
프레임 쌍 2083, 임계 0.25
  검출 총 개수     CPU 86    ISP 82
  둘 다 검출 62 | CPU 만 24 | ISP 만 20 | 둘 다 없음 1977
  평균 score       CPU 0.3293  /  ISP 0.3276   (차 -0.0030)
  최상위 박스 IoU   평균 0.959  중앙값 0.971  최소 0.834
```

박스는 IoU 중앙값 0.971 로 사실상 같은 자리를 잡고, score 차는 상대 0.9%. 개수 차는 불일치가
24 대 20 으로 거의 대칭이라 한쪽이 체계적으로 놓치는 것이 아니다.
McNemar χ² = 0.36, **p ≈ 0.55** 로 유의하지 않다.

**한계.** 2083 프레임 중 검출이 있었던 것은 4% 이고 평균 score 0.33 으로 임계값 바로 위였다.
가장 흔들리기 쉬운 약한 검출 위주로 비교된 표본이다.

---

# 6. 알아야 할 제약 (모두 실측으로 확인됨)

**libcamera 는 카메라를 한 프로세스만 연다.** 새 소비자가 필요하면 반드시 이 파이프라인의 `tee` 에
분기를 붙일 것 — 두 번째 프로세스를 띄우면 충돌한다. RTSP 퍼블리셔를 분리하지 않고 이 프로세스가
직접 publish 하는 이유이며, MediaMTX 의 `cam0` 경로가 `source: publisher` 인 이유다.

**추론 입력은 ISP 2nd 스트림으로 받는다.** PiSP 가 640×360 RGB 를 직접 내주므로 카메라 직후 `tee`
자체가 필요 없고, 두 스트림이 같은 캡처 요청에서 나와 영상과 검출의 프레임 정합도 좋다.
협상 실패 시 `tee` + CPU 변환으로 자동 폴백한다(`DUAL_STREAM=0` 으로 강제 가능).

**다중 스트림 분기에는 반드시 `queue` 를 붙인다.** `libcamerasrc` 는 하나의 스트리밍 태스크로 모든
src 패드를 밀기 때문에, 없으면 분기가 직렬화돼 2 fps 까지 떨어진다. CPU 만 보면 "효율이 좋아졌다"
로 오독하기 쉬우니 최적화를 판단할 땐 fps 를 반드시 같이 볼 것.

**`tee` 분기 안에서 framerate 를 고정하면 안 된다.** `videorate ! framerate=1/1` 같은 제약이
tee 를 거쳐 카메라까지 역으로 협상돼 파이프라인 전체가 1 fps 로 묶인다.

**`kmssink` 는 실패한다.** DSI 패널은 `card0`(rp1-dsi)인데 kmssink 가 커넥터 없는
`card2`(vc4/HDMI)를 열어 `allowed caps is empty` 로 죽는다. `fbdevsink device=/dev/fb0` 를 쓴다.

**GStreamer `hailonet` 은 쓰지 않는다.** 추론 결과를 GstMeta 로 붙여 파이프라인 밖에서 읽기 어렵고,
경유 시 1.5 fps 로 무너진다. `libhailort` 를 인프로세스로 직접 호출한다.

**MediaMTX 설정에 `yes`/`no` 를 쓰면 안 된다.** Go yaml.v3 는 YAML 1.2 라 이들을 문자열로 읽는다.

**NMS 출력 포맷** (HAILO NMS BY CLASS / float32) — 클래스마다
`[float32 count][count × {y_min, x_min, y_max, x_max, score}]`, 좌표는 640×640 레터박스 프레임
기준 0..1 정규화값. 총 4008 B = 2클래스 × (1+100×5) × 4.

---

# 7. 구조

| 경로 | 내용 |
|---|---|
| `src/main.cpp` | 기동, Hailo 판정, GMainLoop, 통계 리포트 |
| `src/pipeline.cpp` | GStreamer 파이프라인 + cairo 오버레이 + 추론 워커 |
| `src/hailo_inference.cpp` | libhailort VStream 래퍼 (HailoRT 있을 때만 컴파일) |
| `src/hailo_inference_stub.cpp` | 없을 때 대체 구현 |
| `src/hailo_probe.cpp` | PCIe 카드 존재 확인 (항상 컴파일) |
| `src/detection.cpp` | NMS 파싱, 레터박스 역변환 |
| `src/config.cpp` | 환경변수 → `Config` |
| `include/riposte/` | 비행 로직 (헤더 온리, **MAVSDK 무의존**) — 궤적·기하·안전정책·시험카탈로그·판정 |
| `src/riposte/test_main.cpp` | 시험비행 앱. Arm→이륙→기동→호버→착륙, PASS/FAIL, CSV |
| `src/riposte/gcs_main.cpp` | 지상국 앱 |
| `tests/riposte/` | 기하·판정 단위시험 (ctest) |
| `config/`, `systemd/` | Pi 에 설치되는 파일의 원본. Pi 에서 직접 고치지 말고 여기서 고쳐 배포 |

**비행 앱이 아직 별도 실행 파일인 이유.** `test_main.cpp` 는 1,949 줄에 `main()`·인자 파싱·시험
러너가 얽혀 있고, 그 안에 예외→비상착륙, setpoint watchdog, 소프트웨어 반경 제한, 배터리 게이트
같은 **검증된 안전 로직**이 들어 있다. 통합 편의를 위해 안전 코드를 리팩터링하는 것은 미뤘다.
우선 같은 저장소·같은 빌드·같은 설치로 묶고, 프로세스 통합은 안전 로직을 서브시스템으로 분리할
준비가 됐을 때 별도로 한다.

**영상과 비행을 합칠 때 지킬 것.** 카메라 소유권은 영상 파이프라인에 있다. 비행 쪽에서 검출 결과가
필요하면 `Pipeline` 에서 스냅샷을 받아 가는 형태가 맞다. 원본 Riposte 가 EO/IR 녹화를 일부러 뺀
것도 같은 이유이며(SITL 과 실기체에서 같은 바이너리 사용), 그 성질을 깨지 않도록 비행 앱에
GStreamer 의존을 들이지 말 것.

---

# 8. 문제 해결

| 증상 | 확인 |
|---|---|
| `cam -l` 이 비어 있음 | §1.4 — PiSP IPA 부재. `ls /usr/lib/*/libcamera/ipa_rpi_pisp.so` |
| 센서는 잡히는데 프레임 0 | I²C 는 되고 CSI 만 죽은 것. [docs/setup_notes.md](docs/setup_notes.md) 의 제어선/데이터선 분리 진단 |
| `Camera frontend has timed out` | 위와 동일 — 케이블·커넥터 |
| fps 가 한 자리로 떨어짐 | caps 협상. §6 의 `tee`/`queue` 항목 |
| 검출이 안 됨 | `journalctl -u drone-companion | grep hailo]` 에 사유가 남는다 |
| RTSP 404 | 경로명이 `cam0` 인지 확인 (앱 기본값) |
| 비행 에이전트가 재시작 반복 | `/dev/ttyAMA0` 존재 여부(§1.8), `dialout` 그룹(§1.6), FC 전원 |
| `component 191 is not visible` | PX4 포워딩 미설정 — §3.3 의 `MAV_0/1_FORWARD` |
| `Arming denied` | 지상국(태블릿/QGC) 미연결. PX4 기본 arming 체크 요구사항 — §3.3 |
| 태블릿에서 영상 안 나옴 | 태블릿마다 설정이 따로다. 설정 > 네트워크에서 해당 유닛의 영상 IP 확인 |

진단 스크립트:

```bash
sudo systemctl stop drone-companion     # 카메라는 한 프로세스만 열 수 있다
./scripts/camera_setup.sh --snapshot
sudo systemctl start drone-companion
```

---

# 9. 남은 과제

- **혼잡 제어가 없다는 점이 WHIP 단독 운용의 전제 조건이다.** 아래 항목 참조. RTSP 를 걷어내면
  링크가 나빠질 때 물러설 곳이 없다
- **WHIP 은 `stun.l.google.com` 이름 해석에 묶여 있다.** 못 풀면 ICE 가 host 후보만 모으고
  미디어가 붙지 않는데, 로컬 로그에는 `Can't resolve STUN server` 한 줄만 남고 서비스는 정상으로
  보인다. 2026-08-13 실패의 최종 원인이 이것이었다. DNS 가 죽으면 WHIP 도 죽는다는 뜻이므로
  고정 IP STUN 또는 서버 제공 STUN 으로 바꾸는 편이 낫다
- **혼잡 제어 없이 송출한다.** `rtpgccbwe` 가 설치돼 있지 않아 `webrtcsink` 가
  `not doing any congestion control` 을 남긴다. 링크가 나빠질 때 동작을 확인하지 않았다
- 원격 미션 에이전트의 실기체 검증은 이륙·착륙 1회뿐이다. 지속 운용, 항로 비행, 배터리 게이트,
  선점의 실기체 동작은 SITL 에서만 확인했다
- 미션 실행을 영상 프로세스와 통합 (현재는 별도 바이너리)
- 검출 결과를 영상 burn-in 외에 별도 채널(JSON/MAVLink)로도 내보내기 — 태블릿에서 오버레이 on/off 가 가능해진다
- `mediamtx.service` 가 root 로 뜬다. 카메라를 열지 않으므로 비특권 사용자로 낮출 수 있다
- RTSP 인증 없음 (현재 익명 접근 허용)
- Riposte 문서(`docs/riposte/RIPOSTE_README.md`)의 시험 고도 설명이 코드와 다르다 —
  문서는 50cm/100cm/300cm, 코드는 5m/6m/7m(23종). 저고도 이륙 경고는 현재 카탈로그에 해당하지 않는다.
