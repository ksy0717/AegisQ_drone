# WHIP 송출 운영 가이드

이 제품 브랜치의 영상 프로세스는 카메라를 한 번만 열고 H.264를 한 번만
인코딩한다. 카메라/RTSP GstPipeline과 WHIP GstPipeline은 bounded
`appsink`/`appsrc` bridge로 분리한다. queue만 붙인 한 파이프라인에서는
WHIP signalling의 비동기 상태 전이가 RTSP preroll까지 멈출 수 있기 때문이다.
WHIP 파이프라인이 연결 대기나 재시작 중이어도 카메라와 RTSP는 계속 흐른다.

~~~text
libcamerasrc -> overlay/pass-through -> x264enc -> h264parse -> encoded tee
                                                           +-> RTSP
                                                           +-> bounded appsink
                                                               |
                                       isolated appsrc -> WHIP/WebRTC
~~~

WHIP은 기본적으로 꺼져 있다. `WHIP_URL`, stream key 또는 token을 저장소나
systemd unit에 기록하지 않는다. 실제 값은 Pi의 root 전용
`/etc/default/drone-companion`에만 둔다.

## GStreamer 버전 정책

Raspberry Pi 5 / Ubuntu 24.04의 배포 기준은 배포판과 libcamera가 함께 제공하는
GStreamer 1.24 계열이다. WHIP 때문에 코어와 모든 플러그인을 교체하지 않고,
GStreamer 1.24와 호환되는 공식 `gst-plugins-rs` 0.12.11의
`gst-plugin-webrtc`만 설치한다.

Android GCS가 사용하는 GStreamer 1.29.1과 Pi 버전이 같을 필요는 없다. 두
프로세스는 GStreamer ABI를 공유하지 않고 WHIP/WebRTC, SDP, RTP와 H.264
bitstream으로 상호운용한다. 1.29.x는 1.30으로 가는 API/ABI 불안정 개발
시리즈이므로 비행용 Pi 런타임에는 설치하지 않는다. Pi 업그레이드가 필요하면
현재 안정 시리즈인 1.28.x를 별도 SD 카드 이미지에서 카메라, RTSP, WHIP,
Hailo 유무 조합까지 재검증한 뒤 채택한다.

- [GStreamer 1.29 개발 시리즈 안내](https://gstreamer.freedesktop.org/releases/1.29/)
- [GStreamer 현재 안정 릴리스](https://gstreamer.freedesktop.org/releases/)

## 런타임 설치

~~~bash
sudo ./scripts/install_gst_whip.sh
GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0 \
  gst-inspect-1.0 whipclientsink
~~~

설치 스크립트는 GStreamer 코어를 올리지 않는다. 지원하는 안정 계열
1.24/1.26/1.28인지 확인하고, 공식 저장소의 고정 태그를 빌드해
`/usr/local/lib/gstreamer-1.0/libgstrswebrtc.so`만 설치한다.

## AegisQ endpoint 설정

현재 내부 경로는 다음처럼 역할이 다르다.

- `10.100.0.1:7880`: LiveKit 서비스 주소. WHIP ingest URL이 아니다.
- `10.100.0.1:8085/w/<stream_key>`: WHIP ingest endpoint 형식.

`<stream_key>`에는 서버가 발급한 실제 key를 넣는다. 웹 재생 room 이름인
`stream-DRONE-01`이나 임의로 만든 카메라 ID를 key로 사용하지 않는다.

~~~bash
sudoedit /etc/default/drone-companion
~~~

~~~ini
ENABLE_WHIP=1
WHIP_URL=http://10.100.0.1:8085/w/REPLACE_WITH_SERVER_ISSUED_KEY
WHIP_TOKEN=
WHIP_RETRY_SECS=15
~~~

token이 서버에서 발급된 경우에만 `WHIP_TOKEN`을 채운다. 이 파일은 mode 0600을
유지한다. URL은 `http://` 또는 `https://`와 정확한 `/w/<stream_key>` 한
segment만 허용한다. query, fragment, userinfo, room 이름, 잘못된 bearer token은
카메라를 열기 전에 거부된다. `WHIP_RETRY_SECS`는 1~300초만 허용하며,
publisher가 PLAYING에 도달하지 못하거나 오류를 내면 RTSP를 건드리지 않고
해당 주기로 WHIP 보조 파이프라인만 재시작한다.

WireGuard 경로도 활성화 전에 확인한다.

~~~bash
ip route get 10.100.0.1
curl -fsS -o /dev/null -w '%{http_code}\n' http://10.100.0.1:8085/
~~~

첫 명령은 `dev wg0 src 10.100.0.x`를 보여야 한다. 두 번째 명령의 root 경로는
404여도 서버 도달 확인으로는 정상이다. 이 route 확인은 kill switch가 아니므로,
운영 환경에서는 방화벽/policy routing으로 해당 목적지 트래픽을 wg0에 제한한다.

## 서비스 등록과 검증

~~~bash
sudo install -m600 config/drone-companion.env.example \
  /etc/default/drone-companion
sudo install -m644 systemd/drone-companion.service \
  /etc/systemd/system/drone-companion.service
sudo systemctl daemon-reload
sudo systemctl enable drone-companion
~~~

실제 stream key와 카메라가 준비된 뒤 시작한다.

~~~bash
sudo systemctl start drone-companion
journalctl -u drone-companion -f
~~~

WHIP 활성 상태의 로그에는 endpoint 대신 `(redacted)`만 보여야 한다. 파이프라인
문자열에도 stream key와 token이 없어야 한다. 처리량은 WHIP과 RTSP를 동시에 켠
상태에서 캡처 29~30fps, 드롭 0을 기준으로 확인하고 CPU/RSS도 함께 기록한다.
RTSP 회귀는 저장소 루트 `AGENTS.md`의 되받아보기 명령으로 확인한다.

카메라가 `cam -l`에 나타나지 않으면 서비스의 실제 송출·fps 검증을 진행할 수
없다. 이 경우 서비스 등록까지만 하고 반복 재시작시키지 않는다.

## 2026-08-11 Pi 검증 기록

Raspberry Pi 5 / Ubuntu 24.04 / GStreamer 1.24.2에서 다음을 확인했다.

- `gst-plugins-rs` 0.12.11(`177dd1a`)의 `whipclientsink` 등록
- MQTT 제품 기본 프로필 빌드와 `ctest` 10/10 통과
- WHIP OFF, RTSP-only: 40.3초 시점 29.1fps, CPU 64.1%(0.64코어),
  RSS 67,744KiB
- 폐쇄된 WHIP endpoint를 15초마다 재시작하면서 RTSP 동시 송출: 40.8초 시점
  29.1fps, CPU 64.1%, RSS 74,936KiB
- 위 장애 조건의 RTSP를 H.264 디코딩해 1280x960 JPEG 프레임 생성
- `drone-companion.service`는 `enabled`/`active`, WHIP은 기본 OFF

실제 AegisQ WHIP publish와 웹/Android 재생 E2E는 서버 발급 stream key가 없어
수행하지 않았다. 해당 key와 필요한 token을 받은 뒤에만 `ENABLE_WHIP=1`로
바꾸고 동시 송출 fps와 재생을 다시 검증한다.
