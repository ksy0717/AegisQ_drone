#!/bin/bash
# Pi 5 + Ubuntu 24.04 부트스트랩. sudo 로 실행.
#   이 제품 프로필은 HailoRT 컴파일을 끄고 검출 없는 통과 전용 바이너리를 빌드한다.
#   SKIP_LIBCAMERA=1 로 libcamera 소스 빌드를 건너뛸 수 있다(이미 되어 있을 때).
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_USER="${TARGET_USER:-sky}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
JOBS="${JOBS:-3}"

# Keep install builds isolated by branch and project profile. Clearing cached
# DC_* selections makes config/project.cmake authoritative even after a prior
# diagnostic build used command-line overrides.
BUILD_REF="$(git -C "$REPO" symbolic-ref --quiet --short HEAD 2>/dev/null || true)"
if [ -z "$BUILD_REF" ]; then
  BUILD_REF="detached-$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
fi
BUILD_KEY="$(printf '%s' "$BUILD_REF" | LC_ALL=C tr -c 'A-Za-z0-9._-' '_')"
PROFILE_HASH="$(sha256sum "$REPO/config/project.cmake" | cut -c1-12)"
BUILD_DIR="$REPO/build/install-${BUILD_KEY}-${PROFILE_HASH}"

log() { echo; echo "== $*"; }
[ "$(id -u)" -eq 0 ] || { echo "sudo 로 실행하세요"; exit 1; }

log "[1/7] apt 소스 확인 (noble-updates 없으면 의존성이 통째로 깨진다)"
if ! grep -q 'noble-updates' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null; then
  cp -a /etc/apt/sources.list.d/ubuntu.sources "/etc/apt/sources.list.d/ubuntu.sources.bak.$(date +%F)"
  sed -i 's/^Suites: noble$/Suites: noble noble-updates/' /etc/apt/sources.list.d/ubuntu.sources
  echo "   noble-updates 추가함"
fi
apt-get update -qq

log "[2/7] 런타임 + 빌드 패키지"
apt-get install -y --no-install-recommends \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
  gstreamer1.0-rtsp gstreamer1.0-x \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libcairo2-dev libglib2.0-dev \
  build-essential cmake pkg-config v4l-utils curl bc

log "[3/7] Hailo 드라이버 상태"
if dkms status 2>/dev/null | grep -q hailo_pci; then
  dkms status | grep hailo_pci | sed 's/^/   /'
else
  echo "   Hailo 없음 또는 드라이버 미설치 — 검출 없이 영상만 송출하는 구성으로 진행합니다."
  echo "   나중에 장착하면: apt-get install -y build-essential dkms linux-headers-\$(uname -r)"
  echo "                   dpkg -i hailort_*.deb hailort-pcie-driver_*.deb  후 재빌드"
fi

log "[4/7] libcamera (Pi 5 PiSP) — Ubuntu 기본 패키지에는 없다"
if [ "${SKIP_LIBCAMERA:-0}" = "1" ]; then
  echo "   SKIP_LIBCAMERA=1 — 건너뜀"
elif ls /usr/lib/*/libcamera/ipa_rpi_pisp.so >/dev/null 2>&1; then
  echo "   ipa_rpi_pisp.so 이미 존재 — 건너뜀"
else
  JOBS="$JOBS" bash "$REPO/scripts/build_libcamera.sh"
fi
sudo -u "$TARGET_USER" cam -l 2>/dev/null | grep -E "^[0-9]+:" | sed 's/^/   /' || true

log "[5/7] MediaMTX"
if [ ! -x /usr/local/bin/mediamtx ]; then
  tmp=$(mktemp -d); pushd "$tmp" >/dev/null
  url=$(curl -fsSL https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
        | grep -oE 'https://[^"]*mediamtx_[^"]*_linux_arm64\.tar\.gz' | head -1)
  curl -fsSL -o m.tar.gz "$url" && tar -xzf m.tar.gz
  install -m755 mediamtx /usr/local/bin/mediamtx
  popd >/dev/null; rm -rf "$tmp"
fi
mkdir -p /usr/local/etc
install -m644 "$REPO/config/mediamtx.yml"    /usr/local/etc/mediamtx.yml
install -m755 "$REPO/config/publish-test.sh" /usr/local/bin/publish-test.sh
/usr/local/bin/mediamtx --version | sed 's/^/   mediamtx /'

log "[6/7] MAVSDK (비행 서브시스템) + 빌드"
profile_report="$(cmake -DDC_PRINT_PROFILE=ON -P "$REPO/config/project.cmake" 2>&1)"
if grep -q 'DC_PROFILE_NEEDS_GST_WHIP=ON' <<<"$profile_report"; then
  JOBS="$JOBS" bash "$REPO/scripts/install_gst_whip.sh"
else
  echo "   project.cmake: WHIP runtime plugin not required"
fi
if grep -q 'DC_PROFILE_NEEDS_MQTT=ON' <<<"$profile_report"; then
  apt-get install -y --no-install-recommends \
    libmosquitto-dev nlohmann-json3-dev libsqlite3-dev
else
  echo "   project.cmake: MQTT connector dependencies not required"
fi
if ! grep -q 'DC_PROFILE_NEEDS_MAVSDK=ON' <<<"$profile_report"; then
  echo "   project.cmake: MAVSDK 대상 없음 — 설치 건너뜀"
elif [ "${SKIP_MAVSDK:-0}" = "1" ]; then
  echo "   SKIP_MAVSDK=1 — 건너뜀 (활성화된 비행 앱은 빌드되지 않는다)"
else
  bash "$REPO/scripts/install_mavsdk.sh"
fi

cmake -U 'DC_*' -S "$REPO" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j"$JOBS"
cmake --install "$BUILD_DIR" --prefix /usr/local
# 기하·판정 단위시험은 하드웨어 없이 회귀를 잡는 유일한 수단이므로 설치 시 한 번 돌린다.
ctest --test-dir "$BUILD_DIR" --output-on-failure 2>&1 | tail -4 | sed 's/^/   /'
install -m755 "$REPO/scripts/camera_setup.sh" "$TARGET_HOME/camera_setup.sh"
install -m755 "$REPO/scripts/grab_fb.sh"      "$TARGET_HOME/grab_fb.sh"
chown "$TARGET_USER:$TARGET_USER" "$TARGET_HOME/camera_setup.sh" "$TARGET_HOME/grab_fb.sh"
mkdir -p "$TARGET_HOME/models"; chown "$TARGET_USER:$TARGET_USER" "$TARGET_HOME/models"
usermod -aG video,render "$TARGET_USER"

log "[7/7] systemd"
install -m644 "$REPO/systemd/mediamtx.service"         /etc/systemd/system/mediamtx.service
install -m644 "$REPO/systemd/drone-companion.service"  /etc/systemd/system/drone-companion.service
if [ ! -e /etc/default/drone-companion ]; then
  install -m600 "$REPO/config/drone-companion.env.example" /etc/default/drone-companion
else
  echo "   /etc/default/drone-companion preserved (WHIP credentials were not overwritten)"
fi

# --- 비행 에이전트 (MAVSDK 가 있을 때만) ---
# A stale executable is not proof that the current profile selected the target.
RIPOSTE_AGENT_ENABLED=0
if grep -q '^DC_BUILD_RIPOSTE_AGENT:BOOL=ON$' "$BUILD_DIR/CMakeCache.txt"; then
  [ -x "$BUILD_DIR/riposte_test" ] || {
    echo "riposte_test is enabled but the current build did not produce it" >&2
    exit 1
  }
  RIPOSTE_AGENT_ENABLED=1
fi

if [ "$RIPOSTE_AGENT_ENABLED" = "1" ]; then
  install -m755 "$REPO/scripts/riposte-agent.sh" /usr/local/bin/riposte-agent.sh
  install -m644 "$REPO/systemd/riposte-agent.service" /etc/systemd/system/riposte-agent.service
  # 시리얼(Pixhawk UART) 접근 권한
  usermod -aG dialout "$TARGET_USER"
  # Pi 5 는 dtoverlay=uart0-pi5 가 있어야 /dev/ttyAMA0 가 노출된다.
  if ! grep -q '^dtoverlay=uart0-pi5' /boot/firmware/config.txt 2>/dev/null; then
    cp -a /boot/firmware/config.txt "/boot/firmware/config.txt.bak.$(date +%F-%H%M)"
    echo 'dtoverlay=uart0-pi5' >> /boot/firmware/config.txt
    echo "   config.txt 에 dtoverlay=uart0-pi5 추가 — /dev/ttyAMA0 사용에 재부팅 필요"
  fi
  echo "   비행 에이전트 등록됨. 기체가 움직이면 안 되는 상황이면 먼저 확인할 것:"
  echo "     systemctl show riposte-agent -p Environment | grep RIPOSTE_EXECUTE"
fi
# 디스플레이가 없는 기체에서는 패널 출력을 꺼야 파이프라인이 뜬다.
if [ ! -e /dev/fb0 ]; then
  echo "   /dev/fb0 없음 → ENABLE_DISPLAY=0 으로 설정"
  sed -i 's/^Environment=ENABLE_DISPLAY=1/Environment=ENABLE_DISPLAY=0/' \
      /etc/systemd/system/drone-companion.service
fi
systemctl daemon-reload
systemctl enable --now mediamtx
systemctl enable drone-companion
echo "   시작: sudo systemctl start drone-companion"
if [ "$RIPOSTE_AGENT_ENABLED" = "1" ]; then
  systemctl enable riposte-agent
  echo "   시작: sudo systemctl start riposte-agent   (부팅 자동시작 등록됨)"
elif systemctl is-enabled --quiet riposte-agent 2>/dev/null; then
  echo "   경고: 기존 riposte-agent가 활성화되어 있음 — 운영자가 충돌 여부를 확인할 것"
fi

log "완료"
