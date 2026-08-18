#!/bin/bash
# MAVSDK 설치 (비행 서브시스템 의존성).
#
# 3.17.1 은 고정 버전이 아니라 빌드가 요구하는 최소 버전이다. 여기서 3.17.1 을 받는 것은
# 검증된 조합이기 때문이지 다른 버전이 안 되기 때문이 아니다 — 더 높은 3.x 도 그대로 쓸 수 있고
# MAVSDK_VERSION 으로 바꿀 수 있다. 이미 설치돼 있으면 버전과 무관하게 손대지 않는다
# (배포판 패키지나 소스 빌드로 이미 갖춰둔 환경을 덮어쓰지 않기 위해서다).
#
# 3.17.1 이 바닥인 이유는 코드가 v3 에서 바뀐 의미에 의존하기 때문이다 — remaining_percent 가
# 0~100 퍼센트, CompanionComputer 기본 컴포넌트 ID 195 등. 그 아래로는 조용히 오동작한다.
#
# Ubuntu 24.04 arm64 용 공식 deb 가 없어 debian12 arm64 빌드를 쓴다.
# 더 오래된 glibc 로 빌드된 바이너리는 최신 시스템에서 동작한다 — Pi 5 / Ubuntu 24.04
# 에서 riposte_test·riposte_gcs 빌드와 단위시험 통과를 확인했다.
# Raspberry Pi OS Bookworm 은 Debian 12 자체이므로 이 자산이 네이티브 타깃이다.
set -euo pipefail

VERSION="${MAVSDK_VERSION:-3.17.1}"
[ "$(id -u)" -eq 0 ] || { echo "sudo 로 실행하세요"; exit 1; }

if [ -f /usr/lib/cmake/MAVSDK/MAVSDKConfig.cmake ] || \
   [ -f /usr/local/lib/cmake/MAVSDK/MAVSDKConfig.cmake ]; then
  echo "== MAVSDK 이미 설치됨"
  grep -m1 -oE '[0-9]+\.[0-9]+\.[0-9]+' /usr/lib/cmake/MAVSDK/MAVSDKConfigVersion.cmake 2>/dev/null \
    | sed 's/^/   버전: /' || true
  exit 0
fi

ARCH="$(dpkg --print-architecture)"
case "$ARCH" in
  arm64|amd64) ;;
  *) echo "지원하지 않는 아키텍처: $ARCH"; exit 1 ;;
esac

. /etc/os-release
# 배포판별 자산 이름. arm64 는 ubuntu 빌드가 없어 debian12 를 쓴다.
if [ "$ARCH" = "arm64" ]; then
  ASSET="libmavsdk-dev_${VERSION}_debian12_arm64.deb"
else
  ASSET="libmavsdk-dev_${VERSION}_ubuntu${VERSION_ID}_amd64.deb"
fi
URL="https://github.com/mavlink/MAVSDK/releases/download/v${VERSION}/${ASSET}"

echo "== 다운로드: $ASSET"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
curl -fsSL -o "$TMP/mavsdk.deb" "$URL"

echo "== 설치"
DEBIAN_FRONTEND=noninteractive apt-get install -y "$TMP/mavsdk.deb"

echo "== 확인"
dpkg -l | grep -i mavsdk | sed 's/^/   /'
find /usr -name 'MAVSDKConfig.cmake' 2>/dev/null | head -1 | sed 's/^/   cmake: /'
