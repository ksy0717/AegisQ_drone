#!/bin/bash
# 카메라 점검 · 설정 확인 스크립트 (Pi 5 / Ubuntu 24.04)
#
#   ./camera_setup.sh              전체 점검
#   ./camera_setup.sh --snapshot   점검 + /tmp/camera_snapshot.jpg 저장
#   ./camera_setup.sh --fix        고칠 수 있는 것(그룹·오버레이)을 실제로 반영
#
# 이 파이프라인은 libcamera 의 2nd 스트림(ISP 하드웨어 스케일)에 의존한다.
# 그게 되는지까지 확인하는 것이 이 스크립트의 핵심이다.
set -u

SNAPSHOT=0; FIX=0
for a in "$@"; do
  case "$a" in
    --snapshot) SNAPSHOT=1 ;;
    --fix) FIX=1 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
  esac
done

SRC_W=${SRC_W:-1280}; SRC_H=${SRC_H:-720}
INFER_W=${INFER_W:-640}; INFER_H=${INFER_H:-360}
CONFIG_TXT=/boot/firmware/config.txt
FAIL=0; WARN=0

ok()   { printf "  \033[32m[ OK ]\033[0m %s\n" "$*"; }
bad()  { printf "  \033[31m[FAIL]\033[0m %s\n" "$*"; FAIL=$((FAIL+1)); }
warn() { printf "  \033[33m[WARN]\033[0m %s\n" "$*"; WARN=$((WARN+1)); }
hint() { printf "         → %s\n" "$*"; }

echo "==================== 1. 커널이 센서를 잡았는가 ===================="
SENSOR=$(for d in /sys/bus/i2c/devices/*/; do
           n=$(cat "$d/name" 2>/dev/null)
           case "$n" in imx*|ov*) echo "$n"; esac
         done | head -1)
if [ -n "$SENSOR" ]; then
  ok "센서 인식: $SENSOR"
else
  bad "i2c 버스에서 카메라 센서를 찾지 못함"
  hint "CSI 케이블 방향과 체결을 확인하고 '전원을 끈 뒤' 다시 연결할 것"
  hint "camera_auto_detect 는 부팅 시점에만 동작하므로 연결 후 재부팅 필요"
fi

if [ -e /sys/class/video4linux ]; then
  CFE=$(for v in /sys/class/video4linux/video*; do
          cat "$v/name" 2>/dev/null | grep -q '^rp1-cfe' && basename "$v"
        done | head -1)
  [ -n "$CFE" ] && ok "CSI 프론트엔드 등록됨 (rp1-cfe → /dev/$CFE)" \
                || bad "rp1-cfe 노드 없음 (Pi 5 CSI 수신기 미동작)"
fi

echo
echo "==================== 2. libcamera 스택 ===================="
if ! command -v cam >/dev/null 2>&1; then
  bad "libcamera 도구(cam)가 없음"
  hint "scripts/build_libcamera.sh 로 소스 빌드 필요"
else
  IPA_DIR=$(ls -d /usr/lib/*/libcamera 2>/dev/null | head -1)
  if [ -n "$IPA_DIR" ] && [ -f "$IPA_DIR/ipa_rpi_pisp.so" ]; then
    ok "Pi 5 PiSP IPA 존재 ($IPA_DIR/ipa_rpi_pisp.so)"
  else
    bad "ipa_rpi_pisp.so 없음 — Ubuntu 기본 libcamera 는 Pi 5 를 지원하지 않는다"
    hint "scripts/build_libcamera.sh 실행 (libpisp + rpi 포크 libcamera 소스 빌드)"
  fi
  if [ -n "$SENSOR" ] && [ -f "/usr/share/libcamera/ipa/rpi/pisp/${SENSOR}.json" ]; then
    ok "센서 튜닝 파일 존재 (${SENSOR}.json)"
  elif [ -n "$SENSOR" ]; then
    warn "pisp 튜닝 파일 없음 (${SENSOR}.json) — 화질/AWB 가 기본값으로 동작"
  fi

  CAMLINE=$(cam -l 2>/dev/null | grep -E "^[0-9]+:" | head -1)
  if [ -n "$CAMLINE" ]; then
    ok "libcamera 열거: $CAMLINE"
  else
    bad "libcamera 가 카메라를 열거하지 못함"
    hint "cam -l 을 직접 실행해 오류를 확인할 것"
  fi
fi

echo
echo "==================== 3. 접근 권한 ===================="
U=${SUDO_USER:-$(id -un)}
if id -nG "$U" | tr ' ' '\n' | grep -qx video; then
  ok "$U 가 video 그룹에 속함"
else
  bad "$U 가 video 그룹에 없음 — /dev/media* 접근 불가"
  if [ "$FIX" = 1 ]; then
    sudo usermod -aG video,render "$U" && hint "추가함. 재로그인 또는 재부팅 후 적용된다"
  else
    hint "sudo usermod -aG video,render $U  (재로그인 필요)"
  fi
fi
for n in /dev/media0 /dev/fb0; do
  [ -e "$n" ] || continue
  [ -r "$n" ] && ok "$n 읽기 가능" || warn "$n 접근 불가 (권한 확인)"
done

echo
echo "==================== 4. 부팅 설정 ===================="
if [ -f "$CONFIG_TXT" ]; then
  if grep -qE '^\s*camera_auto_detect=1' "$CONFIG_TXT"; then
    ok "camera_auto_detect=1"
  else
    warn "camera_auto_detect 가 1 이 아님"
    hint "공식 카메라 모듈은 이 값으로 자동 인식된다. 서드파티는 dtoverlay 를 직접 지정"
  fi
  OVL=$(grep -E '^\s*dtoverlay=(imx|ov)[0-9a-z]+' "$CONFIG_TXT" | head -1)
  [ -n "$OVL" ] && ok "명시적 오버레이: $OVL"
else
  warn "$CONFIG_TXT 없음"
fi

echo
echo "==================== 5. 캡처 시험 (${SRC_W}x${SRC_H} NV12) ===================="
if command -v gst-launch-1.0 >/dev/null 2>&1; then
  if timeout 30 gst-launch-1.0 -q libcamerasrc \
       ! video/x-raw,width=$SRC_W,height=$SRC_H,format=NV12 \
       ! identity eos-after=30 ! fakesink sync=false >/dev/null 2>&1; then
    ok "메인 스트림 캡처 성공"
  else
    bad "메인 스트림 캡처 실패"
    hint "다른 프로세스가 카메라를 점유했을 수 있다: sudo systemctl stop hailo-detect"
  fi

  echo
  echo "==================== 6. ISP 2nd 스트림 (CPU 절감의 핵심) ===================="
  # 프로세스가 살아 있는지만 보면 안 된다 — 카메라가 프레임을 한 장도 안 줘도 협상은
  # 성공하고 프로세스는 계속 돌기 때문에 멀쩡해 보인다(실제로 그렇게 오판했다).
  # 반드시 양쪽 패드의 프레임을 세서 판정한다.
  DUAL_OUT=$(python3 - "$SRC_W" "$SRC_H" "$INFER_W" "$INFER_H" 2>/dev/null <<'PY'
import sys, time, gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
Gst.init(None)
W, H, IW, IH = (int(a) for a in sys.argv[1:5])
c = {"a": 0, "b": 0}
p = Gst.parse_launch(
    f"libcamerasrc name=cam "
    f"cam.src ! video/x-raw,width={W},height={H},format=NV12 "
    "! queue leaky=downstream max-size-buffers=3 "
    "! appsink name=a emit-signals=true sync=false max-buffers=1 drop=true "
    f"cam.src_1 ! video/x-raw,width={IW},height={IH},format=RGB "
    "! queue leaky=downstream max-size-buffers=3 "
    "! appsink name=b emit-signals=true sync=false max-buffers=1 drop=true")
def mk(k):
    def cb(s):
        if s.emit("pull-sample") is not None:
            c[k] += 1
        return Gst.FlowReturn.OK
    return cb
p.get_by_name("a").connect("new-sample", mk("a"))
p.get_by_name("b").connect("new-sample", mk("b"))
loop = GLib.MainLoop()
bus = p.get_bus(); bus.add_signal_watch()
bus.connect("message", lambda _b, m: loop.quit()
            if m.type in (Gst.MessageType.ERROR, Gst.MessageType.EOS) else None)
t0 = time.time(); p.set_state(Gst.State.PLAYING)
GLib.timeout_add_seconds(8, lambda: (loop.quit(), False)[1])
loop.run(); p.set_state(Gst.State.NULL)
print(f"{c['a']} {c['b']} {max(time.time()-t0, 1e-6):.1f}")
PY
)
  set -- $DUAL_OUT
  DA=${1:-0}; DB=${2:-0}; DEL=${3:-0}
  if [ "$DA" -gt 50 ] && [ "$DB" -gt 50 ]; then
    ok "듀얼 스트림 동작 — main ${DA}프레임 / infer ${DB}프레임 (${DEL}s)"
    hint "추론 입력 변환을 ISP 가 처리한다 (CPU 42.9% → 3.0% 코어)"
  elif [ "$DA" = "0" ] && [ "$DB" = "0" ]; then
    bad "듀얼 스트림에서 프레임이 한 장도 오지 않음"
    hint "5번도 실패했다면 카메라 연결 문제다 (CSI 케이블/커넥터)"
  else
    warn "듀얼 스트림 프레임 부족 — main ${DA} / infer ${DB} (${DEL}s)"
    hint "분기마다 queue 가 있어야 한다. 없으면 스트리밍 스레드가 직렬화돼 수 fps 로 떨어진다"
  fi
else
  warn "gst-launch-1.0 없음 — 캡처 시험 건너뜀"
fi

if [ "$SNAPSHOT" = 1 ] && command -v gst-launch-1.0 >/dev/null 2>&1; then
  echo
  echo "==================== 7. 스냅샷 ===================="
  rm -f /tmp/camera_snapshot.jpg
  # 앞부분은 AWB 수렴 전이라 색이 튄다. 30 프레임 버린 뒤 저장한다.
  timeout 30 gst-launch-1.0 -q libcamerasrc \
    ! video/x-raw,width=$SRC_W,height=$SRC_H,format=NV12 \
    ! identity eos-after=30 ! videoconvert ! jpegenc quality=88 \
    ! multifilesink location=/tmp/camsnap_%02d.jpg >/dev/null 2>&1
  LAST=$(ls -t /tmp/camsnap_*.jpg 2>/dev/null | head -1)
  if [ -n "$LAST" ]; then
    mv -f "$LAST" /tmp/camera_snapshot.jpg; rm -f /tmp/camsnap_*.jpg
    ok "저장: /tmp/camera_snapshot.jpg ($(stat -c%s /tmp/camera_snapshot.jpg) bytes)"
  else
    bad "스냅샷 저장 실패"
  fi
fi

echo
echo "==================== 요약 ===================="
printf "  실패 %d / 경고 %d\n" "$FAIL" "$WARN"
if [ "$FAIL" -eq 0 ]; then
  echo "  카메라 사용 가능."
  exit 0
fi
echo "  위 [FAIL] 항목을 해결한 뒤 다시 실행할 것."
exit 1
