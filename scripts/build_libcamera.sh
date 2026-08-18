#!/bin/bash
# libpisp + libcamera(rpi fork, pisp ?뚯씠?꾨씪???ы븿) ?뚯뒪 鍮뚮뱶 - Ubuntu 24.04 / Pi 5
LOG=/tmp/libcam_build.log
MARK=/tmp/libcam_build.DONE
rm -f "$MARK"
exec > >(tee -a "$LOG") 2>&1

LIBPISP_TAG=v1.0.7
LIBCAM_TAG=v0.3.2+rpt20241119
SRC=/home/sky/src/raspberrypi
JOBS=${JOBS:-3}

fail() { echo "### FAILED: $1"; echo "FAILED:$1" > "$MARK"; exit 1; }

echo "######## ?쒖옉 $(date) ########"
echo "libpisp   : $LIBPISP_TAG"
echo "libcamera : $LIBCAM_TAG"
echo "jobs      : $JOBS (?ㅼ솑 ?놁쓬 ??OOM 諛⑹?)"

# ---------- 0) 湲곗〈 Ubuntu libcamera ?쒓굅 ----------
echo; echo "######## [0/4] 湲곗〈 libcamera 0.2.0 ?⑦궎吏 ?쒓굅 ########"
DEBIAN_FRONTEND=noninteractive apt-get remove -y \
  libcamera0.2 libcamera-ipa libcamera-tools gstreamer1.0-libcamera 2>&1 | tail -5
# ?붿〈 ?뚮윭洹몄씤 ?뺤씤
find /usr/lib -name 'libgstlibcamera.so' 2>/dev/null | sed 's/^/  ?붿〈: /'

# ---------- 1) libpisp ----------
echo; echo "######## [1/4] libpisp $LIBPISP_TAG 鍮뚮뱶 ########"
mkdir -p "$SRC" && cd "$SRC"
rm -rf libpisp
git clone --depth 1 -b "$LIBPISP_TAG" https://github.com/raspberrypi/libpisp.git 2>&1 | tail -3 || fail "libpisp clone"
cd libpisp
meson setup build --buildtype=release --prefix=/usr --libdir=lib/aarch64-linux-gnu 2>&1 | tail -15 || fail "libpisp meson"
ninja -C build -j$JOBS 2>&1 | tail -10 || fail "libpisp ninja"
ninja -C build install 2>&1 | tail -5 || fail "libpisp install"
ldconfig
pkg-config --modversion libpisp && echo "  libpisp OK" || fail "libpisp pkg-config"

# ---------- 2) libcamera ----------
echo; echo "######## [2/4] libcamera $LIBCAM_TAG 鍮뚮뱶 (10~25遺??뚯슂) ########"
cd "$SRC"
rm -rf libcamera
git clone --depth 1 -b "$LIBCAM_TAG" https://github.com/raspberrypi/libcamera.git 2>&1 | tail -3 || fail "libcamera clone"
cd libcamera
meson setup build --buildtype=release \
  --prefix=/usr --libdir=lib/aarch64-linux-gnu \
  -Dpipelines=rpi/vc4,rpi/pisp \
  -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=true \
  -Dgstreamer=enabled \
  -Dcam=enabled \
  -Dqcam=disabled \
  -Dtest=false \
  -Dlc-compliance=disabled \
  -Ddocumentation=disabled \
  -Dpycamera=disabled 2>&1 | tail -30 || fail "libcamera meson"

echo "--- meson ?ㅼ젙 ?붿빟 ---"
meson configure build 2>/dev/null | grep -E 'pipelines|ipas|gstreamer|prefix|libdir' | head -10

ninja -C build -j$JOBS 2>&1 | tail -15 || fail "libcamera ninja"
ninja -C build install 2>&1 | tail -5 || fail "libcamera install"
ldconfig

# ---------- 3) ?ㅼ튂 寃利?----------
echo; echo "######## [3/4] ?ㅼ튂 寃곌낵 ########"
echo "--- IPA 紐⑤뱢 ---"
ls -1 /usr/lib/aarch64-linux-gnu/libcamera/ 2>/dev/null | sed 's/^/  /'
echo "--- pisp ?쒕떇 ?뚯씪 (ov5647 ?꾩닔) ---"
ls -1 /usr/share/libcamera/ipa/rpi/pisp/ 2>/dev/null | head -25 | sed 's/^/  /'
echo "--- gstreamer ?뚮윭洹몄씤 ---"
find /usr/lib -name 'libgstlibcamera.so' 2>/dev/null | sed 's/^/  /'
echo "--- ?쇱씠釉뚮윭由?---"
ldconfig -p | grep -E 'libcamera|libpisp' | sed 's/^/  /'

# ---------- 4) 移대찓???닿굅 ----------
echo; echo "######## [4/4] 移대찓???닿굅 ?뚯뒪??########"
cam -l 2>&1 | grep -vE '^\[[0-9]' | sed 's/^/  /'

if cam -l 2>/dev/null | grep -q 'ov5647\|Available cameras' && cam -l 2>/dev/null | grep -qE '^[0-9]+:'; then
  echo "SUCCESS" > "$MARK"
  echo "### 移대찓???몄떇 ?깃났"
else
  echo "ENUM_FAILED" > "$MARK"
  echo "### 鍮뚮뱶???먯쑝??移대찓???닿굅 ?ㅽ뙣 - 濡쒓렇 ?뺤씤 ?꾩슂"
fi
echo "######## 醫낅즺 $(date) ########"
