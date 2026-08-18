# 셋업 노트 — Pi 5 + Ubuntu 24.04 에서 막히는 지점들

`install.sh` 가 자동으로 처리하지만, 왜 그런 단계가 필요한지와 수동 복구 방법을 남긴다.

## 1. apt 소스에 `noble-updates` 가 빠져 있으면 의존성이 통째로 깨진다

증상은 엉뚱하게 나타난다:

```
bzip2 : Depends: libbz2-1.0 (= 1.0.8-5.1) but 1.0.8-5.1build0.1 is to be installed
E: Unable to correct problems, you have held broken packages.
```

설치된 `libbz2-1.0` 은 updates 버전(`build0.1`)인데 apt 목록에는 릴리스 버전만 있어 짝이 맞는 `bzip2` 를
찾지 못한다. 그 뒤로 `dpkg-dev` → `build-essential` → `hailort-pcie-driver` 가 연쇄로 실패한다.

`apt update` 를 다시 돌려도 소용없다. 포켓 자체가 없어서다.

```bash
sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates/' /etc/apt/sources.list.d/ubuntu.sources
sudo apt-get update
```

## 2. Hailo 드라이버는 DKMS 로 넣어야 한다

`hailort-pcie-driver` 는 `build-essential` 에 의존하고, `dkms` 가 없으면 DKMS 트리 대신 직접 빌드로
떨어진다(`Failed to install PCIe driver to the DKMS tree`). 그러면 **커널 업데이트 때 모듈이 사라진다.**

```bash
sudo apt-get install -y build-essential dkms linux-headers-"$(uname -r)"
sudo dpkg -i hailort_*.deb hailort-pcie-driver_*.deb
dkms status                      # hailo_pci/4.24.0, ...: installed
hailortcli fw-control identify   # Device Architecture: HAILO8
```

## 3. Ubuntu 24.04 의 libcamera 로는 Pi 5 카메라가 안 잡힌다

커널은 센서를 정상 인식한다:

```
rp1-cfe 1f00110000.csi: Using sensor ov5647 10-0036 for capture
```

그런데 `cam -l` 은 비어 있다. 원인:

```
Found registered pipeline handler 'PipelineHandlerVc4'
RPI vc4.cpp:179  Unable to acquire a Unicam instance
```

Ubuntu 의 libcamera 0.2.0 패키지에는 `ipa_rpi_vc4.so`(Pi 4 이하용)만 있고 **Pi 5 의 PiSP 용
`ipa_rpi_pisp.so` 와 `/usr/share/libcamera/ipa/rpi/pisp/` 가 통째로 없다.** `libpisp` 도 저장소에 없다.
설정 문제가 아니라 구성요소 부재라 config.txt 나 오버레이로는 해결되지 않는다.

`scripts/build_libcamera.sh` 가 libpisp + Raspberry Pi 포크 libcamera 를 소스 빌드한다
(Pi 5 에서 `-j3` 기준 약 15~25분). 빌드 전에 Ubuntu 패키지를 먼저 제거해야 구버전 플러그인과 충돌하지 않는다.

버전은 커널과 시기를 맞춘다. 커널 6.8 계열에는 **libcamera `v0.3.2+rpt20241119` + libpisp `v1.0.7`** 조합을 쓴다.
최신 태그(2026년)는 커널보다 2년 앞서 ABI 가 어긋날 위험이 있다.

성공하면:

```
$ cam -l
Available cameras:
1: 'ov5647' (/base/axi/pcie@120000/rp1/i2c@88000/ov5647@36)
```

## 4. CSI 카메라는 전원을 끄고 연결한다

`camera_auto_detect=1` 은 **부팅 시점에만** 동작하므로 연결 후 재부팅이 필요하다.
동작 중 핫플러그는 커넥터·센서 손상 위험이 있다.

### 커넥터 · 오버레이 · i2c 버스 매핑

Pi 5 에는 `CAM/DISP 0` 과 `CAM/DISP 1` 두 커넥터가 있다. `camera_auto_detect=0` 이면 오버레이에
**어느 커넥터인지까지** 적어야 하며, 틀리면 센서에 전원·클럭이 가지 않아 아예 잡히지 않는다.

| 커넥터 | 오버레이 | 센서가 뜨는 i2c 버스 | CSI 블록 |
|---|---|---|---|
| CAM/DISP 0 | `dtoverlay=<센서>,cam0` | `i2c-10` | `1f00110000.csi` |
| CAM/DISP 1 | `dtoverlay=<센서>,cam1` | `i2c-11` | `1f00128000.csi` |

카메라를 옮겼거나 모델을 바꿨으면 `/boot/firmware/config.txt` 의 오버레이를 함께 고치고 재부팅한다.
**버스 번호가 어느 커넥터인지 말해주므로**, 의심스러우면 오버레이를 바꿔 재부팅해 보고
`/sys/bus/i2c/devices/*/name` 에서 센서가 뜨는 버스로 확인하면 된다.

### 카메라가 안 나올 때 — 제어선과 데이터선을 분리해서 본다

CSI 카메라는 **I²C(제어)** 와 **CSI-2 차동 쌍(데이터)** 이 물리적으로 다른 핀을 쓴다.
둘을 나눠 보면 원인이 좁혀진다.

```bash
# ① 제어선: 칩 ID 를 센서에서 직접 읽는다 (imx219 기대값 02 19)
BUS=10; ADDR=0x10                       # 위 표에서 커넥터에 맞는 버스
sudo i2ctransfer -f -y $BUS w2@$ADDR 0x00 0x00 r2

# ② 데이터선: 프레임이 실제로 오는지 센다 (scripts/camera_setup.sh 5·6번이 이 일을 한다)
```

| ① 제어 | ② 데이터 | 해석 |
|---|---|---|
| OK | OK | 정상 |
| OK | 0프레임 | 데이터 레인만 접촉 불량 — 케이블 의심 |
| NAK(-121) | 0프레임 | 케이블이 아예 안 맞물림, 방향 반대, 또는 오버레이의 커넥터가 틀림 |

`dmesg` 에 `rp1-cfe ...: Using a link rate of ... Mbps` 가 몇 초 간격으로 반복되면
스트리밍 시작을 계속 재시도하는 중이라는 뜻이다 — 즉 **데이터가 한 장도 안 오고 있다.**
`Camera frontend has timed out!` 도 같은 상황이다.

**서로 다른 커넥터에서 같은 증상이면 보드보다 케이블·모듈을 의심한다.** 동작이 확인된
카메라+케이블 조합을 꽂아 보는 것이 가장 빠른 분리 실험이다.

#### 두 커넥터가 동시에 죽는 것은 이상한 일이 아니다

CAM/DISP 0 과 1 은 커넥터만 둘이고, 뒤쪽은 **RP1 칩 하나**가 두 CSI 수신기를 모두 품는다.
클럭도 같은 블록에서 나온다(`cam0_clk`/`cam1_clk`, `clk_mipi0_cfg`/`clk_mipi1_cfg`).
따라서 RP1 의 MIPI 입력단 문제 하나로 양쪽이 함께 죽을 수 있다 — 독립된 두 고장이 아니라
**단일 고장점**이다. "설마 둘 다 고장이겠나" 로 보드를 후보에서 빼지 말 것.

#### XCLK(센서 마스터 클럭)은 연속 샘플링으로 확인한다

`clk_summary` 를 한 번 찍어 `enable_cnt=0` 이라고 "클럭이 안 켜진다" 고 판단하면 틀린다.
libcamera 는 실패 시 ~5 초 주기로 센서 전원을 껐다 켜므로 대부분의 순간은 꺼져 있다.
0.5 초 간격으로 수십 번 샘플링해 **한 번이라도 `enable_cnt>0` 이 잡히는지**를 봐야 한다.

```bash
for i in $(seq 1 40); do
  sudo grep -E '^\s*cam[01]_clk' /sys/kernel/debug/clk/clk_summary | awk '{print $2}'
  sleep 0.5
done | sort -u        # 0 만 나오면 클럭이 정말 안 켜지는 것
```

실제 사례: 40 회 중 2 회에서 `enable_cnt=1` 이 잡혀 클럭 가설이 기각되고, 원인이 보드로 좁혀졌다.

> libcamerasrc 는 이 타임아웃 경로에서 `requestCompleted` assertion 으로 segfault 를 내기도 한다.
> 원인이 아니라 증상이니 크래시 자체를 쫓지 말 것.

## 5. 사용자를 `video` 그룹에 넣는다

없으면 `/dev/media*` 에서 `Permission denied` 가 나고 libcamera 가 카메라를 열거하지 못한다.

```bash
sudo usermod -aG video,render sky      # 재로그인 필요
```

## 6. 디스플레이는 `fbdevsink` 로 낸다

Pi 5 는 DRM 카드가 여러 개다 — `card0`=rp1-dsi(DSI 패널), `card2`=vc4(HDMI).
`kmssink` 는 커넥터가 하나도 연결되지 않은 vc4 를 잡고 `allowed caps is empty` 로 죽는다.
`fbdevsink device=/dev/fb0` 는 DSI 에 직결된다(`/sys/class/graphics/fb0/device -> ...dsi`).

SSH 로 화면을 확인하려면 `scripts/grab_fb.sh` 가 `/dev/fb0` 를 되읽어 PNG 로 저장한다.
