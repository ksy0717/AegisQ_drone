# 기체 무선 링크 구성 — 2026-08-11 변경 기록

Pi 5 (`pi-pixhawk-2`)의 WiFi 경로를 내장 라디오 2.4 GHz에서 **USB 동글 5 GHz 장거리 AP**로
옮겼다. 영상(RTSP)·MQTT·WireGuard 백홀이 모두 이 링크를 탄다. 이 문서는 무엇을 왜 바꿨는지,
어떤 함정이 있었는지, 어떻게 되돌리는지를 다룬다.

작업자: Claude (SkyHunter GCS 세션). 코드 변경 없음 — Pi의 netplan 설정만 바뀌었다.

---

## 요약

| | 변경 전 | 변경 후 |
|---|---|---|
| 사용 라디오 | 내장 `wlan0` | **USB 동글 `wlx90de80fad724`** |
| AP / 주파수 | `iptime_drone` · **2412 MHz** | `SkyAP_5G` · **5220 MHz** |
| 링크 속도 | 72.2 Mbit/s | **rx 702 / tx 650 Mbit/s** (VHT 80 MHz, 2 스트림) |
| IP | `192.168.0.6` (DHCP) | **`192.168.0.6` (정적)** — 동일 |
| 내장 `wlan0` | 주 경로 | **비활성** (`ActivationPolicy=always-down`) |

**IP를 그대로 유지한 것이 핵심이다.** 주소를 더 좋은 라디오로 옮겼을 뿐이라 GCS 태블릿의 드론
주소, 포트포워딩, 기존 문서의 `192.168.0.6` 기재가 전부 그대로 유효하다.

---

## 왜 바꿨나

### 1. ELRS와 같은 대역을 쓰고 있었다

변경 전 `wlan0`은 **2412 MHz(2.4 GHz 채널 1)**에 붙어 RTSP를 송출했다. ExpressLRS 2.4 GHz RC
링크는 ISM 대역 전체를 FHSS로 도약하므로, 기체 위에서 고비트레이트 영상을 2.4 GHz로 쏘는 것은
RC 링크와 직접 경합한다. 5 GHz로 옮기면서 영상 트래픽이 RC 대역에서 완전히 빠졌다.

동글 스캔에서 실제로 관측된 이웃:

```
iptime_drone     2412.0 MHz   ← 변경 전 wlan0이 여기 있었음
ExpressLRS RX    2412.0 MHz
```

### 2. 장거리 AP는 기체 쪽 라디오도 받쳐줘야 한다

`SkyAP_5G`는 300 m급 장거리 AP다. 두 라디오가 같은 AP를 듣는 감도를 실측했다:

| 라디오 | SkyAP_5G 수신 감도 |
|---|---|
| 내장 `wlan0` | **-37 dBm** |
| USB 동글 | **-24 dBm** |

**13 dB 차이**로, 자유공간 기준 도달 거리 약 4.5배에 해당한다. 동글 안테나가 5 GHz에서
내장 라디오보다 압도적으로 낫다. `iw dev … info`의 txpower는 wlan0 31 dBm / 동글 23 dBm으로
나오지만 이는 규제 상한이지 실제 복사 전력이 아니며, 측정 가능한 수신 감도가 판단 근거다.

---

## 현재 설정

`/etc/netplan/50-cloud-init.yaml` (root 전용 0600). PSK는 이 문서에 옮기지 않는다.

```yaml
  wifis:
    wlan0:
      activation-mode: "off"          # 평소 뜨지 않음. 복구용으로 설정만 남겨둠
      optional: true
      dhcp4: true
      regulatory-domain: "KR"
      access-points:
        "GL-BE3600-5G": { ... }
        "iptime_drone": { ... }

    wlx90de80fad724:                  # USB 동글. MAC 90:de:80:fa:d7:24
      optional: true
      dhcp4: false
      addresses: [192.168.0.6/24]
      routes:
        - to: default
          via: 192.168.0.1
          metric: 100
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
      regulatory-domain: "KR"
      access-points:
        "SkyAP_5G": { ... }
```

변경 전 원본 백업: `/etc/netplan/50-cloud-init.yaml.bak-20260811-144950`

---

## 함정 (다른 에이전트가 시간 낭비하지 않도록)

### netplan은 wifi에 `match:`를 지원하지 않는다

networkd 백엔드에서 `wifis:` 항목에 `match:`/`set-name:`을 쓰면 generate가 거절한다:

```
ERROR: usbwlan: networkd backend does not support wifi with match:, only by interface name
```

그래서 **인터페이스 이름 `wlx90de80fad724`를 직접 키로 썼다.** 이 이름은 MAC에서 파생되므로
**동글을 다른 개체로 교체하면 이름이 바뀌고 설정이 안 먹는다.** 교체 시 netplan의 키도 같이
바꿔야 한다. (드라이버는 `rtw_8822bu` — Realtek RTL8822BU.)

### SkyAP_5G는 별도 네트워크가 아니다

iptime 공유기에 **LAN으로 물린 브리지 AP**다. 게이트웨이 MAC이 양쪽에서 동일(`b0:38:6c:5e:79:1c`)
하고 DHCP 서버도 같은 `192.168.0.1`이다. BSSID만 다르다:

- `iptime_drone` → `b0:38:6c:5e:79:1e`
- `SkyAP_5G` → `ec:b9:31:5f:48:5d`

따라서 **두 인터페이스를 동시에 올려 metric으로 경로를 가르는 것은 의미가 없다.** 같은 L2,
같은 서브넷, 같은 게이트웨이로 나간다. 얻는 것 없이 ARP flux(양쪽 인터페이스가 같은 주소의
ARP에 응답)와 비대칭 라우팅 위험만 생긴다. `wlan0`을 병행 운용하지 않고 내려둔 이유가 이것이다.

### DHCP가 풀 밖 주소를 내주는 것처럼 보인다

iptime의 DHCP 풀은 `.10~.255`이고 MAC 예약은 설정돼 있지 않은데, `netplan apply` 직후
`wlan0`이 `192.168.0.6`을 새로 받는 것이 로그로 관측됐다:

```
Aug 11 14:51:20  wlan0: DHCPv4 address 192.168.0.6/24 ... acquired from 192.168.0.1
```

원인은 공유기 예약이 아니라 **클라이언트 쪽**이다. systemd-networkd의 DHCP 클라이언트는
재구성 시 직전 주소를 Option 50(Requested IP Address)으로 명시 요청하고, 서버는 그 주소가
놀고 있으면 풀 밖이어도 ACK한다. 임대 기록은 `/run/systemd/netif/leases/`에 있고 재부팅 시
사라진다 — 현재는 비어 있다.

실무적 의미: **`wlan0`을 DHCP로 다시 올려도 `.6`을 받아 동글과 충돌할 가능성은 낮다.** 다만
확실히 하려면 `wlan0`에도 풀 안의 고정 주소(예: `192.168.0.7`)를 박으면 된다. 현재는 하지 않았다.

### 원격에서 무선 설정을 바꿀 때

`netplan apply`는 SSH 세션을 끊는다. `netplan try`는 TTY가 필요해 비대화형에서 쓰기 어렵다.
이번에는 dead-man switch를 걸고 apply를 세션과 분리해 실행했다:

```bash
sudo systemd-run --on-active=300 --unit=netplan-revert \
  /bin/sh -c 'cp /etc/netplan/50-cloud-init.yaml.bak-… /etc/netplan/50-cloud-init.yaml && netplan apply'
sudo systemd-run --unit=netplan-apply-now --no-block netplan apply
# 재접속 성공 후
sudo systemctl stop netplan-revert.timer
```

`--no-block`으로 분리하지 않으면 SSH가 끊기면서 apply가 중단될 수 있다.

---

## 검증 결과 (2026-08-11)

변경 후 실측:

| 항목 | 결과 |
|---|---|
| 동글 링크 | -31 dBm, rx 702 / tx 650 Mbit/s |
| 인터넷 (`8.8.8.8`) | 0% 손실, 36 ms |
| **WireGuard `wg0`** | 핸드셰이크 정상, `10.100.0.1` 20 ms 도달, endpoint `210.94.8.7:51820` |
| Tailscale | `pi-pixhawk-2` 온라인 |
| `mediamtx` / `drone-companion` | 둘 다 active, `cam0` ready |
| GCS 태블릿 영상 | `rtsp://192.168.0.6:8554/cam0` 실영상 표시 확인 |

**AegisQ 백홀(wg0)이 새 링크에서 정상 동작한다.** WHIP publish 경로가 이 링크를 타므로
`ENABLE_WHIP=1`로 전환할 때 별도 네트워크 작업은 필요하지 않다.

GCS 태블릿도 `SkyAP_5G`로 옮겨 두 기기가 같은 AP(`ec:b9:31:5f:48:5d`)에 붙어 있다. iptime AP를
경유하지 않는 최단 경로다.

---

## 복구

동글이 빠지거나 고장나면 **Pi가 네트워크에서 사라진다.** 동글이 유일한 경로다. 물리 접근이
가능하면 내장 라디오를 되살릴 수 있다:

```bash
# 일시적
sudo ip link set wlan0 up && sudo dhclient wlan0

# 영구적: netplan에서 wlan0의 activation-mode 줄을 지우고
sudo netplan apply
```

전체 원복:

```bash
sudo cp /etc/netplan/50-cloud-init.yaml.bak-20260811-144950 /etc/netplan/50-cloud-init.yaml
sudo netplan apply
```

---

## 남은 것

- **`ping` RTT 편차** — 태블릿→기체 ICMP가 25~98 ms(mdev 30 ms)로 흔들린다. RTSP 포트 연결은
  6.8 ms로 빠르므로 안드로이드 절전 응답 지연일 가능성이 높지만, 실제 비행 거리에서 영상
  지연·끊김이 생기면 여기부터 다시 볼 것.
- **동글 물리 고정** — 유일한 네트워크 경로이므로 비행 진동에 대한 커넥터 고정 상태 확인 필요.
- **`wlan0` 고정 주소** — 복구 시 주소를 예측 가능하게 하려면 `192.168.0.7` 등을 박아둘 수 있다.
  현재는 충돌 위험이 낮아 하지 않았다.
- **300 m 실거리 검증** — 벤치 거리에서만 확인했다. 실제 도달 거리와 그 지점의 영상 비트레이트는
  미측정.
