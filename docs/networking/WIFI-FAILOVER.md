# 동글 착탈 자동 전환 — 2026-08-12 변경 기록

[RADIO-LINK.md](RADIO-LINK.md)에서 기체 WiFi 경로를 내장 `wlan0`(2.4 GHz)에서 USB 동글
`wlx90de80fad724`(5 GHz `SkyAP_5G`)로 옮기면서, 내장 라디오는
`activation-mode: "off"`로 내려두고 "복구용으로 설정만 남겨둔" 상태였다.

그 상태에는 실무적 함정이 하나 있었다. **동글을 뽑으면 Pi가 완전히 고립된다.** `wlan0`은
`always-down`이라 올라오지 않고, `eth0`은 케이블이 없으며, `wg0`·`tailscale0`은 모두 동글
링크 위에 올라가 있어 같이 죽는다. 복구 수단이 물리 접근뿐이 된다.

이 문서는 그 자리를 메운 변경을 다룬다. **동글을 뽑으면 `wlan0`이 `iptime_drone`으로 자동
전환되고, 다시 꽂으면 `wlan0`이 내려간다.**

작업자: Claude (Claude Code 세션). 코드 변경 없음 — Pi 설정만 바뀌었다.

---

## 요약

| | 변경 전 | 변경 후 |
|---|---|---|
| `wlan0` 활성화 정책 | `ActivationPolicy=always-down` | **`manual`** (설정 유지, 자동 기동만 안 함) |
| 동글 뽑았을 때 | 네트워크 전무 — 물리 접근만 | **`wlan0` → `iptime_drone` 자동 전환** |
| 동글 꽂았을 때 | — | **`wlan0` 자동 하강**, 동글이 기본 경로 회수 |
| 동글 없이 부팅 | 네트워크 전무 | **부팅 유닛이 `wlan0` 기동** |
| SSH 주소 | `192.168.0.6` | **어느 경로든 `192.168.0.6`** (변동 없음) |

두 AP가 같은 서브넷·같은 게이트웨이라서 주소가 바뀌지 않는다. 기존 문서의 `192.168.0.6`
기재, 포트포워딩, RTSP URL이 전부 그대로 유효하다.

---

## 왜 "한 번에 하나만"인가

처음 검토한 대안은 **둘 다 올려두고 라우트 metric으로 페일오버**하는 방식이었다. udev도
스크립트도 필요 없고 이벤트를 놓칠 일이 없어 더 견고해 보인다. **이 환경에서는 쓸 수 없다.**

`SkyAP_5G`는 iptime 공유기에 LAN으로 물린 브리지 AP이고 `iptime_drone`과 게이트웨이·DHCP
서버가 동일하다(RADIO-LINK.md §브리지 AP 참고). 그래서 `wlan0`을 DHCP로 올리면 iptime이
**동글의 정적 주소와 똑같은 `192.168.0.6`을 내준다.**

2026-08-12 검증 중 이 충돌을 직접, 재현 가능하게 관측했다:

~~~text
wlan0            UP   192.168.0.6/24 metric 700 ...
wlx90de80fad724  UP   192.168.0.6/24 ...
~~~

RADIO-LINK.md는 DHCP 풀이 `.10~.255`라는 점을 들어 "충돌할 가능성은 낮다"고 적었으나,
실제로는 `wlan0`을 올릴 때마다 `.6`을 받았다(2026-08-11 로그 1회, 2026-08-12 검증 2회).
**낮은 가능성이 아니라 재현되는 동작으로 취급해야 한다.**

같은 L2에 같은 IP를 가진 인터페이스 둘은 ARP 응답이 뒤섞이고 비대칭 라우팅을 만든다.
따라서 이 구성의 불변식은 하나다 — **`192.168.0.6`을 들고 있는 인터페이스는 항상 정확히
하나.** 동글이 있으면 동글이, 없으면 `wlan0`이 갖는다.

`wlan0`에 `dhcp4-overrides.route-metric: 700`을 넣어 둔 것은 그 불변식이 깨진 순간에도
기본 경로만은 동글(metric 100)이 이기게 하는 안전벨트다. 충돌 자체를 막지는 못한다.

---

## 설치물

### 1. netplan — `wlan0`을 수동 제어 가능하게

`/etc/netplan/50-cloud-init.yaml`:

~~~yaml
    wlan0:
      activation-mode: "manual"      # was "off"; udev/부팅 유닛이 제어한다
      optional: true
      dhcp4: true
      dhcp4-overrides:
        route-metric: 700            # 동시에 뜨더라도 동글(100)이 기본 경로를 유지
      regulatory-domain: "KR"
      access-points:
        "GL-BE3600-5G": { ... }
        "iptime_drone": { ... }
~~~

`"off"`는 `ActivationPolicy=always-down`이 되어 `networkctl up`조차 듣지 않는다.
`"manual"`은 자동 기동만 막고 명시적 제어를 허용한다 — 이 전환의 전제다.

### 2. udev — 동글 착탈 감지

`/etc/udev/rules.d/80-wlan0-failover.rules`:

~~~text
SUBSYSTEM=="net", ACTION=="remove", ENV{INTERFACE}=="wlx90de80fad724", RUN+="/usr/bin/systemctl --no-block start wlan0-failover@up.service"
SUBSYSTEM=="net", ACTION=="add", ENV{INTERFACE}=="wlx90de80fad724", RUN+="/usr/bin/systemctl --no-block start wlan0-failover@down.service"
~~~

`ENV{INTERFACE}`로 매칭한다 — `remove` 이벤트에서도 항상 실려 오기 때문이다.
`--no-block`은 udev 규칙 안에서 `systemctl start`가 블록되어 교착되는 것을 막는다.
동글 식별자는 Realtek `0bda:b812`, 드라이버 `rtw_8822bu`.

### 3. systemd — 실제 동작 주체

`/etc/systemd/system/wlan0-failover@.service` (인스턴스 `up` / `down`):

~~~ini
[Service]
Type=oneshot
ExecStart=/usr/bin/networkctl %i wlan0
~~~

`/etc/systemd/system/wlan0-failover-boot.service` (enabled):

~~~ini
ExecStart=/bin/sh -c 'test -e /sys/class/net/wlx90de80fad724 || /usr/bin/networkctl up wlan0'
~~~

**부팅 유닛이 없으면 빈틈이 생긴다.** 동글이 애초에 없는 채로 부팅하면 `add`/`remove`
이벤트가 발생하지 않으므로 udev 규칙만으로는 아무 일도 일어나지 않고, `manual` 정책상
`wlan0`은 내려간 채로 남는다. 즉 네트워크 없이 부팅된다. 동글이 있으면 이 유닛은 no-op다.

---

## 검증 (2026-08-12, `pi-pixhawk-2`)

동글이 **꽂힌 상태에서** 양방향을 모의 실행했다.

- `wlan0-failover@up` → `wlan0`이 `iptime_drone`에 결합, `192.168.0.6` 획득
- `wlan0-failover@down` → `wlan0` 하강, 동글이 기본 경로(metric 100) 회수
- `wlan0-failover-boot` → 동글 존재 시 no-op 확인
- `udevadm verify` → 규칙 문법 통과 (Success 1 / Fail 0)
- `netplan generate` / `netplan apply` 통과

**아직 검증하지 못한 것: 동글을 물리적으로 뽑았을 때 udev가 실제로 발화하는지.**
모의 실행은 유닛을 직접 기동한 것이라 udev 경로 자체는 아직 확인되지 않았다. 뽑기 시험을
하기 전까지 이 구성을 원격 복구 수단으로 신뢰하지 말 것.

### 뽑기 시험 절차

~~~bash
# 뽑기 전 (다른 단말에서)
ping 192.168.0.6

# 동글 제거 후 20초 정도 기다린 뒤
ssh sky@192.168.0.6 'iwgetid -r wlan0; ip route show default'
#   기대: iptime_drone / default ... dev wlan0

# 다시 꽂고
ssh sky@192.168.0.6 'ip -br a | grep -E "wlan0|wlx"'
#   기대: wlan0 DOWN, wlx… UP 192.168.0.6
~~~

udev가 발화하지 않으면 `journalctl -u wlan0-failover@up.service`와
`udevadm monitor --property --subsystem-match=net`으로 이벤트가 오는지부터 본다.

---

## 되돌리기

~~~bash
sudo cp /root/netplan-rollback-20260812-113715.yaml /etc/netplan/50-cloud-init.yaml
sudo chmod 600 /etc/netplan/50-cloud-init.yaml
sudo netplan apply
sudo systemctl disable --now wlan0-failover-boot.service
sudo rm -f /etc/udev/rules.d/80-wlan0-failover.rules \
           /etc/systemd/system/wlan0-failover@.service \
           /etc/systemd/system/wlan0-failover-boot.service
sudo systemctl daemon-reload
sudo udevadm control --reload-rules
~~~

되돌리면 RADIO-LINK.md 시점 상태(내장 `wlan0` `always-down`)로 정확히 복귀한다.

---

## 함정 기록

- **`netplan apply`는 동글 링크를 끊었다 되살린다.** 원격으로 이 작업을 할 때는 먼저
  자동 롤백 안전망을 걸어라. 이번에는 5분짜리로 걸었다:
  `sudo systemd-run --on-active=300 --unit=netplan-rollback /bin/bash -c "cp <백업> <원본> && netplan apply"`.
  복구를 확인한 뒤 `systemctl stop netplan-rollback.timer`로 취소한다. 재결합에 10초 이상
  걸리므로 즉시 확인하면 끊긴 것처럼 보인다 — 한 번은 더 기다려 볼 것.
- **`activation-mode: "off"`는 `networkctl up`도 거부한다.** 복구용으로 설정만 남겨둘
  의도였다면 `"manual"`이 맞다. `"off"`는 "이 인터페이스를 절대 올리지 않는다"는 뜻이다.
- 이 Pi는 벤치 작업 중 사용자가 전원을 직접 내리는 일이 잦다. 긴 검증은 로그를 `/tmp`
  밖에 쓰고 ssh 세션과 분리해 돌린 뒤 재접속해서 수거하는 편이 낫다.
