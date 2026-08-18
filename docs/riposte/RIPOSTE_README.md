# X500 + Pixhawk 시험 애플리케이션 (Riposten_CPP_v1.4.2-test)

> 기존 `Bullet_Drone_Mission_CPP_v1.4.2-ChatGPT` 를 기반으로 작성한 **시험 전용**
> 애플리케이션입니다. 안전 골격(예외 → 비상 착륙, setpoint watchdog, 소프트웨어
> 반경 제한, 배터리 게이트)은 기존 코드의 것을 그대로 유지했고, 바뀐 것은
> "무엇을 비행하는가"입니다.

시험 계획서의 Test 1 ~ 22 를 번호로 선택해 실행하고, 호버 구간의 오차를 누적해
PASS/FAIL 을 판정한 뒤 요약 CSV 를 남깁니다.

> **운영자용 사용 설명서는 `docs/TEST_USAGE.md`** 에 있습니다 (옵션 표, 실행
> 절차, 판정 기준, CSV 열, 문제 해결). 이 README 는 설계 배경과 근거 중심입니다.

---

## 1. 시험 카탈로그

> 이 표는 `include/riposte/test_cases.hpp` 의 `kTestCatalog` 와 일치한다. 카탈로그를 바꿨으면
> 이 표도 함께 고칠 것. 실제 값은 `riposte_test --list` 로 언제든 확인된다.

| ID | 고도 | 남북 1 m 왕복 | 반복 | 비고 |
|----|------|--------------|------|------|
| 1~3 | 5 m | 없음 | 1 / 3 / 10 | 호버 5초 |
| 4~6 | 6 m | 없음 | 1 / 3 / 10 | 호버 5초 |
| 7~10 | 7 m | 없음 | 1 / 3 / 5 / 10 | 호버 5초 |
| 11~13 | 5 m | 있음 | 1 / 3 / 10 | 호버 5초 |
| 14~16 | 6 m | 있음 | 1 / 3 / 10 | 호버 5초 |
| 17~20 | 7 m | 있음 | 1 / 3 / 5 / 10 | 호버 5초 |
| 21 | 5 m | — | 1 lap | POC Bullet 경로, D=100 m, 폭 2 m, 코너 2 m/s, 순항 4 m/s |
| 22 | 6 m | — | 1 lap | POC Bullet 경로, D=200 m, 폭 5 m, 코너 3 m/s, 순항 6 m/s |
| 23 | 7 m | — | 1 lap | Test 22 와 같은 경로, 고도만 7 m |

1회 사이클: `이륙 → (남북 왕복) → 호버 5초 → 착륙 → Disarm`.
남북 왕복은 편도 **1 m 를 0.3 m/s** 로 이동한다(편도 약 3.3 초).
반복 시험은 **매 사이클 다시 Arm/이륙** 하며 사이클 사이에 **5 초** 쉰다(모터 열, EKF 재수렴,
육안 점검). 반복 시험의 목적이 Arm~Disarm 전 구간의 재현성 확인이기 때문이다.

### 판정 기준 (`include/riposte/test_config.hpp`)

| 항목 | 값 |
|------|-----|
| 수평 허용오차 | 0.6 m |
| 수직 허용오차 | `max(0.30 m, 고도 × 0.10)` → 5 m 에서 0.50 m, 6 m 0.60 m, 7 m 0.70 m |
| 도달 판정 | 속도 0.5 m/s 이하로 1.0 초 유지, 도달 타임아웃 30 초 |
| 배터리 게이트 | 첫 이륙 전 60 %, 매 사이클 전 35 %, 비행 중 30 % 이하면 복귀 |
| 소프트웨어 반경 | 호버 시험 60 m / Bullet 은 산출 경로 최대반경 + 20 m |
| Offboard setpoint | 20 Hz, 생성 정지 감시 300 ms |

```bash
./build/riposte_test --list
```

---

## 2. 기존 코드에서 바뀐 점

| 항목 | 기존 `main.cpp` | 본 시험 앱 |
|------|----------------|-----------|
| 시나리오 | 10 m 이륙 + 원주비행 고정 1개 | 카탈로그에서 번호로 선택 (23개) |
| 반복 | 없음 | N회, 사이클마다 Arm/Disarm |
| 이륙 | `Action::takeoff()` 만 | 고도에 따라 2개 경로 (아래 §3) |
| 판정 | "임무 완료 여부" | 호버 오차 기반 PASS/FAIL + 요약 CSV |
| 목표 좌표 | 이륙점 기준 상대 좌표만 | 위/경도 → Local NED 변환 추가 |
| 속도 프로파일 | 등속 | 코너 서행 ↔ 장변 사다리꼴 가감속 (POC §7.7) |
| 반경 제한 | 60 m 고정 | 호버 시험 60 m, Bullet 은 경로에서 산출 |
| EO/IR 녹화 | 포함 (GStreamer/Hailo) | **제외** |

**EO/IR 녹화를 뺀 이유**: 본 앱의 목적은 비행 거동 검증입니다. GStreamer/Hailo
의존을 빼면 SITL 장비와 실기체에서 **같은 바이너리**를 쓸 수 있어, 시험 결과에
영상 파이프라인 부하가 섞이지 않습니다. 영상 증빙이 필요하면 기존 미션 앱을
사용하십시오.

---

## 3. 이륙 경로가 두 개인 이유

PX4 의 `Action::takeoff()` 는 파라미터 `MIS_TAKEOFF_ALT` 를 사용하며 **하한이 있습니다
(기본 2.5 m)**. 그보다 낮은 고도를 요구하면 PX4 가 값을 거부하거나 클램프하므로
`Action::takeoff()` 로는 저고도 시험을 만들 수 없습니다. 그래서 고도에 따라 경로를 나눕니다.

| 고도 | 경로 | 순서 |
|------|------|------|
| ≥ 2.5 m | PX4 `Action::takeoff` | arm → takeoff → 고도 확인 → Offboard 진입 |
| < 2.5 m | Offboard 직접 상승 (0.5 m/s) | setpoint 스트림 → Offboard 진입 → arm → 램프 상승 |

임계값은 `TestTuning::action_takeoff_min_altitude_m = 2.5F` 입니다.

> **현재 카탈로그(Test 1~23)는 모두 5 m 이상이므로 전부 `Action::takeoff` 경로를 씁니다.**
> Offboard 직접 상승 경로는 코드에 남아 있지만 지금은 어떤 시험도 타지 않습니다.
> 저고도 시험을 다시 넣는다면 아래 "저고도의 현실적 한계"를 먼저 읽으십시오.

Offboard 경로에서 `prime → offboard.start → arm` 순서인 이유는 PX4 가 Offboard 진입 전에
**유효한 setpoint 스트림이 이미 살아 있기를 요구**하기 때문입니다.
(기존 미션 코드는 `arm → takeoff → offboard.start` 순서였습니다.)

### 저고도 시험을 다시 넣을 때의 현실적 한계

- 기압계만으로는 고도 드리프트가 **±0.3 m 수준**이 정상입니다. 50 cm 목표라면 20 cm ~ 80 cm 를
  오간다는 뜻입니다. 그래서 수직 허용오차 하한을 0.30 m 로 두었습니다.
  **이보다 좁은 판정을 원하면 거리센서가 필요합니다.**
- 권장: 하방 거리센서 장착 후 `EKF2_HGT_REF = Range` (또는 `EKF2_RNG_CTRL` 활성화).
  X500 프레임에서 50 cm 는 지면효과 영향권이므로 거리센서 없이는 재현성이 떨어집니다.
- 거리센서 없이 저고도 시험을 수행해 FAIL 이 나왔다면, 그것이 **소프트웨어 결함이 아니라
  센서 구성의 한계**일 수 있습니다. ULog 의 `vehicle_local_position.z` 와 기압/거리 소스를
  함께 확인하십시오.
- 현재 카탈로그의 5~7 m 에서는 수직 허용오차가 0.50~0.70 m 로 넓어지고 지면효과에서도
  벗어나므로, 위 제약은 해당하지 않습니다.

---

## 4. 빌드

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

MAVSDK 가 `/opt/mavsdk` 등 별도 위치에 있으면:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/mavsdk
```

MAVSDK 없이 기하/판정 단위시험만 돌리려면:

```bash
cmake -S . -B build-tests -G Ninja -DBUILD_TEST_APP=OFF
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

Sanitizer 는 별도 빌드 디렉터리에서 각각 실행합니다.

```bash
cmake -S . -B build-asan -DBUILD_TEST_APP=OFF -DENABLE_ASAN=ON
cmake --build build-asan
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 ctest --test-dir build-asan --output-on-failure

cmake -S . -B build-ubsan -DBUILD_TEST_APP=OFF -DENABLE_UBSAN=ON
cmake --build build-ubsan
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 ctest --test-dir build-ubsan --output-on-failure
```

기존 프로젝트와 동일하게 `-Wall -Wextra -Wpedantic -Wconversion -Werror` 를 앱과
단위시험 양쪽에 적용합니다.

---

## 5. 실행

### 5.1 계획 확인 (기체 연결 불필요)

비행 전에 **무엇을 비행할지 반드시 눈으로 확인**하십시오.

```bash
./build/riposte_test --test 13 --plan-only
./build/riposte_test --test 21 --plan-only --plan-home 36.6606482 126.3417854
```

### 5.2 PX4 SITL

```bash
# 터미널 1
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# 터미널 2
./scripts/run_sitl_test.sh 1
./scripts/run_sitl_test.sh 13 2     # 10회 대신 2회만
```

### 5.3 실기체 (Pixhawk)

```bash
# USB
./build/riposte_test --test 1 --execute --connection serial:///dev/ttyACM0:115200

# TELEM2 UART
./build/riposte_test --test 1 --execute --connection serial:///dev/ttyAMA0:921600
```

`--execute` 가 없으면 기체에 **어떤 명령도 보내지 않습니다**. SITL/벤치 점검이
끝난 뒤에만 붙이십시오.

### 5.4 Ground Station 에서 원격 실행

지상국 PC 에서 텔레메트리 링크로 시험을 요청할 수 있습니다. companion 쪽은
`--serve` 로 대기시키고, 지상국은 같은 빌드에서 나오는 `riposte_gcs` 를 씁니다.

```bash
# companion (RPi)
./build/riposte_test --serve --execute --connection serial:///dev/ttyAMA0:921600

# 지상국 PC
./build/riposte_gcs --test 1 --execute --connection serial:///dev/ttyUSB0:57600
```

프로토콜, PX4 포워딩 설정, SITL 검증 절차, 안전 설계는 **`docs/GCS_USAGE.md`** 를
반드시 먼저 읽으십시오. 원격 실행이라도 §9 의 실기체 체크리스트는 그대로
적용됩니다.

---

## 6. Test 21 / 22 — POC Bullet 경로

경로를 좌표로 하드코딩하지 않고 POC 시나리오 V9 §7.2 의 **4개 입력**에서
산출합니다.

```
입력 : S(이륙 지점), T(공격 목표 지점), wL/wR(좌/우 폭), D(전체 비행거리)

  d = |T - S|
  u = (T - S) / d
  n = 좌측 법선
  W = wL + wR
  L = D/2 - W
  F = S + L*u
  P_L = P + wL*n , P_R = P - wR*n   for P in {S, T, F}

출력 : S_L → T_L → F_L → F_R → T_R → S_R → (S_L 복귀)
```

`S` 는 **실제 이륙 위치**이므로 실행 시각에 정해집니다. 목표 `T` 는 위/경도로
주어지므로 `include/geo.hpp` 가 Local NED 로 변환합니다.

### ⚠ 필수 제약: 이륙 지점이 목표에서 L 이내여야 합니다

§7.6 의 `L > d` 조건 때문입니다. 위반하면 원단점 F 가 목표 앞에 생겨 T_L/T_R 이
장변을 벗어나고 6점 포함이 깨집니다. 프로그램이 이륙 직후 이를 검사하여
**위반 시 비행하지 않고 중단**합니다.

| 시험 | D | 폭 | W | **L** | 이륙 지점 허용 범위 |
|------|---|-----|---|-------|-------------------|
| Test 21 | 100 m | 2 m | 4 m | **46 m** | 목표에서 46 m 이내 |
| Test 22 | 200 m | 5 m | 10 m | **90 m** | 목표에서 90 m 이내 |

즉 **Test 21 은 목표 지점 46 m 안쪽에서 이륙해야 합니다.** 현장 배치를 먼저
정하고 `--plan-only --plan-home <lat> <lon>` 로 확인하십시오.

### 좌표계 주의

POC 문서는 x=주행방향, y=폭 인 수학 XY 평면을 쓰며 좌측 법선을 `n = (-u_y, u_x)`
로 적었습니다. 본 코드는 **NED** 이고 위에서 보면 North 가 위, East 가 오른쪽이라
회전 부호가 반대입니다. NED 에서 좌측 법선은 `n = (u_east, -u_north)` 입니다.
`wL == wR` 인 POC 적용값에서는 모양이 같아 표시가 안 나지만, 순회 방향이 뒤집히고
`wL != wR` 이면 경로가 틀어집니다. `tests/test_bullet_path.cpp` 가 이 부호를
고정합니다.

### 속도 (POC §7.7 대비 보수적)

| | POC 권고 | 본 시험 Test 21 | Test 22 |
|---|---|---|---|
| 코너 | ~3 m/s | 2 m/s | 3 m/s |
| 장변 순항 | ~8 m/s | 4 m/s | 6 m/s |

POC 권고보다 낮춘 이유는 시험 고도가 3 m / 5 m 로 낮기 때문입니다. 현장에서
안정성을 확인한 뒤 `include/test_cases.hpp` 에서 올리십시오.

---

## 7. 판정 기준

호버 5초 구간의 표본으로 판정합니다.

| 항목 | 기준 |
|------|------|
| 최대 고도오차 | `max(0.30 m, 고도 × 0.10)` |
| 최대 수평오차 | 0.60 m |
| 호버 유지시간 | 요구시간 − 0.5 s 이상 |
| 표본 수 | 기대치(20 Hz × 5 s = 100)의 50% 이상 |
| 유효성 | NaN/Inf 표본 0개 |

- 반복 시험은 **계획한 횟수를 모두 마치고 전부 PASS** 여야 전체 PASS 입니다.
- 표본 수 기준이 있는 이유: setpoint 루프가 밀리면(CPU 부하, UART 대역폭) 오차
  통계 자체를 신뢰할 수 없기 때문입니다. 오차가 아무리 좋아 보여도 FAIL 입니다.

**이 PASS 가 보증하지 않는 것**: 자세, 진동, EKF 건전성, actuator 여유는 여기서
보지 않습니다. PX4 ULog 를 반드시 함께 검토하십시오.

---

## 8. 출력

```
logs/test13_20260717T044012Z.csv          # 20 Hz 상세 로그
logs/test13_summary_20260717T044530Z.csv  # 사이클별 PASS/FAIL 요약
```

상세 CSV 열: `elapsed_s, test_id, iteration, phase, target_n/e/d, actual_n/e/d,
horizontal_error_m, altitude_error_m, battery_remaining_percent`

- `iteration` 열이 있어 "3회차 호버가 유독 나빴다" 같은 분석이 가능합니다.
- `altitude_error_m` 은 **양수 = 목표보다 높이 떠 있음** 입니다 (NED Down 부호를
  뒤집어 직관에 맞췄습니다).
- `phase` 로 단계 구분이 됩니다. Bullet 경로는 `LAP1_SL_TL` → `LAP1_TL_FL` 처럼
  라벨이 바뀌므로 **T_L/T_R 통과 시각**을 CSV 에서 바로 읽을 수 있습니다.

---

## 9. 안전 사항

기존 미션 앱의 안전 계층을 그대로 유지합니다.

- **setpoint watchdog** (300 ms): MAVSDK 는 마지막 setpoint 를 자동 재전송하므로
  이 앱의 궤적 생성 스레드가 멈춰도 PX4 는 정상으로 착각합니다. watchdog 은
  MAVSDK 재전송이 아니라 **이 앱의 새 명령 생성**만 감시하여, 정지 시 Offboard
  중단 후 Land 를 요청합니다.
- **소프트웨어 반경 제한**: 실제 위치가 아니라 **생성된 목표**를 검사하여, 잘못된
  수식/설정이 만든 명령을 PX4 로 보내기 전에 차단합니다.
- **배터리 3단 게이트**: 이륙 전 60% / 사이클 시작 35% / 비행 중 30%.
  10회 반복 도중 배터리가 마르면 사이클 경계에서 안전하게 종료합니다.
- **SIGINT/SIGTERM**: 플래그만 세우고, 실제 착륙은 정상 실행 흐름에서 처리합니다.

### 첫 실기체 시험 전 확인

1. SITL 에서 해당 시험 번호를 10회 이상 반복해 모두 PASS 할 것
2. 첫 실기체 비행은 `--repeat 1` 로 단계적으로 확장할 것
3. RC 조종사가 즉시 Position/Altitude/Land 모드로 전환할 수 있을 것
4. PX4 geofence 를 시험장 경계 안에 설정할 것
5. `COM_OF_LOSS_T` 와 `COM_OBL_RC_ACT` 를 시험장에서 검증할 것
6. **50 cm / 100 cm 시험은 거리센서 구성을 먼저 확인할 것** (§3)
7. Test 21/22 는 이륙 지점이 목표에서 L 이내인지 `--plan-only` 로 확인할 것
8. Test 21/22 는 목표 너머로 `L - d` 만큼의 개활 공역이 필요함 (§7.5)

---

## 10. 파일 구성

```
include/
  test_cases.hpp        시험 카탈로그 (Test 1~23)          [신규]
  test_config.hpp       튜닝값 (mission_config.hpp 에서 파생) [파생]
  test_verdict.hpp      PASS/FAIL 판정과 집계               [신규]
  test_trajectory.hpp   수직 상승 + 사다리꼴 속도 프로파일   [신규]
  bullet_path.hpp       POC §7.2 직사각형 경로 산출         [신규]
  geo.hpp               위/경도 → Local NED 변환            [신규]
  gcs_protocol.hpp      Ground Station 원격 요청 프로토콜    [신규]
  trajectory.hpp        직선/원 궤적 수학                   [기존 재사용, 무수정]
  safety_policy.hpp     비상조치 선택 + watchdog 판정        [기존 재사용, 무수정]
src/
  test_main.cpp         TestRunner + --serve 에이전트        [파생]
  gcs_main.cpp          지상국 프로그램 (riposte_gcs)        [신규]
tests/
  test_bullet_path.cpp  POC §7.4 검산표 대조
  test_gcs_protocol.cpp 원격 요청 해석/진행 라인 형식
  test_geo.cpp          좌표 변환 부호/축척
  test_test_trajectory.cpp  상승 부호, 속도 프로파일
  test_verdict.cpp      판정 로직 + 카탈로그가 시험 계획서와 일치하는가
scripts/
  run_sitl_test.sh      단위시험 → 계획 출력 → SITL 실행
docs/
  TEST_USAGE.md         시험 앱 운영자용 사용 설명서
  GCS_USAGE.md          Ground Station 원격 실행 사용법
```

`trajectory.hpp` 와 `safety_policy.hpp` 는 **한 줄도 고치지 않았습니다.** 검증된
코드이기 때문입니다. 새로 필요한 궤적 함수는 `test_trajectory.hpp` 로 분리했습니다.

---

## 11. 검증 현황

작성 시점에 다음을 확인했습니다.

| 항목 | 결과 |
|------|------|
| 4개 단위시험 (`-Wall -Wextra -Wpedantic -Wconversion -Werror`) | 통과 |
| AddressSanitizer | 통과 |
| UndefinedBehaviorSanitizer (+ float-cast-overflow, float-divide-by-zero) | 통과 |
| `test_main.cpp` 컴파일 (`-Wconversion -Werror`) | 통과 |
| `--list` / `--plan-only` 실제 실행 | 통과 |
| POC §7.4 검산표 (d=60, W=10, L=140, F=(380,0), 둘레=300) | 일치 |

**진행 현황 (2026-07-17 갱신)**

- [x] 실제 MAVSDK v3.17.1 링크 빌드 — 완료. 이 과정에서 **v3 의
      `remaining_percent` 가 0~100 퍼센트로 바뀐 것**(기존 코드는 0~1 비율 가정)을
      발견해 수정했습니다. 기존 미션 앱에도 같은 문제가 있을 가능성이 높습니다.
- [x] 저고도 Offboard 이륙 경로의 SITL 검증 (§3 의 `prime → start → arm` 순서)
      — PX4 v1.17.0 SITL(gz_x500)에서 Test 1/3 으로 확인.
- [x] GCS 원격 실행/중단 흐름의 SITL end-to-end — `docs/GCS_USAGE.md` §9 참조.
- [x] PX4 SITL 전 시험 케이스(1~22) 실행 — **전부 PASS** (2026-07-17, PX4
      v1.17.0 gz_x500 headless, 총 90+ Arm~Disarm 사이클). Test 21/22 는 태안
      좌표 월드(`PX4_GZ_WORLD` + spherical_coordinates)에서 6점 경로 완주.
      배치에서 발견·수정된 사항은 `docs/GCS_USAGE.md` §9.2 참조.
- [ ] 실기체 검증

SITL 반복 시험에서 한 가지 운영 주의: PX4 기본 arming 체크가 GCS 연결을
요구하므로, 반복 사이클 중 GCS(QGC 또는 riposte_gcs)가 끊겨 있으면 재-Arm 이
거부됩니다 (`docs/GCS_USAGE.md` §8).
