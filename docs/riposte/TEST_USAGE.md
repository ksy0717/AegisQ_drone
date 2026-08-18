# 시험 애플리케이션 사용 설명서 (riposte_test)

X500 + Pixhawk 시험 앱 `riposte_test` 의 운영자용 매뉴얼입니다. 설계 배경과
근거는 `README.md`, Ground Station 원격 실행은 `docs/GCS_USAGE.md` 를 보십시오.

---

## 1. 빌드

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

| 상황 | 명령 |
|------|------|
| MAVSDK 가 별도 위치에 있음 | `cmake ... -DCMAKE_PREFIX_PATH=/opt/mavsdk` |
| MAVSDK 없이 단위시험만 | `cmake -S . -B build-tests -G Ninja -DBUILD_TEST_APP=OFF` |

빌드 산출물:

| 파일 | 용도 |
|------|------|
| `build/riposte_test` | 시험 앱 (companion 컴퓨터/SITL 공용, 이 문서의 대상) |
| `build/riposte_gcs` | 지상국 프로그램 (`docs/GCS_USAGE.md`) |

### 1.1 단위시험 (기체 불필요)

코드를 수정했거나 처음 받았다면 비행 전에 반드시 돌립니다.

```bash
ctest --test-dir build --output-on-failure
```

5개 스위트(bullet_path, gcs_protocol, geo, test_trajectory, verdict)가 모두
Passed 여야 합니다. Sanitizer 검증은 별도 빌드 디렉터리에서:

```bash
cmake -S . -B build-asan -DBUILD_TEST_APP=OFF -DENABLE_ASAN=ON
cmake --build build-asan
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 ctest --test-dir build-asan --output-on-failure

cmake -S . -B build-ubsan -DBUILD_TEST_APP=OFF -DENABLE_UBSAN=ON
cmake --build build-ubsan
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 ctest --test-dir build-ubsan --output-on-failure
```

---

## 2. 명령줄 옵션

```
riposte_test --test <1-23> --execute [options]
```

| 옵션 | 의미 |
|------|------|
| `--test <id>` | 실행할 시험 번호 (1~23) |
| `--list` | 시험 카탈로그 출력 후 종료 |
| `--plan-only` | 기체 연결 없이 산출 경로/예상 시간만 출력 |
| `--plan-home <lat> <lon>` | `--plan-only` 에서 쓸 가상 이륙 지점 (Test 21/22 확인용) |
| `--repeat <n>` | 반복 횟수를 카탈로그 값 대신 사용 |
| `--serve` | Ground Station 원격 요청 대기 모드 (`docs/GCS_USAGE.md`) |
| `--connection <url>` | MAVSDK 연결 URL (기본 `udpin://0.0.0.0:14540` = SITL) |
| `--execute` | **안전 게이트.** 이 옵션이 없으면 기체에 어떤 명령도 보내지 않음 |
| `--help`, `-h` | 도움말 |

- 오타가 있는 옵션은 무시되지 않고 **즉시 오류로 거부**됩니다 (의도와 다른
  시험이 비행하는 것을 막기 위함).
- `--serve` 는 `--test`/`--plan-only`/`--repeat` 와 함께 쓸 수 없습니다
  (시험 번호와 반복 횟수는 GCS 요청이 지정).

### 종료 코드

| 코드 | 의미 |
|------|------|
| 0 | 시험 전체 PASS (`--list`/`--plan-only`/`--help` 는 정상 출력) |
| 1 | FAIL, 조기 종료(배터리/중단), 연결 실패, 사용법 오류 |

---

## 3. 시험 카탈로그

```bash
./build/riposte_test --list
```

| ID | 내용 | 고도 | 반복 |
|----|------|------|------|
| 1~3 | 호버 5초 | 50 cm | 1 / 3 / 10 |
| 4~6 | 호버 5초 | 100 cm | 1 / 3 / 10 |
| 7~10 | 호버 5초 | 300 cm | 1 / 3 / 5 / 10 |
| 11~13 | 남북 1 m 왕복 + 호버 5초 | 50 cm | 1 / 3 / 10 |
| 14~16 | 남북 1 m 왕복 + 호버 5초 | 100 cm | 1 / 3 / 10 |
| 17~20 | 남북 1 m 왕복 + 호버 5초 | 300 cm | 1 / 3 / 5 / 10 |
| 21 | POC Bullet 경로 1랩 (D=100 m, 폭 2 m) | 3 m | 1 사이클 |
| 22 | POC Bullet 경로 1랩 (D=200 m, 폭 5 m) | 5 m | 1 사이클 |

- 1회 사이클: `Arm → 이륙 → (왕복) → 호버 → 착륙 → Disarm`. 반복 시험은 매
  사이클 Arm/이륙을 다시 합니다 (전 구간 재현성 확인 목적).
- 사이클 사이 5초 대기가 있습니다 (모터 열, EKF 재수렴).

### 3.1 저고도 시험(1~6, 11~16) 주의

50 cm / 100 cm 는 PX4 `Action::takeoff()` 의 하한(기본 2.5 m) 아래이므로 앱이
자동으로 **Offboard 직접 상승** 경로를 씁니다. 기압계만으로는 ±0.3 m 드리프트가
정상이므로, **하방 거리센서 없이 FAIL 이 나면 소프트웨어 결함이 아니라 센서
구성의 한계**일 수 있습니다 (README §3).

### 3.2 Test 21/22 필수 제약

이륙 지점이 목표 지점에서 **L 이내**여야 합니다 (21: 46 m, 22: 90 m).
위반 시 프로그램이 이륙 직후 검사해서 **비행하지 않고 중단**합니다.
현장 배치를 정한 뒤 반드시 계획을 먼저 확인하십시오:

```bash
./build/riposte_test --test 21 --plan-only --plan-home <이륙지점 위도> <경도>
```

출력의 6개 waypoint (S_L→T_L→F_L→F_R→T_R→S_R) 와 `overshoot`(목표 너머 필요
공역)을 눈으로 확인합니다. `INVALID` 가 나오면 이륙 위치를 옮겨야 합니다.

---

## 4. 실행 절차

### 4.1 계획 확인 (기체 불필요 — 항상 먼저)

```bash
./build/riposte_test --test 13 --plan-only
./build/riposte_test --test 21 --plan-only --plan-home 36.6606482 126.3417854
```

### 4.2 PX4 SITL

```bash
# 터미널 1
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# 터미널 2 — 스크립트 사용 (단위시험 → 계획 출력 → 실행 순으로 진행)
./scripts/run_sitl_test.sh 1
./scripts/run_sitl_test.sh 13 2      # 카탈로그 10회 대신 2회만

# 스크립트 없이 직접
./build/riposte_test --test 1 --execute --connection udpin://0.0.0.0:14540
```

**SITL 주의 두 가지** (2026-07-17 전 케이스 배치에서 확인):

1. PX4 기본 arming 체크가 GCS 연결을 요구하므로, QGC 또는
   `riposte_gcs --monitor` 를 붙여 두고 실행하십시오 (§7 참조).
2. **Test 21/22 는 gz 월드의 `spherical_coordinates` 가 홈 좌표를 결정**합니다
   (`PX4_HOME_LAT/LON` 환경변수는 gz 에 적용되지 않음). 기본 월드는 취리히라
   목표(태안)에서 8,900 km 떨어져 L>d 검증이 비행을 거부합니다. 태안 좌표
   월드를 만들어 실행하십시오:

   ```bash
   W=~/PX4-Autopilot/Tools/simulation/gz/worlds
   sed -e 's/<world name="default">/<world name="taean">/' \
       -e 's|<latitude_deg>.*<|<latitude_deg>36.6606482<|' \
       -e 's|<longitude_deg>.*<|<longitude_deg>126.3417854<|' \
       $W/default.sdf > $W/taean.sdf
   cd ~/PX4-Autopilot && PX4_GZ_WORLD=taean HEADLESS=1 make px4_sitl gz_x500
   ```

   위 좌표는 목표 지점에서 약 40 m 남쪽으로, Test 21(L=46 m)/22(L=90 m) 의
   이륙 지점 제약을 만족합니다.

### 4.3 실기체 (Pixhawk)

```bash
# USB 직결 (벤치)
./build/riposte_test --test 1 --execute --connection serial:///dev/ttyACM0:115200

# TELEM2 UART (RPi companion, 기본 구성)
./build/riposte_test --test 1 --execute --connection serial:///dev/ttyAMA0:921600
```

**첫 실기체 시험 전 체크리스트 (README §9 요약):**

1. SITL 에서 해당 시험 번호 10회 이상 반복 모두 PASS
2. 첫 비행은 `--repeat 1` 로 시작해 단계적으로 확장
3. RC 조종사가 즉시 Position/Altitude/Land 로 전환할 수 있을 것
4. PX4 geofence 를 시험장 경계 안에 설정
5. `COM_OF_LOSS_T`, `COM_OBL_RC_ACT` 를 시험장에서 검증
6. 50/100 cm 시험은 거리센서 구성 먼저 확인
7. Test 21/22 는 `--plan-only` 로 L 제약과 개활 공역 확인

### 4.4 실행 중 중단

`Ctrl+C` (SIGINT) 를 누르면 즉시 모터를 끊지 않고 **정상 흐름에서 착륙 →
Disarm** 을 수행합니다. 급할 때는 RC 조종사의 모드 전환이 최우선입니다.

### 4.5 배터리 게이트

| 시점 | 기준 | 미달 시 |
|------|------|--------|
| 첫 이륙 전 | 60 % | 시험 시작 거부 |
| 매 사이클 시작 전 | 35 % | 남은 반복 포기, 요약 출력 |
| 비행 중 | 30 % | 다음 안전 전이점에서 착륙 |

---

## 5. 판정 기준 (PASS/FAIL)

호버 5초 구간의 표본(20 Hz)으로 사이클마다 판정합니다.

| 항목 | 기준 |
|------|------|
| 최대 고도오차 | `max(0.30 m, 고도 × 0.10)` — 0.5/1/3 m → 0.30 m, 5 m → 0.50 m |
| 최대 수평오차 | 0.60 m |
| 호버 유지시간 | 요구시간 − 0.5 s 이상 |
| 표본 수 | 기대치(20 Hz × 5 s = 100)의 50 % 이상 |
| 유효성 | NaN/Inf 표본 0개 |

- **시험 전체 PASS** = 계획한 반복을 모두 마치고 + 전 사이클 PASS.
  조기 종료(배터리, 중단)는 오차가 좋아도 전체 FAIL 입니다.
- 표본 수 미달은 setpoint 루프가 밀렸다는 뜻이므로 오차와 무관하게 FAIL 입니다.
- **이 PASS 가 보증하지 않는 것**: 자세, 진동, EKF 건전성, actuator 여유.
  PX4 ULog 를 반드시 함께 검토하십시오.

---

## 6. 출력물

```
logs/test13_20260717T044012Z.csv          # 20 Hz 상세 로그
logs/test13_summary_20260717T044530Z.csv  # 사이클별 요약
```

파일명 시각은 UTC 입니다 (PX4 ULog 와 대조 용이).

### 6.1 상세 CSV 열

```
elapsed_s, test_id, iteration, phase,
target_n_m, target_e_m, target_d_m, actual_n_m, actual_e_m, actual_d_m,
horizontal_error_m, altitude_error_m, battery_remaining_percent
```

- `altitude_error_m` 은 **양수 = 목표보다 높이 떠 있음** (NED 부호를 뒤집어 직관화).
- `phase` 예: `TAKEOFF` → `SETTLE_AT_ALTITUDE` → `MOVE_NORTH` → … → `HOVER`.
  Bullet 경로는 `LAP1_SL_TL` → `LAP1_TL_FL` 처럼 바뀌므로 **T_L/T_R 통과 시각**을
  바로 읽을 수 있습니다.

### 6.2 요약 CSV 열

```
test_id, test_name, iteration, verdict, commanded_altitude_m,
mean_altitude_error_m, max_altitude_error_m, max_horizontal_error_m,
hover_duration_s, hover_samples, cycle_duration_s,
battery_start_percent, battery_end_percent, note
```

- `verdict` = PASS/FAIL, `note` = FAIL 사유.
- `battery_*_percent` 가 **−1** 이면 해당 시점에 유효한 배터리 텔레메트리가
  없었다는 표시입니다 (측정값 아님).

---

## 7. 문제 해결

| 증상 | 원인 후보 | 조치 |
|------|----------|------|
| `Safety gate: add --execute ...` | `--execute` 누락 | 의도된 동작. SITL/벤치 점검 후 붙일 것 |
| `No autopilot discovered` | 포트/장치명/baud 오류, TX/RX 교차, `MAV_1_CONFIG` | 연결 URL 과 PX4 시리얼 설정 확인 |
| `Vehicle health check timed out` | GPS fix, home, compass, EKF 미완 | QGroundControl preflight 메시지 확인. 코드를 고치지 말 것 |
| `Preflight battery ... below required` | 배터리 60 % 미만 | 충전 후 재시도 |
| `arm failed ... Command Denied` (PX4: `Arming denied`) | GCS 미연결 — PX4 기본 arming 체크가 GCS 연결 요구 | QGC 또는 `riposte_gcs --monitor` 를 연결해 두고 실행 |
| `Bullet path is not valid from this takeoff point` | L > d 제약 위반 | 이륙 위치 이동 또는 D 조정, `--plan-only` 재확인 |
| `Generated setpoint ... exceeds the software radius limit` | 경로 산출/설정 오류 | 비행 전 차단이 목적. 설정과 계획 출력 재확인 |
| `Setpoint generation watchdog expired` | 제어 루프 정지(CPU 부하, I/O 지연) | companion 부하 확인. 앱이 이미 착륙을 요청했음 |
| 호버 FAIL (표본 수 미달) | setpoint 루프 밀림 | CPU/UART 대역폭 확인, 로그 저장장치 속도 확인 |
| 50 cm 시험 고도 FAIL 반복 | 기압계 드리프트 | 하방 거리센서 + `EKF2_HGT_REF=Range` (README §3) |

---

## 8. 관련 문서

| 문서 | 내용 |
|------|------|
| `README.md` | 설계 배경, 기존 코드와의 차이, 검증 현황 |
| `docs/GCS_USAGE.md` | Ground Station 원격 실행 (riposte_gcs + --serve) |
| POC 시나리오 V9 | Test 21/22 경로 산출식(§7.2)과 검산표(§7.4)의 원 문서 |
