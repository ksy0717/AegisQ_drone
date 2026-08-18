#pragma once

#include <algorithm>

/*
 * 시험 튜닝값 (기존 include/mission_config.hpp 에서 파생)
 * ------------------------------------------------------------------------
 * 기존 미션 코드의 MissionConfig 는 "10 m 이륙 + 원주비행" 한 가지 시나리오에
 * 맞춰 상수를 고정해 두었습니다. 본 시험 앱은 50 cm ~ 5 m 를 오가므로,
 * 고도에 무관한 값만 상수로 남기고 고도 의존적인 허용오차는 함수로 바꿨습니다.
 *
 * 값을 바꿨다면 반드시 다음을 다시 수행하십시오.
 *   1) ctest (경로/기하 단위시험)
 *   2) --plan-only 로 산출 경로 확인
 *   3) PX4 SITL 전체 시험
 *
 * 단위는 변수명에 표시했습니다: _m, _m_s, _s, _hz.
 */

namespace riposte_test {

struct TestTuning {
    // ---- Offboard setpoint 스트림 -------------------------------------
    // PX4 는 최소 2 Hz 의 지속적인 Offboard 생존 신호를 요구합니다. 기존
    // 미션과 동일하게 여유를 두어 20 Hz 로 위치+속도 setpoint 를 보냅니다.
    static constexpr float setpoint_rate_hz = 20.0F;

    // setpoint "생성"이 멈추면(스레드 stall) MAVSDK 는 마지막 값을 계속
    // 재전송하므로 PX4 는 정상으로 착각합니다. 이 watchdog 이 생성 정지를
    // 감시하여 Offboard 중단 후 Land 를 요청합니다. 300 ms = 20 Hz 기준 6주기.
    static constexpr float setpoint_watchdog_timeout_s = 0.30F;
    static constexpr float setpoint_watchdog_poll_s = 0.025F;

    // ---- 저고도 이륙 경로 선택 ----------------------------------------
    // PX4 의 Action::takeoff() 는 MIS_TAKEOFF_ALT 를 사용하며, 기본값은
    // 2.5 m 이고 하한이 있습니다. 50 cm / 100 cm 목표를 Action::takeoff() 로
    // 요구하면 PX4 가 값을 거부하거나 클램프하여 시험 고도가 어긋납니다.
    // 따라서 이 임계값 미만은 Offboard 로 직접 상승합니다.
    static constexpr float action_takeoff_min_altitude_m = 2.5F;

    // Offboard 직접 상승 시의 수직 속도입니다. 지면효과 구간을 천천히
    // 벗어나야 50 cm 목표에서 오버슈트가 줄어듭니다.
    static constexpr float offboard_climb_speed_m_s = 0.5F;

    // ---- 도달 판정 허용오차 -------------------------------------------
    static constexpr float horizontal_tolerance_m = 0.6F;
    static constexpr float arrival_speed_tolerance_m_s = 0.5F;
    static constexpr float arrival_settle_s = 1.0F;
    static constexpr float arrival_timeout_s = 30.0F;

    // 수직 허용오차의 하한입니다. 기압계만으로는 ±0.3 m 수준의 드리프트가
    // 정상이므로 이보다 좁히려면 거리센서가 필요합니다(README 참조).
    //
    // 이 두 값만 double 입니다. vertical_tolerance_for() 가 double 로 계산되고
    // 그 결과가 double 인 위치 오차와 직접 비교되기 때문입니다. float 로 두면
    // 0.30F -> double 승격에서 약 1.2e-8 m 의 오차가 섞여 들어옵니다. 비행에는
    // 무해하지만, 허용오차 자체가 정확히 표현되지 않으면 판정 근거를 설명하기
    // 어려워집니다. (단위시험 test_vertical_tolerance_scaling 이 이를 고정합니다.)
    static constexpr double vertical_tolerance_floor_m = 0.30;

    // 고도가 높을수록 허용오차를 비례해 넓힙니다.
    static constexpr double vertical_tolerance_ratio = 0.10;

    // ---- 타임아웃 ------------------------------------------------------
    static constexpr float takeoff_timeout_s = 90.0F;
    static constexpr float landing_timeout_s = 90.0F;
    static constexpr float health_timeout_s = 60.0F;
    static constexpr float discovery_timeout_s = 15.0F;
    static constexpr float local_position_timeout_s = 10.0F;
    static constexpr float battery_telemetry_timeout_s = 10.0F;

    // ---- 배터리 게이트 -------------------------------------------------
    // 이 임계값들은 0.0~1.0 "비율"입니다. MAVSDK v3 의 remaining_percent 는
    // 0~100 퍼센트이므로, test_main.cpp 의 battery_remaining_fraction() 이
    // 비율로 정규화한 값과 비교합니다 (SITL E2E 에서 확인된 v2→v3 단위 변경).
    // 첫 이륙 전 게이트입니다.
    static constexpr float preflight_min_battery_fraction = 0.60F;

    // 반복 시험은 사이클마다 다시 이륙하므로, 매 사이클 시작 전에 이 값을
    // 확인합니다. 10회 반복 도중 배터리가 마르면 여기서 조기 종료합니다.
    static constexpr float iteration_min_battery_fraction = 0.35F;

    // 비행 중 이 값 이하가 되면 남은 구간을 포기하고 착륙합니다.
    static constexpr float inflight_return_battery_fraction = 0.30F;

    // ---- 반복 사이클 ---------------------------------------------------
    // 착륙/disarm 후 다음 이륙까지의 대기입니다. 모터 열, EKF 재수렴,
    // 육안 점검 시간을 위해 둡니다.
    static constexpr float inter_iteration_pause_s = 5.0F;

    // ---- 로깅 ----------------------------------------------------------
    static constexpr float log_flush_interval_s = 1.0F;

    // ---- 지오펜스 (소프트웨어) ----------------------------------------
    // Test 1~20 은 반경 몇 m 안에서 끝나므로 기존 60 m 를 그대로 씁니다.
    static constexpr float hover_radius_limit_m = 60.0F;

    // Bullet 경로(Test 21~23)는 원단점 F 가 목표 너머까지 나가므로 60 m 로는
    // 부족합니다. 산출된 경로 최대 반경에 이 여유를 더해 한계를 정합니다.
    static constexpr float bullet_radius_margin_m = 20.0F;
};

// 고도에 비례한 수직 허용오차입니다.
//   0.5 m  -> 0.30 m (하한)
//   3.0 m  -> 0.30 m (하한)
//   5.0 m  -> 0.50 m
inline double vertical_tolerance_for(const double altitude_m)
{
    const double scaled = altitude_m * TestTuning::vertical_tolerance_ratio;
    return std::max(TestTuning::vertical_tolerance_floor_m, scaled);
}

} // namespace riposte_test
