#pragma once

#include <algorithm>
#include <cmath>

#include "trajectory.hpp"

/*
 * 시험 전용 궤적 보조 함수
 * ------------------------------------------------------------------------
 * 기존 trajectory.hpp 는 그대로 재사용하되, 본 시험에서 새로 필요한 두 가지를
 * 여기에 추가합니다. 검증된 파일을 건드리지 않기 위해 분리했습니다.
 *
 *   1) vertical_setpoint()  : 순수 수직 이동.
 *      기존 line_setpoint() 는 속도 feed-forward 를 수평 성분으로만 만듭니다.
 *      (dn, de 가 0 이면 속도가 0). 50 cm / 100 cm 목표는 Action::takeoff() 를
 *      쓸 수 없어 Offboard 로 직접 상승해야 하므로 수직 feed-forward 가 필요합니다.
 *
 *   2) trapezoid_speed_at() : 사다리꼴 속도 프로파일.
 *      기존 move_line() 은 등속입니다. POC §7.7 은 "코너에서 서행, 장변에서
 *      서서히 가속 후 다음 코너 전 감속"을 요구하므로 거리 기반 프로파일을
 *      추가했습니다.
 *
 * 둘 다 순수 함수이므로 tests/test_test_trajectory.cpp 로 검증합니다.
 */

namespace riposte_test {

/*
 * 순수 수직 이동 setpoint
 *
 *   위치 : down = start.down + progress * (finish.down - start.down)
 *   속도 : down_m_s = +speed (하강) 또는 -speed (상승)
 *   yaw  : 호출자가 지정 (수직 이동 중에는 기수를 유지하는 편이 안전)
 *
 * progress 가 1 이면 속도를 0 으로 두어 끝점에서 정지합니다.
 * 수평 좌표는 start 값을 그대로 유지합니다.
 */
inline bullet_mission::TrajectorySetpoint vertical_setpoint(
    const bullet_mission::NedPoint& start,
    const bullet_mission::NedPoint& finish,
    const double progress,
    const double speed_m_s,
    const double yaw_deg)
{
    const double p = bullet_mission::clamp01(progress);
    const double delta_down = finish.down_m - start.down_m;

    bullet_mission::TrajectorySetpoint setpoint{};
    setpoint.position = {
        start.north_m,
        start.east_m,
        start.down_m + p * delta_down,
    };
    setpoint.yaw_deg = yaw_deg;

    if (std::abs(delta_down) > 1.0e-6 && p < 1.0) {
        // Down 이 커지는 방향이 하강입니다. 부호를 delta 에서 가져와
        // 상승/하강 어느 쪽이든 같은 코드로 처리합니다.
        setpoint.velocity.down_m_s = std::copysign(speed_m_s, delta_down);
    }
    return setpoint;
}

// 사다리꼴 속도 프로파일의 입력입니다.
struct SpeedProfile {
    double entry_speed_m_s{};  // 구간 시작 속도 (앞 코너의 서행 속도)
    double cruise_speed_m_s{}; // 장변 순항 속도 상한
    double exit_speed_m_s{};   // 구간 종료 속도 (다음 코너의 서행 속도)
    double accel_m_s2{};       // 가감속도 (가속과 감속에 같은 값 사용)
};

/*
 * 구간 시작점에서 distance_m 만큼 진행한 지점의 목표 속도입니다.
 *
 *   가속 한계 : v <= sqrt(v_entry^2 + 2*a*s)
 *   감속 한계 : v <= sqrt(v_exit^2  + 2*a*(L-s))     ← 남은 거리로 멈출 수 있어야 함
 *   순항 한계 : v <= v_cruise
 *
 * 셋 중 최솟값이 목표 속도입니다. 구간이 짧아 순항까지 못 올라가면 자동으로
 * 삼각 프로파일이 되므로 별도 분기가 필요 없습니다.
 *
 *   v ^      ______________
 *     |     /              \        cruise
 *     |    /                \
 *     |___/                  \___   entry / exit
 *     +------------------------------> s
 */
inline double trapezoid_speed_at(
    const SpeedProfile& profile, const double distance_m, const double segment_length_m)
{
    if (segment_length_m <= 1.0e-6 || profile.accel_m_s2 <= 0.0) {
        return std::max(0.0, profile.cruise_speed_m_s);
    }
    const double travelled_m = std::clamp(distance_m, 0.0, segment_length_m);
    const double remaining_m = segment_length_m - travelled_m;

    const double accel_limit_m_s = std::sqrt(
        profile.entry_speed_m_s * profile.entry_speed_m_s
        + 2.0 * profile.accel_m_s2 * travelled_m);
    const double decel_limit_m_s = std::sqrt(
        profile.exit_speed_m_s * profile.exit_speed_m_s
        + 2.0 * profile.accel_m_s2 * remaining_m);

    const double speed_m_s = std::min(
        profile.cruise_speed_m_s, std::min(accel_limit_m_s, decel_limit_m_s));

    // 0 을 반환하면 적분이 그 자리에 멈춰 진행이 없습니다. entry/exit 가 모두 0 인
    // 경우를 대비해 아주 작은 하한을 둡니다.
    return std::max(0.05, speed_m_s);
}

} // namespace riposte_test
