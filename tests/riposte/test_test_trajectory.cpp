#include "test_support.hpp"
#include "test_trajectory.hpp"

#include <cmath>

/*
 * 시험 전용 궤적 보조 함수 단위시험
 * ------------------------------------------------------------------------
 * 1) vertical_setpoint  : 저고도 Offboard 이륙의 핵심. 부호를 틀리면 이륙 대신
 *                         지면을 향해 추력을 넣습니다. 반드시 여기서 걸러야 합니다.
 * 2) trapezoid_speed_at : POC §7.7 의 "코너 서행, 장변 가속" 프로파일.
 *
 * NED 이므로 상승은 down_m 이 작아지는 방향, 상승 속도는 down_m_s 가 음수입니다.
 */

using bullet_mission::NedPoint;

namespace {

// 상승: 지상 down=+0.2 에서 0.5 m 목표 -> down=-0.3
void test_climb_sign_is_negative_down()
{
    const NedPoint ground{10.0, 20.0, 0.2};
    const NedPoint air{10.0, 20.0, -0.3};

    const auto middle = riposte_test::vertical_setpoint(ground, air, 0.5, 0.5, 0.0);
    test_support::check_close(
        middle.position.down_m, -0.05, 1.0e-9, "halfway up sits between the two altitudes");
    test_support::check(
        middle.velocity.down_m_s < 0.0, "climbing must command a negative down velocity");
    test_support::check_close(
        middle.velocity.down_m_s, -0.5, 1.0e-9, "climb speed magnitude matches the request");
}

// 하강: 부호가 반대여야 합니다.
void test_descent_sign_is_positive_down()
{
    const NedPoint air{0.0, 0.0, -3.0};
    const NedPoint ground{0.0, 0.0, 0.0};

    const auto middle = riposte_test::vertical_setpoint(air, ground, 0.5, 0.4, 0.0);
    test_support::check(
        middle.velocity.down_m_s > 0.0, "descending must command a positive down velocity");
    test_support::check_close(middle.velocity.down_m_s, 0.4, 1.0e-9, "descent speed magnitude");
}

void test_vertical_endpoints_and_horizontal_hold()
{
    const NedPoint ground{5.0, -7.0, 0.1};
    const NedPoint air{5.0, -7.0, -1.0};

    const auto start = riposte_test::vertical_setpoint(ground, air, 0.0, 0.5, 30.0);
    test_support::check_close(start.position.down_m, 0.1, 1.0e-9, "progress 0 is the start");
    test_support::check_close(start.yaw_deg, 30.0, 1.0e-9, "yaw is passed through");

    const auto finish = riposte_test::vertical_setpoint(ground, air, 1.0, 0.5, 30.0);
    test_support::check_close(finish.position.down_m, -1.0, 1.0e-9, "progress 1 is the finish");
    test_support::check_close(
        finish.velocity.down_m_s, 0.0, 1.0e-9, "velocity is zero at the end point");

    // 수평 좌표는 흔들리면 안 됩니다.
    test_support::check_close(finish.position.north_m, 5.0, 1.0e-9, "north is held during a climb");
    test_support::check_close(finish.position.east_m, -7.0, 1.0e-9, "east is held during a climb");

    // progress 범위 밖은 clamp 되어야 합니다.
    const auto over = riposte_test::vertical_setpoint(ground, air, 1.5, 0.5, 0.0);
    test_support::check_close(over.position.down_m, -1.0, 1.0e-9, "progress above 1 is clamped");
    const auto under = riposte_test::vertical_setpoint(ground, air, -0.5, 0.5, 0.0);
    test_support::check_close(under.position.down_m, 0.1, 1.0e-9, "progress below 0 is clamped");

    // 시작과 끝이 같으면 속도는 0 이어야 합니다.
    const auto still = riposte_test::vertical_setpoint(ground, ground, 0.5, 0.5, 0.0);
    test_support::check_close(
        still.velocity.down_m_s, 0.0, 1.0e-9, "a zero-length climb commands no velocity");

    test_support::check(bullet_mission::finite_setpoint(start), "vertical setpoint is finite");
}

// 사다리꼴: 시작/끝은 entry/exit, 중간은 cruise 여야 합니다.
void test_trapezoid_shape()
{
    riposte_test::SpeedProfile profile{};
    profile.entry_speed_m_s = 2.0;
    profile.cruise_speed_m_s = 8.0;
    profile.exit_speed_m_s = 2.0;
    profile.accel_m_s2 = 1.5;

    const double length_m = 140.0;

    test_support::check_close(
        riposte_test::trapezoid_speed_at(profile, 0.0, length_m), 2.0, 1.0e-9,
        "speed at the entry corner equals the entry speed");
    test_support::check_close(
        riposte_test::trapezoid_speed_at(profile, length_m, length_m), 2.0, 1.0e-9,
        "speed at the exit corner equals the exit speed");
    test_support::check_close(
        riposte_test::trapezoid_speed_at(profile, length_m / 2.0, length_m), 8.0, 1.0e-9,
        "the middle of a long side reaches cruise speed");

    // 가속 구간의 해석해: v = sqrt(v0^2 + 2*a*s). s=10 -> sqrt(4+30)=5.831
    test_support::check_close(
        riposte_test::trapezoid_speed_at(profile, 10.0, length_m), std::sqrt(34.0), 1.0e-9,
        "the acceleration ramp follows v = sqrt(v0^2 + 2as)");
    // 감속 구간: 남은 거리 10 -> 동일한 값
    test_support::check_close(
        riposte_test::trapezoid_speed_at(profile, length_m - 10.0, length_m), std::sqrt(34.0),
        1.0e-9, "the deceleration ramp mirrors the acceleration ramp");
}

void test_trapezoid_is_monotone_then_symmetric()
{
    riposte_test::SpeedProfile profile{};
    profile.entry_speed_m_s = 2.0;
    profile.cruise_speed_m_s = 8.0;
    profile.exit_speed_m_s = 2.0;
    profile.accel_m_s2 = 1.5;
    const double length_m = 140.0;

    // 전반부는 단조 증가여야 합니다.
    double previous = riposte_test::trapezoid_speed_at(profile, 0.0, length_m);
    for (double s = 1.0; s <= length_m / 2.0; s += 1.0) {
        const double speed = riposte_test::trapezoid_speed_at(profile, s, length_m);
        test_support::check(speed >= previous - 1.0e-9, "the first half never slows down");
        previous = speed;
    }
    // 대칭성: v(s) == v(L-s)
    for (double s = 0.0; s <= length_m; s += 7.0) {
        test_support::check_close(
            riposte_test::trapezoid_speed_at(profile, s, length_m),
            riposte_test::trapezoid_speed_at(profile, length_m - s, length_m), 1.0e-9,
            "a symmetric profile is mirror-symmetric about the midpoint");
    }
    // 상한 초과 없음
    for (double s = 0.0; s <= length_m; s += 3.0) {
        test_support::check(
            riposte_test::trapezoid_speed_at(profile, s, length_m) <= 8.0 + 1.0e-9,
            "speed never exceeds the cruise limit");
    }
}

// 구간이 짧으면 순항에 도달하지 못하고 삼각 프로파일이 되어야 합니다.
void test_short_segment_becomes_triangular()
{
    riposte_test::SpeedProfile profile{};
    profile.entry_speed_m_s = 2.0;
    profile.cruise_speed_m_s = 8.0;
    profile.exit_speed_m_s = 2.0;
    profile.accel_m_s2 = 1.5;

    // 10 m 짧은 단변. 2 m/s 에서 8 m/s 까지 가속하려면 (64-4)/(2*1.5)=20 m 가 필요합니다.
    const double length_m = 10.0;
    const double peak = riposte_test::trapezoid_speed_at(profile, length_m / 2.0, length_m);
    test_support::check(peak < 8.0, "a short segment never reaches cruise speed");
    test_support::check(peak > 2.0, "a short segment still accelerates above the entry speed");
    // 해석해: v_peak = sqrt(v0^2 + 2*a*(L/2)) = sqrt(4 + 15) = 4.359
    test_support::check_close(peak, std::sqrt(19.0), 1.0e-9, "the triangular peak is analytic");
}

// 비대칭 코너 속도
void test_asymmetric_entry_and_exit()
{
    riposte_test::SpeedProfile profile{};
    profile.entry_speed_m_s = 1.0;
    profile.cruise_speed_m_s = 6.0;
    profile.exit_speed_m_s = 3.0;
    profile.accel_m_s2 = 1.0;
    const double length_m = 100.0;

    test_support::check_close(
        riposte_test::trapezoid_speed_at(profile, 0.0, length_m), 1.0, 1.0e-9, "entry speed");
    test_support::check_close(
        riposte_test::trapezoid_speed_at(profile, length_m, length_m), 3.0, 1.0e-9, "exit speed");
}

void test_degenerate_profiles_do_not_stall_the_integrator()
{
    riposte_test::SpeedProfile profile{};
    profile.entry_speed_m_s = 0.0;
    profile.cruise_speed_m_s = 5.0;
    profile.exit_speed_m_s = 0.0;
    profile.accel_m_s2 = 1.0;

    // entry 가 0 이어도 적분이 멈추지 않도록 하한이 있어야 합니다.
    test_support::check(
        riposte_test::trapezoid_speed_at(profile, 0.0, 50.0) > 0.0,
        "a zero entry speed still returns a positive floor so the integrator advances");
    test_support::check(
        riposte_test::trapezoid_speed_at(profile, 50.0, 50.0) > 0.0,
        "a zero exit speed still returns a positive floor");

    // 길이 0 이나 가속도 0 에서도 유한한 값을 반환해야 합니다.
    test_support::check(
        std::isfinite(riposte_test::trapezoid_speed_at(profile, 0.0, 0.0)),
        "a zero-length segment returns a finite speed");
    profile.accel_m_s2 = 0.0;
    test_support::check(
        std::isfinite(riposte_test::trapezoid_speed_at(profile, 10.0, 50.0)),
        "a zero acceleration returns a finite speed");

    // 범위 밖 거리도 clamp 되어야 합니다.
    profile.accel_m_s2 = 1.0;
    test_support::check(
        std::isfinite(riposte_test::trapezoid_speed_at(profile, -5.0, 50.0)),
        "a negative distance is clamped, not propagated into a NaN square root");
    test_support::check(
        std::isfinite(riposte_test::trapezoid_speed_at(profile, 999.0, 50.0)),
        "a distance beyond the segment is clamped");
}

} // namespace

int main()
{
    test_climb_sign_is_negative_down();
    test_descent_sign_is_positive_down();
    test_vertical_endpoints_and_horizontal_hold();
    test_trapezoid_shape();
    test_trapezoid_is_monotone_then_symmetric();
    test_short_segment_becomes_triangular();
    test_asymmetric_entry_and_exit();
    test_degenerate_profiles_do_not_stall_the_integrator();
    return test_support::report("test_trajectory_tests");
}
