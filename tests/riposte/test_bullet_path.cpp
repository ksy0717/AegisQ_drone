#include "bullet_path.hpp"
#include "test_support.hpp"

#include <cmath>

/*
 * Bullet 경로 기하 단위시험
 * ------------------------------------------------------------------------
 * POC 시나리오 V9 §7.4 의 검산표를 그대로 시험 벡터로 씁니다.
 *
 *   입력 : S=(240, 0), T=(300, 0), wL=wR=5 m, D=300 m
 *   기대 : d=60, W=10, L=140, F=(380, 0), 둘레=300, 목표 너머 연장=80
 *
 * 문서의 좌표계는 x=주행방향, y=폭 입니다. 이를 NED 로 옮기기 위해
 * x -> North, y -> East 로 두고, 문서가 좌측이라 부르는 방향(+y)이 NED 에서
 * 어디인지 확인합니다. 자세한 부호 논의는 bullet_path.hpp 주석 참조.
 */

using bullet_mission::NedPoint;
using riposte_test::BulletWaypoint;

namespace {

constexpr std::size_t index_of(const BulletWaypoint waypoint)
{
    return static_cast<std::size_t>(waypoint);
}

// §7.4 검산표: 스칼라 산출값이 문서와 일치하는가
void test_poc_reference_values()
{
    riposte_test::BulletPathInputs inputs{};
    inputs.start = {240.0, 0.0, -20.0};
    inputs.target = {300.0, 0.0, -20.0};
    inputs.width_left_m = 5.0;
    inputs.width_right_m = 5.0;
    inputs.total_distance_m = 300.0;

    std::string reason;
    test_support::check(
        riposte_test::validate_bullet_path(inputs, reason),
        "POC reference inputs must be valid: " + reason);

    const auto path = riposte_test::compute_bullet_path(inputs);
    test_support::check_close(path.target_distance_m, 60.0, 1.0e-9, "d = 60 m");
    test_support::check_close(path.width_m, 10.0, 1.0e-9, "W = 10 m");
    test_support::check_close(path.long_side_m, 140.0, 1.0e-9, "L = D/2 - W = 140 m");
    test_support::check_close(path.perimeter_m, 300.0, 1.0e-9, "perimeter equals input D");
    test_support::check_close(path.overshoot_m, 80.0, 1.0e-9, "overshoot L - d = 80 m");
    test_support::check_close(path.far_point.north_m, 380.0, 1.0e-9, "F north = 380 m");
    test_support::check_close(path.far_point.east_m, 0.0, 1.0e-9, "F east = 0 m");
    test_support::check_close(path.heading_deg, 0.0, 1.0e-9, "heading is due north");
}

// §7.4 검산표: 6개 필수 통과점의 좌표
// 문서 표는 (x, y) = (240,+5) (300,+5) (380,+5) (380,-5) (300,-5) (240,-5) 입니다.
// x -> North 이므로 north 성분은 그대로, y 성분은 폭이므로 |east| = 5 여야 합니다.
void test_poc_reference_waypoints()
{
    riposte_test::BulletPathInputs inputs{};
    inputs.start = {240.0, 0.0, -20.0};
    inputs.target = {300.0, 0.0, -20.0};
    inputs.width_left_m = 5.0;
    inputs.width_right_m = 5.0;
    inputs.total_distance_m = 300.0;

    const auto path = riposte_test::compute_bullet_path(inputs);

    const double expected_north[] = {240.0, 300.0, 380.0, 380.0, 300.0, 240.0};
    for (std::size_t i = 0; i < riposte_test::kBulletWaypointCount; ++i) {
        test_support::check_close(
            path.waypoints[i].north_m, expected_north[i], 1.0e-9,
            std::string("waypoint ") + std::to_string(i + 1) + " north");
        test_support::check_close(
            std::abs(path.waypoints[i].east_m), 5.0, 1.0e-9,
            std::string("waypoint ") + std::to_string(i + 1) + " lateral offset is wL/wR");
        // 고도는 6점 모두 출발점 고도와 같아야 합니다.
        test_support::check_close(
            path.waypoints[i].down_m, -20.0, 1.0e-9,
            std::string("waypoint ") + std::to_string(i + 1) + " keeps the start altitude");
    }

    // 첫 3점(SL, TL, FL)은 같은 쪽, 뒤 3점(FR, TR, SR)은 반대쪽에 있어야 합니다.
    const double left_side = path.waypoints[index_of(BulletWaypoint::StartLeft)].east_m;
    const double right_side = path.waypoints[index_of(BulletWaypoint::StartRight)].east_m;
    test_support::check(left_side * right_side < 0.0, "L and R offsets are on opposite sides");
    test_support::check(
        path.waypoints[index_of(BulletWaypoint::TargetLeft)].east_m * left_side > 0.0,
        "T_L is on the same side as S_L");
    test_support::check(
        path.waypoints[index_of(BulletWaypoint::FarLeft)].east_m * left_side > 0.0,
        "F_L is on the same side as S_L");
}

// 북진할 때 좌측은 서쪽(east 음수)이어야 합니다. NED 회전 부호 확인.
void test_left_normal_direction_in_ned()
{
    riposte_test::BulletPathInputs inputs{};
    inputs.start = {0.0, 0.0, -10.0};
    inputs.target = {50.0, 0.0, -10.0}; // 정북 방향
    inputs.width_left_m = 4.0;
    inputs.width_right_m = 1.0;
    inputs.total_distance_m = 300.0;

    const auto path = riposte_test::compute_bullet_path(inputs);
    // 북진 중 좌측 = 서쪽 = east 음수
    test_support::check_close(
        path.waypoints[index_of(BulletWaypoint::StartLeft)].east_m, -4.0, 1.0e-9,
        "left offset of a northbound leg points west by wL");
    test_support::check_close(
        path.waypoints[index_of(BulletWaypoint::StartRight)].east_m, 1.0, 1.0e-9,
        "right offset of a northbound leg points east by wR");
}

// 동진할 때 좌측은 북쪽(north 양수)이어야 합니다.
void test_left_normal_for_eastbound_leg()
{
    riposte_test::BulletPathInputs inputs{};
    inputs.start = {0.0, 0.0, -10.0};
    inputs.target = {0.0, 50.0, -10.0}; // 정동 방향
    inputs.width_left_m = 3.0;
    inputs.width_right_m = 3.0;
    inputs.total_distance_m = 300.0;

    const auto path = riposte_test::compute_bullet_path(inputs);
    test_support::check_close(
        path.waypoints[index_of(BulletWaypoint::StartLeft)].north_m, 3.0, 1.0e-9,
        "left offset of an eastbound leg points north by wL");
    test_support::check_close(
        path.waypoints[index_of(BulletWaypoint::FarLeft)].east_m,
        path.long_side_m + 0.0, 1.0e-9,
        "F_L lies L metres east of the start");
}

// 사선 방향에서도 폭이 정확히 유지되는가 (§7.6 일반성)
void test_diagonal_and_asymmetric_width()
{
    riposte_test::BulletPathInputs inputs{};
    inputs.start = {10.0, 20.0, -15.0};
    inputs.target = {40.0, 60.0, -15.0}; // 3-4-5 삼각형 -> d = 50
    inputs.width_left_m = 7.0;
    inputs.width_right_m = 2.0;
    inputs.total_distance_m = 400.0;

    std::string reason;
    test_support::check(
        riposte_test::validate_bullet_path(inputs, reason), "diagonal case is valid: " + reason);

    const auto path = riposte_test::compute_bullet_path(inputs);
    test_support::check_close(path.target_distance_m, 50.0, 1.0e-9, "diagonal d = 50 m");
    test_support::check_close(path.width_m, 9.0, 1.0e-9, "asymmetric W = wL + wR = 9 m");
    test_support::check_close(path.long_side_m, 191.0, 1.0e-9, "L = 400/2 - 9 = 191 m");
    test_support::check_close(path.perimeter_m, 400.0, 1.0e-9, "perimeter equals D");

    // S_L 은 S 에서 정확히 wL, S_R 은 정확히 wR 만큼 떨어져야 합니다.
    const auto distance_from_start = [&](const BulletWaypoint waypoint) {
        return bullet_mission::horizontal_distance(inputs.start, path.waypoints[index_of(waypoint)]);
    };
    test_support::check_close(
        distance_from_start(BulletWaypoint::StartLeft), 7.0, 1.0e-9, "S_L is wL from S");
    test_support::check_close(
        distance_from_start(BulletWaypoint::StartRight), 2.0, 1.0e-9, "S_R is wR from S");

    // 단변 SL-SR 의 길이는 W 여야 합니다.
    test_support::check_close(
        bullet_mission::horizontal_distance(
            path.waypoints[index_of(BulletWaypoint::StartLeft)],
            path.waypoints[index_of(BulletWaypoint::StartRight)]),
        9.0, 1.0e-9, "short side S_L..S_R equals W");

    // 장변 SL-FL 의 길이는 L 여야 합니다.
    test_support::check_close(
        bullet_mission::horizontal_distance(
            path.waypoints[index_of(BulletWaypoint::StartLeft)],
            path.waypoints[index_of(BulletWaypoint::FarLeft)]),
        191.0, 1.0e-6, "long side S_L..F_L equals L");
}

// §7.6 필수 조건: L > d. 위반 입력은 거부되어야 합니다.
void test_containment_constraint_rejects_small_d()
{
    riposte_test::BulletPathInputs inputs{};
    inputs.start = {0.0, 0.0, -10.0};
    inputs.target = {60.0, 0.0, -10.0};
    inputs.width_left_m = 5.0;
    inputs.width_right_m = 5.0;
    // 조건: D > 2(d + W) = 2(60 + 10) = 140. 140 은 경계이므로 거부되어야 합니다.
    inputs.total_distance_m = 140.0;

    std::string reason;
    test_support::check(
        !riposte_test::validate_bullet_path(inputs, reason),
        "D = 2(d + W) must be rejected because L == d, not L > d");
    test_support::check(!reason.empty(), "rejection must explain why");

    // 조금만 키우면 통과해야 합니다.
    inputs.total_distance_m = 141.0;
    test_support::check(
        riposte_test::validate_bullet_path(inputs, reason), "D slightly above the bound is valid");
}

// Test 21/22 의 실제 파라미터가 성립하는 최대 이륙 거리 확인
void test_catalog_bullet_cases_bounds()
{
    // Test 21: D=100, w=2 -> W=4, L=46. 이륙점이 목표에서 46 m 미만이어야 합니다.
    {
        riposte_test::BulletPathInputs inputs{};
        inputs.width_left_m = 2.0;
        inputs.width_right_m = 2.0;
        inputs.total_distance_m = 100.0;
        inputs.start = {0.0, 0.0, -3.0};
        inputs.target = {45.0, 0.0, -3.0};
        std::string reason;
        test_support::check(
            riposte_test::validate_bullet_path(inputs, reason),
            "Test 21 is valid when the takeoff point is 45 m from the target");

        inputs.target = {47.0, 0.0, -3.0};
        test_support::check(
            !riposte_test::validate_bullet_path(inputs, reason),
            "Test 21 is rejected when the takeoff point is 47 m from the target (L = 46 m)");
    }
    // Test 22: D=200, w=5 -> W=10, L=90.
    {
        riposte_test::BulletPathInputs inputs{};
        inputs.width_left_m = 5.0;
        inputs.width_right_m = 5.0;
        inputs.total_distance_m = 200.0;
        inputs.start = {0.0, 0.0, -5.0};
        inputs.target = {89.0, 0.0, -5.0};
        std::string reason;
        test_support::check(
            riposte_test::validate_bullet_path(inputs, reason),
            "Test 22 is valid when the takeoff point is 89 m from the target");

        inputs.target = {91.0, 0.0, -5.0};
        test_support::check(
            !riposte_test::validate_bullet_path(inputs, reason),
            "Test 22 is rejected when the takeoff point is 91 m from the target (L = 90 m)");
    }
}

void test_rejects_degenerate_inputs()
{
    std::string reason;
    {
        // S 와 T 가 같으면 진행 방향이 정의되지 않습니다.
        riposte_test::BulletPathInputs inputs{};
        inputs.start = {5.0, 5.0, -3.0};
        inputs.target = {5.0, 5.0, -3.0};
        inputs.width_left_m = 2.0;
        inputs.width_right_m = 2.0;
        inputs.total_distance_m = 100.0;
        test_support::check(
            !riposte_test::validate_bullet_path(inputs, reason),
            "coincident start and target are rejected");
    }
    {
        // 폭이 0 이면 직사각형이 선분으로 무너집니다.
        riposte_test::BulletPathInputs inputs{};
        inputs.start = {0.0, 0.0, -3.0};
        inputs.target = {30.0, 0.0, -3.0};
        inputs.width_left_m = 0.0;
        inputs.width_right_m = 0.0;
        inputs.total_distance_m = 100.0;
        test_support::check(
            !riposte_test::validate_bullet_path(inputs, reason), "zero total width is rejected");
    }
    {
        // 음수 폭
        riposte_test::BulletPathInputs inputs{};
        inputs.start = {0.0, 0.0, -3.0};
        inputs.target = {30.0, 0.0, -3.0};
        inputs.width_left_m = -1.0;
        inputs.width_right_m = 5.0;
        inputs.total_distance_m = 100.0;
        test_support::check(
            !riposte_test::validate_bullet_path(inputs, reason), "negative width is rejected");
    }
    {
        // NaN 입력
        riposte_test::BulletPathInputs inputs{};
        inputs.start = {0.0, 0.0, -3.0};
        inputs.target = {30.0, 0.0, -3.0};
        inputs.width_left_m = 2.0;
        inputs.width_right_m = 2.0;
        inputs.total_distance_m = std::nan("");
        test_support::check(
            !riposte_test::validate_bullet_path(inputs, reason), "NaN D is rejected");
    }
}

void test_corner_classification()
{
    test_support::check(
        riposte_test::is_bullet_corner(BulletWaypoint::StartLeft), "S_L is a corner");
    test_support::check(
        riposte_test::is_bullet_corner(BulletWaypoint::FarLeft), "F_L is a corner");
    test_support::check(
        riposte_test::is_bullet_corner(BulletWaypoint::FarRight), "F_R is a corner");
    test_support::check(
        riposte_test::is_bullet_corner(BulletWaypoint::StartRight), "S_R is a corner");
    test_support::check(
        !riposte_test::is_bullet_corner(BulletWaypoint::TargetLeft), "T_L is a pass-through");
    test_support::check(
        !riposte_test::is_bullet_corner(BulletWaypoint::TargetRight), "T_R is a pass-through");
}

} // namespace

int main()
{
    test_poc_reference_values();
    test_poc_reference_waypoints();
    test_left_normal_direction_in_ned();
    test_left_normal_for_eastbound_leg();
    test_diagonal_and_asymmetric_width();
    test_containment_constraint_rejects_small_d();
    test_catalog_bullet_cases_bounds();
    test_rejects_degenerate_inputs();
    test_corner_classification();
    return test_support::report("bullet_path_tests");
}
