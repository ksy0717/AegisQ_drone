#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

#include "trajectory.hpp"

/*
 * Bullet Drone 정찰 경로 산출 (POC 시나리오 V9 §7.2)
 * ------------------------------------------------------------------------
 * 경로를 좌표로 하드코딩하지 않고 4개 입력에서 산출합니다.
 *
 *   입력 : S(출발 지점), T(공격 목표 지점), wL/wR(좌/우 폭), D(전체 비행거리)
 *   출력 : 반드시 거치는 6점을 포함하는 직사각형 경로
 *
 *   d = |T - S|
 *   u = (T - S) / d                 (진행 방향 단위벡터)
 *   n = 좌측 법선
 *   W = wL + wR                     (단변)
 *   L = D/2 - W                     (장변; 둘레 D = 2L + 2W 로부터)
 *   F = S + L*u                     (원단점, 산출값)
 *   P_L = P + wL*n , P_R = P - wR*n  for P in {S, T, F}
 *
 *   경로 : SL -> TL -> FL -> FR -> TR -> SR -> (SL 복귀)
 *
 * ── 좌표계 주의 (시나리오 문서와의 차이) ──────────────────────────────
 * 문서는 x=주행방향, y=폭 인 일반 수학 XY 평면을 쓰며 좌측 법선을
 * n = (-u_y, u_x) 로 적었습니다. 그러나 본 코드의 좌표계는 Local NED 이고,
 * 위에서 내려다보면 North 가 위, East 가 오른쪽이라 회전 부호가 반대입니다.
 * NED 에서 (-u_east, u_north) 는 우측 법선이 됩니다. 따라서 좌측 법선은
 *
 *   n_left = (u_east, -u_north)
 *
 * 입니다. 검산: 북진(u=(1,0)) 시 좌측은 서쪽(0,-1) 이고 위 식이 (0,-1) 을 줍니다.
 * wL == wR 인 POC 적용값에서는 직사각형 모양이 같아 이 부호를 틀려도 표시가
 * 안 나지만, 순회 방향이 뒤집히고 wL != wR 이면 경로 자체가 틀어집니다.
 *
 * 이 파일은 MAVSDK/하드웨어에 의존하지 않는 순수 수학이므로
 * tests/test_bullet_path.cpp 로 검증합니다.
 */

namespace riposte_test {

// 필수 통과 6점의 경로상 순번입니다. 배열 인덱스와 일치합니다.
enum class BulletWaypoint : std::size_t {
    StartLeft = 0,  // S_L  코너 (서행)
    TargetLeft = 1, // T_L  장변 위 통과점 (정지 없이 통과)
    FarLeft = 2,    // F_L  코너 (서행)
    FarRight = 3,   // F_R  코너 (서행)
    TargetRight = 4,// T_R  장변 위 통과점
    StartRight = 5, // S_R  코너 (서행)
};

inline constexpr std::size_t kBulletWaypointCount = 6;

struct BulletPathInputs {
    bullet_mission::NedPoint start{};  // S
    bullet_mission::NedPoint target{}; // T
    double width_left_m{};             // wL
    double width_right_m{};            // wR
    double total_distance_m{};         // D
};

struct BulletPathResult {
    double target_distance_m{}; // d = |T - S|
    double width_m{};           // W = wL + wR
    double long_side_m{};       // L = D/2 - W
    double perimeter_m{};       // 2L + 2W, D 와 일치해야 함
    double overshoot_m{};       // L - d, 목표 너머 필요 공역
    double heading_deg{};       // S -> T 방위
    bullet_mission::NedPoint far_point{}; // F
    std::array<bullet_mission::NedPoint, kBulletWaypointCount> waypoints{};
};

// 경로 산출 전 §7.6 유효성 제약을 검사합니다.
// 실패하면 false 를 반환하고 reason 에 사유를 채웁니다.
inline bool validate_bullet_path(const BulletPathInputs& inputs, std::string& reason)
{
    if (!std::isfinite(inputs.total_distance_m) || !std::isfinite(inputs.width_left_m)
        || !std::isfinite(inputs.width_right_m)) {
        reason = "bullet path inputs contain NaN or infinity";
        return false;
    }
    if (inputs.width_left_m < 0.0 || inputs.width_right_m < 0.0) {
        reason = "width must not be negative";
        return false;
    }
    const double width_m = inputs.width_left_m + inputs.width_right_m;
    if (width_m <= 0.0) {
        reason = "total width (wL + wR) must be positive";
        return false;
    }
    const double target_distance_m =
        bullet_mission::horizontal_distance(inputs.start, inputs.target);
    if (target_distance_m <= 1.0e-3) {
        // d = 0 이면 진행 방향 단위벡터가 정의되지 않습니다. 이륙 지점이
        // 공격 목표 지점과 사실상 같은 경우이므로 시험 배치가 잘못된 것입니다.
        reason = "start and target coincide, heading is undefined";
        return false;
    }
    const double long_side_m = inputs.total_distance_m / 2.0 - width_m;
    if (long_side_m <= target_distance_m) {
        // §7.6 필수 조건: L > d  <=>  D > 2(d + W).
        // 위반하면 원단점 F 가 목표 앞에 생겨 T_L/T_R 이 장변을 벗어납니다.
        reason = "D must exceed 2*(d + W) so that the target lies inside the path"
                 " (L=" + std::to_string(long_side_m)
            + " m must be greater than d=" + std::to_string(target_distance_m) + " m)";
        return false;
    }
    return true;
}

// §7.2 의 산출식을 그대로 옮깁니다. 호출 전에 validate_bullet_path() 로
// 유효성을 확인하십시오. 유효하지 않은 입력에 대한 출력은 의미가 없습니다.
inline BulletPathResult compute_bullet_path(const BulletPathInputs& inputs)
{
    BulletPathResult result{};

    // ① 방향 벡터
    const double delta_north = inputs.target.north_m - inputs.start.north_m;
    const double delta_east = inputs.target.east_m - inputs.start.east_m;

    // ② 목표거리 d
    result.target_distance_m = std::hypot(delta_north, delta_east);

    const double unit_north = delta_north / result.target_distance_m;
    const double unit_east = delta_east / result.target_distance_m;

    // 좌측 법선 (NED 기준). 위쪽 주석의 부호 설명을 참고하십시오.
    const double normal_north = unit_east;
    const double normal_east = -unit_north;

    // ③ 단변 W, ④ 장변 L
    result.width_m = inputs.width_left_m + inputs.width_right_m;
    result.long_side_m = inputs.total_distance_m / 2.0 - result.width_m;
    result.perimeter_m = 2.0 * result.long_side_m + 2.0 * result.width_m;
    result.overshoot_m = result.long_side_m - result.target_distance_m;
    result.heading_deg = bullet_mission::line_heading_deg(inputs.start, inputs.target);

    // ⑤ 원단점 F = S + L*u. 고도는 경로 전체가 동일합니다.
    result.far_point = {
        inputs.start.north_m + result.long_side_m * unit_north,
        inputs.start.east_m + result.long_side_m * unit_east,
        inputs.start.down_m,
    };

    // ⑥ 기준점 3개(S, T, F)를 좌/우로 이격해 필수 통과 6점을 만듭니다.
    const auto offset_point = [&](const bullet_mission::NedPoint& base, const double signed_width_m) {
        return bullet_mission::NedPoint{
            base.north_m + signed_width_m * normal_north,
            base.east_m + signed_width_m * normal_east,
            inputs.start.down_m,
        };
    };

    // T 는 입력 고도가 다를 수 있으므로 중심선 좌표만 취하고 고도는 S 를 따릅니다.
    const bullet_mission::NedPoint target_on_path{
        inputs.target.north_m, inputs.target.east_m, inputs.start.down_m};

    // ⑦ 출력 경로 순서: SL -> TL -> FL -> FR -> TR -> SR
    result.waypoints[static_cast<std::size_t>(BulletWaypoint::StartLeft)] =
        offset_point(inputs.start, inputs.width_left_m);
    result.waypoints[static_cast<std::size_t>(BulletWaypoint::TargetLeft)] =
        offset_point(target_on_path, inputs.width_left_m);
    result.waypoints[static_cast<std::size_t>(BulletWaypoint::FarLeft)] =
        offset_point(result.far_point, inputs.width_left_m);
    result.waypoints[static_cast<std::size_t>(BulletWaypoint::FarRight)] =
        offset_point(result.far_point, -inputs.width_right_m);
    result.waypoints[static_cast<std::size_t>(BulletWaypoint::TargetRight)] =
        offset_point(target_on_path, -inputs.width_right_m);
    result.waypoints[static_cast<std::size_t>(BulletWaypoint::StartRight)] =
        offset_point(inputs.start, -inputs.width_right_m);

    return result;
}

// 코너 여부입니다. SL/FL/FR/SR 는 회전 코너라 서행하고, TL/TR 은 장변 위의
// 통과점이라 정지하지 않고 지나갑니다 (§7.7).
inline bool is_bullet_corner(const BulletWaypoint waypoint) noexcept
{
    return waypoint != BulletWaypoint::TargetLeft && waypoint != BulletWaypoint::TargetRight;
}

inline const char* bullet_waypoint_name(const BulletWaypoint waypoint) noexcept
{
    switch (waypoint) {
    case BulletWaypoint::StartLeft: return "S_L";
    case BulletWaypoint::TargetLeft: return "T_L";
    case BulletWaypoint::FarLeft: return "F_L";
    case BulletWaypoint::FarRight: return "F_R";
    case BulletWaypoint::TargetRight: return "T_R";
    case BulletWaypoint::StartRight: return "S_R";
    }
    return "UNKNOWN";
}

// 기준점에서 경로상 가장 먼 점까지의 거리입니다. 소프트웨어 지오펜스 한계를
// 정할 때 사용합니다.
inline double bullet_max_radius_m(
    const bullet_mission::NedPoint& origin, const BulletPathResult& path)
{
    double farthest_m = 0.0;
    for (const auto& waypoint : path.waypoints) {
        farthest_m = std::max(farthest_m, bullet_mission::horizontal_distance(origin, waypoint));
    }
    return farthest_m;
}

} // namespace riposte_test
