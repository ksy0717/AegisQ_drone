#pragma once

#include <cmath>

/*
 * 궤적 생성 전용 함수
 * -----------------------------------------------------------------------------
 * 이 파일은 MAVSDK나 하드웨어에 의존하지 않는 순수 수학 코드입니다.
 * 따라서 원 궤적이 이상하면 먼저 tests/test_trajectory.cpp로 이 함수들을 검증하고,
 * 수학 출력이 정상이면 그 다음 PX4 위치제어/EKF/기체 튜닝을 확인하십시오.
 *
 * 좌표계는 PX4 Local NED입니다.
 *   North + : 북쪽
 *   East  + : 동쪽
 *   Down  + : 아래쪽 (상승할수록 값이 작아짐)
 *   Yaw 0° : 북쪽, +90° : 동쪽
 */

namespace bullet_mission {

constexpr double kPi = 3.14159265358979323846;

// Local NED 위치입니다. double은 경로 누적 계산 오차를 줄이기 위해 사용합니다.
struct NedPoint {
    double north_m{};
    double east_m{};
    double down_m{};
};

// Local NED 속도입니다. Down 속도가 음수이면 상승, 양수이면 하강입니다.
struct NedVelocity {
    double north_m_s{};
    double east_m_s{};
    double down_m_s{};
};

// 한 시점에 PX4로 보낼 위치, feed-forward 속도, yaw를 묶은 내부 표현입니다.
struct TrajectorySetpoint {
    NedPoint position{};
    NedVelocity velocity{};
    double yaw_deg{};
};

inline bool finite_setpoint(const TrajectorySetpoint& setpoint) noexcept
{
    // NaN은 모든 크기 비교를 false로 만들기 때문에 단순 반경 검사만으로는 차단되지
    // 않습니다. MAVSDK float 변환 전에 위치·속도·yaw 전 성분의 유한성을 확인합니다.
    return std::isfinite(setpoint.position.north_m)
        && std::isfinite(setpoint.position.east_m)
        && std::isfinite(setpoint.position.down_m)
        && std::isfinite(setpoint.velocity.north_m_s)
        && std::isfinite(setpoint.velocity.east_m_s)
        && std::isfinite(setpoint.velocity.down_m_s)
        && std::isfinite(setpoint.yaw_deg);
}

inline double clamp01(const double value)
{
    // 시간 지연이나 부동소수점 오차로 progress가 0 미만/1 초과가 되어도
    // 직선 목표가 시작점 또는 끝점을 넘어가지 않도록 제한합니다.
    return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
}

// 두 수평점 사이의 진행방향을 NED yaw 각도로 계산합니다.
// 직선 종료(progress=1)에서도 이 값을 유지해야 다음 정지 setpoint에서 기수가
// 갑자기 북쪽(0도)으로 돌아가지 않습니다. 길이가 0이면 방향이 정의되지 않으므로
// 안전하고 예측 가능한 기본값 0도를 반환합니다.
inline double line_heading_deg(const NedPoint& start, const NedPoint& finish)
{
    const double dn = finish.north_m - start.north_m;
    const double de = finish.east_m - start.east_m;
    if (std::hypot(dn, de) <= 1.0e-6) {
        return 0.0;
    }
    return std::atan2(de, dn) * 180.0 / kPi;
}

/*
 * 두 점 사이 직선 setpoint
 *
 * 위치: p = start + progress * (finish - start)
 * 속도: 수평 진행방향 단위벡터 * speed_m_s
 * Yaw : atan2(East 방향량, North 방향량)
 *
 * progress가 1이면 velocity는 0으로 두어 끝점에서 정지합니다.
 * horizontal_length가 0에 가까울 때 0으로 나누지 않도록 1e-6을 검사합니다.
 */
inline TrajectorySetpoint line_setpoint(
    const NedPoint& start,
    const NedPoint& finish,
    const double progress,
    const double speed_m_s)
{
    const double p = clamp01(progress);
    const double dn = finish.north_m - start.north_m;
    const double de = finish.east_m - start.east_m;
    const double dd = finish.down_m - start.down_m;
    // 고도 변화 dd는 위치 보간에는 사용하지만, 이번 미션의 이동시간과 수평속도는
    // 평면 거리로 정의하므로 방향 정규화에는 dn/de만 사용합니다.
    const double horizontal_length = std::hypot(dn, de);

    TrajectorySetpoint setpoint{};
    setpoint.position = {
        start.north_m + p * dn,
        start.east_m + p * de,
        start.down_m + p * dd,
    };

    // 진행이 끝나 velocity를 0으로 만든 뒤에도 직선의 진행방향 yaw는 유지합니다.
    // 기존 구현은 p==1에서 yaw가 value-initialization 값 0으로 돌아가 동/남쪽 이동
    // 종료 시 기수가 순간적으로 북쪽을 향할 수 있었습니다.
    setpoint.yaw_deg = line_heading_deg(start, finish);
    if (horizontal_length > 1.0e-6 && p < 1.0) {
        setpoint.velocity.north_m_s = speed_m_s * dn / horizontal_length;
        setpoint.velocity.east_m_s = speed_m_s * de / horizontal_length;
    }
    return setpoint;
}

/*
 * 원형 setpoint
 *
 * 원 출발점의 동쪽에 중심을 둡니다.
 * theta=0에서 출발점에 있고 북쪽 방향으로 움직이며, theta=2*pi에서 복귀합니다.
 *
 *   center = (start.N, start.E + R)
 *   N(theta) = center.N + R*sin(theta)
 *   E(theta) = center.E - R*cos(theta)
 *   Vn       = v*cos(theta)
 *   Ve       = v*sin(theta)
 *
 * 디버깅 기준:
 *   theta=0    -> 위치=start,           속도=(+v, 0)
 *   theta=pi   -> 위치=start.E+2R 지점, 속도=(-v, 0)
 *   theta=2pi  -> 위치=start,           속도=(+v, 0)
 */
inline TrajectorySetpoint circle_setpoint(
    const NedPoint& start,
    const double radius_m,
    const double speed_m_s,
    const double theta_rad)
{
    // 모든 지름의 원은 같은 start를 사용하지만 반지름에 따라 중심은 달라집니다.
    const double center_north = start.north_m;
    const double center_east = start.east_m + radius_m;

    TrajectorySetpoint setpoint{};
    setpoint.position.north_m = center_north + radius_m * std::sin(theta_rad);
    setpoint.position.east_m = center_east - radius_m * std::cos(theta_rad);
    setpoint.position.down_m = start.down_m;

    setpoint.velocity.north_m_s = speed_m_s * std::cos(theta_rad);
    setpoint.velocity.east_m_s = speed_m_s * std::sin(theta_rad);
    setpoint.velocity.down_m_s = 0.0;

    // NED yaw: 북쪽 0도, 동쪽 90도. 기수가 원의 접선 방향을 따라갑니다.
    // atan2의 인자 순서가 (East, North)임에 주의하십시오. 일반 XY 수식의
    // atan2(y, x)를 그대로 복사하면 yaw가 90도 회전하는 오류가 생깁니다.
    setpoint.yaw_deg = std::atan2(
        setpoint.velocity.east_m_s, setpoint.velocity.north_m_s) * 180.0 / kPi;
    return setpoint;
}

inline double horizontal_distance(const NedPoint& a, const NedPoint& b)
{
    // geofence 성격의 software guard와 도착 판정에는 수평거리만 사용합니다.
    // 고도 오차는 settle_at()에서 별도 검사합니다.
    return std::hypot(a.north_m - b.north_m, a.east_m - b.east_m);
}

/*
 * 기준점에서 원 궤적까지의 정확한 최대 수평거리
 *
 * 원 중심 C는 start에서 동쪽으로 R만큼 떨어져 있습니다. 기준점 O에서 원 위의
 * 가장 먼 점까지 거리는 |OC| + R이므로 다음과 같습니다.
 *
 *   center = (start.N, start.E + R)
 *   max_distance = hypot(center.N-origin.N, center.E-origin.E) + R
 *
 * 단순히 hypot(start.N-origin.N, 2R)로 계산하면 특정방향의 한 점만 검사하게 되어
 * 실제 최원점을 과소평가합니다. 이 함수는 하드웨어와 무관하므로 단위시험에서
 * software radius 사전검증과 런타임 setpoint guard의 일치를 확인할 수 있습니다.
 */
inline double circle_max_horizontal_distance(
    const NedPoint& origin,
    const NedPoint& start,
    const double radius_m)
{
    const double center_north = start.north_m;
    const double center_east = start.east_m + radius_m;
    return std::hypot(
        center_north - origin.north_m,
        center_east - origin.east_m) + radius_m;
}

} // namespace bullet_mission
