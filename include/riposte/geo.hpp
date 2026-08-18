#pragma once

#include <cmath>

#include "trajectory.hpp"

/*
 * 측지좌표 → Local NED 변환
 * ------------------------------------------------------------------------
 * Test 21/22 는 공격 목표 지점을 위/경도로 받습니다. 반면 Offboard setpoint 는
 * Local NED 이므로 둘을 이어 줄 변환이 필요합니다.
 *
 * 기존 코드에는 이 변환이 없었습니다(모든 목표가 이륙점 기준 상대 좌표였음).
 * 그래서 이 파일을 새로 추가했고, MAVSDK/하드웨어 의존이 없는 순수 함수로
 * 유지하여 tests/test_geo.cpp 로 단위시험합니다.
 *
 * 방식: WGS-84 타원체의 자오선 곡률반경 R_M, 묘유선 곡률반경 R_N 을 원점 위도에서
 * 구해 선형화합니다. 수 km 이내에서는 오차가 cm 수준이므로 본 시험 범위
 * (최대 200 m)에는 충분합니다. 수십 km 이상이면 정식 ECEF 변환을 쓰십시오.
 *
 * 주의: PX4 의 Local NED 원점은 EKF origin 이며 이륙점과 정확히 같지 않습니다.
 * 그래서 절대 NED 를 만들지 않고, "이륙 시점의 기체 위/경도"를 원점으로 한
 * 상대 오프셋만 계산한 뒤, 그 시점에 읽은 Local NED 값에 더합니다. 이렇게 하면
 * EKF origin 이 어디에 있든 자기일관성이 유지됩니다.
 */

namespace riposte_test {

struct GeoPoint {
    double latitude_deg{};
    double longitude_deg{};
};

// 원점 기준 수평 오프셋입니다. Down 은 이 변환의 관심사가 아닙니다.
struct NedOffset {
    double north_m{};
    double east_m{};
};

namespace geo_detail {

// WGS-84 타원체 상수
constexpr double kSemiMajorAxisM = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;
constexpr double kEccentricitySquared = kFlattening * (2.0 - kFlattening);

constexpr double deg_to_rad(const double degrees)
{
    return degrees * bullet_mission::kPi / 180.0;
}

} // namespace geo_detail

// 원점 위도에서의 자오선 곡률반경 R_M [m] 입니다.
inline double meridian_radius_m(const double latitude_deg)
{
    const double sin_lat = std::sin(geo_detail::deg_to_rad(latitude_deg));
    const double denominator = 1.0 - geo_detail::kEccentricitySquared * sin_lat * sin_lat;
    return geo_detail::kSemiMajorAxisM * (1.0 - geo_detail::kEccentricitySquared)
        / (denominator * std::sqrt(denominator));
}

// 원점 위도에서의 묘유선 곡률반경 R_N [m] 입니다.
inline double normal_radius_m(const double latitude_deg)
{
    const double sin_lat = std::sin(geo_detail::deg_to_rad(latitude_deg));
    const double denominator = 1.0 - geo_detail::kEccentricitySquared * sin_lat * sin_lat;
    return geo_detail::kSemiMajorAxisM / std::sqrt(denominator);
}

/*
 * origin 을 기준으로 한 point 의 North/East 오프셋입니다.
 *
 *   north = (lat - lat0) * R_M
 *   east  = (lon - lon0) * R_N * cos(lat0)
 *
 * 경도 차이는 ±180° 를 넘지 않도록 정규화합니다. 본 시험지(태안)에서는 발생하지
 * 않지만, 날짜변경선 근처에서 좌표를 잘못 넣었을 때 수천 km 짜리 setpoint 가
 * 생성되는 것을 막아 줍니다.
 */
inline NedOffset geodetic_to_ned_offset(const GeoPoint& origin, const GeoPoint& point)
{
    double delta_longitude_deg = point.longitude_deg - origin.longitude_deg;
    if (delta_longitude_deg > 180.0) {
        delta_longitude_deg -= 360.0;
    } else if (delta_longitude_deg < -180.0) {
        delta_longitude_deg += 360.0;
    }
    const double delta_latitude_deg = point.latitude_deg - origin.latitude_deg;
    const double cos_lat = std::cos(geo_detail::deg_to_rad(origin.latitude_deg));

    NedOffset offset{};
    offset.north_m = geo_detail::deg_to_rad(delta_latitude_deg) * meridian_radius_m(origin.latitude_deg);
    offset.east_m = geo_detail::deg_to_rad(delta_longitude_deg) * normal_radius_m(origin.latitude_deg) * cos_lat;
    return offset;
}

// 두 측지좌표 사이의 수평 거리 [m] 입니다.
inline double geodetic_distance_m(const GeoPoint& a, const GeoPoint& b)
{
    const auto offset = geodetic_to_ned_offset(a, b);
    return std::hypot(offset.north_m, offset.east_m);
}

// 위/경도가 유효 범위이고 NaN 이 아닌지 확인합니다.
// MAVSDK 는 GNSS fix 이전에 0 이나 NaN 을 반환할 수 있습니다.
inline bool geo_point_valid(const GeoPoint& point) noexcept
{
    return std::isfinite(point.latitude_deg)
        && std::isfinite(point.longitude_deg)
        && std::abs(point.latitude_deg) <= 90.0
        && std::abs(point.longitude_deg) <= 180.0;
}

} // namespace riposte_test
