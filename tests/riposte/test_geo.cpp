#include "geo.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

/*
 * 측지 -> Local NED 변환 단위시험
 * ------------------------------------------------------------------------
 * Test 21/22 는 목표를 위/경도로 받으므로 이 변환이 틀리면 드론이 엉뚱한 곳으로
 * 갑니다. 하드웨어 없이 여기서 반드시 걸러야 합니다.
 *
 * 기준점은 POC 공격 목표 지점(태안) 인근입니다.
 */

namespace {

// POC 공격 목표 지점
constexpr double kTargetLat = 36.6610082;
constexpr double kTargetLon = 126.3417854;

void test_same_point_is_zero()
{
    const riposte_test::GeoPoint point{kTargetLat, kTargetLon};
    const auto offset = riposte_test::geodetic_to_ned_offset(point, point);
    test_support::check_close(offset.north_m, 0.0, 1.0e-9, "same point has zero north offset");
    test_support::check_close(offset.east_m, 0.0, 1.0e-9, "same point has zero east offset");
    test_support::check_close(
        riposte_test::geodetic_distance_m(point, point), 0.0, 1.0e-9, "same point distance is 0");
}

// 위도를 올리면 북쪽(+north), 경도를 올리면 동쪽(+east) 이어야 합니다.
void test_sign_conventions()
{
    const riposte_test::GeoPoint origin{kTargetLat, kTargetLon};
    {
        const riposte_test::GeoPoint north_of{kTargetLat + 0.001, kTargetLon};
        const auto offset = riposte_test::geodetic_to_ned_offset(origin, north_of);
        test_support::check(offset.north_m > 0.0, "higher latitude is north");
        test_support::check_close(offset.east_m, 0.0, 1.0e-9, "pure latitude change has no east");
    }
    {
        const riposte_test::GeoPoint east_of{kTargetLat, kTargetLon + 0.001};
        const auto offset = riposte_test::geodetic_to_ned_offset(origin, east_of);
        test_support::check(offset.east_m > 0.0, "higher longitude is east");
        test_support::check_close(offset.north_m, 0.0, 1.0e-9, "pure longitude change has no north");
    }
}

/*
 * 위도 1도는 자오선 곡률반경 기준 약 111 km 입니다. 위도 36.66도에서는
 * R_M ~= 6,366 km 이므로 1도 ~= 111.1 km, 0.001도 ~= 111 m 입니다.
 * 0.5% 여유로 검사합니다(타원체 근사와 상수 자릿수 차이 흡수).
 */
void test_latitude_scale()
{
    const riposte_test::GeoPoint origin{kTargetLat, kTargetLon};
    const riposte_test::GeoPoint north_of{kTargetLat + 0.001, kTargetLon};
    const auto offset = riposte_test::geodetic_to_ned_offset(origin, north_of);
    test_support::check_close(offset.north_m, 111.0, 0.6, "0.001 deg of latitude is about 111 m");
}

/*
 * 경도 1도는 위도에 따라 cos(lat) 로 줄어듭니다.
 * 위도 36.66도: 111.32 km * cos(36.66) ~= 89.3 km. 0.001도 ~= 89.3 m.
 */
void test_longitude_scale_shrinks_with_latitude()
{
    const riposte_test::GeoPoint origin{kTargetLat, kTargetLon};
    const riposte_test::GeoPoint east_of{kTargetLat, kTargetLon + 0.001};
    const auto offset = riposte_test::geodetic_to_ned_offset(origin, east_of);
    test_support::check_close(offset.east_m, 89.3, 0.6, "0.001 deg of longitude is about 89 m");

    // 적도에서는 같은 경도 차이가 더 긴 거리여야 합니다.
    const riposte_test::GeoPoint equator{0.0, kTargetLon};
    const riposte_test::GeoPoint equator_east{0.0, kTargetLon + 0.001};
    const auto equator_offset = riposte_test::geodetic_to_ned_offset(equator, equator_east);
    test_support::check(
        equator_offset.east_m > offset.east_m,
        "the same longitude delta spans more ground at the equator");
}

// 왕복 변환: A->B 오프셋과 B->A 오프셋은 부호만 달라야 합니다(단거리 근사 범위).
void test_reciprocity()
{
    const riposte_test::GeoPoint a{kTargetLat, kTargetLon};
    const riposte_test::GeoPoint b{kTargetLat + 0.0005, kTargetLon + 0.0007};
    const auto forward = riposte_test::geodetic_to_ned_offset(a, b);
    const auto backward = riposte_test::geodetic_to_ned_offset(b, a);
    // 원점 위도가 달라 곡률반경이 미세하게 다르므로 mm 수준의 차이는 허용합니다.
    test_support::check_close(
        forward.north_m, -backward.north_m, 0.01, "north offset is reciprocal");
    test_support::check_close(forward.east_m, -backward.east_m, 0.01, "east offset is reciprocal");
    test_support::check_close(
        riposte_test::geodetic_distance_m(a, b), riposte_test::geodetic_distance_m(b, a), 0.01,
        "distance is symmetric");
}

// 피타고라스 성립 확인: 북 3, 동 4 이면 거리 5.
void test_distance_is_euclidean_in_the_tangent_plane()
{
    const riposte_test::GeoPoint origin{kTargetLat, kTargetLon};
    // 북쪽으로 약 30 m, 동쪽으로 약 40 m 인 점을 역산해서 만듭니다.
    const double meridian_m = riposte_test::meridian_radius_m(kTargetLat);
    const double normal_m = riposte_test::normal_radius_m(kTargetLat);
    const double cos_lat = std::cos(kTargetLat * bullet_mission::kPi / 180.0);
    const double delta_lat_deg = 30.0 / meridian_m * 180.0 / bullet_mission::kPi;
    const double delta_lon_deg = 40.0 / (normal_m * cos_lat) * 180.0 / bullet_mission::kPi;

    const riposte_test::GeoPoint point{kTargetLat + delta_lat_deg, kTargetLon + delta_lon_deg};
    const auto offset = riposte_test::geodetic_to_ned_offset(origin, point);
    test_support::check_close(offset.north_m, 30.0, 0.01, "reconstructed north offset is 30 m");
    test_support::check_close(offset.east_m, 40.0, 0.01, "reconstructed east offset is 40 m");
    test_support::check_close(
        riposte_test::geodetic_distance_m(origin, point), 50.0, 0.02,
        "3-4-5 triangle gives a 50 m distance");
}

// 날짜변경선 근처에서 경도차가 ±180 을 넘으면 정규화되어야 합니다.
void test_longitude_wrap()
{
    const riposte_test::GeoPoint west{0.0, 179.9995};
    const riposte_test::GeoPoint east{0.0, -179.9995};
    const double distance_m = riposte_test::geodetic_distance_m(west, east);
    // 실제로는 약 111 m 떨어진 이웃입니다. 정규화가 없으면 약 40,000 km 가 나옵니다.
    test_support::check(
        distance_m < 200.0, "antimeridian neighbours are close, not a lap around the planet");
}

void test_validity_checks()
{
    test_support::check(
        riposte_test::geo_point_valid({kTargetLat, kTargetLon}), "the POC target is valid");
    test_support::check(!riposte_test::geo_point_valid({91.0, 0.0}), "latitude above 90 is invalid");
    test_support::check(
        !riposte_test::geo_point_valid({0.0, 181.0}), "longitude above 180 is invalid");
    test_support::check(
        !riposte_test::geo_point_valid({std::nan(""), 0.0}), "NaN latitude is invalid");
    test_support::check(
        !riposte_test::geo_point_valid({0.0, std::numeric_limits<double>::infinity()}),
        "infinite longitude is invalid");
}

} // namespace

int main()
{
    test_same_point_is_zero();
    test_sign_conventions();
    test_latitude_scale();
    test_longitude_scale_shrinks_with_latitude();
    test_reciprocity();
    test_distance_is_euclidean_in_the_tangent_plane();
    test_longitude_wrap();
    test_validity_checks();
    return test_support::report("geo_tests");
}
