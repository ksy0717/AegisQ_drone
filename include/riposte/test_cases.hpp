#pragma once

#include <array>
#include <cstddef>

/*
 * X500 + Pixhawk 시험 케이스 카탈로그
 * ------------------------------------------------------------------------
 * 이 헤더는 "무엇을 시험할 것인가"만 담습니다. "어떻게 비행할 것인가"는
 * src/test_main.cpp 의 TestRunner 가, "얼마나 정밀해야 하는가"는
 * include/test_config.hpp 의 TestTuning 이 담당합니다.
 *
 * 시험 번호는 시험 계획서와 1:1 로 일치시킵니다. 번호를 바꾸면 증빙 로그의
 * 추적성이 깨지므로, 케이스를 추가할 때는 뒤에만 덧붙이십시오.
 *
 *   Test  1~ 3 : 5 m 호버 5초, 1/3/10 회
 *   Test  4~ 6 : 6 m 호버 5초, 1/3/10 회
 *   Test  7~10 : 7 m 호버 5초, 1/3/5/10 회
 *   Test 11~13 : 5 m + North 1 m / South 1 m 저속 왕복, 1/3/10 회
 *   Test 14~16 : 6 m + North 1 m / South 1 m 저속 왕복, 1/3/10 회
 *   Test 17~20 : 7 m + North 1 m / South 1 m 저속 왕복, 1/3/5/10 회
 *   Test 21    : POC Bullet Drone 경로, 이륙 5 m, D=100 m, 좌우 폭 2 m
 *   Test 22    : POC Bullet Drone 경로, 이륙 6 m, D=200 m, 좌우 폭 5 m
 *   Test 23    : POC Bullet Drone 경로, 이륙 7 m, D=200 m, 좌우 폭 5 m
 *
 * 단위는 변수명에 표시했습니다: _m, _m_s, _m_s2, _s, _deg.
 */

namespace riposte_test {

enum class TestKind {
    HoverCycle, // 이륙 → (선택) 남북 왕복 → 호버 → 착륙 을 N회 반복
    BulletPath, // POC 시나리오 §7 파라미터 기반 직사각형 정찰 경로
};

// Test 1~20 의 1회 사이클을 규정합니다.
struct HoverCycleSpec {
    // 이륙 목표 고도입니다. 시험 계획서의 cm 값을 m 로 환산해 둡니다.
    float altitude_m{};

    // true 이면 호버 전에 North 1 m → South 1 m 저속 왕복을 수행합니다.
    bool north_south_leg{};

    // 왕복 구간의 편도 거리입니다.
    float leg_distance_m{};

    // "저속 이동"의 정의입니다. 1 m 구간에서 가감속 여유를 남기려면
    // 0.5 m/s 이하가 현실적입니다. 0.3 m/s 이면 편도 약 3.3 초입니다.
    float leg_speed_m_s{};

    // 착륙 직전 정지 호버 유지 시간입니다.
    float hover_s{};

    // 이륙~착륙 사이클 반복 횟수입니다.
    int repeat{};
};

// Test 21~23 의 Bullet Drone 경로를 규정합니다.
// 경로는 좌표 하드코딩이 아니라 POC 시나리오 §7.2 의 4개 입력에서 산출합니다.
struct BulletPathSpec {
    // 순환 비행 고도(AGL)입니다.
    float altitude_m{};

    // 공격 목표 지점 T 입니다. 이륙 지점 S 는 실제 이륙 위치가 되므로
    // 여기에 적지 않습니다. d = |T - S| 는 실행 시각에 계산합니다.
    double target_latitude_deg{};
    double target_longitude_deg{};

    // D: 1 순환 경로의 총 길이(직사각형 둘레)입니다. 여기서 L = D/2 - W.
    float total_distance_m{};

    // 중심선 기준 좌/우 이격 거리입니다. wL != wR 인 비대칭 폭도 지원합니다.
    float width_left_m{};
    float width_right_m{};

    // 코너 통과 속도입니다. POC 권고는 약 3 m/s 입니다.
    float corner_speed_m_s{};

    // 장변 순항 속도입니다. POC 권고 상한은 약 8 m/s 이지만, 저고도 시험에서는
    // 낮춰서 시작하고 현장에서 올리는 편이 안전합니다.
    float cruise_speed_m_s{};

    // 장변의 사다리꼴 속도 프로파일에 사용할 가감속도입니다.
    float accel_m_s2{};

    // 순환 횟수 N 입니다. 총 비행거리는 N x D 가 됩니다.
    int laps{};
};

struct TestCase {
    int id{};
    const char* name{};
    TestKind kind{};
    HoverCycleSpec hover{};
    BulletPathSpec bullet{};
};

namespace detail {

// 시험 계획서의 cm 값을 m 로 환산합니다.
constexpr float cm(const float centimetres) { return centimetres / 100.0F; }

constexpr TestCase hover_case(
    const int id,
    const char* name,
    const float altitude_cm,
    const bool north_south_leg,
    const int repeat)
{
    return TestCase{
        id,
        name,
        TestKind::HoverCycle,
        HoverCycleSpec{
            cm(altitude_cm),
            north_south_leg,
            1.0F,  // 편도 1 m
            0.3F,  // 저속 이동
            5.0F,  // 호버 5초
            repeat,
        },
        BulletPathSpec{},
    };
}

constexpr TestCase bullet_case(
    const int id,
    const char* name,
    const float altitude_m,
    const float total_distance_m,
    const float width_m,
    const float corner_speed_m_s,
    const float cruise_speed_m_s,
    const int laps)
{
    return TestCase{
        id,
        name,
        TestKind::BulletPath,
        HoverCycleSpec{},
        BulletPathSpec{
            altitude_m,
            36.6610082,   // POC 공격 목표 지점 위도
            126.3417854,  // POC 공격 목표 지점 경도
            total_distance_m,
            width_m,
            width_m,
            corner_speed_m_s,
            cruise_speed_m_s,
            1.5F,
            laps,
        },
    };
}

} // namespace detail

// 시험 카탈로그입니다. 인덱스가 아니라 id 로 조회하십시오.
inline constexpr std::array<TestCase, 23> kTestCatalog{{
    detail::hover_case(1, "5m hover x1", 500.0F, false, 1),
    detail::hover_case(2, "5m hover x3", 500.0F, false, 3),
    detail::hover_case(3, "5m hover x10", 500.0F, false, 10),
    detail::hover_case(4, "6m hover x1", 600.0F, false, 1),
    detail::hover_case(5, "6m hover x3", 600.0F, false, 3),
    detail::hover_case(6, "6m hover x10", 600.0F, false, 10),
    detail::hover_case(7, "7m hover x1", 700.0F, false, 1),
    detail::hover_case(8, "7m hover x3", 700.0F, false, 3),
    detail::hover_case(9, "7m hover x5", 700.0F, false, 5),
    detail::hover_case(10, "7m hover x10", 700.0F, false, 10),
    detail::hover_case(11, "5m N/S 1m hover x1", 500.0F, true, 1),
    detail::hover_case(12, "5m N/S 1m hover x3", 500.0F, true, 3),
    detail::hover_case(13, "5m N/S 1m hover x10", 500.0F, true, 10),
    detail::hover_case(14, "6m N/S 1m hover x1", 600.0F, true, 1),
    detail::hover_case(15, "6m N/S 1m hover x3", 600.0F, true, 3),
    detail::hover_case(16, "6m N/S 1m hover x10", 600.0F, true, 10),
    detail::hover_case(17, "7m N/S 1m hover x1", 700.0F, true, 1),
    detail::hover_case(18, "7m N/S 1m hover x3", 700.0F, true, 3),
    detail::hover_case(19, "7m N/S 1m hover x5", 700.0F, true, 5),
    detail::hover_case(20, "7m N/S 1m hover x10", 700.0F, true, 10),
    // Test 21: 이륙 5 m, D=100 m, 폭 2 m. L = 100/2 - 4 = 46 m 이므로 이륙 지점은
    // 목표 지점에서 46 m 이내여야 합니다(§7.6 의 L > d 조건). 실행 시 검증합니다.
    detail::bullet_case(21, "bullet path D=100m w=2m alt=5m", 5.0F, 100.0F, 2.0F, 2.0F, 4.0F, 1),
    // Test 22: 이륙 6 m, D=200 m, 폭 5 m. L = 200/2 - 10 = 90 m.
    detail::bullet_case(22, "bullet path D=200m w=5m alt=6m", 6.0F, 200.0F, 5.0F, 3.0F, 6.0F, 1),
    // Test 23: 이륙 7 m, 경로 파라미터는 Test 22 와 동일합니다.
    detail::bullet_case(23, "bullet path D=200m w=5m alt=7m", 7.0F, 200.0F, 5.0F, 3.0F, 6.0F, 1),
}};

// id 로 시험 케이스를 찾습니다. 없으면 nullptr 을 반환합니다.
inline const TestCase* find_test_case(const int id)
{
    for (const auto& test_case : kTestCatalog) {
        if (test_case.id == id) {
            return &test_case;
        }
    }
    return nullptr;
}

} // namespace riposte_test
