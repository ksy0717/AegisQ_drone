#include "test_cases.hpp"
#include "test_config.hpp"
#include "test_support.hpp"
#include "test_verdict.hpp"

#include <cmath>

/*
 * 판정 로직과 카탈로그 단위시험
 * ------------------------------------------------------------------------
 * 판정이 헐거우면 불합격을 합격으로 통과시키고, 카탈로그가 시험 계획서와
 * 어긋나면 엉뚱한 시험을 비행합니다. 둘 다 하드웨어 없이 검사할 수 있습니다.
 */

namespace {

// 표본을 채우는 도우미
riposte_test::HoverAccumulator make_accumulator(
    const std::size_t count, const double altitude_error_m, const double horizontal_error_m)
{
    riposte_test::HoverAccumulator accumulator{};
    for (std::size_t i = 0; i < count; ++i) {
        accumulator.add(altitude_error_m, horizontal_error_m);
    }
    return accumulator;
}

void test_accumulator_statistics()
{
    riposte_test::HoverAccumulator accumulator{};
    accumulator.add(0.1, 0.2);
    accumulator.add(-0.3, 0.1); // 부호가 달라도 절대값으로 누적되어야 합니다.
    accumulator.add(0.2, 0.5);

    test_support::check(accumulator.samples() == 3, "three samples were counted");
    test_support::check_close(
        accumulator.mean_altitude_error_m(), (0.1 + 0.3 + 0.2) / 3.0, 1.0e-9,
        "the mean uses absolute altitude error");
    test_support::check_close(
        accumulator.max_altitude_error_m(), 0.3, 1.0e-9,
        "the peak altitude error takes the sign into account via abs()");
    test_support::check_close(
        accumulator.max_horizontal_error_m(), 0.5, 1.0e-9, "the peak horizontal error is tracked");
}

void test_accumulator_rejects_non_finite_samples()
{
    riposte_test::HoverAccumulator accumulator{};
    accumulator.add(0.1, 0.1);
    accumulator.add(std::nan(""), 0.1);
    accumulator.add(0.1, std::numeric_limits<double>::infinity());

    test_support::check(accumulator.samples() == 1, "only the finite sample is counted");
    test_support::check(accumulator.invalid_samples() == 2, "non-finite samples are tallied");
    test_support::check(
        std::isfinite(accumulator.mean_altitude_error_m()),
        "a NaN sample must not poison the mean");
}

void test_empty_accumulator_is_safe()
{
    const riposte_test::HoverAccumulator accumulator{};
    test_support::check_close(
        accumulator.mean_altitude_error_m(), 0.0, 1.0e-9,
        "an empty accumulator does not divide by zero");
}

// 고도별 허용오차: 낮은 고도는 하한, 높은 고도는 비례
void test_vertical_tolerance_scaling()
{
    test_support::check_close(
        riposte_test::vertical_tolerance_for(0.5), 0.30, 1.0e-9,
        "at 50 cm the tolerance is the floor, not 5 cm");
    test_support::check_close(
        riposte_test::vertical_tolerance_for(1.0), 0.30, 1.0e-9, "at 1 m the floor still applies");
    test_support::check_close(
        riposte_test::vertical_tolerance_for(3.0), 0.30, 1.0e-9, "at 3 m the floor still applies");
    test_support::check_close(
        riposte_test::vertical_tolerance_for(5.0), 0.50, 1.0e-9, "at 5 m the ratio takes over");
}

void test_hover_passes_when_within_tolerance()
{
    const auto criteria = riposte_test::hover_criteria_for(0.5, 5.0);
    // 20 Hz x 5 s = 100 표본
    const auto accumulator = make_accumulator(100, 0.05, 0.1);
    std::string note;
    test_support::check(
        riposte_test::evaluate_hover(accumulator, criteria, 5.0, note),
        "a clean 50 cm hover passes");
    test_support::check(note.empty(), "a passing hover leaves no note");
}

void test_hover_fails_on_altitude_error()
{
    const auto criteria = riposte_test::hover_criteria_for(0.5, 5.0);
    // 허용오차 0.30 m 를 넘는 0.45 m
    const auto accumulator = make_accumulator(100, 0.45, 0.1);
    std::string note;
    test_support::check(
        !riposte_test::evaluate_hover(accumulator, criteria, 5.0, note),
        "an altitude error beyond tolerance fails");
    test_support::check(!note.empty(), "a failure explains itself");
}

void test_hover_fails_on_horizontal_error()
{
    const auto criteria = riposte_test::hover_criteria_for(3.0, 5.0);
    const auto accumulator = make_accumulator(100, 0.05, 1.2); // 허용 0.6 m
    std::string note;
    test_support::check(
        !riposte_test::evaluate_hover(accumulator, criteria, 5.0, note),
        "a horizontal error beyond tolerance fails");
}

// 표본이 부족하면 통계를 믿을 수 없으므로 FAIL 이어야 합니다.
void test_hover_fails_when_the_setpoint_loop_was_starved()
{
    const auto criteria = riposte_test::hover_criteria_for(1.0, 5.0);
    const auto accumulator = make_accumulator(10, 0.01, 0.01); // 기대 100, 최소 50
    std::string note;
    test_support::check(
        !riposte_test::evaluate_hover(accumulator, criteria, 5.0, note),
        "too few samples fails even when the errors look perfect");
}

void test_hover_fails_when_shorter_than_required()
{
    const auto criteria = riposte_test::hover_criteria_for(1.0, 5.0);
    const auto accumulator = make_accumulator(100, 0.01, 0.01);
    std::string note;
    test_support::check(
        !riposte_test::evaluate_hover(accumulator, criteria, 3.0, note),
        "a 3 s hover does not satisfy a 5 s requirement");
    // 미세한 초과분은 허용해야 합니다(스케줄러 지터).
    test_support::check(
        riposte_test::evaluate_hover(accumulator, criteria, 4.8, note),
        "a hover within the jitter tolerance still passes");
}

void test_hover_fails_when_telemetry_was_non_finite()
{
    const auto criteria = riposte_test::hover_criteria_for(1.0, 5.0);
    auto accumulator = make_accumulator(100, 0.01, 0.01);
    accumulator.add(std::nan(""), 0.01);
    std::string note;
    test_support::check(
        !riposte_test::evaluate_hover(accumulator, criteria, 5.0, note),
        "non-finite telemetry during hover fails the iteration");
}

void test_report_requires_every_iteration()
{
    riposte_test::TestReport report{};
    report.test_id = 3;
    report.planned_iterations = 10;
    for (int i = 1; i <= 10; ++i) {
        riposte_test::IterationResult result{};
        result.iteration = i;
        result.passed = true;
        report.iterations.push_back(result);
    }
    test_support::check(report.passed(), "ten of ten passing iterations is a pass");

    // 하나라도 FAIL 이면 전체 FAIL
    report.iterations[4].passed = false;
    test_support::check(!report.passed(), "one failing iteration fails the whole test");
    test_support::check(report.passed_count() == 9, "the passing count is still reported");

    // 조기 종료는 전체 FAIL
    report.iterations[4].passed = true;
    report.abort_reason = "battery";
    test_support::check(!report.passed(), "an aborted test is not a pass");

    // 반복이 모자라면 FAIL
    report.abort_reason.clear();
    report.iterations.pop_back();
    test_support::check(!report.passed(), "nine of ten iterations is not a pass");
}

// ---- 카탈로그가 시험 계획서와 일치하는가 --------------------------------

void test_catalog_ids_are_unique_and_contiguous()
{
    for (std::size_t i = 0; i < riposte_test::kTestCatalog.size(); ++i) {
        test_support::check(
            riposte_test::kTestCatalog[i].id == static_cast<int>(i) + 1,
            "catalog ids run 1..23 in order");
    }
    test_support::check(riposte_test::find_test_case(1) != nullptr, "test 1 exists");
    test_support::check(riposte_test::find_test_case(23) != nullptr, "test 23 exists");
    test_support::check(riposte_test::find_test_case(0) == nullptr, "test 0 does not exist");
    test_support::check(riposte_test::find_test_case(24) == nullptr, "test 24 does not exist");
}

void test_catalog_matches_the_test_plan()
{
    struct Expected {
        int id;
        double altitude_m;
        bool north_south;
        int repeat;
    };
    // 시험 계획서 Test 1~20
    constexpr Expected expected[] = {
        {1, 5.0, false, 1},   {2, 5.0, false, 3},   {3, 5.0, false, 10},
        {4, 6.0, false, 1},   {5, 6.0, false, 3},   {6, 6.0, false, 10},
        {7, 7.0, false, 1},   {8, 7.0, false, 3},   {9, 7.0, false, 5},
        {10, 7.0, false, 10}, {11, 5.0, true, 1},   {12, 5.0, true, 3},
        {13, 5.0, true, 10},  {14, 6.0, true, 1},   {15, 6.0, true, 3},
        {16, 6.0, true, 10},  {17, 7.0, true, 1},   {18, 7.0, true, 3},
        {19, 7.0, true, 5},   {20, 7.0, true, 10},
    };
    for (const auto& item : expected) {
        const auto* test_case = riposte_test::find_test_case(item.id);
        test_support::check(test_case != nullptr, "hover case exists");
        if (test_case == nullptr) {
            continue;
        }
        test_support::check(
            test_case->kind == riposte_test::TestKind::HoverCycle, "tests 1-20 are hover cycles");
        test_support::check_close(
            static_cast<double>(test_case->hover.altitude_m), item.altitude_m, 1.0e-6,
            "altitude matches the test plan for test " + std::to_string(item.id));
        test_support::check(
            test_case->hover.north_south_leg == item.north_south,
            "the north/south leg flag matches for test " + std::to_string(item.id));
        test_support::check(
            test_case->hover.repeat == item.repeat,
            "the repeat count matches for test " + std::to_string(item.id));
        test_support::check_close(
            static_cast<double>(test_case->hover.hover_s), 5.0, 1.0e-6,
            "every hover cycle holds for 5 s");
        test_support::check_close(
            static_cast<double>(test_case->hover.leg_distance_m), 1.0, 1.0e-6,
            "every north/south leg is 1 m");
    }
}

void test_catalog_bullet_cases_match_the_test_plan()
{
    {
        const auto* test_case = riposte_test::find_test_case(21);
        test_support::check(test_case != nullptr, "test 21 exists");
        if (test_case != nullptr) {
            test_support::check(
                test_case->kind == riposte_test::TestKind::BulletPath, "test 21 is a bullet path");
            test_support::check_close(
                static_cast<double>(test_case->bullet.altitude_m), 5.0, 1.0e-6,
                "test 21 flies at 5 m");
            test_support::check_close(
                static_cast<double>(test_case->bullet.total_distance_m), 100.0, 1.0e-6,
                "test 21 uses D = 100 m");
            test_support::check_close(
                static_cast<double>(test_case->bullet.width_left_m), 2.0, 1.0e-6,
                "test 21 uses a 2 m left width");
            test_support::check_close(
                static_cast<double>(test_case->bullet.width_right_m), 2.0, 1.0e-6,
                "test 21 uses a 2 m right width");
            test_support::check_close(
                test_case->bullet.target_latitude_deg, 36.6610082, 1.0e-9, "test 21 target lat");
            test_support::check_close(
                test_case->bullet.target_longitude_deg, 126.3417854, 1.0e-9, "test 21 target lon");
        }
    }
    {
        const auto* test_case = riposte_test::find_test_case(22);
        test_support::check(test_case != nullptr, "test 22 exists");
        if (test_case != nullptr) {
            test_support::check_close(
                static_cast<double>(test_case->bullet.altitude_m), 6.0, 1.0e-6,
                "test 22 flies at 6 m");
            test_support::check_close(
                static_cast<double>(test_case->bullet.total_distance_m), 200.0, 1.0e-6,
                "test 22 uses D = 200 m");
            test_support::check_close(
                static_cast<double>(test_case->bullet.width_left_m), 5.0, 1.0e-6,
                "test 22 uses a 5 m width");
        }
    }
    {
        const auto* test_case = riposte_test::find_test_case(23);
        test_support::check(test_case != nullptr, "test 23 exists");
        if (test_case != nullptr) {
            test_support::check(
                test_case->kind == riposte_test::TestKind::BulletPath, "test 23 is a bullet path");
            test_support::check_close(
                static_cast<double>(test_case->bullet.altitude_m), 7.0, 1.0e-6,
                "test 23 flies at 7 m");
            test_support::check_close(
                static_cast<double>(test_case->bullet.total_distance_m), 200.0, 1.0e-6,
                "test 23 uses D = 200 m");
            test_support::check_close(
                static_cast<double>(test_case->bullet.width_left_m), 5.0, 1.0e-6,
                "test 23 uses a 5 m width");
        }
    }
}

// 5/6/7 m 케이스는 모두 Action::takeoff 하한(2.5 m) 이상이어야 합니다.
void test_low_altitude_cases_avoid_action_takeoff()
{
    for (const auto& test_case : riposte_test::kTestCatalog) {
        if (test_case.kind != riposte_test::TestKind::HoverCycle) {
            continue;
        }
        const bool below_floor =
            test_case.hover.altitude_m < riposte_test::TestTuning::action_takeoff_min_altitude_m;
        test_support::check(
            !below_floor, "the 5/6/7 m cases can use PX4 Action::takeoff");
    }
}

void test_battery_gates_are_ordered()
{
    // 비행 중 임계 < 사이클 게이트 < 이륙 전 게이트 여야 의미가 있습니다.
    test_support::check(
        riposte_test::TestTuning::inflight_return_battery_fraction
            < riposte_test::TestTuning::iteration_min_battery_fraction,
        "the in-flight threshold sits below the per-iteration gate");
    test_support::check(
        riposte_test::TestTuning::iteration_min_battery_fraction
            < riposte_test::TestTuning::preflight_min_battery_fraction,
        "the per-iteration gate sits below the preflight gate");
}

void test_watchdog_timeout_is_compatible_with_the_setpoint_rate()
{
    // watchdog 이 2주기보다 짧으면 정상 비행에서도 오발동합니다.
    test_support::check(
        riposte_test::TestTuning::setpoint_watchdog_timeout_s
            > 2.0F / riposte_test::TestTuning::setpoint_rate_hz,
        "the watchdog allows at least two setpoint periods");
    test_support::check(
        riposte_test::TestTuning::setpoint_rate_hz >= 10.0F,
        "the setpoint rate keeps a healthy margin above the PX4 minimum of 2 Hz");
}

} // namespace

int main()
{
    test_accumulator_statistics();
    test_accumulator_rejects_non_finite_samples();
    test_empty_accumulator_is_safe();
    test_vertical_tolerance_scaling();
    test_hover_passes_when_within_tolerance();
    test_hover_fails_on_altitude_error();
    test_hover_fails_on_horizontal_error();
    test_hover_fails_when_the_setpoint_loop_was_starved();
    test_hover_fails_when_shorter_than_required();
    test_hover_fails_when_telemetry_was_non_finite();
    test_report_requires_every_iteration();
    test_catalog_ids_are_unique_and_contiguous();
    test_catalog_matches_the_test_plan();
    test_catalog_bullet_cases_match_the_test_plan();
    test_low_altitude_cases_avoid_action_takeoff();
    test_battery_gates_are_ordered();
    test_watchdog_timeout_is_compatible_with_the_setpoint_rate();
    return test_support::report("verdict_tests");
}
