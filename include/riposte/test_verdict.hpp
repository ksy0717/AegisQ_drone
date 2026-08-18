#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "test_config.hpp"

/*
 * 시험 판정
 * ------------------------------------------------------------------------
 * 기존 미션 앱은 "임무를 끝냈는가"만 알려 줍니다(mission_completed_). 시험
 * 애플리케이션은 거기서 한 발 더 나가 "요구 성능을 만족했는가"를 판정해야
 * 하므로, 호버 구간의 오차를 누적해 PASS/FAIL 을 내는 부분을 분리했습니다.
 *
 * 판정은 순수 데이터 처리이므로 tests/test_verdict.cpp 로 단위시험합니다.
 *
 * 주의: 여기서의 PASS 는 "SW 가 지시한 대로 기체가 따라왔는가"입니다.
 * 기체 자세, EKF 건전성, 진동, actuator saturation 은 PX4 ULog 로 별도
 * 확인해야 하며 이 판정이 대체하지 않습니다.
 */

namespace riposte_test {

// 호버 구간에서 누적하는 표본입니다.
class HoverAccumulator {
public:
    void add(const double altitude_error_m, const double horizontal_error_m)
    {
        if (!std::isfinite(altitude_error_m) || !std::isfinite(horizontal_error_m)) {
            // NaN 표본은 오차 통계를 통째로 오염시킵니다. 개수만 세어 두고
            // 평균에서 제외한 뒤, 판정 단계에서 유효 표본 수로 걸러 냅니다.
            ++invalid_samples_;
            return;
        }
        const double absolute_altitude_error_m = std::abs(altitude_error_m);
        altitude_error_sum_m_ += absolute_altitude_error_m;
        max_altitude_error_m_ = std::max(max_altitude_error_m_, absolute_altitude_error_m);
        max_horizontal_error_m_ = std::max(max_horizontal_error_m_, horizontal_error_m);
        ++samples_;
    }

    std::size_t samples() const { return samples_; }
    std::size_t invalid_samples() const { return invalid_samples_; }
    double max_altitude_error_m() const { return max_altitude_error_m_; }
    double max_horizontal_error_m() const { return max_horizontal_error_m_; }

    double mean_altitude_error_m() const
    {
        return samples_ == 0 ? 0.0
                             : altitude_error_sum_m_ / static_cast<double>(samples_);
    }

private:
    double altitude_error_sum_m_{};
    double max_altitude_error_m_{};
    double max_horizontal_error_m_{};
    std::size_t samples_{};
    std::size_t invalid_samples_{};
};

// 사이클 1회의 결과입니다.
struct IterationResult {
    int iteration{};
    bool passed{};
    double commanded_altitude_m{};
    double mean_altitude_error_m{};
    double max_altitude_error_m{};
    double max_horizontal_error_m{};
    double hover_duration_s{};
    double cycle_duration_s{};
    double battery_start_percent{-1.0};
    double battery_end_percent{-1.0};
    std::size_t hover_samples{};
    std::string note{}; // FAIL 사유 또는 특이사항
};

// 호버 구간 판정 기준입니다.
struct HoverCriteria {
    double altitude_tolerance_m{};
    double horizontal_tolerance_m{};
    double required_hover_s{};
    double hover_duration_tolerance_s{0.5};
    std::size_t minimum_samples{};
};

inline HoverCriteria hover_criteria_for(const double altitude_m, const double hover_s)
{
    HoverCriteria criteria{};
    criteria.altitude_tolerance_m = vertical_tolerance_for(altitude_m);
    criteria.horizontal_tolerance_m = static_cast<double>(TestTuning::horizontal_tolerance_m);
    criteria.required_hover_s = hover_s;
    // 20 Hz x 5 초 = 100 표본이 기대값입니다. 절반 미만이면 setpoint 루프가
    // 밀린 것이므로 통계를 신뢰할 수 없습니다.
    const double expected_samples =
        hover_s * static_cast<double>(TestTuning::setpoint_rate_hz);
    criteria.minimum_samples = static_cast<std::size_t>(expected_samples * 0.5);
    return criteria;
}

// 누적된 호버 표본과 기준으로 PASS/FAIL 을 판정합니다.
// FAIL 이면 note 에 사유를 채웁니다.
inline bool evaluate_hover(
    const HoverAccumulator& accumulator,
    const HoverCriteria& criteria,
    const double hover_duration_s,
    std::string& note)
{
    if (accumulator.samples() < criteria.minimum_samples) {
        note = "hover samples " + std::to_string(accumulator.samples())
            + " below minimum " + std::to_string(criteria.minimum_samples)
            + " (setpoint loop was starved)";
        return false;
    }
    if (accumulator.invalid_samples() > 0) {
        note = "telemetry produced " + std::to_string(accumulator.invalid_samples())
            + " non-finite samples during hover";
        return false;
    }
    if (hover_duration_s + criteria.hover_duration_tolerance_s < criteria.required_hover_s) {
        note = "hover lasted " + std::to_string(hover_duration_s) + " s, required "
            + std::to_string(criteria.required_hover_s) + " s";
        return false;
    }
    if (accumulator.max_altitude_error_m() > criteria.altitude_tolerance_m) {
        note = "peak altitude error " + std::to_string(accumulator.max_altitude_error_m())
            + " m exceeds tolerance " + std::to_string(criteria.altitude_tolerance_m) + " m";
        return false;
    }
    if (accumulator.max_horizontal_error_m() > criteria.horizontal_tolerance_m) {
        note = "peak horizontal error " + std::to_string(accumulator.max_horizontal_error_m())
            + " m exceeds tolerance " + std::to_string(criteria.horizontal_tolerance_m) + " m";
        return false;
    }
    note.clear();
    return true;
}

// 시험 1건(반복 전체)의 결과입니다.
struct TestReport {
    int test_id{};
    std::string test_name{};
    int planned_iterations{};
    std::vector<IterationResult> iterations{};
    std::string abort_reason{}; // 비어 있지 않으면 조기 종료된 것입니다.

    int completed() const { return static_cast<int>(iterations.size()); }

    int passed_count() const
    {
        int count = 0;
        for (const auto& iteration : iterations) {
            if (iteration.passed) {
                ++count;
            }
        }
        return count;
    }

    // 계획한 반복을 모두 마쳤고 전부 PASS 여야 시험 전체가 PASS 입니다.
    bool passed() const
    {
        return abort_reason.empty() && completed() == planned_iterations
            && passed_count() == planned_iterations;
    }
};

} // namespace riposte_test
