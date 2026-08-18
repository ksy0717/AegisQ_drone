#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "test_cases.hpp"

/*
 * Ground Station 원격 시험 프로토콜
 * ------------------------------------------------------------------------
 * GCS 가 텔레메트리(MAVLink) 링크로 companion 컴퓨터의 시험 앱에 시험 실행을
 * 요청하는 프로토콜입니다. 경로는 다음과 같습니다.
 *
 *   riposte_gcs ──(radio/UDP)── PX4 (MAVLink 라우터) ──(TELEM2/UDP)── riposte_test --serve
 *
 * 메시지는 표준 MAVLink 를 그대로 씁니다. 커스텀 dialect 를 만들지 않는 이유는
 * QGroundControl 등 다른 GCS 와 같은 링크를 공유해도 깨지지 않아야 하기 때문입니다.
 *
 *   요청  : COMMAND_LONG, command = MAV_CMD_USER_1(31010)
 *             param1 = 시험 번호 (1~23)
 *             param2 = 반복 횟수 덮어쓰기 (0 = 카탈로그 값)
 *             param3 = 안전 매직 코드 (kRequestMagic)
 *             target_component = 191 (MAV_COMP_ID_ONBOARD_COMPUTER)
 *   중단  : COMMAND_LONG, command = MAV_CMD_USER_2(31011), param1 = 매직 코드
 *   응답  : COMMAND_ACK (ACCEPTED / TEMPORARILY_REJECTED=busy / DENIED=잘못된 요청)
 *   진행  : STATUSTEXT "RT|..." (아래 형식). 50자 제한을 지킵니다.
 *
 * param3 의 매직 코드는 다른 GCS 소프트웨어가 MAV_CMD_USER_1 을 다른 용도로
 * 보냈을 때 우연히 시험이 시작되는 것을 막는 안전장치입니다. 보안 장치가
 * 아니므로(링크를 볼 수 있으면 알 수 있음) 물리적 링크 보안은 별도입니다.
 *
 * 이 헤더는 MAVLink/MAVSDK 헤더를 포함하지 않는 순수 코드입니다. 필요한 상수는
 * 값으로 재정의했고(컴파일 시 실제 헤더와 대조하는 것은 각 앱 쪽 static_assert),
 * 덕분에 tests/test_gcs_protocol.cpp 가 MAVSDK 없이 이 프로토콜을 검증합니다.
 */

namespace riposte_test::gcs {

// MAV_CMD_USER_1 / MAV_CMD_USER_2 (MAVLink common)
inline constexpr std::uint16_t kCommandRunTest = 31010;
inline constexpr std::uint16_t kCommandAbort = 31011;

// MAV_COMP_ID_ONBOARD_COMPUTER. 시험 앱(--serve)이 쓰는 컴포넌트 ID 입니다.
inline constexpr std::uint8_t kAgentComponentId = 191;

// param3 안전 매직 코드. float 로 왕복해도 정확히 표현되는 값을 골랐습니다.
inline constexpr float kRequestMagic = 7305.0F;

// STATUSTEXT 의 text 필드는 50바이트(널 포함)입니다.
inline constexpr std::size_t kStatusTextMaxLength = 49;

struct TestRequest {
    int test_id{};

    // 0 이면 카탈로그의 기본 반복 횟수를 씁니다 (--repeat 미지정과 동일).
    int repeat_override{};
};

// COMMAND_LONG 의 param1~3 입니다.
struct RequestParams {
    float param1{};
    float param2{};
    float param3{};
};

inline RequestParams encode_request(const TestRequest& request)
{
    return RequestParams{
        static_cast<float>(request.test_id),
        static_cast<float>(request.repeat_override),
        kRequestMagic,
    };
}

inline bool magic_matches(const float param) noexcept
{
    // 매직 코드는 정수값이라 float 전송에서 손실이 없지만, 미래의 코드 변경이나
    // 별난 GCS 구현이 double->float 반올림을 섞어도 견디도록 0.5 여유를 둡니다.
    return std::isfinite(param) && std::abs(param - kRequestMagic) < 0.5F;
}

/*
 * 수신한 param1~3 을 검증하며 해석합니다. 유효하지 않으면 nullopt 를 반환하고
 * reason 에 사유를 채웁니다. 여기서 걸리는 요청은 COMMAND_ACK 의 DENIED 로
 * 응답해야 합니다.
 */
inline std::optional<TestRequest> decode_request(
    const float param1, const float param2, const float param3, std::string& reason)
{
    if (!std::isfinite(param1) || !std::isfinite(param2) || !std::isfinite(param3)) {
        reason = "request parameters contain NaN or infinity";
        return std::nullopt;
    }
    if (!magic_matches(param3)) {
        // 매직 불일치는 우리 프로토콜이 아닌 MAV_CMD_USER_1 사용일 가능성이
        // 높습니다. 시험을 시작하지 않는 것이 요점입니다.
        reason = "safety magic code mismatch (param3)";
        return std::nullopt;
    }

    const long test_id = std::lround(param1);
    if (std::abs(param1 - static_cast<float>(test_id)) > 1.0e-3F) {
        reason = "test id is not an integer";
        return std::nullopt;
    }
    if (riposte_test::find_test_case(static_cast<int>(test_id)) == nullptr) {
        reason = "unknown test id " + std::to_string(test_id);
        return std::nullopt;
    }

    const long repeat = std::lround(param2);
    if (std::abs(param2 - static_cast<float>(repeat)) > 1.0e-3F) {
        reason = "repeat override is not an integer";
        return std::nullopt;
    }
    if (repeat < 0 || repeat > 1000) {
        reason = "repeat override out of range (0-1000)";
        return std::nullopt;
    }

    return TestRequest{static_cast<int>(test_id), static_cast<int>(repeat)};
}

// main() 과 동일한 규칙입니다: Bullet 시험의 기본 반복(사이클) 수는 laps 가
// 아니라 1 입니다. laps 는 한 사이클 안의 순환 횟수로 run_bullet_path() 가
// 따로 처리합니다.
inline int effective_repeat(const riposte_test::TestCase& test_case, const int repeat_override)
{
    if (repeat_override > 0) {
        return repeat_override;
    }
    return test_case.kind == riposte_test::TestKind::HoverCycle ? test_case.hover.repeat : 1;
}

// STATUSTEXT 의 50바이트 제한에 맞춰 자릅니다.
inline std::string clip_status_text(std::string text)
{
    if (text.size() > kStatusTextMaxLength) {
        text.resize(kStatusTextMaxLength);
    }
    return text;
}

// ---- STATUSTEXT 라인 형식 --------------------------------------------------
//
//   RT|READY                        에이전트가 요청 대기 중
//   RT|START|<id>|x<repeat>         시험 시작
//   RT|IT|<id>|<i>/<n>|<PASS|FAIL>  반복 1회 종료
//   RT|RESULT|<id>|<PASS|FAIL>|<completed>/<planned>   시험 전체 종료
//   RT|ABORTED|<id>                 GCS 중단 요청으로 착륙 완료 (0 = 유휴, 중단할 것 없음)
//   RT|STOPPED                      운영자가 에이전트를 중지 (오류 아님)
//   RT|ERROR|<사유>                 오류로 에이전트 종료 (사람 점검 필요)
//
// 모든 라인은 49자 이하입니다. PX4 자체 STATUSTEXT 와 섞여 수신되므로
// "RT|" 접두사로 구분합니다.

inline bool is_protocol_text(const std::string& text)
{
    return text.rfind("RT|", 0) == 0;
}

inline std::string ready_text()
{
    return "RT|READY";
}

inline std::string start_text(const int test_id, const int repeat)
{
    return clip_status_text(
        "RT|START|" + std::to_string(test_id) + "|x" + std::to_string(repeat));
}

inline std::string iteration_text(
    const int test_id, const int iteration, const int planned, const bool passed)
{
    return clip_status_text(
        "RT|IT|" + std::to_string(test_id) + '|' + std::to_string(iteration) + '/'
        + std::to_string(planned) + '|' + (passed ? "PASS" : "FAIL"));
}

inline std::string result_text(
    const int test_id, const bool passed, const int completed, const int planned)
{
    return clip_status_text(
        "RT|RESULT|" + std::to_string(test_id) + '|' + (passed ? "PASS" : "FAIL") + '|'
        + std::to_string(completed) + '/' + std::to_string(planned));
}

inline std::string aborted_text(const int test_id)
{
    return clip_status_text("RT|ABORTED|" + std::to_string(test_id));
}

inline std::string error_text(const std::string& reason)
{
    return clip_status_text("RT|ERROR|" + reason);
}

// 운영자가 에이전트 쪽에서 SIGINT/SIGTERM 으로 중지한 경우입니다. 오류가 아니라
// 의도된 중지이므로 RT|ERROR 와 구분합니다 (GCS 안내 문구가 달라집니다).
inline std::string stopped_text()
{
    return "RT|STOPPED";
}

// GCS 쪽 해석 결과입니다.
struct ResultLine {
    int test_id{};
    bool passed{};
    int completed{};
    int planned{};
};

namespace detail {

inline std::vector<std::string> split(const std::string& text, const char separator)
{
    std::vector<std::string> tokens;
    std::size_t begin = 0;
    while (true) {
        const std::size_t end = text.find(separator, begin);
        if (end == std::string::npos) {
            tokens.push_back(text.substr(begin));
            return tokens;
        }
        tokens.push_back(text.substr(begin, end - begin));
        begin = end + 1;
    }
}

inline std::optional<int> parse_int(const std::string& text)
{
    if (text.empty()) {
        return std::nullopt;
    }
    try {
        std::size_t consumed = 0;
        const int value = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

} // namespace detail

// "RT|RESULT|..." 라인을 해석합니다. 형식이 아니면 nullopt 입니다.
inline std::optional<ResultLine> parse_result_text(const std::string& text)
{
    const auto tokens = detail::split(text, '|');
    if (tokens.size() != 5 || tokens[0] != "RT" || tokens[1] != "RESULT") {
        return std::nullopt;
    }
    const auto test_id = detail::parse_int(tokens[2]);
    if (!test_id) {
        return std::nullopt;
    }
    if (tokens[3] != "PASS" && tokens[3] != "FAIL") {
        return std::nullopt;
    }
    const auto counts = detail::split(tokens[4], '/');
    if (counts.size() != 2) {
        return std::nullopt;
    }
    const auto completed = detail::parse_int(counts[0]);
    const auto planned = detail::parse_int(counts[1]);
    if (!completed || !planned) {
        return std::nullopt;
    }
    return ResultLine{*test_id, tokens[3] == "PASS", *completed, *planned};
}

} // namespace riposte_test::gcs
