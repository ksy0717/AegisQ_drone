#include "gcs_protocol.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

/*
 * Ground Station 프로토콜 단위시험
 * ------------------------------------------------------------------------
 * 요청 해석이 헐거우면 다른 GCS 의 MAV_CMD_USER_1 로 시험이 시작될 수 있고,
 * 결과 라인 해석이 틀리면 GCS 가 FAIL 을 PASS 로 보고합니다. 둘 다 하드웨어
 * 없이 여기서 걸러야 합니다.
 */

namespace gcs = riposte_test::gcs;

namespace {

void test_request_roundtrip()
{
    const gcs::TestRequest request{13, 2};
    const auto params = gcs::encode_request(request);
    test_support::check_close(
        static_cast<double>(params.param1), 13.0, 1.0e-9, "param1 carries the test id");
    test_support::check_close(
        static_cast<double>(params.param2), 2.0, 1.0e-9, "param2 carries the repeat override");
    test_support::check_close(
        static_cast<double>(params.param3), static_cast<double>(gcs::kRequestMagic), 1.0e-9,
        "param3 carries the safety magic");

    std::string reason;
    const auto decoded =
        gcs::decode_request(params.param1, params.param2, params.param3, reason);
    test_support::check(decoded.has_value(), "a valid request decodes: " + reason);
    if (decoded) {
        test_support::check(decoded->test_id == 13, "test id survives the roundtrip");
        test_support::check(decoded->repeat_override == 2, "repeat survives the roundtrip");
    }
}

void test_all_catalog_ids_roundtrip()
{
    for (const auto& test_case : riposte_test::kTestCatalog) {
        std::string reason;
        const auto params = gcs::encode_request({test_case.id, 0});
        const auto decoded =
            gcs::decode_request(params.param1, params.param2, params.param3, reason);
        test_support::check(
            decoded.has_value(),
            "catalog id " + std::to_string(test_case.id) + " roundtrips: " + reason);
    }
}

void test_magic_is_required()
{
    std::string reason;
    test_support::check(
        !gcs::decode_request(1.0F, 0.0F, 0.0F, reason).has_value(),
        "a request without the magic code is denied");
    test_support::check(
        !gcs::decode_request(1.0F, 0.0F, gcs::kRequestMagic + 5.0F, reason).has_value(),
        "a request with the wrong magic code is denied");
    test_support::check(
        !gcs::magic_matches(std::nanf("")), "a NaN magic never matches");
    test_support::check(
        gcs::magic_matches(gcs::kRequestMagic), "the exact magic matches");
}

void test_invalid_requests_are_denied()
{
    std::string reason;
    test_support::check(
        !gcs::decode_request(0.0F, 0.0F, gcs::kRequestMagic, reason).has_value(),
        "test id 0 is denied");
    test_support::check(
        !gcs::decode_request(24.0F, 0.0F, gcs::kRequestMagic, reason).has_value(),
        "test id 24 is denied");
    test_support::check(
        !gcs::decode_request(13.5F, 0.0F, gcs::kRequestMagic, reason).has_value(),
        "a fractional test id is denied");
    test_support::check(
        !gcs::decode_request(13.0F, -1.0F, gcs::kRequestMagic, reason).has_value(),
        "a negative repeat is denied");
    test_support::check(
        !gcs::decode_request(13.0F, 2000.0F, gcs::kRequestMagic, reason).has_value(),
        "an absurd repeat is denied");
    test_support::check(
        !gcs::decode_request(
             std::numeric_limits<float>::quiet_NaN(), 0.0F, gcs::kRequestMagic, reason)
             .has_value(),
        "a NaN test id is denied");
    test_support::check(!reason.empty(), "denial always explains why");
}

void test_effective_repeat_rules()
{
    const auto* hover = riposte_test::find_test_case(3);   // 5 m x10
    const auto* bullet = riposte_test::find_test_case(21); // laps=1
    test_support::check(hover != nullptr && bullet != nullptr, "catalog lookups work");
    if (hover == nullptr || bullet == nullptr) {
        return;
    }
    test_support::check(
        gcs::effective_repeat(*hover, 0) == 10, "hover default repeat comes from the catalog");
    test_support::check(
        gcs::effective_repeat(*hover, 2) == 2, "the override wins for hover tests");
    test_support::check(
        gcs::effective_repeat(*bullet, 0) == 1,
        "bullet default repeat is one cycle, not laps");
    test_support::check(
        gcs::effective_repeat(*bullet, 3) == 3, "the override wins for bullet tests");
}

void test_status_texts_fit_the_mavlink_limit()
{
    const std::string lines[] = {
        gcs::ready_text(),
        gcs::start_text(22, 1000),
        gcs::iteration_text(22, 1000, 1000, false),
        gcs::result_text(22, false, 1000, 1000),
        gcs::aborted_text(22),
        gcs::stopped_text(),
        gcs::error_text("a very long explanation that will not fit into a status text at all"),
    };
    for (const auto& line : lines) {
        test_support::check(
            line.size() <= gcs::kStatusTextMaxLength,
            "status text fits 49 chars: " + line);
        test_support::check(gcs::is_protocol_text(line), "protocol text has the RT| prefix");
    }
    test_support::check(
        !gcs::is_protocol_text("[logger] preflight checks passed"),
        "ordinary PX4 text is not protocol text");
}

void test_result_line_roundtrip()
{
    const auto line = gcs::result_text(13, true, 10, 10);
    const auto parsed = gcs::parse_result_text(line);
    test_support::check(parsed.has_value(), "a result line parses");
    if (parsed) {
        test_support::check(parsed->test_id == 13, "result test id");
        test_support::check(parsed->passed, "result verdict");
        test_support::check(parsed->completed == 10 && parsed->planned == 10, "result counts");
    }

    const auto failed = gcs::parse_result_text(gcs::result_text(21, false, 0, 1));
    test_support::check(
        failed.has_value() && !failed->passed && failed->completed == 0,
        "a failing result parses too");
}

void test_result_parser_rejects_garbage()
{
    test_support::check(!gcs::parse_result_text("").has_value(), "empty line");
    test_support::check(
        !gcs::parse_result_text("RT|READY").has_value(), "ready line is not a result");
    test_support::check(
        !gcs::parse_result_text("RT|STOPPED").has_value(), "stopped line is not a result");
    test_support::check(
        !gcs::parse_result_text("RT|IT|13|2/10|PASS").has_value(),
        "iteration line is not a result");
    test_support::check(
        !gcs::parse_result_text("RT|RESULT|abc|PASS|1/1").has_value(),
        "non-numeric test id");
    test_support::check(
        !gcs::parse_result_text("RT|RESULT|13|MAYBE|1/1").has_value(),
        "unknown verdict word");
    test_support::check(
        !gcs::parse_result_text("RT|RESULT|13|PASS|11").has_value(),
        "missing planned count");
    test_support::check(
        !gcs::parse_result_text("XX|RESULT|13|PASS|1/1").has_value(),
        "wrong prefix");
}

void test_clip_is_a_hard_limit()
{
    const std::string long_text(200, 'x');
    test_support::check(
        gcs::clip_status_text(long_text).size() == gcs::kStatusTextMaxLength,
        "clipping enforces the 49 char limit");
    test_support::check(
        gcs::clip_status_text("short") == "short", "short text is untouched");
}

} // namespace

int main()
{
    test_request_roundtrip();
    test_all_catalog_ids_roundtrip();
    test_magic_is_required();
    test_invalid_requests_are_denied();
    test_effective_repeat_rules();
    test_status_texts_fit_the_mavlink_limit();
    test_result_line_roundtrip();
    test_result_parser_rejects_garbage();
    test_clip_is_a_hard_limit();
    return test_support::report("gcs_protocol_tests");
}
