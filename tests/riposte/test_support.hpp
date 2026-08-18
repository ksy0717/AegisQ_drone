#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

/*
 * 최소 시험 지원 헤더
 * ------------------------------------------------------------------------
 * 기존 프로젝트와 마찬가지로 외부 시험 프레임워크에 의존하지 않습니다.
 * assert 는 Release 의 -DNDEBUG 에서 사라지므로 쓰지 않습니다. 시험은
 * Release/ASan/UBSan 어느 빌드에서도 동일하게 검사되어야 합니다.
 */

namespace test_support {

inline int g_failures = 0;

inline void check(const bool condition, const std::string& what)
{
    if (!condition) {
        std::cerr << "  FAIL: " << what << '\n';
        ++g_failures;
    }
}

inline void check_close(
    const double actual, const double expected, const double tolerance, const std::string& what)
{
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::cerr << "  FAIL: " << what << " (actual=" << actual << ", expected=" << expected
                  << ", tolerance=" << tolerance << ")\n";
        ++g_failures;
    }
}

inline int report(const std::string& suite)
{
    if (g_failures == 0) {
        std::cout << suite << ": all checks passed\n";
        return EXIT_SUCCESS;
    }
    std::cerr << suite << ": " << g_failures << " check(s) failed\n";
    return EXIT_FAILURE;
}

} // namespace test_support
