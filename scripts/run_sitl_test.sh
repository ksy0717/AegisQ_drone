#!/usr/bin/env bash
# PX4 SITL 에서 시험 케이스를 실행합니다.
#
# 사용 전 별도 터미널에서 X500 SITL 을 띄우십시오.
#   cd ~/PX4-Autopilot && make px4_sitl gz_x500
#
# 사용법:
#   ./scripts/run_sitl_test.sh 1          # Test 1 을 카탈로그 반복 횟수로
#   ./scripts/run_sitl_test.sh 13 2       # Test 13 을 2회만 (10회 대신)
#
# 실기체에서는 이 스크립트를 쓰지 말고 --connection 을 직접 지정하십시오.
# 이 스크립트의 기본 연결은 SITL 전용입니다.

set -euo pipefail

TEST_ID="${1:-}"
REPEAT="${2:-}"
BUILD_DIR="${BUILD_DIR:-build}"
CONNECTION="${CONNECTION:-udpin://0.0.0.0:14540}"

if [[ -z "${TEST_ID}" ]]; then
    echo "Usage: $0 <test id 1-23> [repeat override]" >&2
    echo "Run '${BUILD_DIR}/riposte_test --list' to see the catalogue." >&2
    exit 1
fi

if [[ ! -x "${BUILD_DIR}/riposte_test" ]]; then
    echo "Build first:" >&2
    echo "  cmake -S . -B ${BUILD_DIR} -DCMAKE_BUILD_TYPE=Release" >&2
    echo "  cmake --build ${BUILD_DIR}" >&2
    exit 1
fi

# 비행 전에 기하 단위시험을 먼저 통과시킵니다. 경로 계산이 틀린 채로 SITL 을
# 돌리면 원인 분리가 어려워집니다.
echo "== unit tests =="
ctest --test-dir "${BUILD_DIR}" --output-on-failure

# 무엇을 비행할지 먼저 사람이 눈으로 확인합니다.
echo
echo "== plan =="
"${BUILD_DIR}/riposte_test" --test "${TEST_ID}" --plan-only

ARGS=(--test "${TEST_ID}" --execute --connection "${CONNECTION}")
if [[ -n "${REPEAT}" ]]; then
    ARGS+=(--repeat "${REPEAT}")
fi

echo
echo "== flying test ${TEST_ID} on ${CONNECTION} =="
exec "${BUILD_DIR}/riposte_test" "${ARGS[@]}"
