#!/bin/bash
# riposte-agent.service 의 실행 래퍼.
#
# systemd 유닛에서 조건부 인자를 만들기 어려워 래퍼로 분리했다. 특히 --execute 는
# 기체를 실제로 움직이게 하는 게이트이므로, 유닛 파일을 편집하지 않고 환경변수 하나로
# 껐다 켤 수 있어야 한다.
#
# 안전: riposte 는 이중 --execute 게이트를 쓴다. 이 에이전트에 --execute 가 있어도
# 지상국(riposte_gcs)이 자기 쪽 --execute 로 요청해야만 비행이 시작된다.
# 지상 점검 중이라 기체가 절대 움직이면 안 되는 상황에서는 RIPOSTE_EXECUTE=0 으로 두면
# 에이전트가 연결·대기·보고만 하고 어떤 명령도 보내지 않는다.
set -u

BIN="${RIPOSTE_BIN:-/usr/local/bin/riposte_test}"
CONNECTION="${RIPOSTE_CONNECTION:-serial:///dev/ttyAMA0:921600}"
EXECUTE="${RIPOSTE_EXECUTE:-0}"

ARGS=(--serve --connection "$CONNECTION")
if [ "$EXECUTE" = "1" ]; then
    ARGS+=(--execute)
    echo "[riposte-agent] 비행 명령 허용(--execute). 링크: $CONNECTION"
else
    echo "[riposte-agent] 관찰 전용 — RIPOSTE_EXECUTE=0 이므로 기체에 명령을 보내지 않습니다."
    echo "[riposte-agent] 링크: $CONNECTION"
fi

exec "$BIN" "${ARGS[@]}"
