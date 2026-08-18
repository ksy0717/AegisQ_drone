#pragma once

#include <chrono>

namespace bullet_mission {

// 예외가 발생한 시점의 기체 상태를 순수 함수로 분류합니다. 이 로직을 MAVSDK에서
// 분리해 Arm 직후 Takeoff 실패 같은 고장도 하드웨어 없이 단위시험할 수 있습니다.
enum class EmergencyAction {
    None,
    Land,
    DisarmOnGround,
};

inline EmergencyAction choose_emergency_action(
    const bool connected,
    const bool armed,
    const bool in_air,
    const bool takeoff_command_accepted = false) noexcept
{
    if (!connected) {
        return EmergencyAction::None;
    }
    // Takeoff가 수락된 직후에는 PX4의 in_air telemetry가 아직 false일 수 있습니다.
    // 이 구간을 지상으로 오판해 Disarm하면 초기 상승 중 모터가 정지할 수 있으므로,
    // Takeoff 수락 이후의 armed 오류는 보수적으로 Land를 선택합니다.
    if (in_air || (armed && takeoff_command_accepted)) {
        return EmergencyAction::Land;
    }
    return armed ? EmergencyAction::DisarmOnGround : EmergencyAction::None;
}

// setpoint deadline 판정도 순수 함수로 유지합니다. MAVSDK가 마지막 setpoint를 계속
// 재전송하는지와 무관하게 companion application의 새 명령 생성 정지를 검출합니다.
inline bool setpoint_deadline_expired(
    const std::chrono::steady_clock::time_point now,
    const std::chrono::steady_clock::time_point last_generated,
    const std::chrono::steady_clock::duration timeout) noexcept
{
    return now - last_generated > timeout;
}

} // namespace bullet_mission
