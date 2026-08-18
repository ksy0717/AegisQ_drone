#include "bullet_path.hpp"
#include "gcs_protocol.hpp"
#include "geo.hpp"
#include "mavsdk_offboard_control.hpp"
#include "safety_policy.hpp"
#include "test_cases.hpp"
#include "test_config.hpp"
#include "test_trajectory.hpp"
#include "test_verdict.hpp"
#include "trajectory.hpp"

/*
 * X500 + Pixhawk 시험 애플리케이션
 * ============================================================================
 * 기존 Bullet_Drone_Mission_CPP_v1.4.2-ChatGPT/src/main.cpp 의 구조를 그대로
 * 따릅니다. 안전 골격(예외 -> emergency_land_if_needed, setpoint watchdog,
 * 소프트웨어 반경 제한, 배터리 게이트)은 검증된 것이므로 손대지 않았고,
 * 바뀐 것은 "무엇을 비행하는가"뿐입니다.
 *
 * 기존 main.cpp 와의 차이
 * ----------------------------------------------------------------------------
 *   1) MissionRunner 는 고정 시나리오 1개를 실행했지만, TestRunner 는
 *      test_cases.hpp 의 카탈로그에서 케이스를 받아 실행합니다.
 *   2) 이륙~착륙을 N회 반복합니다. 매 사이클마다 Arm/Disarm 을 다시 합니다.
 *      (반복 시험의 목적이 Arm~Disarm 전 구간의 재현성 확인이기 때문입니다.)
 *   3) 50 cm / 100 cm 는 PX4 의 Action::takeoff() 로 만들 수 없어(MIS_TAKEOFF_ALT
 *      하한) Offboard 직접 상승 경로를 별도로 구현했습니다.
 *   4) 호버 구간 오차를 누적해 PASS/FAIL 을 판정하고 요약 CSV 를 남깁니다.
 *   5) EO/IR recorder 는 제외했습니다. 본 앱의 목적은 비행 거동 검증이며,
 *      GStreamer/Hailo 의존을 빼면 SITL 과 실기체에서 동일 바이너리를 씁니다.
 *      영상 증빙이 필요하면 기존 미션 앱을 사용하십시오.
 *
 * 시험 흐름
 * ----------------------------------------------------------------------------
 *   main()
 *     -> parse_options()
 *     -> (--list / --plan-only 이면 여기서 종료)
 *     -> MAVSDK 연결, telemetry rate 설정
 *     -> TestRunner::run(test_case)
 *          -> validate_case()
 *          -> wait_for_vehicle_health()
 *          -> check_preflight_battery()
 *          -> for iteration in 1..repeat:
 *               -> run_hover_cycle() 또는 run_bullet_path()
 *          -> TestReport 반환
 *     -> 요약 출력 + summary CSV
 *
 * 주의: 이 코드를 수정할 때는 먼저 ctest(기하/판정 단위시험)와 PX4 SITL 을
 * 통과시키십시오. 프로펠러가 장착된 기체에서 바로 시험하지 마십시오.
 */

// MAVSDK v3 API 를 씁니다. 최소 버전은 3.17.1 이며(CMake 의 find_package 와 동일 기준),
// 더 높은 3.x 는 그대로 동작합니다. 3.17.1 이 바닥인 이유는 아래 v3 의미 변경들에
// 의존하기 때문입니다 — remaining_percent 가 0~100 퍼센트, CompanionComputer 기본
// 컴포넌트 ID 195. 그 아래 버전에서는 빌드는 되더라도 조용히 오동작합니다.
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// gcs_protocol.hpp 는 단위시험이 MAVSDK 없이 돌도록 MAVLink 상수를 값으로
// 재정의합니다. 실제 MAVLink 헤더가 있는 이 파일에서 두 정의가 어긋나지
// 않는지 컴파일 시점에 대조합니다.
static_assert(riposte_test::gcs::kCommandRunTest == MAV_CMD_USER_1,
              "gcs_protocol.hpp command id must match MAV_CMD_USER_1");
static_assert(riposte_test::gcs::kCommandAbort == MAV_CMD_USER_2,
              "gcs_protocol.hpp abort id must match MAV_CMD_USER_2");
static_assert(riposte_test::gcs::kAgentComponentId == MAV_COMP_ID_ONBOARD_COMPUTER,
              "gcs_protocol.hpp component id must match MAV_COMP_ID_ONBOARD_COMPUTER");

using namespace std::chrono_literals;

namespace {

using bullet_mission::NedPoint;
using bullet_mission::TrajectorySetpoint;
using riposte_test::TestTuning;

// Ctrl+C(SIGINT) 또는 서비스 종료(SIGTERM)를 모든 제어 루프에 전달하는 플래그입니다.
// signal handler 안에서 안전하게 쓸 수 있는 것은 lock-free atomic 뿐입니다.
std::atomic_bool g_stop_requested{false};
static_assert(std::atomic_bool::is_always_lock_free,
              "Signal stop flag must use a lock-free atomic_bool");

// GCS 가 텔레메트리로 보낸 중단 요청입니다. SIGINT 와 달리 현재 시험만 중단하고
// (--serve 모드에서는) 다음 요청을 계속 기다립니다.
std::atomic_bool g_remote_abort_requested{false};

void signal_handler(int)
{
    // signal handler 안에서는 MAVSDK 호출, 파일 저장, cout 출력이 모두 위험합니다.
    // 여기서는 플래그만 세우고, 실제 착륙은 정상 실행 흐름의
    // emergency_land_if_needed() 가 담당합니다.
    g_stop_requested.store(true);
}

struct ProgramOptions {
    // PX4 SITL 기본 포트입니다. 실기체는 serial:///dev/ttyAMA0:921600 형태입니다.
    std::string connection_url{"udpin://0.0.0.0:14540"};

    // 실행할 시험 번호입니다. 0 이면 미지정입니다.
    int test_id{0};

    // 카탈로그만 출력하고 종료합니다.
    bool list{false};

    // 기체에 연결하지 않고 산출된 경로/시간만 출력합니다.
    bool plan_only{false};

    // --plan-only 로 Bullet 경로를 확인할 때 쓸 가상 이륙 지점입니다.
    std::optional<riposte_test::GeoPoint> plan_home{};

    // 반복 횟수를 카탈로그 값 대신 덮어씁니다. 현장에서 10회 대신 2회만
    // 돌려 보는 용도입니다. 0 이면 카탈로그 값을 씁니다.
    int repeat_override{0};

    // 실수로 프로그램만 실행했을 때 Arm/Takeoff 가 나가지 않도록 하는 안전 게이트입니다.
    bool execute{false};

    // Ground Station 원격 요청 대기 모드입니다. --execute 와 함께 써야 하며,
    // 시험 번호는 GCS 의 COMMAND_LONG(MAV_CMD_USER_1) 으로 받습니다.
    bool serve{false};
};

void print_usage(const char* program)
{
    std::cout
        << "Usage: " << program << " --test <1-23> --execute [options]\n\n"
        << "Examples:\n"
        << "  카탈로그 확인 : " << program << " --list\n"
        << "  경로만 산출   : " << program << " --test 21 --plan-only"
           " --plan-home 36.6606 126.3417\n"
        << "  PX4 SITL      : " << program
        << " --test 1 --execute --connection udpin://0.0.0.0:14540\n"
        << "  Pixhawk UART  : " << program
        << " --test 1 --execute --connection serial:///dev/ttyAMA0:921600\n"
        << "  GCS 대기 모드 : " << program
        << " --serve --execute --connection serial:///dev/ttyAMA0:921600\n\n"
        << "Options:\n"
        << "  --test <id>              시험 번호 (1~23)\n"
        << "  --list                   시험 카탈로그 출력 후 종료\n"
        << "  --plan-only              기체 연결 없이 산출 경로/예상 시간만 출력\n"
        << "  --plan-home <lat> <lon>  --plan-only 에서 쓸 가상 이륙 지점\n"
        << "  --repeat <n>             반복 횟수를 카탈로그 값 대신 사용\n"
        << "  --serve                  Ground Station 의 텔레메트리 요청을 대기\n"
        << "                           (riposte_gcs 와 짝, docs/GCS_USAGE.md 참조)\n"
        << "  --connection <url>       MAVSDK 연결 URL\n"
        << "  --help, -h               이 도움말\n\n"
        << "--execute 가 없으면 기체에 어떤 명령도 보내지 않습니다.\n";
}

int parse_integer(const std::string& text, const std::string& option)
{
    try {
        std::size_t consumed = 0;
        const int value = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            throw std::runtime_error("trailing characters");
        }
        return value;
    } catch (...) {
        throw std::runtime_error(option + " requires an integer: " + text);
    }
}

double parse_double(const std::string& text, const std::string& option)
{
    try {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value)) {
            throw std::runtime_error("not a finite number");
        }
        return value;
    } catch (...) {
        throw std::runtime_error(option + " requires a finite number: " + text);
    }
}

ProgramOptions parse_options(const int argc, char** argv)
{
    // 인자를 허용 목록 방식으로 처리합니다. 오타가 있는 옵션을 조용히 무시하면
    // 의도와 다른 시험이 비행할 수 있으므로 즉시 예외를 던집니다.
    ProgramOptions options{};
    for (int i = 1; i < argc; ++i) {
        const std::string argument{argv[i]};
        if (argument == "--execute") {
            options.execute = true;
        } else if (argument == "--serve") {
            options.serve = true;
        } else if (argument == "--list") {
            options.list = true;
        } else if (argument == "--plan-only") {
            options.plan_only = true;
        } else if (argument == "--test" && i + 1 < argc) {
            options.test_id = parse_integer(argv[++i], "--test");
        } else if (argument == "--repeat" && i + 1 < argc) {
            options.repeat_override = parse_integer(argv[++i], "--repeat");
            if (options.repeat_override <= 0) {
                throw std::runtime_error("--repeat must be positive");
            }
        } else if (argument == "--connection" && i + 1 < argc) {
            options.connection_url = argv[++i];
        } else if (argument == "--plan-home" && i + 2 < argc) {
            riposte_test::GeoPoint home{};
            home.latitude_deg = parse_double(argv[++i], "--plan-home");
            home.longitude_deg = parse_double(argv[++i], "--plan-home");
            if (!riposte_test::geo_point_valid(home)) {
                throw std::runtime_error("--plan-home is out of range");
            }
            options.plan_home = home;
        } else if (argument == "--help" || argument == "-h") {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::runtime_error("Unknown or incomplete argument: " + argument);
        }
    }
    return options;
}

std::string now_for_filename()
{
    // 로그 파일명은 UTC 를 씁니다. Raspberry Pi 와 PX4 ULog 의 시각을 비교할 때
    // 지역 시간대 변경의 영향을 받지 않도록 하기 위함입니다.
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &time);
#else
    gmtime_r(&time, &utc);
#endif
    std::ostringstream output;
    output << std::put_time(&utc, "%Y%m%dT%H%M%SZ");
    return output.str();
}

void require_telemetry_success(const mavsdk::Telemetry::Result result, const std::string& operation)
{
    if (result != mavsdk::Telemetry::Result::Success) {
        std::ostringstream message;
        message << operation << " failed, MAVSDK Telemetry result=" << result;
        throw std::runtime_error(message.str());
    }
}

/*
 * 시험 로그 (20 Hz)
 * ----------------------------------------------------------------------------
 * 기존 CsvLogger 와 같은 역할이되, test_id 와 iteration 열을 추가했습니다.
 * 반복 시험에서 "3회차 호버가 유독 나빴다" 같은 분석을 하려면 사이클을 구분할
 * 열이 반드시 있어야 합니다.
 *
 * 이 CSV 는 "companion 이 무엇을 지시했고 기체가 어떻게 따라왔는가"만 담습니다.
 * 자세/진동/EKF/actuator 는 PX4 ULog 를 함께 보십시오.
 */
class TestCsvLogger {
public:
    explicit TestCsvLogger(const int test_id)
    {
        std::filesystem::create_directories("logs");
        path_ = "logs/test" + std::to_string(test_id) + "_" + now_for_filename() + ".csv";
        stream_.open(path_);
        if (!stream_) {
            throw std::runtime_error("Cannot open log file: " + path_);
        }
        stream_ << "elapsed_s,test_id,iteration,phase,"
                   "target_n_m,target_e_m,target_d_m,"
                   "actual_n_m,actual_e_m,actual_d_m,"
                   "horizontal_error_m,altitude_error_m,battery_remaining_percent\n";
        stream_.flush();
        if (!stream_) {
            throw std::runtime_error("Cannot write CSV header: " + path_);
        }
        test_id_ = test_id;
        start_ = std::chrono::steady_clock::now();
        last_flush_ = start_;
    }

    ~TestCsvLogger()
    {
        // ofstream 소멸자도 close 를 하지만, 명시적으로 flush/close 하여
        // 정상 종료와 예외 종료 모두에서 마지막 로그를 최대한 보존합니다.
        flush();
        if (stream_.is_open()) {
            stream_.close();
        }
    }

    void set_iteration(const int iteration) { iteration_ = iteration; }

    // battery_remaining 은 0~1 비율(음수 = 텔레메트리 무효)입니다.
    void write(
        const std::string& phase,
        const TrajectorySetpoint& target,
        const mavsdk::Telemetry::PositionVelocityNed& actual,
        const float battery_remaining)
    {
        // 이전 실패로 로깅이 비활성화되었으면 즉시 반환합니다(비행은 계속).
        if (!stream_.is_open()) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start_).count();
        const NedPoint actual_point{
            actual.position.north_m,
            actual.position.east_m,
            actual.position.down_m,
        };
        const double horizontal_error_m =
            bullet_mission::horizontal_distance(target.position, actual_point);
        // Down 축 부호를 그대로 두면 "고도 오차"의 방향이 직관과 반대가 됩니다.
        // 양수 = 목표보다 높이 떠 있음, 이 되도록 부호를 뒤집습니다.
        const double altitude_error_m = target.position.down_m - actual_point.down_m;

        stream_ << std::fixed << std::setprecision(3)
                << elapsed << ',' << test_id_ << ',' << iteration_ << ',' << phase << ','
                << target.position.north_m << ',' << target.position.east_m << ','
                << target.position.down_m << ',' << actual.position.north_m << ','
                << actual.position.east_m << ',' << actual.position.down_m << ','
                << horizontal_error_m << ',' << altitude_error_m << ','
                << (battery_remaining >= 0.0F ? battery_remaining * 100.0F : -1.0F) << '\n';

        // 매 20 Hz 마다 flush 하면 저장장치 I/O 가 제어 루프를 밀어냅니다.
        // 단계 전환 시와 1초마다만 flush 하여 크래시 시 유실 구간을 제한합니다.
        const bool phase_changed = phase != last_phase_;
        const bool interval_elapsed =
            std::chrono::duration<double>(now - last_flush_).count()
            >= static_cast<double>(TestTuning::log_flush_interval_s);
        if (phase_changed || interval_elapsed) {
            flush();
            last_flush_ = now;
        }
        if (!stream_) {
            // 로그 I/O 실패는 비행 중단 사유가 아닙니다. 비활성화하고 계속 비행합니다.
            std::cerr << "[LOG] CSV write/flush failed - logging disabled, test continues: "
                      << path_ << '\n';
            stream_.close();
            return;
        }
        last_phase_ = phase;
    }

    const std::string& path() const { return path_; }

    void flush() noexcept
    {
        if (stream_.is_open()) {
            stream_.flush();
            if (!stream_) {
                std::cerr << "[LOG] CSV flush failed: " << path_ << '\n';
            }
        }
    }

private:
    std::ofstream stream_{};
    std::string path_{};
    int test_id_{};
    int iteration_{};
    std::chrono::steady_clock::time_point start_{};
    std::chrono::steady_clock::time_point last_flush_{};
    std::string last_phase_{};
};

/*
 * TestRunner
 * ----------------------------------------------------------------------------
 * 기존 MissionRunner 의 구조를 그대로 유지합니다. 각 단계는 실패 시 예외를
 * 던지고, main() 의 catch 가 emergency_land_if_needed() 를 호출합니다.
 */
riposte_runtime::OffboardControlConfig test_offboard_control_config()
{
    riposte_runtime::OffboardControlConfig config{};
    config.setpoint_rate_hz = static_cast<double>(TestTuning::setpoint_rate_hz);
    config.setpoint_watchdog_timeout_s =
        static_cast<double>(TestTuning::setpoint_watchdog_timeout_s);
    config.setpoint_watchdog_poll_s =
        static_cast<double>(TestTuning::setpoint_watchdog_poll_s);
    config.watchdog_owner_grace_s = 0.5;
    config.offboard_mode_confirmation_s = 1.5;
    config.action_takeoff_min_altitude_m =
        static_cast<double>(TestTuning::action_takeoff_min_altitude_m);
    config.offboard_climb_speed_m_s =
        static_cast<double>(TestTuning::offboard_climb_speed_m_s);
    config.offboard_prime_seconds = 1.0;
    config.offboard_prime_retry_increment_s = 0.35;
    config.offboard_start_attempts = 3U;
    config.takeoff_altitude_tolerance_m = 0.5;
    config.takeoff_vertical_speed_tolerance_m_s = 0.5;
    config.takeoff_settle_s = 1.0;
    config.takeoff_timeout_s = static_cast<double>(TestTuning::takeoff_timeout_s);
    config.landing_timeout_s = static_cast<double>(TestTuning::landing_timeout_s);
    config.rtl_timeout_s = static_cast<double>(TestTuning::landing_timeout_s) * 2.0;
    config.command_poll_s = 0.1;
    config.disarm_timeout_s = 10.0;
    return config;
}

class TestRunner {
public:
    TestRunner(
        std::shared_ptr<mavsdk::System> system,
        mavsdk::Action& action,
        mavsdk::Telemetry& telemetry,
        TestCsvLogger& logger)
        : telemetry_(telemetry)
        , logger_(logger)
        , flight_control_(
              std::move(system),
              action,
              telemetry,
              test_offboard_control_config(),
              [] { return g_stop_requested.load(); },
              [](const std::string&) { g_stop_requested.store(true); },
              [&logger](
                  const std::string& phase,
                  const TrajectorySetpoint& target,
                  const mavsdk::Telemetry::PositionVelocityNed& actual,
                  const float battery_fraction) {
                  logger.write(phase, target, actual, battery_fraction);
              },
              [&logger] { logger.flush(); })
    {
    }

    ~TestRunner() = default;
    // --serve 모드에서 GCS 로 진행 상황을 중계할 때 씁니다. 시험 시작과 반복
    // 1회 종료 시점에 호출됩니다. 비행 로직에는 관여하지 않습니다.
    void set_progress_callback(std::function<void(const std::string&)> callback)
    {
        progress_ = std::move(callback);
    }

    riposte_test::TestReport run(const riposte_test::TestCase& test_case, const int repeat)
    {
        riposte_test::TestReport report{};
        report.test_id = test_case.id;
        report.test_name = test_case.name;
        report.planned_iterations = repeat;

        validate_case(test_case, repeat);
        notify_progress(riposte_test::gcs::start_text(test_case.id, repeat));
        wait_for_vehicle_health();
        check_preflight_battery();

        for (int iteration = 1; iteration <= repeat; ++iteration) {
            // 반복 시험은 매 사이클 이륙하므로 사이클마다 배터리를 다시 봅니다.
            // 10회 반복 도중 배터리가 마르면 여기서 조기 종료합니다.
            const double battery_percent = battery_remaining_fraction();
            if (battery_percent >= 0.0
                && battery_percent
                    < static_cast<double>(TestTuning::iteration_min_battery_fraction)) {
                std::ostringstream reason;
                reason << "battery " << std::fixed << std::setprecision(1)
                       << battery_percent * 100.0 << "% below iteration gate "
                       << TestTuning::iteration_min_battery_fraction * 100.0F
                       << "% before iteration " << iteration;
                report.abort_reason = reason.str();
                std::cerr << "[SAFETY] " << report.abort_reason << ". Stopping test.\n";
                break;
            }

            std::cout << "\n===== Test " << test_case.id << " iteration " << iteration << '/'
                      << repeat << " =====\n";
            logger_.set_iteration(iteration);
            iteration_ = iteration;

            // 사이클마다 배터리 조기복귀 래치를 초기화합니다. 이전 사이클에서
            // 걸린 래치가 남아 있으면 다음 사이클을 이유 없이 건너뜁니다.
            battery_return_requested_ = false;

            riposte_test::IterationResult result{};
            if (test_case.kind == riposte_test::TestKind::HoverCycle) {
                result = run_hover_cycle(test_case, iteration);
            } else {
                result = run_bullet_path(test_case, iteration);
            }
            report.iterations.push_back(result);
            notify_progress(riposte_test::gcs::iteration_text(
                test_case.id, iteration, repeat, result.passed));

            std::cout << "  -> iteration " << iteration << ": "
                      << (result.passed ? "PASS" : "FAIL");
            if (!result.note.empty()) {
                std::cout << " (" << result.note << ')';
            }
            std::cout << '\n';

            if (battery_return_requested_) {
                report.abort_reason = "battery reached in-flight return threshold";
                std::cerr << "[SAFETY] " << report.abort_reason << ". Stopping test.\n";
                break;
            }
            if (iteration < repeat) {
                pause_between_iterations();
            }
        }
        return report;
    }

    void emergency_land_if_needed() noexcept
    {
        flight_control_.emergency_land_and_wait();
    }

private:
    void notify_progress(const std::string& text) const
    {
        if (progress_) {
            progress_(text);
        }
    }

    // ---- 계획 검증 ---------------------------------------------------------

    void validate_case(const riposte_test::TestCase& test_case, const int repeat) const
    {
        // PX4 Offboard 최소 요구(2 Hz)보다 충분한 여유를 확보합니다.
        if (TestTuning::setpoint_rate_hz < 10.0F) {
            throw std::runtime_error("Setpoint rate must be at least 10 Hz");
        }
        if (TestTuning::setpoint_watchdog_timeout_s <= 2.0F / TestTuning::setpoint_rate_hz) {
            throw std::runtime_error("Setpoint watchdog timeout is too short for configured rate");
        }
        if (repeat <= 0) {
            throw std::runtime_error("Repeat count must be positive");
        }
        if (TestTuning::inflight_return_battery_fraction
            >= TestTuning::preflight_min_battery_fraction) {
            throw std::runtime_error("Battery thresholds are inconsistent");
        }

        if (test_case.kind == riposte_test::TestKind::HoverCycle) {
            const auto& hover = test_case.hover;
            if (hover.altitude_m <= 0.0F || hover.hover_s < 0.0F || hover.leg_speed_m_s <= 0.0F
                || hover.leg_distance_m <= 0.0F) {
                throw std::runtime_error("Hover cycle parameters are invalid");
            }
            // 1 m 를 0.3 m/s 로 가려면 3.3 초입니다. arrival_timeout 안에 못 끝나는
            // 조합이면 시험이 아니라 설정 실수입니다.
            const double leg_duration_s =
                static_cast<double>(hover.leg_distance_m) / static_cast<double>(hover.leg_speed_m_s);
            if (leg_duration_s >= static_cast<double>(TestTuning::arrival_timeout_s)) {
                throw std::runtime_error("Leg duration exceeds arrival timeout");
            }
        } else {
            const auto& bullet = test_case.bullet;
            if (bullet.altitude_m <= 0.0F || bullet.laps <= 0 || bullet.cruise_speed_m_s <= 0.0F
                || bullet.corner_speed_m_s <= 0.0F || bullet.accel_m_s2 <= 0.0F) {
                throw std::runtime_error("Bullet path parameters are invalid");
            }
            if (bullet.corner_speed_m_s > bullet.cruise_speed_m_s) {
                throw std::runtime_error("Corner speed must not exceed cruise speed");
            }
            const riposte_test::GeoPoint target{
                bullet.target_latitude_deg, bullet.target_longitude_deg};
            if (!riposte_test::geo_point_valid(target)) {
                throw std::runtime_error("Bullet path target coordinate is out of range");
            }
            // S 는 실행 시각에 정해지므로 여기서는 S 와 무관한 부분만 봅니다.
            // L > d 조건은 이륙 후 실제 S 를 알게 된 시점에 검사합니다.
            const double width_m =
                static_cast<double>(bullet.width_left_m) + static_cast<double>(bullet.width_right_m);
            const double long_side_m =
                static_cast<double>(bullet.total_distance_m) / 2.0 - width_m;
            if (long_side_m <= 0.0) {
                throw std::runtime_error("D is too small for the requested width (L <= 0)");
            }
        }
        std::cout << "Plan validated for test " << test_case.id << " (" << test_case.name
                  << "), repeat=" << repeat << '\n';
    }

    // ---- 사전 점검 ---------------------------------------------------------

    void wait_for_vehicle_health()
    {
        // timeout 이면 코드를 고치지 말고 QGroundControl 의 preflight 메시지를
        // 확인하십시오: GPS fix, home position, compass calibration, EKF innovation,
        // battery warning, safety switch, airframe/센서 calibration.
        std::cout << "Waiting for PX4 health, GNSS and home position...\n";
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(TestTuning::health_timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            check_stop_requested();
            if (telemetry_.health_all_ok()) {
                return;
            }
            std::this_thread::sleep_for(500ms);
        }
        throw std::runtime_error("Vehicle health check timed out");
    }

    // 배터리 잔량 비율(0~1)입니다. 유효하지 않으면 음수를 반환합니다.
    //
    // 주의: MAVSDK v3 의 remaining_percent 는 이름 그대로 0~100 "퍼센트"입니다.
    // (v1/v2 는 0~1 비율이었고 기존 미션 코드는 그 가정으로 작성되어 있었음.)
    // v3.17.1 + PX4 SITL 실측에서 100.0 이 왔고, 0~1 가정이면 이 값을 무효로
    // 버려 배터리 게이트 전체가 무력화됩니다. 그래서 여기 한 곳에서 비율로
    // 정규화하고, 다른 코드는 전부 이 함수만 쓰게 합니다.
    double battery_remaining_fraction() const
    {
        const float remaining = telemetry_.battery().remaining_percent;
        if (!std::isfinite(remaining) || remaining < 0.0F || remaining > 100.0F) {
            return -1.0;
        }
        return static_cast<double>(remaining) / 100.0;
    }

    // 보고용 배터리 % 입니다. 텔레메트리가 유효하지 않으면 IterationResult 의
    // 기본값과 같은 -1 을 반환하여, 요약 CSV 에 -100 처럼 실제 측정값으로 오해할
    // 수 있는 수치가 남지 않게 합니다.
    double battery_percent_for_report() const
    {
        const double fraction = battery_remaining_fraction();
        return fraction >= 0.0 ? fraction * 100.0 : -1.0;
    }

    void check_preflight_battery()
    {
        // 최초 Battery 메시지가 오기 전에는 remaining_percent 가 NaN 일 수 있으므로
        // 유효한 0~1 값을 받을 때까지 잠깐 기다립니다. 미수신을 100% 로 간주하면 안 됩니다.
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(TestTuning::battery_telemetry_timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            check_stop_requested();
            const double remaining = battery_remaining_fraction();
            if (remaining >= 0.0) {
                if (remaining < static_cast<double>(TestTuning::preflight_min_battery_fraction)) {
                    std::ostringstream message;
                    message << "Preflight battery " << std::fixed << std::setprecision(1)
                            << remaining * 100.0 << "% is below required "
                            << TestTuning::preflight_min_battery_fraction * 100.0F << '%';
                    throw std::runtime_error(message.str());
                }
                std::cout << "Preflight battery check passed: " << std::fixed
                          << std::setprecision(1) << remaining * 100.0 << "%\n";
                return;
            }
            std::this_thread::sleep_for(100ms);
        }
        throw std::runtime_error("Valid battery telemetry was not received before timeout");
    }

    mavsdk::Telemetry::PositionVelocityNed wait_for_valid_local_position()
    {
        // health_all_ok 직후에도 첫 LOCAL_POSITION_NED 샘플이 아직 도착하지 않았을
        // 수 있습니다. MAVSDK 의 초기값/미수신 값이 NaN 이면 기준점으로 저장하지 않습니다.
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(TestTuning::local_position_timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            check_stop_requested();
            const auto state = telemetry_.position_velocity_ned();
            if (std::isfinite(state.position.north_m) && std::isfinite(state.position.east_m)
                && std::isfinite(state.position.down_m)) {
                return state;
            }
            std::this_thread::sleep_for(100ms);
        }
        throw std::runtime_error("Local NED position is not valid");
    }

    // 위/경도 텔레메트리가 유효해질 때까지 기다립니다. MAVSDK 의 position() 캐시는
    // 첫 GLOBAL_POSITION_INT 수신 전에는 NaN 이라, health_all_ok 직후 곧바로
    // 읽으면 유효하지 않을 수 있습니다 (PX4 SITL 배치에서 Test 21/22 가 시작
    // 1초 만에 실패한 실측 사례). local position 대기와 같은 정책을 씁니다.
    riposte_test::GeoPoint wait_for_valid_global_position()
    {
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(TestTuning::local_position_timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            check_stop_requested();
            const auto global = telemetry_.position();
            const riposte_test::GeoPoint point{global.latitude_deg, global.longitude_deg};
            if (riposte_test::geo_point_valid(point)) {
                return point;
            }
            std::this_thread::sleep_for(100ms);
        }
        throw std::runtime_error("Global position is not valid; cannot resolve the target");
    }

    // ---- 사이클 -------------------------------------------------------------

    riposte_test::IterationResult run_hover_cycle(
        const riposte_test::TestCase& test_case, const int iteration)
    {
        const auto& spec = test_case.hover;
        const double altitude_m = static_cast<double>(spec.altitude_m);
        const auto cycle_start = std::chrono::steady_clock::now();

        riposte_test::IterationResult result{};
        result.iteration = iteration;
        result.commanded_altitude_m = altitude_m;
        result.battery_start_percent = battery_percent_for_report();

        // 이륙 전 local NED 위치를 기준점으로 저장합니다. PX4 local origin 은
        // 항상 (0,0,0) 이라고 가정하면 안 됩니다. 사이클마다 다시 읽는 이유는
        // 반복 중 EKF 가 재수렴하며 원점이 미세하게 이동할 수 있기 때문입니다.
        const auto ground_state = wait_for_valid_local_position();
        ground_reference_ = {
            ground_state.position.north_m,
            ground_state.position.east_m,
            ground_state.position.down_m,
        };
        radius_limit_m_ = static_cast<double>(TestTuning::hover_radius_limit_m);

        // NED 에서 Down 은 아래가 양수입니다. 지상 down=+0.2 에서 0.5 m 상승 목표는
        // +0.2-0.5 = -0.3 이 됩니다.
        const NedPoint hover_point{
            ground_reference_.north_m,
            ground_reference_.east_m,
            ground_reference_.down_m - altitude_m,
        };

        std::cout << "[1] Take off to " << std::fixed << std::setprecision(2) << altitude_m
                  << " m\n";
        takeoff_to(ground_reference_, altitude_m, "TAKEOFF");
        settle_at("SETTLE_AT_ALTITUDE", hover_point, 0.0, altitude_m);

        if (spec.north_south_leg && !battery_return_requested_) {
            const double leg_m = static_cast<double>(spec.leg_distance_m);
            const double leg_speed_m_s = static_cast<double>(spec.leg_speed_m_s);
            const NedPoint north_point{
                hover_point.north_m + leg_m, hover_point.east_m, hover_point.down_m};

            std::cout << "[2] Move " << leg_m << " m north at " << leg_speed_m_s << " m/s\n";
            move_line("MOVE_NORTH", hover_point, north_point, leg_speed_m_s);
            settle_at("SETTLE_NORTH", north_point,
                      bullet_mission::line_heading_deg(hover_point, north_point), altitude_m);

            std::cout << "[3] Move " << leg_m << " m south at " << leg_speed_m_s << " m/s\n";
            move_line("MOVE_SOUTH", north_point, hover_point, leg_speed_m_s);
            settle_at("SETTLE_SOUTH", hover_point,
                      bullet_mission::line_heading_deg(north_point, hover_point), altitude_m);
        }

        std::cout << "[4] Hover " << spec.hover_s << " s\n";
        riposte_test::HoverAccumulator accumulator{};
        const double hover_duration_s = hold(
            "HOVER", hover_point, static_cast<double>(spec.hover_s), 0.0, &accumulator);

        std::cout << "[5] Land\n";
        land_and_wait();

        result.hover_duration_s = hover_duration_s;
        result.hover_samples = accumulator.samples();
        result.mean_altitude_error_m = accumulator.mean_altitude_error_m();
        result.max_altitude_error_m = accumulator.max_altitude_error_m();
        result.max_horizontal_error_m = accumulator.max_horizontal_error_m();
        result.battery_end_percent = battery_percent_for_report();
        result.cycle_duration_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - cycle_start).count();

        const auto criteria =
            riposte_test::hover_criteria_for(altitude_m, static_cast<double>(spec.hover_s));
        result.passed =
            riposte_test::evaluate_hover(accumulator, criteria, hover_duration_s, result.note);
        return result;
    }

    riposte_test::IterationResult run_bullet_path(
        const riposte_test::TestCase& test_case, const int iteration)
    {
        const auto& spec = test_case.bullet;
        const double altitude_m = static_cast<double>(spec.altitude_m);
        const auto cycle_start = std::chrono::steady_clock::now();

        riposte_test::IterationResult result{};
        result.iteration = iteration;
        result.commanded_altitude_m = altitude_m;
        result.battery_start_percent = battery_percent_for_report();

        const auto ground_state = wait_for_valid_local_position();
        ground_reference_ = {
            ground_state.position.north_m,
            ground_state.position.east_m,
            ground_state.position.down_m,
        };

        // 목표는 위/경도로 주어지므로 Local NED 로 옮겨야 합니다. PX4 의 local
        // origin 은 EKF origin 이라 이륙점과 정확히 같지 않습니다. 그래서 절대
        // 좌표를 만들지 않고, "지금 여기"의 위/경도를 원점으로 한 상대 오프셋을
        // 계산해 "지금 여기"의 local NED 에 더합니다.
        const riposte_test::GeoPoint home_geo = wait_for_valid_global_position();
        const riposte_test::GeoPoint target_geo{
            spec.target_latitude_deg, spec.target_longitude_deg};
        const auto offset = riposte_test::geodetic_to_ned_offset(home_geo, target_geo);

        const NedPoint start_point{
            ground_reference_.north_m,
            ground_reference_.east_m,
            ground_reference_.down_m - altitude_m,
        };
        const NedPoint target_point{
            ground_reference_.north_m + offset.north_m,
            ground_reference_.east_m + offset.east_m,
            start_point.down_m,
        };

        riposte_test::BulletPathInputs inputs{};
        inputs.start = start_point;
        inputs.target = target_point;
        inputs.width_left_m = static_cast<double>(spec.width_left_m);
        inputs.width_right_m = static_cast<double>(spec.width_right_m);
        inputs.total_distance_m = static_cast<double>(spec.total_distance_m);

        std::string reason;
        if (!riposte_test::validate_bullet_path(inputs, reason)) {
            // 대개 이륙 지점이 목표에서 너무 먼 경우입니다. 코드가 아니라
            // 이륙 위치나 D 를 조정해야 합니다.
            throw std::runtime_error("Bullet path is not valid from this takeoff point: " + reason);
        }
        const auto path = riposte_test::compute_bullet_path(inputs);
        print_bullet_path(path, inputs);

        // 산출된 경로에 맞춰 소프트웨어 반경 한계를 넓힙니다. 기존 60 m 고정값은
        // Test 22(원단점이 90 m) 에서 send_setpoint 가 전부 거부합니다.
        radius_limit_m_ = riposte_test::bullet_max_radius_m(ground_reference_, path)
            + static_cast<double>(TestTuning::bullet_radius_margin_m);
        std::cout << "  software radius limit for this test: " << std::fixed
                  << std::setprecision(1) << radius_limit_m_ << " m\n";

        std::cout << "[1] Take off to " << std::setprecision(2) << altitude_m << " m\n";
        takeoff_to(ground_reference_, altitude_m, "TAKEOFF");
        settle_at("SETTLE_AT_ALTITUDE", start_point, 0.0, altitude_m);

        const auto& waypoints = path.waypoints;
        const auto index = [](const riposte_test::BulletWaypoint waypoint) {
            return static_cast<std::size_t>(waypoint);
        };
        const NedPoint& start_left = waypoints[index(riposte_test::BulletWaypoint::StartLeft)];
        const NedPoint& far_left = waypoints[index(riposte_test::BulletWaypoint::FarLeft)];
        const NedPoint& far_right = waypoints[index(riposte_test::BulletWaypoint::FarRight)];
        const NedPoint& start_right = waypoints[index(riposte_test::BulletWaypoint::StartRight)];

        const double corner_speed_m_s = static_cast<double>(spec.corner_speed_m_s);
        riposte_test::SpeedProfile long_side_profile{};
        long_side_profile.entry_speed_m_s = corner_speed_m_s;
        long_side_profile.cruise_speed_m_s = static_cast<double>(spec.cruise_speed_m_s);
        long_side_profile.exit_speed_m_s = corner_speed_m_s;
        long_side_profile.accel_m_s2 = static_cast<double>(spec.accel_m_s2);

        // 코너 settle 의 yaw 는 도착 직전 구간의 진행방향을 유지합니다. 0(북쪽)을
        // 명령하면 코너마다 기수가 북쪽으로 돌아갔다가 다음 구간 시작에서 다시
        // 돌아오는 불필요한 회전이 생깁니다. (trajectory.hpp 의 line_setpoint 가
        // 구간 종료 시 진행방향 yaw 를 유지하도록 고친 것과 같은 취지입니다.)
        const double enter_heading_deg =
            bullet_mission::line_heading_deg(start_point, start_left);
        const double left_side_heading_deg =
            bullet_mission::line_heading_deg(start_left, far_left);
        const double far_side_heading_deg =
            bullet_mission::line_heading_deg(far_left, far_right);
        const double right_side_heading_deg =
            bullet_mission::line_heading_deg(far_right, start_right);
        const double home_side_heading_deg =
            bullet_mission::line_heading_deg(start_right, start_left);
        const double return_heading_deg =
            bullet_mission::line_heading_deg(start_left, start_point);

        // S 에서 첫 필수 통과점 S_L 로 진입합니다.
        std::cout << "[2] Enter the loop at S_L\n";
        move_line("ENTER_S_L", start_point, start_left, corner_speed_m_s);
        settle_at("SETTLE_S_L", start_left, enter_heading_deg, altitude_m);

        for (int lap = 1; lap <= spec.laps && !battery_return_requested_; ++lap) {
            const std::string tag = "LAP" + std::to_string(lap) + '_';
            std::cout << "[3] Lap " << lap << '/' << spec.laps << '\n';

            // 좌측 장변: S_L -> (T_L 통과) -> F_L. T_L 에서 정지하지 않습니다(§7.7).
            move_line_profiled(tag + "SL_TL", tag + "TL_FL", start_left, far_left,
                               long_side_profile, path.target_distance_m);
            settle_at(tag + "SETTLE_F_L", far_left, left_side_heading_deg, altitude_m);

            // 원단측 단변: 길이 W 로 짧아 서행 유지.
            move_line(tag + "FL_FR", far_left, far_right, corner_speed_m_s);
            settle_at(tag + "SETTLE_F_R", far_right, far_side_heading_deg, altitude_m);

            // 우측 장변: F_R -> (T_R 통과) -> S_R.
            move_line_profiled(tag + "FR_TR", tag + "TR_SR", far_right, start_right,
                               long_side_profile, path.long_side_m - path.target_distance_m);
            settle_at(tag + "SETTLE_S_R", start_right, right_side_heading_deg, altitude_m);

            // 출발측 단변으로 순환을 닫습니다.
            move_line(tag + "SR_SL", start_right, start_left, corner_speed_m_s);
            settle_at(tag + "SETTLE_S_L", start_left, home_side_heading_deg, altitude_m);
        }

        std::cout << "[4] Return to the takeoff point and hold\n";
        move_line("RETURN_TO_START", start_left, start_point, corner_speed_m_s);
        settle_at("SETTLE_AT_START", start_point, return_heading_deg, altitude_m);

        riposte_test::HoverAccumulator accumulator{};
        const double hover_duration_s =
            hold("HOVER", start_point, 5.0, return_heading_deg, &accumulator);

        std::cout << "[5] Land\n";
        land_and_wait();

        result.hover_duration_s = hover_duration_s;
        result.hover_samples = accumulator.samples();
        result.mean_altitude_error_m = accumulator.mean_altitude_error_m();
        result.max_altitude_error_m = accumulator.max_altitude_error_m();
        result.max_horizontal_error_m = accumulator.max_horizontal_error_m();
        result.battery_end_percent = battery_percent_for_report();
        result.cycle_duration_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - cycle_start).count();

        const auto criteria = riposte_test::hover_criteria_for(altitude_m, 5.0);
        result.passed =
            riposte_test::evaluate_hover(accumulator, criteria, hover_duration_s, result.note);
        if (battery_return_requested_ && result.passed) {
            result.passed = false;
            result.note = "battery aborted the loop before all laps finished";
        }
        return result;
    }

    static void print_bullet_path(
        const riposte_test::BulletPathResult& path, const riposte_test::BulletPathInputs& inputs)
    {
        std::cout << std::fixed << std::setprecision(2)
                  << "  Bullet path computed from D=" << inputs.total_distance_m
                  << " m, wL=" << inputs.width_left_m << " m, wR=" << inputs.width_right_m
                  << " m\n"
                  << "    d (start->target) = " << path.target_distance_m << " m\n"
                  << "    W (short side)    = " << path.width_m << " m\n"
                  << "    L (long side)     = " << path.long_side_m << " m\n"
                  << "    perimeter         = " << path.perimeter_m << " m (input D="
                  << inputs.total_distance_m << " m)\n"
                  << "    overshoot (L-d)   = " << path.overshoot_m << " m of clear airspace"
                  << " needed beyond the target\n"
                  << "    heading           = " << path.heading_deg << " deg\n"
                  << "    mandatory waypoints (local NED, north/east):\n";
        for (std::size_t i = 0; i < riposte_test::kBulletWaypointCount; ++i) {
            const auto waypoint = static_cast<riposte_test::BulletWaypoint>(i);
            std::cout << "      " << (i + 1) << ") "
                      << riposte_test::bullet_waypoint_name(waypoint) << "  ("
                      << path.waypoints[i].north_m << ", " << path.waypoints[i].east_m << ")  "
                      << (riposte_test::is_bullet_corner(waypoint) ? "corner (slow)"
                                                                   : "pass-through")
                      << '\n';
        }
    }

    void pause_between_iterations()
    {
        // 착륙 후 다음 이륙까지의 대기입니다. 모터 열, EKF 재수렴, 육안 점검 시간.
        std::cout << "  waiting " << TestTuning::inter_iteration_pause_s
                  << " s before the next iteration...\n";
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(TestTuning::inter_iteration_pause_s);
        while (std::chrono::steady_clock::now() < deadline) {
            check_stop_requested();
            std::this_thread::sleep_for(200ms);
        }
    }

    // ---- 이륙 ---------------------------------------------------------------

    void takeoff_to(const NedPoint& ground, const double altitude_m, const std::string& tag)
    {
        flight_control_.set_flight_volume(ground, radius_limit_m_);
        flight_control_.takeoff_to(ground, altitude_m, tag);
    }
    static TrajectorySetpoint stationary_setpoint(const NedPoint& point, const double yaw_deg)
    {
        // value-initialization({})으로 velocity 세 성분을 모두 0 으로 만듭니다.
        // 정지 구간에서 이전 이동의 feed-forward 속도가 남지 않게 하는 것이 중요합니다.
        TrajectorySetpoint setpoint{};
        setpoint.position = point;
        setpoint.yaw_deg = yaw_deg;
        return setpoint;
    }

    void check_stop_requested() const
    {
        if (flight_control_.watchdog_triggered()) {
            throw std::runtime_error("Setpoint generation watchdog expired");
        }
        if (g_stop_requested.load()) {
            throw std::runtime_error("Stop requested (SIGINT/SIGTERM or GCS abort)");
        }
        if (!flight_control_.connected()) {
            throw std::runtime_error("Vehicle heartbeat lost");
        }
    }
    mavsdk::Telemetry::PositionVelocityNed send_setpoint(
        const std::string& phase, const TrajectorySetpoint& target)
    {
        const auto actual = flight_control_.send_setpoint(phase, target);
        const double battery_fraction = battery_remaining_fraction();
        if (!battery_return_requested_ && battery_fraction >= 0.0
            && battery_fraction
                <= static_cast<double>(TestTuning::inflight_return_battery_fraction)) {
            battery_return_requested_ = true;
            std::cerr << "[SAFETY] Battery reached " << std::fixed << std::setprecision(1)
                      << battery_fraction * 100.0
                      << "%. Landing at the next safe transition.\n";
            logger_.flush();
        }
        return actual;
    }
    double hold(
        const std::string& phase,
        const NedPoint& point,
        const double duration_s,
        const double yaw_deg,
        riposte_test::HoverAccumulator* accumulator = nullptr)
    {
        // sleep_for(period) 를 반복하면 처리시간만큼 주기가 계속 늘어집니다.
        // sleep_until(next) 를 쓰면 각 반복의 실행시간이 누적 드리프트로 이어지지 않습니다.
        const auto period =
            std::chrono::duration<double>(1.0 / static_cast<double>(TestTuning::setpoint_rate_hz));
        const auto start = std::chrono::steady_clock::now();
        auto next = start;
        const auto target = stationary_setpoint(point, yaw_deg);

        double elapsed_s = 0.0;
        while (elapsed_s < duration_s) {
            const auto actual = send_setpoint(phase, target);
            if (accumulator != nullptr) {
                const NedPoint actual_point{
                    actual.position.north_m, actual.position.east_m, actual.position.down_m};
                accumulator->add(
                    point.down_m - actual_point.down_m,
                    bullet_mission::horizontal_distance(point, actual_point));
            }
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next);
            elapsed_s =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        }
        return elapsed_s;
    }

    void move_line(
        const std::string& phase,
        const NedPoint& start,
        const NedPoint& finish,
        const double speed_m_s)
    {
        // 직선 구간 시간 = 수평거리 / 속도입니다. trajectory.hpp 의 line_setpoint() 가
        // 경과시간으로 progress 를 만들고 위치 보간과 속도 feed-forward 를 생성합니다.
        const double distance_m = bullet_mission::horizontal_distance(start, finish);
        if (distance_m <= 1.0e-6) {
            send_setpoint(phase, stationary_setpoint(
                finish, bullet_mission::line_heading_deg(start, finish)));
            return;
        }
        const double duration_s = distance_m / speed_m_s;
        const auto period =
            std::chrono::duration<double>(1.0 / static_cast<double>(TestTuning::setpoint_rate_hz));
        const auto begin = std::chrono::steady_clock::now();
        auto next = begin;

        while (true) {
            const double elapsed_s =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
            const double progress = elapsed_s / duration_s;
            if (progress >= 1.0) {
                break;
            }
            send_setpoint(
                phase, bullet_mission::line_setpoint(start, finish, progress, speed_m_s));
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next);
        }
        // 루프의 마지막 반복이 progress<1 에서 끝나므로, 정확한 끝점을 한 번 더
        // 보내고 velocity=0 으로 전환합니다. 이후 settle_at() 이 도착을 검증합니다.
        send_setpoint(phase, stationary_setpoint(
            finish, bullet_mission::line_heading_deg(start, finish)));
    }

    /*
     * 사다리꼴 속도 프로파일 직선 이동 (POC §7.7)
     *
     * 등속인 move_line() 과 달리, 코너 서행 -> 장변 순항 -> 코너 서행을 만듭니다.
     * 시간이 아니라 "진행 거리 s" 를 상태로 두고 s += v(s)*dt 로 적분합니다.
     * 속도가 위치의 함수이므로 시간 기반 progress 로는 표현할 수 없기 때문입니다.
     *
     * mark_m 지점을 지나면 phase 라벨이 phase_before -> phase_after 로 바뀝니다.
     * T_L / T_R 통과 시각을 CSV 에 남기기 위한 것으로, 비행에는 영향이 없습니다.
     */
    void move_line_profiled(
        const std::string& phase_before,
        const std::string& phase_after,
        const NedPoint& start,
        const NedPoint& finish,
        const riposte_test::SpeedProfile& profile,
        const double mark_m)
    {
        const double segment_length_m = bullet_mission::horizontal_distance(start, finish);
        if (segment_length_m <= 1.0e-6) {
            send_setpoint(phase_after, stationary_setpoint(
                finish, bullet_mission::line_heading_deg(start, finish)));
            return;
        }
        const double dt_s = 1.0 / static_cast<double>(TestTuning::setpoint_rate_hz);
        const auto period = std::chrono::duration<double>(dt_s);
        auto next = std::chrono::steady_clock::now();

        // 적분이 어떤 이유로든 진행하지 못할 때를 대비한 상한입니다. 이상적인
        // 소요시간의 3배 + 10초를 넘으면 무언가 잘못된 것이므로 예외를 던집니다.
        const double nominal_duration_s =
            segment_length_m / std::max(0.1, profile.cruise_speed_m_s);
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(nominal_duration_s * 3.0 + 10.0);

        double travelled_m = 0.0;
        while (travelled_m < segment_length_m) {
            if (std::chrono::steady_clock::now() > deadline) {
                throw std::runtime_error("Profiled move timed out in phase: " + phase_before);
            }
            const double speed_m_s =
                riposte_test::trapezoid_speed_at(profile, travelled_m, segment_length_m);
            const double progress = travelled_m / segment_length_m;
            const std::string& phase =
                (mark_m >= 0.0 && travelled_m >= mark_m) ? phase_after : phase_before;
            send_setpoint(
                phase, bullet_mission::line_setpoint(start, finish, progress, speed_m_s));
            travelled_m += speed_m_s * dt_s;
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next);
        }
        send_setpoint(phase_after, stationary_setpoint(
            finish, bullet_mission::line_heading_deg(start, finish)));
    }

    // 순수 수직 이동입니다. 저고도 Offboard 이륙에 사용합니다.
    void settle_at(
        const std::string& phase,
        const NedPoint& target,
        const double yaw_deg,
        const double altitude_m)
    {
        // "setpoint 를 보냈다"와 "기체가 도착했다"는 다릅니다. 세 조건을 모두
        // 만족해야 합니다.
        //   1) 수평 위치오차 <= horizontal_tolerance_m
        //   2) 고도오차      <= vertical_tolerance_for(altitude)
        //   3) 수평속도      <= arrival_speed_tolerance_m_s
        // 한 샘플만 만족해서 다음 단계로 넘어가지 않도록 settle 시간 동안 유지합니다.
        const double vertical_tolerance_m = riposte_test::vertical_tolerance_for(altitude_m);
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration<double>(TestTuning::arrival_timeout_s);
        const auto period =
            std::chrono::duration<double>(1.0 / static_cast<double>(TestTuning::setpoint_rate_hz));
        std::optional<std::chrono::steady_clock::time_point> within_tolerance_since;
        auto next = std::chrono::steady_clock::now();
        const auto setpoint = stationary_setpoint(target, yaw_deg);

        while (std::chrono::steady_clock::now() < deadline) {
            const auto state = send_setpoint(phase, setpoint);
            const NedPoint actual{
                state.position.north_m,
                state.position.east_m,
                state.position.down_m,
            };
            const bool horizontal_ok = bullet_mission::horizontal_distance(actual, target)
                <= static_cast<double>(TestTuning::horizontal_tolerance_m);
            const bool vertical_ok =
                std::abs(actual.down_m - target.down_m) <= vertical_tolerance_m;
            const bool speed_ok =
                std::hypot(state.velocity.north_m_s, state.velocity.east_m_s)
                <= TestTuning::arrival_speed_tolerance_m_s;

            if (horizontal_ok && vertical_ok && speed_ok) {
                if (!within_tolerance_since) {
                    within_tolerance_since = std::chrono::steady_clock::now();
                }
                const double settled_s = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - *within_tolerance_since).count();
                if (settled_s >= static_cast<double>(TestTuning::arrival_settle_s)) {
                    return;
                }
            } else {
                // 바람이나 overshoot 로 조건을 벗어나면 안정화 시간을 다시 셉니다.
                within_tolerance_since.reset();
            }
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next);
        }
        throw std::runtime_error("Arrival/settling timeout in phase: " + phase);
    }

    void land_and_wait()
    {
        flight_control_.land_and_wait();
    }
    mavsdk::Telemetry& telemetry_;
    TestCsvLogger& logger_;
    riposte_runtime::MavsdkOffboardControl flight_control_;
    std::function<void(const std::string&)> progress_{};
    NedPoint ground_reference_{};
    double radius_limit_m_{static_cast<double>(TestTuning::hover_radius_limit_m)};
    int iteration_{};
    bool battery_return_requested_{false};
};

// ---- 카탈로그 / 계획 출력 (기체 연결 불필요) --------------------------------

void print_catalog()
{
    std::cout << "ID  Description                        Altitude  N/S leg  Repeat\n"
              << "--  ---------------------------------  --------  -------  ------\n";
    for (const auto& test_case : riposte_test::kTestCatalog) {
        std::cout << std::setw(2) << test_case.id << "  " << std::left << std::setw(33)
                  << test_case.name << std::right;
        if (test_case.kind == riposte_test::TestKind::HoverCycle) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(2)
                      << test_case.hover.altitude_m << " m"
                      << std::setw(9) << (test_case.hover.north_south_leg ? "yes" : "no")
                      << std::setw(8) << test_case.hover.repeat << '\n';
        } else {
            std::cout << std::setw(7) << std::fixed << std::setprecision(2)
                      << test_case.bullet.altitude_m << " m" << std::setw(9) << "-"
                      << std::setw(8) << test_case.bullet.laps << "  (bullet path, D="
                      << test_case.bullet.total_distance_m << " m)\n";
        }
    }
}

void print_plan(const riposte_test::TestCase& test_case, const ProgramOptions& options)
{
    // 반복 규칙은 gcs_protocol.hpp 의 effective_repeat() 한 곳에만 둡니다.
    // (Bullet 시험의 기본 반복은 laps 가 아니라 1 — 단위시험으로 고정됨)
    const int repeat = riposte_test::gcs::effective_repeat(test_case, options.repeat_override);
    std::cout << "Test " << test_case.id << ": " << test_case.name << "\n\n";

    if (test_case.kind == riposte_test::TestKind::HoverCycle) {
        const auto& spec = test_case.hover;
        const double leg_duration_s = static_cast<double>(spec.leg_distance_m)
            / static_cast<double>(spec.leg_speed_m_s);
        const bool offboard_takeoff =
            spec.altitude_m < TestTuning::action_takeoff_min_altitude_m;
        std::cout << std::fixed << std::setprecision(2)
                  << "  altitude          : " << spec.altitude_m << " m\n"
                  << "  takeoff method    : "
                  << (offboard_takeoff ? "Offboard climb (below MIS_TAKEOFF_ALT floor)"
                                       : "PX4 Action::takeoff")
                  << '\n'
                  << "  hover             : " << spec.hover_s << " s\n"
                  << "  altitude tolerance: "
                  << riposte_test::vertical_tolerance_for(static_cast<double>(spec.altitude_m))
                  << " m\n"
                  << "  iterations        : " << repeat << '\n';
        if (spec.north_south_leg) {
            std::cout << "  north/south leg   : " << spec.leg_distance_m << " m each way at "
                      << spec.leg_speed_m_s << " m/s (" << leg_duration_s << " s per leg)\n";
        }
        // 사이클당 대략의 소요시간입니다. 이륙/착륙은 기체와 PX4 에 달려 있으므로
        // 어림값이며, 배터리 계획용 참고치입니다.
        const double climb_s = offboard_takeoff
            ? static_cast<double>(spec.altitude_m)
                / static_cast<double>(TestTuning::offboard_climb_speed_m_s)
            : static_cast<double>(spec.altitude_m) / 1.5;
        const double cycle_s = climb_s + 2.0
            + (spec.north_south_leg ? 2.0 * (leg_duration_s + 2.0) : 0.0)
            + static_cast<double>(spec.hover_s) + climb_s + 5.0;
        std::cout << "  rough cycle time  : " << cycle_s << " s\n"
                  << "  rough total time  : "
                  << cycle_s * repeat
                + static_cast<double>(TestTuning::inter_iteration_pause_s) * (repeat - 1)
                  << " s\n";
        return;
    }

    const auto& spec = test_case.bullet;
    const double width_m =
        static_cast<double>(spec.width_left_m) + static_cast<double>(spec.width_right_m);
    const double long_side_m = static_cast<double>(spec.total_distance_m) / 2.0 - width_m;
    std::cout << std::fixed << std::setprecision(2)
              << "  altitude       : " << spec.altitude_m << " m\n"
              << "  target         : " << std::setprecision(7) << spec.target_latitude_deg << ", "
              << spec.target_longitude_deg << '\n'
              << std::setprecision(2)
              << "  D (perimeter)  : " << spec.total_distance_m << " m\n"
              << "  wL / wR        : " << spec.width_left_m << " / " << spec.width_right_m
              << " m\n"
              << "  W (short side) : " << width_m << " m\n"
              << "  L (long side)  : " << long_side_m << " m\n"
              << "  laps per cycle : " << spec.laps << "  (path length per cycle "
              << static_cast<double>(spec.total_distance_m) * spec.laps << " m)\n"
              << "  iterations     : " << repeat << "  (arm/takeoff/land cycles)\n"
              << "  speeds         : corner " << spec.corner_speed_m_s << " m/s, cruise "
              << spec.cruise_speed_m_s << " m/s\n\n"
              << "  Constraint (POC 7.6): the takeoff point must lie within L = " << long_side_m
              << " m of the target,\n"
              << "  otherwise T_L / T_R fall outside the long sides and the 6-point"
                 " containment breaks.\n";

    if (!options.plan_home) {
        std::cout << "\n  Pass --plan-home <lat> <lon> to resolve the six waypoints"
                     " for a specific takeoff point.\n";
        return;
    }

    const riposte_test::GeoPoint target{spec.target_latitude_deg, spec.target_longitude_deg};
    const double distance_m = riposte_test::geodetic_distance_m(*options.plan_home, target);
    const auto offset = riposte_test::geodetic_to_ned_offset(*options.plan_home, target);

    // --plan-only 에서는 이륙점을 local NED 원점으로 두고 상대 좌표만 봅니다.
    riposte_test::BulletPathInputs inputs{};
    inputs.start = {0.0, 0.0, -static_cast<double>(spec.altitude_m)};
    inputs.target = {offset.north_m, offset.east_m, -static_cast<double>(spec.altitude_m)};
    inputs.width_left_m = static_cast<double>(spec.width_left_m);
    inputs.width_right_m = static_cast<double>(spec.width_right_m);
    inputs.total_distance_m = static_cast<double>(spec.total_distance_m);

    std::cout << "\n  With takeoff at " << std::setprecision(7)
              << options.plan_home->latitude_deg << ", " << options.plan_home->longitude_deg
              << ":\n"
              << std::setprecision(2) << "    d (start->target) = " << distance_m << " m\n";

    std::string reason;
    if (!riposte_test::validate_bullet_path(inputs, reason)) {
        std::cout << "    INVALID: " << reason << '\n';
        return;
    }
    const auto path = riposte_test::compute_bullet_path(inputs);
    std::cout << "    L = " << path.long_side_m << " m, overshoot beyond target = "
              << path.overshoot_m << " m, perimeter = " << path.perimeter_m << " m\n"
              << "    waypoints (north, east) relative to the takeoff point:\n";
    for (std::size_t i = 0; i < riposte_test::kBulletWaypointCount; ++i) {
        const auto waypoint = static_cast<riposte_test::BulletWaypoint>(i);
        std::cout << "      " << (i + 1) << ") " << riposte_test::bullet_waypoint_name(waypoint)
                  << "  (" << path.waypoints[i].north_m << ", " << path.waypoints[i].east_m
                  << ")  "
                  << (riposte_test::is_bullet_corner(waypoint) ? "corner (slow)" : "pass-through")
                  << '\n';
    }
}

// ---- 결과 요약 --------------------------------------------------------------

void write_summary(const riposte_test::TestReport& report, const std::string& csv_path)
{
    std::filesystem::create_directories("logs");
    const std::string path = "logs/test" + std::to_string(report.test_id) + "_summary_"
        + now_for_filename() + ".csv";
    std::ofstream stream(path);
    if (!stream) {
        std::cerr << "[LOG] Cannot write the summary file: " << path << '\n';
        return;
    }
    stream << "test_id,test_name,iteration,verdict,commanded_altitude_m,"
              "mean_altitude_error_m,max_altitude_error_m,max_horizontal_error_m,"
              "hover_duration_s,hover_samples,cycle_duration_s,"
              "battery_start_percent,battery_end_percent,note\n";
    for (const auto& iteration : report.iterations) {
        stream << report.test_id << ',' << report.test_name << ',' << iteration.iteration << ','
               << (iteration.passed ? "PASS" : "FAIL") << ',' << std::fixed
               << std::setprecision(3) << iteration.commanded_altitude_m << ','
               << iteration.mean_altitude_error_m << ',' << iteration.max_altitude_error_m << ','
               << iteration.max_horizontal_error_m << ',' << iteration.hover_duration_s << ','
               << iteration.hover_samples << ',' << iteration.cycle_duration_s << ','
               << iteration.battery_start_percent << ',' << iteration.battery_end_percent << ',';
        // note 에 쉼표가 들어가면 열이 밀리므로 큰따옴표로 감쌉니다.
        stream << '"' << iteration.note << "\"\n";
    }
    stream.flush();
    if (!stream) {
        std::cerr << "[LOG] Summary write failed: " << path << '\n';
        return;
    }
    std::cout << "Summary CSV: " << path << "\nDetailed CSV: " << csv_path << '\n';
}

void print_summary(const riposte_test::TestReport& report)
{
    std::cout << "\n================ Test " << report.test_id << " summary ================\n"
              << report.test_name << '\n'
              << "iterations: " << report.completed() << '/' << report.planned_iterations
              << ", passed: " << report.passed_count() << '\n';
    if (!report.abort_reason.empty()) {
        std::cout << "aborted: " << report.abort_reason << '\n';
    }
    std::cout << "\n  it  verdict  mean_alt_err  max_alt_err  max_horiz_err  hover_s\n"
              << "  --  -------  ------------  -----------  -------------  -------\n";
    for (const auto& iteration : report.iterations) {
        std::cout << "  " << std::setw(2) << iteration.iteration << "  " << std::setw(7)
                  << (iteration.passed ? "PASS" : "FAIL") << "  " << std::fixed
                  << std::setprecision(3) << std::setw(12) << iteration.mean_altitude_error_m
                  << "  " << std::setw(11) << iteration.max_altitude_error_m << "  "
                  << std::setw(13) << iteration.max_horizontal_error_m << "  " << std::setw(7)
                  << iteration.hover_duration_s << '\n';
        if (!iteration.note.empty()) {
            std::cout << "      note: " << iteration.note << '\n';
        }
    }
    std::cout << "\nOVERALL: " << (report.passed() ? "PASS" : "FAIL") << '\n'
              << "Attitude, vibration, EKF health and actuator margins are not covered here;\n"
              << "review the PX4 ULog for those.\n";
}

/*
 * ---- Ground Station 원격 요청 처리 (--serve) --------------------------------
 *
 * GCS(riposte_gcs)가 텔레메트리 링크로 보낸 COMMAND_LONG(MAV_CMD_USER_1) 을
 * 받아 카탈로그 시험을 실행합니다. 프로토콜 정의와 요청 검증은
 * include/gcs_protocol.hpp 에 있고 단위시험으로 고정되어 있습니다.
 *
 * 안전 정책:
 *   - --serve 는 --execute 와 함께 줄 때만 동작합니다. 원격으로 Arm 이 나가는
 *     모드이므로, 에이전트를 켜는 사람이 명시적으로 허용해야 합니다.
 *   - param3 매직 코드가 다르면 DENIED. 다른 GCS 소프트웨어의 MAV_CMD_USER_1
 *     오발송으로 시험이 시작되는 것을 막습니다.
 *   - 시험 중 새 요청은 TEMPORARILY_REJECTED(busy) 로 거절합니다.
 *   - MAV_CMD_USER_2 는 진행 중 시험만 중단(착륙)하고 대기를 계속합니다.
 *   - 그 밖의 오류(watchdog, health, 반경 초과 등)는 비상 착륙 후 에이전트를
 *     종료합니다. 사람이 원인을 확인하기 전에는 다음 원격 시험을 받지 않습니다.
 */

void send_agent_status(mavsdk::MavlinkPassthrough& passthrough, const std::string& text)
{
    const std::string clipped = riposte_test::gcs::clip_status_text(text);
    passthrough.queue_message([clipped](MavlinkAddress address, uint8_t channel) {
        mavlink_message_t message;
        char buffer[50] = {};
        std::snprintf(buffer, sizeof(buffer), "%s", clipped.c_str());
        mavlink_msg_statustext_pack_chan(
            address.system_id, address.component_id, channel, &message,
            MAV_SEVERITY_INFO, buffer, 0, 0);
        return message;
    });
}

void send_command_ack(
    mavsdk::MavlinkPassthrough& passthrough,
    const uint16_t command,
    const uint8_t result,
    const uint8_t to_system,
    const uint8_t to_component)
{
    passthrough.queue_message(
        [command, result, to_system, to_component](MavlinkAddress address, uint8_t channel) {
            mavlink_message_t message;
            mavlink_msg_command_ack_pack_chan(
                address.system_id, address.component_id, channel, &message,
                command, result, 0, 0, to_system, to_component);
            return message;
        });
}

int serve_requests(
    mavsdk::Mavsdk& mavsdk,
    const std::shared_ptr<mavsdk::System>& system,
    mavsdk::Action& action,
    mavsdk::Telemetry& telemetry)
{
    // 수신 스레드(MAVSDK 콜백)와 실행 루프(main) 사이의 공유 상태입니다.
    //
    // 선언 순서가 곧 안전 조건입니다: 아래 콜백이 이 상태들을 참조로 캡처하므로,
    // passthrough(구독 해제)가 "먼저" 소멸해야 합니다. C++ 는 선언 역순으로
    // 소멸하므로 상태를 passthrough 보다 앞에 선언합니다. 순서를 바꾸면 함수
    // 종료 순간에 도착한 메시지가 파괴된 mutex 를 잠그는 UB 가 됩니다.
    std::mutex request_mutex;
    std::optional<riposte_test::gcs::TestRequest> pending{};
    bool test_running = false;

    // 송신 전용입니다 (ACK, STATUSTEXT). PX4 가 대상/브로드캐스트에 맞춰 GCS
    // 링크로 포워딩합니다.
    mavsdk::MavlinkPassthrough passthrough{system};

    const auto handle_command = [&](const mavlink_message_t& message) {
            mavlink_command_long_t command;
            mavlink_msg_command_long_decode(&message, &command);
            // 이 컴포넌트(191) 앞으로 온 명령만 처리합니다. 브로드캐스트(0)는
            // 받지 않습니다. 원격 Arm 을 유발하는 명령이므로 명시적 지정을 요구합니다.
            //
            // system 은 두 값을 모두 허용합니다: MAVSDK companion 의 기본 sysid(1)와
            // 기체(autopilot)의 실제 sysid. 기체의 MAV_SYS_ID 를 1 이 아닌 값으로
            // 바꾼 구성에서 GCS 는 기체 sysid 로 보내는데, 이 에이전트의 자체
            // sysid 는 여전히 1 이라 한쪽만 검사하면 요청이 조용히 무시됩니다.
            const bool system_matches =
                command.target_system == passthrough.get_our_sysid()
                || command.target_system == passthrough.get_target_sysid();
            if (!system_matches
                || command.target_component != passthrough.get_our_compid()) {
                return;
            }

            if (command.command == riposte_test::gcs::kCommandRunTest) {
                std::string reason;
                const auto request = riposte_test::gcs::decode_request(
                    command.param1, command.param2, command.param3, reason);
                uint8_t ack_result;
                if (!request) {
                    ack_result = static_cast<uint8_t>(MAV_RESULT_DENIED);
                    std::cerr << "[SERVE] request denied: " << reason << '\n';
                } else {
                    const std::lock_guard<std::mutex> lock(request_mutex);
                    if (test_running || pending) {
                        ack_result = static_cast<uint8_t>(MAV_RESULT_TEMPORARILY_REJECTED);
                        std::cerr << "[SERVE] request rejected: a test is already running\n";
                    } else {
                        pending = *request;
                        ack_result = static_cast<uint8_t>(MAV_RESULT_ACCEPTED);
                        std::cout << "[SERVE] accepted test " << request->test_id
                                  << " (repeat override " << request->repeat_override
                                  << ") from GCS " << static_cast<int>(message.sysid) << '/'
                                  << static_cast<int>(message.compid) << '\n';
                    }
                }
                send_command_ack(
                    passthrough, command.command, ack_result, message.sysid, message.compid);
            } else if (command.command == riposte_test::gcs::kCommandAbort) {
                uint8_t ack_result;
                if (!riposte_test::gcs::magic_matches(command.param1)) {
                    ack_result = static_cast<uint8_t>(MAV_RESULT_DENIED);
                } else {
                    ack_result = static_cast<uint8_t>(MAV_RESULT_ACCEPTED);
                    bool was_running = false;
                    {
                        const std::lock_guard<std::mutex> lock(request_mutex);
                        pending.reset();
                        was_running = test_running;
                    }
                    if (was_running) {
                        // 실행 루프의 check_stop_requested() 가 이 플래그를 보고
                        // 예외를 던지면 emergency_land_if_needed() 가 착륙시킵니다.
                        // RT|ABORTED|<id> 는 착륙이 끝난 뒤 실행 루프가 보냅니다.
                        g_remote_abort_requested.store(true);
                        g_stop_requested.store(true);
                        std::cerr << "[SERVE] GCS abort received. Landing the current test.\n";
                    } else {
                        std::cout << "[SERVE] GCS abort received while idle. Nothing to stop.\n";
                        // 유휴 상태라 착륙할 것이 없다는 확인입니다. GCS 의 --abort 가
                        // 30초 확인 대기를 하지 않고 바로 끝나게 해 줍니다.
                        send_agent_status(passthrough, riposte_test::gcs::aborted_text(0));
                    }
                }
                send_command_ack(
                    passthrough, command.command, ack_result, message.sysid, message.compid);
            }
    };

    /*
     * 수신 구독은 "발견되는 모든 system" 에 겁니다.
     *
     * MAVSDK 는 수신 메시지를 발신자 sysid 의 System 객체로 분배합니다. 그래서
     * autopilot system(위 passthrough)에만 구독하면 PX4(sysid 1)가 보낸 메시지만
     * 보이고, 지상국(별도 sysid, 기본 245)이 보낸 COMMAND_LONG 은 지상국용
     * System 으로 가 버려 요청이 조용히 사라집니다. PX4 SITL end-to-end 시험에서
     * 실제로 발견된 동작입니다.
     *
     * receivers 는 상태/primary passthrough 보다 뒤에 선언하여 가장 먼저
     * 소멸(구독 해제)되게 합니다.
     */
    std::vector<std::shared_ptr<mavsdk::System>> covered_systems;
    std::vector<std::unique_ptr<mavsdk::MavlinkPassthrough>> receivers;
    const auto ensure_receivers = [&]() {
        for (const auto& discovered : mavsdk.systems()) {
            bool already_covered = false;
            for (const auto& covered : covered_systems) {
                if (covered.get() == discovered.get()) {
                    already_covered = true;
                    break;
                }
            }
            if (already_covered) {
                continue;
            }
            covered_systems.push_back(discovered);
            receivers.push_back(std::make_unique<mavsdk::MavlinkPassthrough>(discovered));
            receivers.back()->subscribe_message(MAVLINK_MSG_ID_COMMAND_LONG, handle_command);
        }
    };
    ensure_receivers();

    std::cout << "[SERVE] Waiting for GCS test requests as MAVLink component "
              << static_cast<int>(passthrough.get_our_compid()) << " (sysid "
              << static_cast<int>(passthrough.get_our_sysid())
              << "). Press Ctrl+C to stop.\n";
    send_agent_status(passthrough, riposte_test::gcs::ready_text());

    while (true) {
        // 지상국이 뒤늦게 링크에 나타날 수 있으므로 매 주기 새 system 을 확인합니다.
        ensure_receivers();

        if (g_stop_requested.load()) {
            if (g_remote_abort_requested.load()) {
                // run() 종료 직후(요약 출력/CSV 기록 중)에 도착한 늦은 중단
                // 명령입니다. 시험은 이미 끝나 착륙했으므로 중단할 것이 없습니다.
                // 운영자 SIGINT 로 오인해 에이전트를 종료하지 않도록 플래그만
                // 걷어내고 계속 대기합니다.
                g_remote_abort_requested.store(false);
                g_stop_requested.store(false);
                std::cout << "[SERVE] Late GCS abort arrived after the test finished;"
                             " nothing to stop.\n";
                continue;
            }
            // 시험이 돌지 않는 동안의 SIGINT/SIGTERM 은 대기 종료 요청입니다.
            std::cout << "[SERVE] Stopped by operator.\n";
            return EXIT_SUCCESS;
        }

        std::optional<riposte_test::gcs::TestRequest> request{};
        {
            const std::lock_guard<std::mutex> lock(request_mutex);
            if (pending) {
                request = pending;
                pending.reset();
                test_running = true;
            }
        }
        if (!request) {
            std::this_thread::sleep_for(200ms);
            continue;
        }

        // decode_request() 가 이미 확인했지만, 카탈로그 조회는 방어적으로 반복합니다.
        const riposte_test::TestCase* test_case =
            riposte_test::find_test_case(request->test_id);
        if (test_case == nullptr) {
            const std::lock_guard<std::mutex> lock(request_mutex);
            test_running = false;
            continue;
        }
        const int repeat =
            riposte_test::gcs::effective_repeat(*test_case, request->repeat_override);

        std::cout << "\n[SERVE] ===== Running test " << test_case->id << " ("
                  << test_case->name << "), repeat=" << repeat << " =====\n";

        TestCsvLogger logger{test_case->id};
        TestRunner runner{system, action, telemetry, logger};
        runner.set_progress_callback([&passthrough](const std::string& text) {
            send_agent_status(passthrough, text);
        });

        try {
            const riposte_test::TestReport report = runner.run(*test_case, repeat);
            print_summary(report);
            write_summary(report, logger.path());
            send_agent_status(
                passthrough,
                riposte_test::gcs::result_text(
                    report.test_id, report.passed(), report.completed(),
                    report.planned_iterations));
        } catch (const std::exception& error) {
            runner.emergency_land_if_needed();
            const std::string what{error.what()};
            const bool watchdog_fired = what.find("watchdog") != std::string::npos;
            if (g_remote_abort_requested.load() && !watchdog_fired) {
                // GCS 가 의도한 중단입니다. 착륙까지 끝났으므로 플래그를 걷어내고
                // 다음 요청을 계속 기다립니다.
                std::cerr << "[SERVE] Test aborted by GCS: " << what << '\n';
                send_agent_status(
                    passthrough, riposte_test::gcs::aborted_text(test_case->id));
                g_remote_abort_requested.store(false);
                g_stop_requested.store(false);
            } else if (g_stop_requested.load() && !g_remote_abort_requested.load()
                       && !watchdog_fired) {
                // 에이전트 쪽 운영자의 SIGINT/SIGTERM 입니다. 착륙까지 끝났고
                // 의도된 중지이므로 오류(RT|ERROR)가 아니라 RT|STOPPED 로 알리고
                // 정상 종료합니다.
                std::cout << "[SERVE] Operator stop during a test. Landed; shutting down.\n";
                send_agent_status(passthrough, riposte_test::gcs::stopped_text());
                return EXIT_SUCCESS;
            } else {
                // 예상하지 못한 오류입니다. 원격으로 재시도하다 상황을 키우지 않도록
                // 에이전트를 종료하고 사람의 점검을 요구합니다.
                std::cerr << "[SERVE] Stopping after an error: " << what << '\n';
                send_agent_status(passthrough, riposte_test::gcs::error_text(what));
                return EXIT_FAILURE;
            }
        }

        {
            const std::lock_guard<std::mutex> lock(request_mutex);
            test_running = false;
        }
        send_agent_status(passthrough, riposte_test::gcs::ready_text());
    }
}

} // namespace

int main(int argc, char** argv)
{
    // 파이프/파일로 리다이렉트되면 stdout 이 블록 버퍼링되어, systemd/nohup 로
    // 띄운 에이전트의 [SERVE] 진행 메시지가 수 KB 가 찰 때까지 보이지 않습니다.
    // 진행 로그는 관제 수단이므로 줄 단위로 즉시 내보냅니다.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    // SIGINT/SIGTERM 을 동일한 안전 중지 경로로 연결합니다.
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        const ProgramOptions options = parse_options(argc, argv);

        if (options.list) {
            print_catalog();
            return EXIT_SUCCESS;
        }

        const riposte_test::TestCase* test_case = nullptr;
        if (options.serve) {
            // 원격 모드에서는 시험 번호가 GCS 요청으로 옵니다. 로컬 옵션과 섞이면
            // "어느 쪽이 이기는가"가 모호해지므로 조합 자체를 거부합니다.
            if (options.test_id != 0 || options.plan_only || options.repeat_override > 0
                || options.plan_home) {
                std::cerr << "--serve cannot be combined with"
                             " --test/--plan-only/--plan-home/--repeat;"
                             " the GCS request carries those.\n";
                return EXIT_FAILURE;
            }
        } else {
            if (options.test_id == 0) {
                print_usage(argv[0]);
                std::cerr << "\nSelect a test with --test <1-23>, or use --list.\n";
                return EXIT_FAILURE;
            }
            test_case = riposte_test::find_test_case(options.test_id);
            if (test_case == nullptr) {
                std::cerr << "Unknown test id: " << options.test_id
                          << " (valid range is 1-23)\n";
                return EXIT_FAILURE;
            }
            if (options.plan_only) {
                print_plan(*test_case, options);
                return EXIT_SUCCESS;
            }
        }
        if (!options.execute) {
            print_usage(argv[0]);
            std::cerr << "\nSafety gate: add --execute only after the SITL/bench checklist"
                         " is complete."
                      << (options.serve ? " --serve arms the vehicle on remote request,"
                                          " so it needs --execute too.\n"
                                        : "\n");
            return EXIT_FAILURE;
        }

        // 반복 규칙은 gcs_protocol.hpp 의 effective_repeat() 한 곳에만 둡니다.
        // (Bullet 시험의 기본 반복은 laps 가 아니라 1 — laps 는 run_bullet_path()
        // 가 사이클 안에서 처리합니다.) --serve 는 요청이 올 때 계산합니다.
        const int repeat = options.serve
            ? 0
            : riposte_test::gcs::effective_repeat(*test_case, options.repeat_override);

        // CompanionComputer 타입이어야 PX4 가 이 링크를 onboard companion 으로
        // 식별합니다. GroundStation 타입으로는 위장하지 않습니다.
        //
        // 컴포넌트 ID 는 MAVSDK 기본값에 맡기지 않고 191 로 고정합니다. MAVSDK
        // v3.17 의 CompanionComputer 기본값은 195(PATHPLANNER)라서, 191 을
        // 대상으로 하는 GCS 원격 요청(gcs_protocol.hpp)이 도달하지 못합니다.
        // (PX4 SITL end-to-end 시험에서 실제로 발견된 불일치입니다.)
        mavsdk::Mavsdk::Configuration configuration{mavsdk::ComponentType::CompanionComputer};
        configuration.set_component_id(riposte_test::gcs::kAgentComponentId);
        mavsdk::Mavsdk mavsdk{configuration};
        const auto connection_result = mavsdk.add_any_connection(options.connection_url);
        if (connection_result != mavsdk::ConnectionResult::Success) {
            std::cerr << "Connection failed: " << connection_result << '\n';
            return EXIT_FAILURE;
        }

        std::cout << "Waiting for the PX4 autopilot on " << options.connection_url << "...\n";
        // timeout 이면 serial 장치명, baud, TX/RX 교차, GND, MAV_1_CONFIG 를 확인하십시오.
        const auto system_optional = mavsdk.first_autopilot(TestTuning::discovery_timeout_s);
        if (!system_optional) {
            throw std::runtime_error("No autopilot discovered before timeout");
        }
        const auto system = *system_optional;

        mavsdk::Action action{system};
        mavsdk::Telemetry telemetry{system};

        // 실제 위치를 20 Hz 로 받아야 settle 판정과 CSV 가 setpoint 주기와 유사한
        // 시간 해상도를 갖습니다.
        require_telemetry_success(
            telemetry.set_rate_position_velocity_ned(TestTuning::setpoint_rate_hz),
            "set_rate_position_velocity_ned");
        require_telemetry_success(telemetry.set_rate_battery(1.0), "set_rate_battery");
        // Bullet 경로는 목표 위/경도를 local NED 로 옮기기 위해 global position 이
        // 필요합니다. 5 Hz 면 충분하며 MAVLink 대역폭도 아낍니다.
        require_telemetry_success(telemetry.set_rate_position(5.0), "set_rate_position");

        if (options.serve) {
            return serve_requests(mavsdk, system, action, telemetry);
        }

        TestCsvLogger logger{test_case->id};
        TestRunner runner{system, action, telemetry, logger};

        riposte_test::TestReport report{};
        try {
            report = runner.run(*test_case, repeat);
        } catch (...) {
            // 원래 예외를 보존하면서 먼저 가능한 범위에서 Land 를 요청합니다.
            runner.emergency_land_if_needed();
            throw;
        }

        print_summary(report);
        write_summary(report, logger.path());
        return report.passed() ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
