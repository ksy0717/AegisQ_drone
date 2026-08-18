#include "gcs_protocol.hpp"
#include "test_cases.hpp"

/*
 * Riposte Ground Station (riposte_gcs)
 * ============================================================================
 * 텔레메트리(MAVLink) 링크로 companion 컴퓨터의 시험 에이전트
 * (riposte_test --serve --execute) 에 시험 실행을 요청하고, 진행 상황과 최종
 * PASS/FAIL 을 수신하는 지상국 프로그램입니다.
 *
 *   riposte_gcs ──(radio/UDP)── PX4 ──(TELEM2/UDP)── riposte_test --serve
 *
 * PX4 는 두 링크 사이에서 MAVLink 라우터 역할을 합니다. 요청이 에이전트까지
 * 가려면 PX4 의 MAVLink 포워딩이 켜져 있어야 합니다(docs/GCS_USAGE.md 참조).
 *
 * 이 프로그램은 기체를 직접 조종하지 않습니다. Arm/Takeoff/Offboard 는 모두
 * companion 쪽 에이전트가 수행하고, 여기서는 요청(COMMAND_LONG)과 수신
 * (COMMAND_ACK, STATUSTEXT)만 합니다. 그래도 원격으로 비행을 시작시키는
 * 프로그램이므로 --execute 안전 게이트를 동일하게 둡니다.
 *
 * 종료 코드:
 *   0   시험 PASS (--abort/--monitor 는 정상 완료)
 *   1   시험 FAIL, 요청 거부(DENIED/busy), 타임아웃, 사용법 오류
 *   130 운영자가 Ctrl+C 로 중단 (진행 중이던 시험에는 중단 명령을 보냄)
 */

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;
namespace gcs = riposte_test::gcs;

// 프로토콜 헤더의 재정의 상수와 실제 MAVLink 헤더를 컴파일 시점에 대조합니다.
static_assert(gcs::kCommandRunTest == MAV_CMD_USER_1, "command id mismatch");
static_assert(gcs::kCommandAbort == MAV_CMD_USER_2, "abort id mismatch");
static_assert(gcs::kAgentComponentId == MAV_COMP_ID_ONBOARD_COMPUTER, "component id mismatch");

namespace {

std::atomic_bool g_interrupted{false};

void signal_handler(int)
{
    g_interrupted.store(true);
}

struct GcsOptions {
    // 기본값은 PX4 SITL 의 GCS 포트입니다. 실기체 무전기는
    // serial:///dev/ttyUSB0:57600 형태입니다.
    std::string connection_url{"udpin://0.0.0.0:14550"};
    int test_id{0};
    int repeat_override{0};
    double timeout_s{0.0}; // 0 = 결과를 무한정 대기
    bool list{false};
    bool abort{false};
    bool monitor{false};
    bool execute{false};
};

void print_usage(const char* program)
{
    std::cout
        << "Usage: " << program << " --test <1-23> --execute [options]\n\n"
        << "Examples:\n"
        << "  카탈로그 확인   : " << program << " --list\n"
        << "  SITL 로 요청    : " << program << " --test 1 --execute\n"
        << "  무전기로 요청   : " << program
        << " --test 13 --repeat 2 --execute --connection serial:///dev/ttyUSB0:57600\n"
        << "  진행 중 시험 중단: " << program << " --abort\n"
        << "  수신만 하기     : " << program << " --monitor\n\n"
        << "Options:\n"
        << "  --test <id>          요청할 시험 번호 (1~23)\n"
        << "  --repeat <n>         반복 횟수 덮어쓰기 (미지정 시 카탈로그 값)\n"
        << "  --abort              진행 중인 시험을 중단(착륙)시키고 종료\n"
        << "  --monitor            요청 없이 STATUSTEXT 수신만 출력 (Ctrl+C 로 종료)\n"
        << "  --timeout <s>        결과 대기 제한시간 (기본 0 = 무한정)\n"
        << "  --list               시험 카탈로그 출력 후 종료\n"
        << "  --connection <url>   MAVSDK 연결 URL (기본 udpin://0.0.0.0:14550)\n"
        << "  --help, -h           이 도움말\n\n"
        << "--execute 가 없으면 시험 요청을 보내지 않습니다. companion 쪽도\n"
        << "riposte_test --serve --execute 로 떠 있어야 합니다 (docs/GCS_USAGE.md).\n";
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

GcsOptions parse_options(const int argc, char** argv)
{
    GcsOptions options{};
    for (int i = 1; i < argc; ++i) {
        const std::string argument{argv[i]};
        if (argument == "--execute") {
            options.execute = true;
        } else if (argument == "--list") {
            options.list = true;
        } else if (argument == "--abort") {
            options.abort = true;
        } else if (argument == "--monitor") {
            options.monitor = true;
        } else if (argument == "--test" && i + 1 < argc) {
            options.test_id = parse_integer(argv[++i], "--test");
        } else if (argument == "--repeat" && i + 1 < argc) {
            options.repeat_override = parse_integer(argv[++i], "--repeat");
            if (options.repeat_override <= 0) {
                throw std::runtime_error("--repeat must be positive");
            }
        } else if (argument == "--timeout" && i + 1 < argc) {
            options.timeout_s = parse_integer(argv[++i], "--timeout");
            if (options.timeout_s < 0.0) {
                throw std::runtime_error("--timeout must not be negative");
            }
        } else if (argument == "--connection" && i + 1 < argc) {
            options.connection_url = argv[++i];
        } else if (argument == "--help" || argument == "-h") {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::runtime_error("Unknown or incomplete argument: " + argument);
        }
    }
    return options;
}

void print_catalog()
{
    std::cout << "ID  Description\n--  ---------------------------------\n";
    for (const auto& test_case : riposte_test::kTestCatalog) {
        std::cout << (test_case.id < 10 ? " " : "") << test_case.id << "  " << test_case.name
                  << '\n';
    }
}

/*
 * GcsLink
 * ----------------------------------------------------------------------------
 * MAVSDK 연결과 수신 구독(COMMAND_ACK, STATUSTEXT)을 묶습니다. 수신은 MAVSDK
 * 콜백 스레드에서 오므로 공유 상태는 mutex/atomic 으로 보호합니다.
 */
class GcsLink {
public:
    explicit GcsLink(const std::shared_ptr<mavsdk::System>& system)
        : passthrough_(system)
    {
        passthrough_.subscribe_message(
            MAVLINK_MSG_ID_COMMAND_ACK, [this](const mavlink_message_t& message) {
                if (message.compid != gcs::kAgentComponentId) {
                    return; // PX4 등 다른 컴포넌트의 ACK 는 이 프로토콜이 아닙니다.
                }
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&message, &ack);
                if (ack.command != gcs::kCommandRunTest && ack.command != gcs::kCommandAbort) {
                    return;
                }
                const std::lock_guard<std::mutex> lock(mutex_);
                last_ack_command_ = ack.command;
                last_ack_result_ = static_cast<int>(ack.result);
            });

        passthrough_.subscribe_message(
            MAVLINK_MSG_ID_HEARTBEAT, [this](const mavlink_message_t& message) {
                // 에이전트 생존 감시용입니다. companion 이 시험 중 죽으면 결과가
                // 영원히 오지 않으므로, heartbeat 정지를 GCS 에서 경고합니다.
                if (message.compid == gcs::kAgentComponentId) {
                    last_agent_heartbeat_ticks_.store(
                        std::chrono::steady_clock::now().time_since_epoch().count());
                }
            });

        passthrough_.subscribe_message(
            MAVLINK_MSG_ID_STATUSTEXT, [this](const mavlink_message_t& message) {
                mavlink_statustext_t status;
                mavlink_msg_statustext_decode(&message, &status);
                // text 는 50바이트에 꽉 차면 널 종료가 없을 수 있습니다.
                char buffer[sizeof(status.text) + 1] = {};
                std::memcpy(buffer, status.text, sizeof(status.text));
                const std::string text{buffer};
                const bool from_agent = message.compid == gcs::kAgentComponentId;

                std::cout << (from_agent ? "[AGENT] " : "[PX4]   ") << text << '\n';

                if (from_agent && gcs::is_protocol_text(text)) {
                    const auto result = gcs::parse_result_text(text);
                    const std::lock_guard<std::mutex> lock(mutex_);
                    if (result) {
                        last_result_ = *result;
                    }
                    if (text.rfind("RT|ABORTED|", 0) == 0) {
                        abort_confirmed_ = true;
                    }
                    if (text.rfind("RT|ERROR|", 0) == 0) {
                        agent_error_ = true;
                    }
                    if (text == "RT|STOPPED") {
                        agent_stopped_ = true;
                    }
                }
            });
    }

    // 요청/중단 명령을 보냅니다. ACK 를 기다렸다가 MAV_RESULT 값을 반환하고,
    // 재시도(2초 간격, 3회) 후에도 응답이 없으면 nullopt 를 반환합니다.
    std::optional<int> send_command_and_wait_ack(
        const uint16_t command, const float param1, const float param2, const float param3)
    {
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            last_ack_command_ = 0;
            last_ack_result_ = -1;
            if (command == gcs::kCommandRunTest) {
                // 직전 세션의 같은 번호 시험이 막 끝난 시점에 접속하면 옛
                // RESULT/ERROR 라인이 이미 수신돼 있을 수 있습니다. 지금 보내는
                // 요청의 결과로 오인하지 않도록 여기서 비웁니다.
                last_result_.reset();
                agent_error_ = false;
            } else if (command == gcs::kCommandAbort) {
                abort_confirmed_ = false;
            }
        }
        for (uint8_t attempt = 0; attempt < 3; ++attempt) {
            queue_command(command, param1, param2, param3, attempt);
            const auto deadline = std::chrono::steady_clock::now() + 2s;
            while (std::chrono::steady_clock::now() < deadline) {
                if (g_interrupted.load()) {
                    return std::nullopt;
                }
                {
                    const std::lock_guard<std::mutex> lock(mutex_);
                    if (last_ack_command_ == command && last_ack_result_ >= 0) {
                        return last_ack_result_;
                    }
                }
                std::this_thread::sleep_for(50ms);
            }
            std::cout << "[GCS]   no ACK yet, retrying (" << static_cast<int>(attempt) + 1
                      << "/3)...\n";
        }
        return std::nullopt;
    }

    std::optional<gcs::ResultLine> take_result(const int test_id)
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (last_result_ && last_result_->test_id == test_id) {
            return last_result_;
        }
        return std::nullopt;
    }

    bool abort_confirmed() const
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        return abort_confirmed_;
    }

    bool agent_error() const
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        return agent_error_;
    }

    bool agent_stopped() const
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        return agent_stopped_;
    }

    // 마지막 에이전트 heartbeat 이후 경과 시간입니다. 한 번도 못 봤으면 nullopt.
    std::optional<double> agent_heartbeat_age_s() const
    {
        const auto ticks = last_agent_heartbeat_ticks_.load();
        if (ticks == 0) {
            return std::nullopt;
        }
        const auto last = std::chrono::steady_clock::time_point{
            std::chrono::steady_clock::duration{ticks}};
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count();
    }

    uint8_t target_sysid() const { return passthrough_.get_target_sysid(); }

private:
    void queue_command(
        const uint16_t command,
        const float param1,
        const float param2,
        const float param3,
        const uint8_t confirmation)
    {
        const uint8_t target_system = passthrough_.get_target_sysid();
        passthrough_.queue_message([command, param1, param2, param3, confirmation,
                                    target_system](MavlinkAddress address, uint8_t channel) {
            mavlink_message_t message;
            mavlink_msg_command_long_pack_chan(
                address.system_id, address.component_id, channel, &message,
                target_system, gcs::kAgentComponentId, command, confirmation,
                param1, param2, param3, 0.0F, 0.0F, 0.0F, 0.0F);
            return message;
        });
    }

    // 선언 순서가 곧 안전 조건입니다: 콜백이 mutex_ 와 수신 상태를 참조하므로,
    // passthrough_(구독 해제)가 "먼저" 소멸해야 합니다. C++ 는 선언 역순으로
    // 소멸하므로 passthrough_ 를 마지막에 둡니다. 순서를 바꾸면 객체 소멸 순간에
    // 도착한 메시지가 파괴된 mutex 를 잠그는 UB 가 됩니다.
    mutable std::mutex mutex_;
    uint16_t last_ack_command_{0};
    int last_ack_result_{-1};
    std::optional<gcs::ResultLine> last_result_{};
    bool abort_confirmed_{false};
    bool agent_error_{false};
    bool agent_stopped_{false};
    std::atomic<std::chrono::steady_clock::duration::rep> last_agent_heartbeat_ticks_{0};
    mavsdk::MavlinkPassthrough passthrough_;
};

const char* ack_result_name(const int result)
{
    switch (result) {
    case MAV_RESULT_ACCEPTED: return "ACCEPTED";
    case MAV_RESULT_TEMPORARILY_REJECTED: return "BUSY (a test is already running)";
    case MAV_RESULT_DENIED: return "DENIED (invalid request)";
    case MAV_RESULT_FAILED: return "FAILED";
    default: return "UNKNOWN";
    }
}

// 중단 명령을 보내고 확인을 기다립니다. Ctrl+C 처리와 --abort 가 공용합니다.
bool send_abort(GcsLink& link)
{
    std::cout << "[GCS]   sending abort...\n";
    const auto ack =
        link.send_command_and_wait_ack(gcs::kCommandAbort, gcs::kRequestMagic, 0.0F, 0.0F);
    if (!ack) {
        std::cerr << "[GCS]   abort was not acknowledged. Use the RC pilot switch"
                     " (Position/Land) as the primary stop.\n";
        return false;
    }
    std::cout << "[GCS]   abort " << ack_result_name(*ack) << '\n';
    if (*ack != MAV_RESULT_ACCEPTED) {
        return false;
    }

    // ACK 는 "명령을 받았다"일 뿐입니다. 착륙까지 끝났다는 RT|ABORTED 확인을
    // 기다립니다 (유휴 중이었어도 에이전트가 RT|ABORTED|0 을 보냅니다).
    // 확인이 안 와도 실패로 치지는 않습니다 — 포워딩 대역폭 문제일 수 있고,
    // 착륙 자체는 companion 이 링크와 무관하게 수행하기 때문입니다.
    std::cout << "[GCS]   waiting for the landing confirmation (RT|ABORTED)...\n";
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    while (std::chrono::steady_clock::now() < deadline && !g_interrupted.load()) {
        if (link.abort_confirmed()) {
            std::cout << "[GCS]   the agent confirmed the abort.\n";
            return true;
        }
        std::this_thread::sleep_for(200ms);
    }
    std::cout << "[GCS]   no confirmation within 30 s; if a test was flying, keep"
                 " watching with --monitor and stand by the RC pilot.\n";
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    // 리다이렉트 시에도 진행/수신 라인이 즉시 보이도록 줄 단위 버퍼링을 씁니다.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        const GcsOptions options = parse_options(argc, argv);

        if (options.list) {
            print_catalog();
            return EXIT_SUCCESS;
        }
        const bool wants_request = options.test_id != 0;
        const int mode_count = (wants_request ? 1 : 0) + (options.abort ? 1 : 0)
            + (options.monitor ? 1 : 0);
        if (mode_count == 0) {
            print_usage(argv[0]);
            std::cerr << "\nChoose one of --test, --abort or --monitor.\n";
            return EXIT_FAILURE;
        }
        if (mode_count > 1) {
            // 어느 동작이 이기는지 조용히 정하지 않습니다. 시험을 시작하는
            // 명령과 중단하는 명령이 한 줄에 섞이면 대개 실수입니다.
            std::cerr << "--test, --abort and --monitor are mutually exclusive;"
                         " pass exactly one.\n";
            return EXIT_FAILURE;
        }
        if (options.repeat_override > 0 && !wants_request) {
            std::cerr << "--repeat only makes sense together with --test.\n";
            return EXIT_FAILURE;
        }
        if (wants_request) {
            if (riposte_test::find_test_case(options.test_id) == nullptr) {
                std::cerr << "Unknown test id: " << options.test_id
                          << " (valid range is 1-23)\n";
                return EXIT_FAILURE;
            }
            if (!options.execute) {
                print_usage(argv[0]);
                std::cerr << "\nSafety gate: --test remotely arms the vehicle, so it"
                             " requires --execute.\n";
                return EXIT_FAILURE;
            }
        }

        mavsdk::Mavsdk mavsdk{
            mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
        const auto connection_result = mavsdk.add_any_connection(options.connection_url);
        if (connection_result != mavsdk::ConnectionResult::Success) {
            std::cerr << "Connection failed: " << connection_result << '\n';
            return EXIT_FAILURE;
        }

        std::cout << "Waiting for the vehicle on " << options.connection_url << "...\n";
        const auto system_optional = mavsdk.first_autopilot(15.0);
        if (!system_optional) {
            throw std::runtime_error("No autopilot discovered before timeout");
        }
        const auto& system = *system_optional;

        // 에이전트(컴포넌트 191)의 heartbeat 이 이 링크까지 포워딩되는지 확인합니다.
        // 없어도 계속 진행은 하되(늦게 켜졌을 수 있음), 원인 후보를 알려 줍니다.
        {
            const auto deadline = std::chrono::steady_clock::now() + 5s;
            bool agent_seen = false;
            while (std::chrono::steady_clock::now() < deadline && !agent_seen) {
                for (const auto component_id : system->component_ids()) {
                    if (component_id == gcs::kAgentComponentId) {
                        agent_seen = true;
                        break;
                    }
                }
                std::this_thread::sleep_for(200ms);
            }
            if (agent_seen) {
                std::cout << "Test agent (component 191) is visible on this link.\n";
            } else {
                std::cerr << "[WARN] The test agent (component 191) is not visible yet.\n"
                             "       Check that riposte_test --serve --execute is running\n"
                             "       and that PX4 MAVLink forwarding is enabled"
                             " (MAV_0_FORWARD/MAV_1_FORWARD, docs/GCS_USAGE.md).\n";
            }
        }

        GcsLink link{system};

        if (options.abort) {
            return send_abort(link) ? EXIT_SUCCESS : EXIT_FAILURE;
        }

        if (options.monitor) {
            std::cout << "Monitoring STATUSTEXT (Ctrl+C to quit)...\n";
            while (!g_interrupted.load()) {
                std::this_thread::sleep_for(200ms);
            }
            return EXIT_SUCCESS;
        }

        // ---- 시험 요청 ------------------------------------------------------
        std::cout << "Requesting test " << options.test_id;
        if (options.repeat_override > 0) {
            std::cout << " (repeat " << options.repeat_override << ')';
        }
        std::cout << " from the agent on vehicle sysid "
                  << static_cast<int>(link.target_sysid()) << "...\n";

        const auto params = gcs::encode_request(
            gcs::TestRequest{options.test_id, options.repeat_override});
        const auto ack = link.send_command_and_wait_ack(
            gcs::kCommandRunTest, params.param1, params.param2, params.param3);

        if (g_interrupted.load()) {
            std::cerr << "Interrupted before the request was acknowledged.\n";
            return 130;
        }
        if (!ack) {
            std::cerr << "The agent did not acknowledge the request. Check --serve and"
                         " MAVLink forwarding (docs/GCS_USAGE.md).\n";
            return EXIT_FAILURE;
        }
        std::cout << "Request " << ack_result_name(*ack) << '\n';
        if (*ack != MAV_RESULT_ACCEPTED) {
            return EXIT_FAILURE;
        }

        // ---- 결과 대기 ------------------------------------------------------
        const bool has_timeout = options.timeout_s > 0.0;
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(options.timeout_s));
        bool heartbeat_warned = false;
        while (true) {
            if (g_interrupted.load()) {
                std::cerr << "\nInterrupted. Asking the agent to land...\n";
                g_interrupted.store(false); // send_abort 의 ACK 대기가 돌 수 있게 함
                send_abort(link);
                return 130;
            }
            if (has_timeout && std::chrono::steady_clock::now() > deadline) {
                std::cerr << "Timed out waiting for the result. The test may still be"
                             " running; use --monitor to keep watching or --abort to stop.\n";
                return EXIT_FAILURE;
            }
            const auto result = link.take_result(options.test_id);
            if (result) {
                std::cout << "\n================ Remote test " << result->test_id
                          << " ================\n"
                          << "iterations: " << result->completed << '/' << result->planned
                          << "\nOVERALL: " << (result->passed ? "PASS" : "FAIL") << '\n'
                          << "Detailed and summary CSV files are on the companion computer"
                             " (logs/).\n";
                return result->passed ? EXIT_SUCCESS : EXIT_FAILURE;
            }
            if (link.agent_error()) {
                std::cerr << "\nThe agent reported an error and stopped serving."
                             " Inspect the companion computer before retrying.\n";
                return EXIT_FAILURE;
            }
            if (link.agent_stopped()) {
                std::cerr << "\nThe operator stopped the agent; the test did not finish.\n";
                return EXIT_FAILURE;
            }
            // 에이전트 heartbeat 이 끊기면 결과가 영원히 오지 않을 수 있습니다.
            // 링크가 순간적으로 흔들릴 수 있으므로 종료하지는 않고, 정지가
            // 이어지는 동안 한 번만 경고합니다.
            const auto heartbeat_age = link.agent_heartbeat_age_s();
            if (heartbeat_age && *heartbeat_age > 10.0) {
                if (!heartbeat_warned) {
                    std::cerr << "[WARN] no agent heartbeat for "
                              << static_cast<int>(*heartbeat_age)
                              << " s. The companion may be down or the link lost;"
                                 " stand by the RC pilot. (Ctrl+C aborts, or use"
                                 " --timeout next time.)\n";
                    heartbeat_warned = true;
                }
            } else {
                heartbeat_warned = false;
            }
            std::this_thread::sleep_for(200ms);
        }
    } catch (const std::exception& error) {
        std::cerr << "riposte_gcs failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
