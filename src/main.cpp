// drone_companion — 드론 온보드 컴패니언 소프트웨어
//
// 현재 담당: 카메라 → (Hailo-8 검출) → DSI 패널 + RTSP/WHIP 송출.
// 이동(MAVLink)·미션 실행은 별도 서브시스템으로 이 위에 붙는다 — README 참고.
//
// libcamera 는 카메라를 한 프로세스만 연다. 새 소비자가 필요하면 이 파이프라인의
// tee 에 분기를 붙일 것 — 두 번째 프로세스를 띄우면 충돌한다.
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>

#include <glib.h>
#include <gst/gst.h>

#include "dc/config.hpp"
#include "dc/hailo_inference.hpp"
#include "dc/pipeline.hpp"

namespace {

GMainLoop* g_loop = nullptr;

void on_signal(int sig) {
    std::printf("\n[main] 신호 %d 수신 — 종료합니다.\n", sig);
    std::fflush(stdout);
    if (g_loop) g_main_loop_quit(g_loop);
}

gboolean on_bus(GstBus*, GstMessage* msg, gpointer) {
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError* err = nullptr;
            gchar* dbg = nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            std::fprintf(stderr, "[ERROR] %s\n%s\n", err->message, dbg ? dbg : "");
            g_error_free(err);
            g_free(dbg);
            if (g_loop) g_main_loop_quit(g_loop);
            break;
        }
        case GST_MESSAGE_EOS:
            std::printf("[main] EOS\n");
            if (g_loop) g_main_loop_quit(g_loop);
            break;
        default:
            break;
    }
    return TRUE;
}

struct ReportCtx {
    dc::Pipeline* pipe;
};

gboolean on_report(gpointer user) {
    auto* ctx = static_cast<ReportCtx*>(user);
    const auto& s = ctx->pipe->stats();
    const double el = s.elapsed();
    if (ctx->pipe->detecting() && ctx->pipe->inference_alive()) {
        std::printf("  t=%6.1fs  캡처 %5llu(%5.1ffps)  추론 %5llu(%5.1ffps)  "
                    "누적 %llu  NMS %zuB  드롭 %llu\n",
                    el,
                    static_cast<unsigned long long>(s.captured.load()), s.captured.load() / el,
                    static_cast<unsigned long long>(s.inferred.load()), s.inferred.load() / el,
                    static_cast<unsigned long long>(s.det_total.load()),
                    s.nms_bytes.load(),
                    static_cast<unsigned long long>(s.dropped.load()));
    } else {
        std::printf("  t=%6.1fs  캡처 %5llu(%5.1ffps)  통과 모드(검출 없음)\n",
                    el,
                    static_cast<unsigned long long>(s.captured.load()),
                    s.captured.load() / std::max(el, 1e-6));
    }
    std::fflush(stdout);
    return TRUE;
}

gboolean on_deadline(gpointer) {
    if (g_loop) g_main_loop_quit(g_loop);
    return FALSE;
}

}  // namespace

int main(int argc, char** argv) {
    gst_init(&argc, &argv);

    const dc::Config cfg = dc::Config::from_env();
    const std::string config_error = cfg.validation_error();
    if (!config_error.empty()) {
        std::fprintf(stderr, "[FATAL] invalid configuration: %s\n", config_error.c_str());
        return 2;
    }

    if (cfg.enable_whip) {
        GstElementFactory* factory = gst_element_factory_find("whipclientsink");
        if (!factory) {
            std::fprintf(
                stderr,
                "[FATAL] ENABLE_WHIP=1 but the GStreamer whipclientsink plugin is missing\n");
            return 2;
        }
        gst_object_unref(factory);
    }
    cfg.print();

    // Hailo 가용성 판정. 못 쓰면 사유를 남기고 통과 모드로 간다.
    std::unique_ptr<dc::HailoInference> hailo;
    if (!cfg.hailo_requested) {
        std::printf("[hailo] 통과 모드 — HAILO=0 으로 검출을 끔\n");
    } else {
        dc::HailoStatus st;
        hailo = dc::HailoInference::create(cfg.hef, cfg.hailo_wait_secs, st);
        if (hailo)
            std::printf("[hailo] 검출 모드 — %s\n", st.reason.c_str());
        else
            std::printf("[hailo] 통과 모드 — %s\n", st.reason.c_str());
    }
    if (!hailo)
        std::printf("[hailo] 검출 없이 카메라 영상을 설정된 영상 출력으로 송출합니다.\n");
    std::fflush(stdout);

    dc::Pipeline pipe(cfg, hailo.get());
    if (!pipe.start()) {
        std::fprintf(stderr, "[FATAL] 파이프라인을 PLAYING 으로 올리지 못했습니다.\n");
        return 1;
    }
    std::printf("=== 파이프라인 ===\n%s\n", pipe.description().c_str());
    if (cfg.enable_rtsp) std::printf("[rtsp] publish -> %s\n", cfg.rtsp_url.c_str());
    if (cfg.enable_whip)
        std::printf("[whip] publish -> configured server-issued endpoint (redacted)\n");
    std::fflush(stdout);

    g_loop = g_main_loop_new(nullptr, FALSE);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    GstBus* bus = gst_element_get_bus(pipe.element());
    gst_bus_add_watch(bus, on_bus, nullptr);

    ReportCtx rc{&pipe};
    g_timeout_add_seconds(static_cast<guint>(cfg.report_secs), on_report, &rc);
    if (cfg.run_secs > 0)
        g_timeout_add_seconds(static_cast<guint>(cfg.run_secs), on_deadline, nullptr);

    g_main_loop_run(g_loop);

    gst_object_unref(bus);
    pipe.stop();

    const auto& s = pipe.stats();
    const double el = s.elapsed();
    if (pipe.detecting())
        std::printf("\n=== 종료: 캡처 %llu / 추론 %llu in %.1fs = %.1f fps, 누적검출 %llu ===\n",
                    static_cast<unsigned long long>(s.captured.load()),
                    static_cast<unsigned long long>(s.inferred.load()), el,
                    s.inferred.load() / std::max(el, 1e-6),
                    static_cast<unsigned long long>(s.det_total.load()));
    else
        std::printf("\n=== 종료: 통과 모드 %.1fs ===\n", el);

    g_main_loop_unref(g_loop);
    return 0;
}
