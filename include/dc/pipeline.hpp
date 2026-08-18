// GStreamer 파이프라인 + cairo 오버레이. 카메라를 여는 유일한 지점이다.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cairo/cairo.h>
#include <gst/gst.h>

#include "dc/config.hpp"
#include "dc/detection.hpp"
#include "dc/hailo_inference.hpp"

namespace dc {

struct Stats {
    std::atomic<uint64_t> captured{0};
    std::atomic<uint64_t> inferred{0};
    std::atomic<uint64_t> det_total{0};
    std::atomic<uint64_t> dropped{0};
    std::atomic<size_t> nms_bytes{0};
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    double elapsed() const {
        using namespace std::chrono;
        return duration<double>(steady_clock::now() - t0).count();
    }
};

class Pipeline {
public:
    Pipeline(const Config& cfg, HailoInference* hailo);
    ~Pipeline();

    // detect=false 면 추론 없이 카메라 영상만 내보낸다(BGRA 변환·오버레이 생략).
    // dual=true 로 먼저 시도하고 실패하면 tee 로 폴백한다.
    bool start();
    void stop();

    GstElement* element() const { return pipeline_; }
    const std::string& description() const { return desc_; }
    bool detecting() const { return detect_; }
    bool inference_alive() const { return infer_alive_.load(); }
    const Stats& stats() const { return stats_; }

private:
    std::string build(bool dual) const;
    bool launch(bool dual);
    void stop_whip();
    void infer_loop();

    static GstFlowReturn on_new_sample(GstElement* sink, gpointer user);
    static GstFlowReturn on_whip_sample(GstElement* sink, gpointer user);
    static void on_frame(GstElement* identity, GstBuffer* buffer, gpointer user);
    static gboolean on_whip_bus(GstBus* bus, GstMessage* message, gpointer user);
    static gboolean on_whip_watchdog(gpointer user);
    // Sums every "bytes-sent" the sink reports, however deeply nested, so the
    // walk survives layout changes between gst-plugins-rs versions.
    static guint64 sum_bytes_sent(const GstStructure* stats, unsigned int& sessions);
    void report_whip_delivery();
    static void on_draw(GstElement* ov, cairo_t* cr, guint64 ts, guint64 dur, gpointer user);
    static void on_caps_changed(GstElement* ov, GstCaps* caps, gpointer user);

    void draw(cairo_t* cr);

    Config cfg_;
    HailoInference* hailo_;          // 소유하지 않는다
    bool detect_;
    Letterbox lb_;

    GstElement* pipeline_ = nullptr;
    GstElement* whip_pipeline_ = nullptr;
    GstElement* whip_source_ = nullptr;
    GstElement* whip_sink_ = nullptr;
    guint whip_bus_watch_id_ = 0U;
    guint whip_watchdog_id_ = 0U;
    // Set before every NULL->PLAYING so the publisher starts on an IDR rather
    // than feeding the peer a mid-GOP it cannot decode.
    std::atomic<bool> whip_needs_keyframe_{true};
    std::atomic<unsigned int> whip_push_failures_{0U};
    // A publisher that signals successfully but delivers nothing looks
    // identical to a healthy one from every other angle. Only the sink's own
    // byte counter can tell them apart.
    guint64 whip_last_bytes_sent_ = 0U;
    bool whip_delivering_ = false;
    bool whip_delivery_logged_ = false;
    std::string desc_;

    mutable std::mutex mu_;
    std::vector<Detection> dets_;
    std::vector<uint8_t> pending_;   // 최신 프레임 1장만 유지
    bool has_pending_ = false;
    int canvas_w_, canvas_h_;

    std::condition_variable cv_;
    std::thread worker_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> infer_alive_{true};
    Stats stats_;
};

}  // namespace dc
