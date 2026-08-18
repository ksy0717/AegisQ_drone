#include "dc/pipeline.hpp"
#include "dc/video_pipeline.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

namespace dc {

namespace {

struct Color { double r, g, b; };
const Color kColors[] = {{1.0, 0.25, 0.25}, {0.25, 0.85, 1.0},
                         {0.4, 1.0, 0.4},   {1.0, 0.85, 0.2}};

std::string fmt(const char* f, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof(buf), f, ap);
    va_end(ap);
    return buf;
}

// GStreamer error text can quote the endpoint it failed on, and that carries
// the stream key. Keep the reason, drop the credential.
std::string redact_stream_key(const char* text) {
    if (text == nullptr) return "unknown";
    std::string message{text};
    for (std::size_t at = message.find("/w/"); at != std::string::npos;
         at = message.find("/w/", at + 3U)) {
        const std::size_t key = at + 3U;
        std::size_t end = key;
        while (end < message.size()
               && std::isspace(static_cast<unsigned char>(message[end])) == 0
               && message[end] != '"' && message[end] != '\'') {
            ++end;
        }
        message.replace(key, end - key, "<redacted>");
    }
    return message;
}

bool configure_whip_signaller(GstElement* pipeline, const Config& config) {
    if (!config.enable_whip) return true;

    GstElement* whip = gst_bin_get_by_name(GST_BIN(pipeline), "whip");
    if (!whip) {
        std::fprintf(stderr, "[whip] pipeline element is missing\n");
        return false;
    }

    if (!g_object_class_find_property(G_OBJECT_GET_CLASS(whip), "signaller")) {
        std::fprintf(stderr, "[whip] plugin does not expose a signaller property\n");
        gst_object_unref(whip);
        return false;
    }

    GObject* signaller = nullptr;
    g_object_get(G_OBJECT(whip), "signaller", &signaller, nullptr);
    gst_object_unref(whip);
    if (!signaller) {
        std::fprintf(stderr, "[whip] plugin did not create its signaller\n");
        return false;
    }

    if (!g_object_class_find_property(G_OBJECT_GET_CLASS(signaller), "whip-endpoint")) {
        std::fprintf(stderr, "[whip] signaller is incompatible: no whip-endpoint property\n");
        g_object_unref(signaller);
        return false;
    }
    g_object_set(signaller, "whip-endpoint", config.whip_url.c_str(), nullptr);

    if (!config.whip_token.empty()) {
        if (!g_object_class_find_property(G_OBJECT_GET_CLASS(signaller), "auth-token")) {
            std::fprintf(stderr, "[whip] signaller is incompatible: no auth-token property\n");
            g_object_unref(signaller);
            return false;
        }
        g_object_set(signaller, "auth-token", config.whip_token.c_str(), nullptr);
    }
    g_object_unref(signaller);
    return true;
}

}  // namespace

Pipeline::Pipeline(const Config& cfg, HailoInference* hailo)
    : cfg_(cfg),
      hailo_(hailo),
      detect_(hailo != nullptr),
      lb_(Letterbox::make(cfg.src_w, cfg.src_h, cfg.net)),
      canvas_w_(cfg.src_w),
      canvas_h_(cfg.src_h) {
    if (!detect_) infer_alive_.store(false);
}

Pipeline::~Pipeline() { stop(); }

std::string Pipeline::build(bool dual) const {
    return build_video_pipeline(cfg_, detect_, dual);
}

bool Pipeline::launch(bool dual) {
    const std::string s = build(dual);
    GError* err = nullptr;
    GstElement* pl = gst_parse_launch(s.c_str(), &err);
    if (!pl) {
        std::fprintf(stderr, "[pipeline] 생성 실패(dual=%d): %s\n",
                     dual ? 1 : 0, err ? err->message : "unknown");
        if (err) g_error_free(err);
        return false;
    }
    if (err) g_error_free(err);

    GstElement* counter = gst_bin_get_by_name(GST_BIN(pl), "frame_counter");
    if (!counter) {
        std::fprintf(stderr, "[pipeline] frame counter element is missing\n");
        gst_object_unref(pl);
        return false;
    }
    g_signal_connect(counter, "handoff", G_CALLBACK(&Pipeline::on_frame), this);
    gst_object_unref(counter);

    if (detect_) {
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pl), "det");
        GstElement* ov = gst_bin_get_by_name(GST_BIN(pl), "ov");
        if (!sink || !ov) {
            std::fprintf(stderr, "[pipeline] det/ov 요소를 찾지 못함\n");
            gst_object_unref(pl);
            return false;
        }
        g_signal_connect(sink, "new-sample", G_CALLBACK(&Pipeline::on_new_sample), this);
        g_signal_connect(ov, "draw", G_CALLBACK(&Pipeline::on_draw), this);
        g_signal_connect(ov, "caps-changed", G_CALLBACK(&Pipeline::on_caps_changed), this);
        gst_object_unref(sink);
        gst_object_unref(ov);
    }

    std::string whip_description;
    if (cfg_.enable_whip) {
        whip_description = build_whip_pipeline();
        GError* whip_error = nullptr;
        GstElement* whip_pipeline =
            gst_parse_launch(whip_description.c_str(), &whip_error);
        if (!whip_pipeline) {
            std::fprintf(
                stderr,
                "[whip] bridge pipeline creation failed: %s\n",
                whip_error ? whip_error->message : "unknown");
            if (whip_error) g_error_free(whip_error);
            gst_object_unref(pl);
            return false;
        }
        if (whip_error) g_error_free(whip_error);

        if (!configure_whip_signaller(whip_pipeline, cfg_)) {
            gst_object_unref(whip_pipeline);
            gst_object_unref(pl);
            return false;
        }

        GstElement* bridge = gst_bin_get_by_name(GST_BIN(pl), "whip_bridge");
        GstElement* source =
            gst_bin_get_by_name(GST_BIN(whip_pipeline), "whip_source");
        if (!bridge || !source) {
            std::fprintf(stderr, "[whip] bounded bridge elements are missing\n");
            if (bridge) gst_object_unref(bridge);
            if (source) gst_object_unref(source);
            gst_element_set_state(whip_pipeline, GST_STATE_NULL);
            gst_object_unref(whip_pipeline);
            gst_object_unref(pl);
            return false;
        }

        whip_pipeline_ = whip_pipeline;
        whip_source_ = source;
        // May be absent on a build without the element; the delivery check
        // simply stays quiet rather than blocking the publisher.
        whip_sink_ = gst_bin_get_by_name(GST_BIN(whip_pipeline), "whip");
        g_signal_connect(
            bridge, "new-sample", G_CALLBACK(&Pipeline::on_whip_sample), this);
        gst_object_unref(bridge);

        GstBus* whip_bus = gst_element_get_bus(whip_pipeline_);
        whip_bus_watch_id_ =
            gst_bus_add_watch(whip_bus, &Pipeline::on_whip_bus, this);
        gst_object_unref(whip_bus);
        whip_watchdog_id_ = g_timeout_add_seconds(
            static_cast<guint>(cfg_.whip_retry_secs),
            &Pipeline::on_whip_watchdog,
            this);

        whip_needs_keyframe_.store(true);
        if (gst_element_set_state(whip_pipeline_, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
            std::fprintf(
                stderr,
                "[whip] publisher did not start; RTSP remains active and retry is scheduled\n");
            gst_element_set_state(whip_pipeline_, GST_STATE_NULL);
        }
    }

    if (gst_element_set_state(pl, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        stop_whip();
        gst_element_set_state(pl, GST_STATE_NULL);
        gst_object_unref(pl);
        return false;
    }

    pipeline_ = pl;
    desc_ = cfg_.enable_whip
        ? s + "\nWHIP bridge: " + whip_description
        : s;
    return true;
}

bool Pipeline::start() {
    stats_.t0 = std::chrono::steady_clock::now();

    // 듀얼 스트림(ISP 하드웨어 스케일)을 먼저 시도하고, 안 되면 tee 로 내려간다.
    bool ok = false;
    if (detect_ && cfg_.dual_stream) {
        ok = launch(true);
        if (!ok)
            std::fprintf(stderr,
                         "[pipeline] ISP 2nd 스트림 협상 실패 — tee 방식으로 폴백합니다.\n");
    }
    if (!ok) ok = launch(false);
    if (!ok) return false;

    if (detect_) worker_ = std::thread(&Pipeline::infer_loop, this);
    return true;
}

void Pipeline::stop() {
    stop_.store(true);
    cv_.notify_all();
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (worker_.joinable()) worker_.join();
    stop_whip();
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

void Pipeline::stop_whip() {
    if (whip_watchdog_id_ != 0U) {
        g_source_remove(whip_watchdog_id_);
        whip_watchdog_id_ = 0U;
    }
    if (whip_bus_watch_id_ != 0U) {
        g_source_remove(whip_bus_watch_id_);
        whip_bus_watch_id_ = 0U;
    }
    if (whip_pipeline_) {
        gst_element_set_state(whip_pipeline_, GST_STATE_NULL);
    }
    if (whip_source_) {
        gst_object_unref(whip_source_);
        whip_source_ = nullptr;
    }
    if (whip_sink_) {
        gst_object_unref(whip_sink_);
        whip_sink_ = nullptr;
    }
    if (whip_pipeline_) {
        gst_object_unref(whip_pipeline_);
        whip_pipeline_ = nullptr;
    }
}

GstFlowReturn Pipeline::on_new_sample(GstElement* sink, gpointer user) {
    auto* self = static_cast<Pipeline*>(user);
    GstSample* smp = nullptr;
    g_signal_emit_by_name(sink, "pull-sample", &smp);
    if (!smp) return GST_FLOW_OK;

    GstBuffer* buf = gst_sample_get_buffer(smp);
    GstMapInfo mi;
    if (buf && gst_buffer_map(buf, &mi, GST_MAP_READ)) {
        {
            std::lock_guard<std::mutex> lk(self->mu_);
            if (self->has_pending_) self->stats_.dropped.fetch_add(1);
            self->pending_.assign(mi.data, mi.data + mi.size);
            self->has_pending_ = true;
        }
        gst_buffer_unmap(buf, &mi);
        self->cv_.notify_one();
    }
    gst_sample_unref(smp);
    return GST_FLOW_OK;
}

GstFlowReturn Pipeline::on_whip_sample(GstElement* sink, gpointer user) {
    auto* self = static_cast<Pipeline*>(user);
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) return GST_FLOW_OK;

    GstBuffer* input = gst_sample_get_buffer(sample);
    if (input != nullptr && self->whip_source_ != nullptr) {
        // A publisher that attaches mid-GOP feeds the peer frames it cannot
        // decode until the next IDR. Wait for one after every restart.
        const bool delta =
            GST_BUFFER_FLAG_IS_SET(input, GST_BUFFER_FLAG_DELTA_UNIT);
        if (!delta || !self->whip_needs_keyframe_.load()) {
            // Shallow copy: a writable header over the same encoded memory.
            GstBuffer* output = gst_buffer_copy(input);
            // These timestamps belong to the camera pipeline's timeline and are
            // meaningless here; appsrc do-timestamp restamps them. GstBaseSrc
            // only stamps a field that is invalid, so clearing is required.
            GST_BUFFER_PTS(output) = GST_CLOCK_TIME_NONE;
            GST_BUFFER_DTS(output) = GST_CLOCK_TIME_NONE;

            GstSample* bridged = gst_sample_new(
                output, gst_sample_get_caps(sample), nullptr, nullptr);
            gst_buffer_unref(output);
            // push_sample carries the caps, so the appsrc advertises the real
            // media type instead of relying on the downstream capsfilter.
            const GstFlowReturn flow = gst_app_src_push_sample(
                GST_APP_SRC(self->whip_source_), bridged);
            gst_sample_unref(bridged);

            if (flow == GST_FLOW_OK) {
                self->whip_needs_keyframe_.store(false);
            } else if (flow != GST_FLOW_FLUSHING) {
                // FLUSHING is normal while the publisher restarts. Anything
                // else is worth one line, not a 30 Hz flood.
                self->whip_push_failures_.fetch_add(1U);
            }
        }
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

void Pipeline::on_frame(GstElement*, GstBuffer*, gpointer user) {
    static_cast<Pipeline*>(user)->stats_.captured.fetch_add(1);
}

gboolean Pipeline::on_whip_bus(GstBus*, GstMessage* message, gpointer user) {
    if (GST_MESSAGE_TYPE(message) != GST_MESSAGE_ERROR) return TRUE;

    auto* self = static_cast<Pipeline*>(user);
    GError* error = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(message, &error, &debug);
    const std::string reason =
        redact_stream_key(error != nullptr ? error->message : nullptr);
    if (error) g_error_free(error);
    g_free(debug);

    std::fprintf(
        stderr,
        "[whip] publisher error (%s); RTSP remains active and retry is scheduled\n",
        reason.c_str());
    if (self->whip_pipeline_) {
        gst_element_set_state(self->whip_pipeline_, GST_STATE_NULL);
    }
    return TRUE;
}

guint64 Pipeline::sum_bytes_sent(
    const GstStructure* stats, unsigned int& sessions) {
    if (!stats) return 0U;
    guint64 total = 0U;
    const gint fields = gst_structure_n_fields(stats);
    for (gint i = 0; i < fields; ++i) {
        const gchar* name = gst_structure_nth_field_name(stats, i);
        if (!name) continue;
        const GValue* value = gst_structure_get_value(stats, name);
        if (value && GST_VALUE_HOLDS_STRUCTURE(value)) {
            ++sessions;
            unsigned int nested = 0U;
            total += sum_bytes_sent(gst_value_get_structure(value), nested);
        }
    }
    guint64 sent = 0U;
    if (gst_structure_get_uint64(stats, "bytes-sent", &sent)) total += sent;
    return total;
}

void Pipeline::report_whip_delivery() {
    if (!whip_sink_) return;
    GstStructure* stats = nullptr;
    g_object_get(whip_sink_, "stats", &stats, nullptr);
    if (!stats) return;

    unsigned int sessions = 0U;
    const guint64 sent = sum_bytes_sent(stats, sessions);
    gst_structure_free(stats);

    // The sink reports one nested structure per connected consumer, so a
    // session count of zero means the far side never joined - which is what a
    // dead stream key and a firewalled ICE path both look like from here.
    const bool delivering = sessions != 0U && sent > whip_last_bytes_sent_;
    whip_last_bytes_sent_ = sent;

    if (delivering != whip_delivering_ || !whip_delivery_logged_) {
        if (delivering) {
            std::fprintf(
                stderr,
                "[whip] delivering to %u session(s), %llu bytes sent\n",
                sessions,
                static_cast<unsigned long long>(sent));
        } else if (sessions == 0U) {
            std::fprintf(
                stderr,
                "[whip] no session is connected; nothing is being delivered\n");
        } else {
            std::fprintf(
                stderr,
                "[whip] %u session(s) connected but no bytes sent since the "
                "last check\n",
                sessions);
        }
        whip_delivering_ = delivering;
        whip_delivery_logged_ = true;
    }
}

gboolean Pipeline::on_whip_watchdog(gpointer user) {
    auto* self = static_cast<Pipeline*>(user);
    if (self->stop_.load() || !self->whip_pipeline_) {
        self->whip_watchdog_id_ = 0U;
        return G_SOURCE_REMOVE;
    }

    GstState current = GST_STATE_NULL;
    GstState pending = GST_STATE_VOID_PENDING;
    const GstStateChangeReturn state = gst_element_get_state(
        self->whip_pipeline_, &current, &pending, 0U);
    // A healthy WHIP publisher spends its first seconds ASYNC: ICE gathering
    // and the DTLS handshake finish long after the state change is requested,
    // so current is still PAUSED while pending is PLAYING. Treating that as a
    // fault tore down a working WebRTC session every retry interval and the
    // stream could never establish - with no error on the bus to explain it,
    // because nothing had actually failed.
    if (state == GST_STATE_CHANGE_FAILURE
        || (current != GST_STATE_PLAYING && pending != GST_STATE_PLAYING)) {
        std::fprintf(
            stderr,
            "[whip] publisher is not PLAYING; restarting it without interrupting RTSP\n");
        gst_element_set_state(self->whip_pipeline_, GST_STATE_NULL);
        self->whip_needs_keyframe_.store(true);
        // A restart resets the sink's counters, so carrying the old total
        // forward would read as "no bytes since the last check" for one tick.
        self->whip_last_bytes_sent_ = 0U;
        self->whip_delivery_logged_ = false;
        if (gst_element_set_state(self->whip_pipeline_, GST_STATE_PLAYING)
            == GST_STATE_CHANGE_FAILURE) {
            gst_element_set_state(self->whip_pipeline_, GST_STATE_NULL);
        }
    }

    self->report_whip_delivery();

    // A publisher can look healthy locally while delivering nothing - that is
    // how a dead stream key went unnoticed for a day. Report rejected pushes
    // once per watchdog tick rather than per frame.
    const unsigned int failures = self->whip_push_failures_.exchange(0U);
    if (failures != 0U) {
        std::fprintf(
            stderr,
            "[whip] publisher rejected %u frame(s) since the last check\n",
            failures);
    }
    return G_SOURCE_CONTINUE;
}

void Pipeline::infer_loop() {
    std::vector<uint8_t> frame;
    std::vector<uint8_t> out(hailo_->output_frame_size());

    while (!stop_.load()) {
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(200),
                         [&] { return has_pending_ || stop_.load(); });
            if (stop_.load()) break;
            if (!has_pending_) continue;
            frame.swap(pending_);
            has_pending_ = false;
        }

        if (!hailo_->infer(frame.data(), frame.size(), out)) {
            std::fprintf(stderr, "[infer] 추론 실패 — 검출 없이 영상만 계속 송출합니다.\n");
            break;
        }
        auto dets = parse_hailo_nms(out.data(), out.size(), cfg_.num_classes,
                                    cfg_.score_th, cfg_.net, lb_);
        stats_.inferred.fetch_add(1);
        stats_.det_total.fetch_add(dets.size());
        stats_.nms_bytes.store(out.size());
        {
            std::lock_guard<std::mutex> lk(mu_);
            dets_ = std::move(dets);
        }
    }

    // 추론이 멈추면 낡은 박스를 계속 그리지 않는다. 영상 송출은 유지된다.
    {
        std::lock_guard<std::mutex> lk(mu_);
        dets_.clear();
    }
    infer_alive_.store(false);
}

void Pipeline::on_caps_changed(GstElement*, GstCaps* caps, gpointer user) {
    auto* self = static_cast<Pipeline*>(user);
    GstStructure* st = gst_caps_get_structure(caps, 0);
    gint w = 0, h = 0;
    if (gst_structure_get_int(st, "width", &w) && gst_structure_get_int(st, "height", &h)) {
        {
            std::lock_guard<std::mutex> lk(self->mu_);
            self->canvas_w_ = w;
            self->canvas_h_ = h;
        }
        std::printf("[overlay] 캔버스 %dx%d\n", w, h);
        std::fflush(stdout);
    }
}

void Pipeline::on_draw(GstElement*, cairo_t* cr, guint64, guint64, gpointer user) {
    static_cast<Pipeline*>(user)->draw(cr);
}

void Pipeline::draw(cairo_t* cr) {
    std::vector<Detection> dets;
    int W, H;
    {
        std::lock_guard<std::mutex> lk(mu_);
        dets = dets_;
        W = canvas_w_;
        H = canvas_h_;
    }
    const double el = std::max(stats_.elapsed(), 1e-6);
    const double scale = W / 1280.0;
    const bool alive = infer_alive_.load();

    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

    for (const auto& d : dets) {
        const double x = d.x0 * W, y = d.y0 * H;
        const double w = (d.x1 - d.x0) * W, h = (d.y1 - d.y0) * H;
        const Color& c = kColors[d.cls % 4];
        cairo_set_source_rgba(cr, c.r, c.g, c.b, 1.0);
        cairo_set_line_width(cr, std::max(2.0, 3 * scale));
        cairo_rectangle(cr, x, y, w, h);
        cairo_stroke(cr);

        const std::string name = (d.cls < static_cast<int>(cfg_.class_names.size()))
                               ? cfg_.class_names[d.cls] : ("cls" + std::to_string(d.cls));
        const std::string label = fmt("%s %.2f", name.c_str(), d.score);
        const double fs = std::max(12.0, 20 * scale);
        cairo_set_font_size(cr, fs);
        cairo_text_extents_t ext;
        cairo_text_extents(cr, label.c_str(), &ext);
        const double bh = fs + 6;
        cairo_set_source_rgba(cr, c.r, c.g, c.b, 0.9);
        cairo_rectangle(cr, x, std::max(y - bh, 0.0), ext.width + 10, bh);
        cairo_fill(cr);
        cairo_set_source_rgba(cr, 0, 0, 0, 1);
        cairo_move_to(cr, x + 5, std::max(y - 6, fs));
        cairo_show_text(cr, label.c_str());
    }

    const double bar = std::max(22.0, 30 * scale);
    cairo_set_source_rgba(cr, 0, 0, 0, 0.55);
    cairo_rectangle(cr, 0, 0, W, bar);
    cairo_fill(cr);
    cairo_set_source_rgba(cr, 1, 1, 1, 1);
    cairo_set_font_size(cr, std::max(13.0, 18 * scale));
    cairo_move_to(cr, 8, bar * 0.7);

    const std::string hud = alive
        ? fmt("Hailo-8 YOLOv5s | cap %4.1f | inf %4.1f fps | det %zu | total %llu | thr %.2f",
              stats_.captured.load() / el, stats_.inferred.load() / el, dets.size(),
              static_cast<unsigned long long>(stats_.det_total.load()), cfg_.score_th)
        : fmt("INFERENCE DOWN — video only | cap %4.1f fps", stats_.captured.load() / el);
    cairo_show_text(cr, hud.c_str());
}

}  // namespace dc
