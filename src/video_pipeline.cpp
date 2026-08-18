#include "dc/video_pipeline.hpp"

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

namespace dc {
namespace {

constexpr const char* kBoundedQueue =
    "queue leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0";

std::string fmt(const char* format, ...)
{
    char buffer[2048];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    return buffer;
}

std::string quote_property(const std::string& value)
{
    std::string result = "\"";
    result.reserve(value.size() + 2U);
    for (const char character : value) {
        if (character == '\\' || character == '"') {
            result.push_back('\\');
        }
        result.push_back(character);
    }
    result.push_back('"');
    return result;
}

std::string branch_tail(
    const std::vector<std::string>& branches, const std::string& tee_name)
{
    if (branches.empty()) {
        return "! fakesink sync=false ";
    }
    if (branches.size() == 1U) {
        return "! " + branches.front();
    }

    std::string result = "! tee name=" + tee_name + " ";
    for (const auto& branch : branches) {
        result += tee_name + ". ! " + kBoundedQueue + " ! " + branch;
    }
    return result;
}

std::string encoded_output(const Config& config)
{
    std::vector<std::string> sinks;
    if (config.enable_rtsp) {
        sinks.push_back(
            "rtspclientsink location=" + quote_property(config.rtsp_url)
            + " protocols=tcp latency=0 name=rtsp ");
    }
    if (config.enable_whip) {
        // WHIP has its own GstPipeline because a signalling state transition can remain
        // ASYNC indefinitely. A bounded bridge keeps that from stalling RTSP preroll.
        sinks.emplace_back(
            "appsink name=whip_bridge emit-signals=true sync=false "
            "max-buffers=2 drop=true ");
    }
    if (sinks.empty()) {
        return {};
    }

    std::string result =
        fmt("videoconvert ! video/x-raw,format=I420 "
            "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=%d "
            "key-int-max=%d sliced-threads=true byte-stream=true bframes=0 "
            "! h264parse config-interval=-1 "
            "! video/x-h264,profile=baseline,stream-format=byte-stream,alignment=au ",
            config.rtsp_bitrate, config.rtsp_fps);
    result += branch_tail(sinks, "encoded_t");
    return result;
}

std::string output_tail(const Config& config)
{
    std::vector<std::string> outputs;
    if (config.enable_display) {
        outputs.push_back(
            fmt("videoconvert ! videoscale ! video/x-raw,width=%d,height=%d "
                "! videoconvert ! fbdevsink device=/dev/fb0 sync=false name=disp ",
                config.disp_w, config.disp_h));
    }

    const std::string network = encoded_output(config);
    if (!network.empty()) {
        outputs.push_back(network);
    }
    return branch_tail(outputs, "output_t");
}

}  // namespace

std::string build_video_pipeline(
    const Config& config, const bool detect, const bool dual_stream)
{
    const std::string tail = output_tail(config);
    const std::string main_caps =
        fmt("video/x-raw,width=%d,height=%d,format=NV12 ", config.src_w, config.src_h);

    if (!detect) {
        return "libcamerasrc ! " + main_caps
            + "! identity name=frame_counter signal-handoffs=true silent=true " + tail;
    }

    const std::string overlay =
        "! queue leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
        + fmt("! videoconvert ! video/x-raw,format=BGRA,width=%d,height=%d ",
              config.src_w, config.src_h)
        + "! cairooverlay name=ov " + tail;
    const std::string appsink =
        "! appsink name=det emit-signals=true sync=false max-buffers=1 drop=true ";

    if (dual_stream) {
        // A second ISP stream avoids paying for full-frame CPU conversion before inference.
        return "libcamerasrc name=cam cam.src ! " + main_caps
            + "! identity name=frame_counter signal-handoffs=true silent=true " + overlay
            + fmt("cam.src_1 ! video/x-raw,width=%d,height=%d,format=RGB ",
                  config.infer_w, config.infer_h)
            + "! queue leaky=downstream max-size-buffers=2 "
              "max-size-time=0 max-size-bytes=0 "
            + "! videoscale add-borders=true "
            + fmt("! video/x-raw,width=%d,height=%d,pixel-aspect-ratio=1/1 ",
                  config.net, config.net)
            + appsink;
    }

    // The fallback keeps one camera owner and performs inference conversion after a tee.
    return "libcamerasrc ! " + main_caps
        + "! identity name=frame_counter signal-handoffs=true silent=true "
        + "! tee name=camera_t "
        + "camera_t. ! queue leaky=downstream max-size-buffers=2 "
          "max-size-time=0 max-size-bytes=0 "
        + "! videoconvert ! videoscale add-borders=true "
        + fmt("! video/x-raw,format=RGB,width=%d,height=%d,pixel-aspect-ratio=1/1 ",
              config.net, config.net)
        + appsink + "camera_t. " + overlay;
}

std::string build_whip_pipeline()
{
    // The bridge carries the encoded tee's byte-stream, but whipclientsink
    // negotiates H.264 only as AVC - byte-stream fails with not-negotiated
    // before any signalling happens, which looks like a broken endpoint. The
    // AegisQ handover note asking for Annex-B describes ffmpeg's WHIP muxer,
    // not this one. h264parse does the repackaging; SPS/PPS are already
    // in-band from config-interval=-1, so it can build the codec_data.
    //
    // Endpoint and bearer token are applied through the signaller GObject later.
    // Keeping secrets out of the parse string also keeps them out of diagnostics.
    // do-timestamp=true, and the bridge clears PTS/DTS before pushing: the
    // buffers carry the camera pipeline's running time, and webrtcsink puts a
    // clocksync in front of its input, so a stale timestamp makes it hold the
    // media until this pipeline's clock catches up. The endpoint sees a session
    // that connects and then delivers nothing. The two pipelines have
    // independent base times and this one restarts on its own, so the gap only
    // grows. max-bytes/max-time are pinned to 0 because appsrc's 200 kB default
    // would otherwise evict a large IDR that max-buffers=2 was meant to admit.
    return "appsrc name=whip_source is-live=true format=time do-timestamp=true "
           "block=false max-buffers=2 max-bytes=0 max-time=0 min-latency=0 "
           "leaky-type=downstream "
           "! video/x-h264,stream-format=byte-stream,alignment=au "
           "! h264parse config-interval=-1 "
           "! video/x-h264,stream-format=avc,alignment=au "
           "! queue leaky=downstream max-size-buffers=2 "
           "max-size-time=0 max-size-bytes=0 "
           "! whipclientsink name=whip ";
}

}  // namespace dc
