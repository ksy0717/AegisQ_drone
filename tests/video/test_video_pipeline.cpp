#include <cstddef>
#include <string>

#include "dc/config.hpp"
#include "dc/video_pipeline.hpp"
#include "test_support.hpp"

namespace {

std::size_t count_occurrences(const std::string& value, const std::string& needle)
{
    std::size_t count = 0U;
    std::size_t offset = 0U;
    while ((offset = value.find(needle, offset)) != std::string::npos) {
        ++count;
        offset += needle.size();
    }
    return count;
}

dc::Config valid_whip_config()
{
    dc::Config config;
    config.enable_whip = true;
    config.whip_url = "http://10.100.0.1:8085/w/key_DRONE-01.abc+123=";
    config.whip_token = "header.payload_signature-1";
    return config;
}

void check_rejected_url(const std::string& url, const std::string& label)
{
    dc::Config config = valid_whip_config();
    config.whip_url = url;
    test_support::check(!config.validation_error().empty(), label);
}

void test_whip_validation()
{
    dc::Config disabled;
    test_support::check(disabled.validation_error().empty(), "WHIP disabled by default");

    dc::Config missing = valid_whip_config();
    missing.whip_url.clear();
    test_support::check(
        !missing.validation_error().empty(), "enabled WHIP requires an endpoint");

    check_rejected_url(
        "ftp://10.100.0.1:8085/w/key", "non-HTTP WHIP endpoint rejected");
    check_rejected_url(
        "http://user@10.100.0.1:8085/w/key", "endpoint userinfo rejected");
    check_rejected_url(
        "http://:8085/w/key", "endpoint without host rejected");
    check_rejected_url(
        "http://10.100.0.1:70000/w/key", "endpoint invalid port rejected");
    check_rejected_url(
        "http://2001:db8::1/w/key", "unbracketed IPv6 endpoint rejected");
    check_rejected_url(
        "http://10.100.0.1:8085/stream/key", "wrong endpoint path rejected");
    check_rejected_url(
        "http://10.100.0.1:8085/w/stream-DRONE-01", "room name rejected as stream key");
    check_rejected_url(
        "http://10.100.0.1:8085/w/key/extra", "multi-segment stream key rejected");
    check_rejected_url(
        "http://10.100.0.1:8085/w/key?debug=1", "endpoint query rejected");

    dc::Config valid = valid_whip_config();
    test_support::check(valid.validation_error().empty(), "server-issued endpoint accepted");

    valid.whip_token = "abc=def";
    test_support::check(
        !valid.validation_error().empty(), "invalid bearer-token padding rejected");
    valid.whip_token = "=";
    test_support::check(
        !valid.validation_error().empty(), "padding-only bearer token rejected");

    valid = valid_whip_config();
    valid.whip_retry_secs = 0;
    test_support::check(
        !valid.validation_error().empty(), "zero WHIP retry interval rejected");
    valid.whip_retry_secs = 301;
    test_support::check(
        !valid.validation_error().empty(), "excessive WHIP retry interval rejected");
}

void test_shared_encoder_topology()
{
    dc::Config config = valid_whip_config();
    config.enable_display = false;
    config.enable_rtsp = true;
    config.rtsp_url = "rtsp://127.0.0.1:8554/cam0";

    const std::string pipeline = dc::build_video_pipeline(config, false, false);
    test_support::check(
        count_occurrences(pipeline, "x264enc") == 1U, "RTSP and WHIP share one encoder");
    test_support::check(
        pipeline.find("identity name=frame_counter") != std::string::npos,
        "output frame-rate counter is present");
    test_support::check(
        pipeline.find("tee name=encoded_t") != std::string::npos, "encoded tee present");
    test_support::check(
        pipeline.find("rtspclientsink") != std::string::npos, "RTSP branch present");
    test_support::check(
        pipeline.find("appsink name=whip_bridge") != std::string::npos,
        "bounded WHIP bridge branch present");
    test_support::check(
        pipeline.find("whipclientsink") == std::string::npos,
        "network WHIP sink is isolated from the camera pipeline");
    test_support::check(
        pipeline.find("bframes=0") != std::string::npos, "B-frames disabled");
    test_support::check(
        pipeline.find("profile=baseline,stream-format=byte-stream,alignment=au")
            != std::string::npos,
        "WHIP-compatible H264 caps fixed");
    test_support::check(
        count_occurrences(pipeline, "queue leaky=downstream") == 2U,
        "each network branch is isolated by a bounded leaky queue");
    test_support::check(
        pipeline.find(config.whip_url) == std::string::npos,
        "WHIP endpoint excluded from pipeline diagnostics");
    test_support::check(
        pipeline.find(config.whip_token) == std::string::npos,
        "WHIP bearer token excluded from pipeline diagnostics");

    const std::string whip_pipeline = dc::build_whip_pipeline();
    test_support::check(
        whip_pipeline.find("appsrc name=whip_source") != std::string::npos,
        "WHIP publisher consumes the encoded bridge");
    test_support::check(
        whip_pipeline.find("block=false max-buffers=2 max-bytes=0 max-time=0")
            != std::string::npos,
        "WHIP appsrc is bounded by buffers alone, not appsrc's byte default");
    // The bridge clears PTS/DTS because they belong to the camera pipeline's
    // timeline; webrtcsink's clocksync would otherwise hold the media.
    test_support::check(
        whip_pipeline.find("do-timestamp=true") != std::string::npos,
        "WHIP appsrc restamps in its own running time");
    test_support::check(
        whip_pipeline.find("whipclientsink name=whip") != std::string::npos,
        "isolated WHIP publisher is present");
    // whipclientsink refuses byte-stream H.264 with not-negotiated, which
    // never reaches signalling and so reads as an endpoint fault.
    test_support::check(
        whip_pipeline.find("video/x-h264,stream-format=avc,alignment=au")
            != std::string::npos,
        "the bridge repackages to AVC for whipclientsink");
    test_support::check(
        whip_pipeline.find("stream-format=byte-stream") <
            whip_pipeline.find("stream-format=avc"),
        "the bridge accepts the encoder's byte-stream before repackaging it");
    test_support::check(
        whip_pipeline.find("x264enc") == std::string::npos,
        "WHIP publisher does not add a second encoder");
}

void test_output_modes()
{
    dc::Config whip_only = valid_whip_config();
    whip_only.enable_display = false;
    whip_only.enable_rtsp = false;
    const std::string single = dc::build_video_pipeline(whip_only, false, false);
    test_support::check(
        count_occurrences(single, "x264enc") == 1U, "WHIP-only mode has one encoder");
    test_support::check(
        single.find("tee name=encoded_t") == std::string::npos,
        "single network consumer is directly linked");

    dc::Config none;
    none.enable_display = false;
    none.enable_rtsp = false;
    const std::string sinkless = dc::build_video_pipeline(none, false, false);
    test_support::check(
        sinkless.find("fakesink sync=false") != std::string::npos,
        "no-output mode remains a valid pipeline");

    dc::Config dual = valid_whip_config();
    dual.enable_rtsp = true;
    const std::string detected = dc::build_video_pipeline(dual, true, true);
    test_support::check(
        detected.find("cam.src_1") != std::string::npos, "dual ISP inference path retained");
    test_support::check(
        count_occurrences(detected, "x264enc") == 1U,
        "detected RTSP and WHIP output still shares one encoder");
}

}  // namespace

int main()
{
    test_whip_validation();
    test_shared_encoder_topology();
    test_output_modes();
    return test_support::report("video_pipeline_tests");
}
