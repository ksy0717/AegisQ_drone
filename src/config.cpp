#include "dc/config.hpp"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace dc {
namespace {

std::string env_s(const char* k, const std::string& d) {
    const char* v = std::getenv(k);
    return (v && *v) ? std::string(v) : d;
}
int env_i(const char* k, int d) {
    const char* v = std::getenv(k);
    return (v && *v) ? std::atoi(v) : d;
}
double env_d(const char* k, double d) {
    const char* v = std::getenv(k);
    return (v && *v) ? std::atof(v) : d;
}
bool env_b(const char* k, bool d) {
    const char* v = std::getenv(k);
    return (v && *v) ? (std::string(v) == "1") : d;
}
std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

bool valid_port(const std::string& value) {
    if (value.empty()) return false;
    unsigned int port = 0U;
    for (const unsigned char character : value) {
        if (std::isdigit(character) == 0) return false;
        const unsigned int digit = static_cast<unsigned int>(character - '0');
        if (port > (65535U - digit) / 10U) return false;
        port = (port * 10U) + digit;
    }
    return port != 0U;
}

bool valid_authority(const std::string& authority) {
    if (authority.empty()) return false;

    if (authority.front() == '[') {
        const std::size_t close = authority.find(']');
        if (close == std::string::npos || close == 1U) return false;
        for (std::size_t index = 1U; index < close; ++index) {
            const unsigned char character =
                static_cast<unsigned char>(authority[index]);
            if (std::isxdigit(character) == 0 && character != ':') return false;
        }
        if (close + 1U == authority.size()) return true;
        return authority[close + 1U] == ':'
            && valid_port(authority.substr(close + 2U));
    }

    const std::size_t colon = authority.find(':');
    if (colon != std::string::npos
        && authority.find(':', colon + 1U) != std::string::npos) {
        return false;
    }
    const std::string host = authority.substr(0U, colon);
    if (host.empty()
        || std::isalnum(static_cast<unsigned char>(host.front())) == 0
        || std::isalnum(static_cast<unsigned char>(host.back())) == 0) {
        return false;
    }
    for (const unsigned char character : host) {
        if (std::isalnum(character) == 0 && character != '.' && character != '-') {
            return false;
        }
    }
    return colon == std::string::npos || valid_port(authority.substr(colon + 1U));
}

bool is_stream_key_char(const unsigned char value) {
    return std::isalnum(value) != 0 || value == '-' || value == '_' || value == '.'
        || value == '~' || value == '+' || value == '=';
}

bool is_bearer_char(const unsigned char value) {
    return std::isalnum(value) != 0 || value == '-' || value == '.' || value == '_'
        || value == '~' || value == '+' || value == '/';
}

}  // namespace

Config Config::from_env() {
    Config c;
    c.src_w = env_i("SRC_W", c.src_w);
    c.src_h = env_i("SRC_H", c.src_h);
    c.infer_w = env_i("INFER_W", c.net);
    // 추론 스트림은 원본과 종횡비가 같아야 레터박스 계산이 맞는다.
    c.infer_h = env_i("INFER_H", static_cast<int>(c.net * 1.0 * c.src_h / c.src_w + 0.5));
    c.dual_stream = env_b("DUAL_STREAM", c.dual_stream);

    c.hef = env_s("HEF", c.hef);
    c.num_classes = env_i("NUM_CLASSES", c.num_classes);
    c.score_th = static_cast<float>(env_d("SCORE_TH", c.score_th));
    auto names = split_csv(env_s("CLASS_NAMES", "class0,class1"));
    if (!names.empty()) c.class_names = names;
    c.hailo_requested = env_b("HAILO", c.hailo_requested);
    c.hailo_wait_secs = env_d("HAILO_WAIT_SECS", c.hailo_wait_secs);

    c.enable_display = env_b("ENABLE_DISPLAY", c.enable_display);
    c.enable_rtsp = env_b("ENABLE_RTSP", c.enable_rtsp);
    c.rtsp_url = env_s("RTSP_URL", c.rtsp_url);
    c.rtsp_bitrate = env_i("RTSP_BITRATE", c.rtsp_bitrate);
    c.rtsp_fps = env_i("RTSP_FPS", c.rtsp_fps);
    c.enable_whip = env_b("ENABLE_WHIP", c.enable_whip);
    c.whip_url = env_s("WHIP_URL", c.whip_url);
    c.whip_token = env_s("WHIP_TOKEN", c.whip_token);
    c.whip_retry_secs = env_i("WHIP_RETRY_SECS", c.whip_retry_secs);

    c.run_secs = env_d("RUN_SECS", c.run_secs);
    c.report_secs = env_d("REPORT_SECS", c.report_secs);
    return c;
}

std::string Config::validation_error() const {
    if (!enable_whip) {
        return {};
    }
    if (whip_url.empty()) {
        return "ENABLE_WHIP=1 requires WHIP_URL";
    }
    if (whip_url.size() > 2048U) {
        return "WHIP_URL exceeds 2048 bytes";
    }

    std::size_t authority_start = 0U;
    if (whip_url.rfind("https://", 0U) == 0U) {
        authority_start = 8U;
    } else if (whip_url.rfind("http://", 0U) == 0U) {
        authority_start = 7U;
    } else {
        return "WHIP_URL must use http:// or https://";
    }

    const std::size_t path_start = whip_url.find('/', authority_start);
    if (path_start == std::string::npos || path_start == authority_start) {
        return "WHIP_URL must include an authority and /w/<stream_key> path";
    }
    const std::string authority =
        whip_url.substr(authority_start, path_start - authority_start);
    if (!valid_authority(authority)) {
        return "WHIP_URL authority is invalid";
    }

    const std::string path = whip_url.substr(path_start);
    if (path.rfind("/w/", 0U) != 0U || path.size() == 3U) {
        return "WHIP_URL path must be exactly /w/<stream_key>";
    }
    const std::string stream_key = path.substr(3U);
    if (stream_key.rfind("stream-", 0U) == 0U) {
        return "WHIP_URL requires the server-issued stream key, not a stream-* room name";
    }
    for (const unsigned char value : stream_key) {
        if (!is_stream_key_char(value)) {
            return "WHIP_URL stream key contains an unsafe character";
        }
    }

    if (whip_token.size() > 4096U) {
        return "WHIP_TOKEN exceeds 4096 bytes";
    }
    if (whip_retry_secs < 1 || whip_retry_secs > 300) {
        return "WHIP_RETRY_SECS must be between 1 and 300";
    }
    bool padding_started = false;
    bool token_value_seen = false;
    for (const unsigned char value : whip_token) {
        if (value == '=') {
            if (!token_value_seen) {
                return "WHIP_TOKEN is not a valid bearer token";
            }
            padding_started = true;
        } else if (padding_started || !is_bearer_char(value)) {
            return "WHIP_TOKEN is not a valid bearer token";
        } else {
            token_value_seen = true;
        }
    }
    return {};
}

void Config::print() const {
    std::printf("[config] %dx%d -> 추론 %dx%d(net %d) 표시 %dx%d | dual=%d "
                "| thr %.2f classes %d | rtsp %s %dkbps | display=%d rtsp=%d whip=%d\n",
                src_w, src_h, infer_w, infer_h, net, disp_w, disp_h,
                dual_stream ? 1 : 0, score_th, num_classes,
                rtsp_url.c_str(), rtsp_bitrate,
                enable_display ? 1 : 0, enable_rtsp ? 1 : 0, enable_whip ? 1 : 0);
    std::fflush(stdout);
}

}  // namespace dc
