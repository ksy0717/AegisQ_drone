#include "dc/detection.hpp"

#include <algorithm>
#include <cstring>

namespace dc {

Letterbox Letterbox::make(int src_w, int src_h, int net) {
    const float s = std::min(static_cast<float>(net) / src_w,
                             static_cast<float>(net) / src_h);
    Letterbox lb;
    lb.inner_w = src_w * s;
    lb.inner_h = src_h * s;
    lb.pad_x = (net - lb.inner_w) * 0.5f;
    lb.pad_y = (net - lb.inner_h) * 0.5f;
    return lb;
}

namespace {
inline float rd(const uint8_t* p) {
    float v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}
}  // namespace

std::vector<Detection> parse_hailo_nms(const uint8_t* data, size_t len,
                                       int num_classes, float score_th,
                                       int net, const Letterbox& lb) {
    std::vector<Detection> out;
    size_t off = 0;
    for (int cls = 0; cls < num_classes; ++cls) {
        if (off + 4 > len) break;
        const float fcount = rd(data + off);
        off += 4;
        // 방어: 포맷이 어긋나면 쓰레기 값을 크게 읽는다. 즉시 중단한다.
        if (!(fcount >= 0.0f) || fcount > 1000.0f) break;
        const int count = static_cast<int>(fcount);
        for (int i = 0; i < count; ++i) {
            if (off + 20 > len) break;
            const float y0 = rd(data + off + 0);
            const float x0 = rd(data + off + 4);
            const float y1 = rd(data + off + 8);
            const float x1 = rd(data + off + 12);
            const float sc = rd(data + off + 16);
            off += 20;
            if (sc < score_th) continue;
            Detection d;
            // 레터박스 역변환: net 프레임 좌표 -> 원본 화면 정규좌표
            d.x0 = (x0 * net - lb.pad_x) / lb.inner_w;
            d.x1 = (x1 * net - lb.pad_x) / lb.inner_w;
            d.y0 = (y0 * net - lb.pad_y) / lb.inner_h;
            d.y1 = (y1 * net - lb.pad_y) / lb.inner_h;
            d.score = sc;
            d.cls = cls;
            out.push_back(d);
        }
    }
    return out;
}

}  // namespace dc
