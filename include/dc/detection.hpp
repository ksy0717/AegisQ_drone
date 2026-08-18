// HAILO NMS 출력 파싱과 레터박스 좌표 환산.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace dc {

// 화면 정규좌표(0..1) 기준 박스.
struct Detection {
    float x0, y0, x1, y1;
    float score;
    int cls;
};

// videoscale add-borders=true 와 동일한 레터박스 기하.
// 원본을 net x net 안에 종횡비를 유지해 넣었을 때의 내부 영역과 여백.
struct Letterbox {
    float inner_w, inner_h, pad_x, pad_y;
    static Letterbox make(int src_w, int src_h, int net);
};

// HEF 가 온칩 NMS 를 수행하므로 출력은 이미 디코드된 박스다.
// 포맷(HAILO NMS BY CLASS / float32): 클래스마다
//   [float32 count][count x {y_min, x_min, y_max, x_max, score}], 좌표는 0..1.
// 총 크기 = num_classes * (1 + max_bboxes_per_class * 5) * 4 byte.
std::vector<Detection> parse_hailo_nms(const uint8_t* data, size_t len,
                                       int num_classes, float score_th,
                                       int net, const Letterbox& lb);

}  // namespace dc
