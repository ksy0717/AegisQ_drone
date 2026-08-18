// libhailort VStream API 직접 호출. 파이썬판과 달리 별도 헬퍼 프로세스가 없어
// 프레임당 1.2 MB 파이프 IPC 가 사라진다.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace dc {

// Hailo 가용성 판정 결과. 왜 못 쓰는지를 로그에 남기기 위해 사유를 함께 돌려준다.
struct HailoStatus {
    bool usable = false;
    std::string reason;
};

class HailoInference {
public:
    ~HailoInference();
    HailoInference(const HailoInference&) = delete;
    HailoInference& operator=(const HailoInference&) = delete;

    // 정적 조건(카드·노드·HEF)을 확인하고 실제로 열어본다. 실패하면 nullptr 과 사유.
    // 파일과 노드가 있어도 드라이버 버전 불일치·HEF 손상·장치 점유로 실패할 수 있으므로
    // '실제로 여는 것' 까지가 가용성 판정이다.
    static std::unique_ptr<HailoInference> create(const std::string& hef_path,
                                                  double wait_secs,
                                                  HailoStatus& status);

    // PCIe 버스에 Hailo 카드(vendor 0x1e60)가 물려 있는가. 드라이버 로드와는 무관.
    static bool card_present();

    // 프레임 하나 추론. in_size 는 input_frame_size() 와 같아야 한다.
    bool infer(const uint8_t* in, size_t in_size, std::vector<uint8_t>& out);

    size_t input_frame_size() const;
    size_t output_frame_size() const;
    std::string describe() const;

private:
    HailoInference();
    struct Impl;
    std::unique_ptr<Impl> p_;
};

}  // namespace dc
