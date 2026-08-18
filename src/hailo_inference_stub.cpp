// libhailort 가 없는 기체에서 쓰는 구현. CMake 가 HailoRT 를 찾지 못하면 이 파일이
// 대신 컴파일된다 — Hailo 미장착 드론에서도 빌드와 실행이 되어야 하기 때문이다.
// (card_present() 는 hailo_probe.cpp 에 있으며 항상 컴파일된다.)
#include "dc/hailo_inference.hpp"

namespace dc {

struct HailoInference::Impl {};

HailoInference::HailoInference() : p_(std::make_unique<Impl>()) {}
HailoInference::~HailoInference() = default;

std::unique_ptr<HailoInference> HailoInference::create(const std::string&, double,
                                                       HailoStatus& status) {
    status.usable = false;
    status.reason = card_present()
        ? "Hailo 카드는 PCIe 에 있으나 이 빌드에 HailoRT 지원이 없음 "
          "(hailort deb 설치 후 재빌드하면 검출이 켜진다)"
        : "Hailo 카드가 PCIe 에 없음 (미장착) — 빌드에도 HailoRT 미포함";
    return nullptr;
}

bool HailoInference::infer(const uint8_t*, size_t, std::vector<uint8_t>&) { return false; }
size_t HailoInference::input_frame_size() const { return 0; }
size_t HailoInference::output_frame_size() const { return 0; }
std::string HailoInference::describe() const { return "HailoRT 미포함 빌드"; }

}  // namespace dc
