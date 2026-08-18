#include "dc/hailo_inference.hpp"

#include <glob.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <thread>

#include <hailo/hailort.hpp>

namespace dc {

namespace {
std::vector<std::string> glob_paths(const char* pattern) {
    std::vector<std::string> out;
    glob_t g{};
    if (glob(pattern, 0, nullptr, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; ++i) out.emplace_back(g.gl_pathv[i]);
    }
    globfree(&g);
    return out;
}

bool file_exists(const std::string& p) {
    struct stat st{};
    return ::stat(p.c_str(), &st) == 0;
}

std::string join(const std::vector<std::string>& v, const char* sep) {
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += sep;
        s += v[i];
    }
    return s;
}
}  // namespace

struct HailoInference::Impl {
    std::unique_ptr<hailort::VDevice> vdevice;
    std::shared_ptr<hailort::ConfiguredNetworkGroup> group;
    std::vector<hailort::InputVStream> inputs;
    std::vector<hailort::OutputVStream> outputs;
    size_t in_size = 0;
    size_t out_size = 0;
    std::string desc;
};

HailoInference::HailoInference() : p_(std::make_unique<Impl>()) {}
HailoInference::~HailoInference() = default;

namespace {
// 부팅 직후에는 PCIe 열거·드라이버 로드가 서비스 기동보다 늦을 수 있다.
// 카드가 실제로 꽂혀 있을 때만 기다린다 — 미장착 기체가 매번 지연되면 안 된다.
std::vector<std::string> wait_for_node(double timeout) {
    auto nodes = glob_paths("/dev/hailo*");
    if (!nodes.empty() || timeout <= 0) return nodes;
    if (!HailoInference::card_present()) return {};

    std::printf("[hailo] 카드는 있으나 장치 노드가 아직 없음 — 최대 %.0fs 대기\n", timeout);
    std::fflush(stdout);
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(static_cast<int>(timeout * 1000));
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        nodes = glob_paths("/dev/hailo*");
        if (!nodes.empty()) return nodes;
    }
    return {};
}
}  // namespace

std::unique_ptr<HailoInference> HailoInference::create(const std::string& hef_path,
                                                       double wait_secs,
                                                       HailoStatus& status) {
    status.usable = false;

    if (!file_exists(hef_path)) {
        status.reason = "HEF 모델이 없음: " + hef_path;
        return nullptr;
    }
    const auto nodes = wait_for_node(wait_secs);
    if (nodes.empty()) {
        status.reason = card_present()
            ? "Hailo 카드는 PCIe 에 있으나 제한 시간 안에 /dev/hailo* 가 나타나지 않음 (드라이버 미로드 의심)"
            : "Hailo 카드가 PCIe 에 없음 (미장착)";
        return nullptr;
    }

    auto self = std::unique_ptr<HailoInference>(new HailoInference());
    auto& im = *self->p_;

    auto vdev = hailort::VDevice::create();
    if (!vdev) {
        status.reason = "VDevice 생성 실패 (status " + std::to_string(vdev.status()) + ")";
        return nullptr;
    }
    im.vdevice = vdev.release();

    auto hef = hailort::Hef::create(hef_path);
    if (!hef) {
        status.reason = "HEF 로드 실패 (status " + std::to_string(hef.status()) + ")";
        return nullptr;
    }

    auto params = im.vdevice->create_configure_params(hef.value());
    if (!params) {
        status.reason = "configure params 생성 실패";
        return nullptr;
    }
    auto groups = im.vdevice->configure(hef.value(), params.value());
    if (!groups || groups->empty()) {
        status.reason = "network group 구성 실패";
        return nullptr;
    }
    im.group = groups->at(0);

    // unused=false, AUTO -> 입력 UINT8 / 출력 FLOAT32 (HEF 정의를 따름)
    auto vs = hailort::VStreamsBuilder::create_vstreams(*im.group, false, HAILO_FORMAT_TYPE_AUTO);
    if (!vs) {
        status.reason = "vstream 생성 실패 (status " + std::to_string(vs.status()) + ")";
        return nullptr;
    }
    auto pair = vs.release();
    im.inputs = std::move(pair.first);
    im.outputs = std::move(pair.second);
    if (im.inputs.empty() || im.outputs.empty()) {
        status.reason = "vstream 이 비어 있음";
        return nullptr;
    }

    im.in_size = im.inputs[0].get_frame_size();
    im.out_size = im.outputs[0].get_frame_size();
    const auto nms = im.outputs[0].get_info().nms_shape;

    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "장치 %s | in %s %zuB | out %s %zuB | classes %u max/class %u",
                  join(nodes, ",").c_str(),
                  im.inputs[0].name().c_str(), im.in_size,
                  im.outputs[0].name().c_str(), im.out_size,
                  nms.number_of_classes, nms.max_bboxes_per_class);
    im.desc = buf;

    status.usable = true;
    status.reason = im.desc;
    return self;
}

bool HailoInference::infer(const uint8_t* in, size_t in_size, std::vector<uint8_t>& out) {
    auto& im = *p_;
    if (in_size != im.in_size) return false;
    if (out.size() != im.out_size) out.resize(im.out_size);

    auto st = im.inputs[0].write(hailort::MemoryView(const_cast<uint8_t*>(in), in_size));
    if (HAILO_SUCCESS != st) return false;
    st = im.outputs[0].read(hailort::MemoryView(out.data(), out.size()));
    return HAILO_SUCCESS == st;
}

size_t HailoInference::input_frame_size() const { return p_->in_size; }
size_t HailoInference::output_frame_size() const { return p_->out_size; }
std::string HailoInference::describe() const { return p_->desc; }

}  // namespace dc
