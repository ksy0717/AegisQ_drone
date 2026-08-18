// HailoRT 링크 여부와 무관하게 항상 컴파일된다.
// 카드가 물리적으로 꽂혀 있는지는 libhailort 없이도 확인할 수 있어야 한다 —
// "카드는 있는데 빌드에 지원이 빠졌다" 와 "카드가 없다" 를 구분해 로그에 남기기 위함이다.
#include "dc/hailo_inference.hpp"

#include <glob.h>

#include <cctype>
#include <fstream>
#include <string>

namespace dc {

namespace {
constexpr const char* kHailoVendor = "0x1e60";
}

bool HailoInference::card_present() {
    glob_t g{};
    bool found = false;
    if (glob("/sys/bus/pci/devices/*/vendor", 0, nullptr, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc && !found; ++i) {
            std::ifstream f(g.gl_pathv[i]);
            std::string v;
            if (f >> v) {
                for (auto& c : v) c = static_cast<char>(std::tolower(c));
                if (v == kHailoVendor) found = true;
            }
        }
    }
    globfree(&g);
    return found;
}

}  // namespace dc
