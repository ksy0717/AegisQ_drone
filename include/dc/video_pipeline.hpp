#pragma once

#include <string>

#include "dc/config.hpp"

namespace dc {

// Kept GStreamer-free so output topology and secret handling can be tested on a host.
std::string build_video_pipeline(const Config& config, bool detect, bool dual_stream);
std::string build_whip_pipeline();

}  // namespace dc
