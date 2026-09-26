#pragma once
// CameraFrame — what CameraPipeline (Phase 4) publishes on frameBus, and what MlInferenceEngine and
// ArRenderer consume. Kept out of common/Types.h so the GPU-free unit suite never has to include
// OpenCV.

#include <cstdint>
#include <opencv2/core.hpp>

namespace ar_drive_assist {

struct CameraFrame {
    std::uint64_t seq = 0;          // monotonically increasing per capture; gaps = dropped frames
    std::uint64_t timestampMs = 0;  // capture time, steady clock
    cv::Mat bgr;  // CV_8UC3. Reference-counted: copies share pixels, not duplicate
};

}  // namespace ar_drive_assist
