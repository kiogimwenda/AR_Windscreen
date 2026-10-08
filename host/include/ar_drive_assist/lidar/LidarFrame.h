#pragma once
// LidarFrame — one LiDAR scan as it travels on lidarBus (Part 5.1, 8.1). Its own header, free of
// the inference engine's CUDA types, so the LiDAR code is testable in the GPU-free unit suite.

#include <cstdint>
#include <vector>

#include "ar_drive_assist/lidar/GroundPlaneModel.h"
#include "ar_drive_assist/scene/SceneReconstruction.h"

namespace ar_drive_assist {

// One scan (Mid-360: 10 Hz). Points in the LiDAR frame; point times in host microseconds.
struct LidarFrame {
    std::uint64_t timestampMs = 0;  // scan end, host steady clock
    std::vector<LidarPoint> points;
    GroundPlaneModel ground;   // the road surface fitted to this scan (LidarProcessor, Part 8.1)
    bool groundValid = false;  // false: no acceptable fit yet (consumers fall back to flat)
};

}  // namespace ar_drive_assist
