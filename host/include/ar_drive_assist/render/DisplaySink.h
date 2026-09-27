#pragma once
// DisplaySink — where the composited picture goes. See docs/BUILD_GUIDE.md Part 10.1.
//
// The renderer hands over the video frame plus an OverlayScene (what to draw, not a finished
// picture); the sink draws both. That seam is what lets a future ProjectorSink draw only the
// overlay scene through combiner optics and ignore the video. Only WindowedSink is implemented
// in this build (Part 0: the optical path is out of scope).

#include <cstdint>
#include <opencv2/core.hpp>

#include "ar_drive_assist/common/Camera.h"
#include "ar_drive_assist/render/OverlayScene.h"

namespace ar_drive_assist {

struct CompositedFrame {
    cv::Mat video;  // BGR, the camera frame being displayed
    OverlayScene overlays;
    std::uint64_t timestampMs = 0;
};

struct FrameGeometry {
    int width = 0;
    int height = 0;
    float aspectRatio = 0;
};

class DisplaySink {
public:
    virtual ~DisplaySink() = default;
    // `camera` and `cameraFromVehicle`: the calibration road-space items are projected with.
    virtual bool init(const FrameGeometry& geometry, const CameraModel& camera,
                      const Eigen::Isometry3d& cameraFromVehicle) = 0;
    virtual void present(const CompositedFrame& frame) = 0;
    virtual FrameGeometry outputGeometry() const = 0;
};

}  // namespace ar_drive_assist
