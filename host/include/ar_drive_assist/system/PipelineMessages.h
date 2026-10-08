#pragma once
// The messages that travel between the host's threads (Part 5.1), and the buses that carry them.
// Every bus is a single-producer / single-consumer RingBuffer (Part 5.3): where two threads need
// the same messages, the producer writes to two buses.
//
//   CameraThread     --frameBus-->        InferenceThread --detectionBus, maskBus-->  FusionThread
//                    --displayBus-->      RenderThread
//   LidarThread      --lidarBus-->        FusionThread
//   FusionThread     --sceneBus-->        DecisionThread
//                    --renderSceneBus-->  RenderThread
//   DecisionThread   --decisionBus-->     RenderThread      (and ActuationRequests to the hub)
//   NavigationThread --navBus-->          RenderThread
//
// Messages are plain copies: a thread owns what it popped and shares nothing with the producer
// (cv::Mat pixels are the exception, shared read-only; CameraPipeline.h explains why that is safe).

#include <cstdint>
#include <vector>

#include "ar_drive_assist/common/RingBuffer.h"
#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/decision/DecisionArbiter.h"
#include "ar_drive_assist/inference/MlInferenceEngine.h"
#include "ar_drive_assist/lidar/LidarFrame.h"
#include "ar_drive_assist/nav/RoadSurfaceProjector.h"
#include "ar_drive_assist/safety/MultiObjectTracker.h"
#include "ar_drive_assist/safety/RecklessDrivingDetector.h"
#include "ar_drive_assist/scene/SceneReconstruction.h"

namespace ar_drive_assist {

// What FusionThread knows after one camera frame: the fused scene and the tracks.
struct SceneSnapshot {
    std::uint64_t timestampMs = 0;  // the camera frame's capture time
    std::uint64_t frameSeq = 0;
    bool egoValid = false;
    EgoState ego;
    ImmFilter::State egoX = ImmFilter::State::Zero();  // the EKF's state and covariance at that
    ImmFilter::Cov egoP = ImmFilter::Cov::Identity();  // time, for MotionPredictor::risk
    std::vector<Track> tracks;
    SceneModel scene;                 // one FusedObject per detection, plus LiDAR-only obstacles
    DetectionFrame detections;        // boxes, lanes, signs: what the renderer draws from
    std::uint64_t lidarAgeMs = 0;     // camera time minus the scan used (0 also when none was)
    bool lidarUsed = false;           // a scan was fused with this frame
    bool extrinsicsDegraded = false;  // ExtrinsicMonitor DEGRADED: no mask-based ranging (12.2.1)
    GroundPlaneModel ground;          // the newest fitted road surface (for navigation)
    bool groundValid = false;
};

// One track as the decision saw it, with no pointer into another thread's memory.
struct TrackView {
    int trackId = 0;
    int32_t objectClass = -1;
    Eigen::Vector2d posVehicle = Eigen::Vector2d::Zero();  // x forward, y left (m)
    double gapM = 0, closingSpeedMps = 0;
    bool inEgoPath = false, confirmed = false, rangeMeasured = false;
    bool inStraightPath = false;
    std::uint64_t measuredRunMs = 0;  // unbroken LiDAR-measured run (rule 1 needs 200 ms)
    int state = 0;                    // TrackState
    ThreatAssessment threat;
};

struct DecisionSnapshot {
    std::uint64_t timestampMs = 0;
    Decision decision;
    std::vector<TrackView> tracks;
};

using FrameBus = RingBuffer<CameraFrame, 4>;
using LidarBus = RingBuffer<LidarFrame, 4>;
using SceneBus = RingBuffer<SceneSnapshot, 4>;
using DecisionBus = RingBuffer<DecisionSnapshot, 4>;
using NavBus = RingBuffer<ProjectedRoute, 4>;

}  // namespace ar_drive_assist
