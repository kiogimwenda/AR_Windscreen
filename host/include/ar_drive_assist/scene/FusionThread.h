#pragma once
// FusionThread — Part 5.1's thread 4: camera detections + LiDAR + the ego estimate -> the fused
// scene and the object tracks. See docs/BUILD_GUIDE.md Part 8.3 and Part 9.1.
//
// ---------------------------------------------------------------------------------------------
// One camera frame, step by step
//
//   detections  the newest DetectionFrame (popLatest: never work on a stale frame), with the
//               masks of the SAME frame (MaskFrame.seq == DetectionFrame.mask_ref)
//   scan        the newest LiDAR scan, used only if it is within maxLidarAgeMs of the frame;
//               SceneReconstruction moves each point to the frame's time (rule 1)
//   ego         the EKF's snapshot (EgoEstimator)
//   merge       SceneReconstruction: one FusedObject per detection, plus LiDAR-only obstacles
//   measure     each object with a range becomes a tracker measurement. A LiDAR range is a
//               MEASURED range; a MiDaS estimate may move a track but never counts as a measured
//               range for braking (Part 8.3 rule 5); an object with no range is not tracked.
//               LiDAR-only obstacles in the corridor are tracked as class -1.
//   track       MultiObjectTracker, in the world frame (it needs the ego pose, so nothing is
//               tracked before the EKF has its first GNSS fix)
//   publish     a SceneSnapshot to the decision thread and, if they run, the renderer and the
//               navigation thread (which needs the detected lanes)
// ---------------------------------------------------------------------------------------------

#include <atomic>
#include <cstdint>
#include <map>

#include "ar_drive_assist/fusion/EgoEstimator.h"
#include "ar_drive_assist/scene/ExtrinsicMonitor.h"
#include "ar_drive_assist/system/PipelineMessages.h"

namespace ar_drive_assist {

class EventLog;

struct FusionThreadConfig {
    CameraModel camera;  // the model of the PUBLISHED frames: CameraPipeline::frameModel()
    FusionExtrinsics extrinsics;
    GroundPlane ground;  // flat road, only until the first scan with a fitted surface (Part 8.1)
    FusionConfig fusion;
    TrackerConfig tracker;
    double lidarSigmaM = 0.2;           // 1-sigma position error of a LiDAR-measured object
    double estimatedSigmaM = 2.0;       // ... of a MiDaS-scaled estimate
    std::uint64_t maxLidarAgeMs = 150;  // older scans are not fused with the frame
    // Two detections over one object (a box and an occluded neighbour's partial mask) choose the
    // same LiDAR cluster and so the same near face. Measurements this close are one object: the
    // duplicate would start a second track and break the real one's measured run (KITTI replay,
    // 2026-10-07). Small enough never to merge two people walking shoulder to shoulder.
    double duplicateRadiusM = 0.3;
    double reportEveryS = 10.0;
    // Part 12.2.1, optional: the camera-from-LiDAR transform to project with (a refinement moves
    // the camera's projection only; the LiDAR's place in the vehicle, which braking uses, stays
    // the calibrated one), and in DEGRADED no camera mask is used to range anything: every point
    // goes to the LiDAR-only obstacles in the vehicle-frame corridor.
    const ExtrinsicMonitor* monitor = nullptr;
};

class FusionThread {
public:
    FusionThread(FusionThreadConfig cfg, MlInferenceEngine::DetectionBus& detections,
                 MlInferenceEngine::MaskBus* masks, LidarBus* lidar, EgoEstimator& ego,
                 SceneBus& toDecision, SceneBus* toRender = nullptr, EventLog* log = nullptr,
                 SceneBus* toNavigation = nullptr);

    void run(const std::atomic<bool>& stop);

    // One frame, synchronously (run() calls it; tests call it directly).
    SceneSnapshot process(const DetectionFrame& det, const MaskFrame* masks, const LidarFrame* scan,
                          const EgoEstimator::Snapshot& ego);

    // Tracker measurements for a fused scene (exposed for tests).
    std::vector<ObjectMeasurement> measurements(const SceneModel& scene,
                                                const FusionFrame& frame) const;

    struct Stats {
        std::uint64_t frames = 0, withLidar = 0, staleScans = 0, untrackedNoEgo = 0;
    };
    Stats stats() const { return stats_; }

private:
    FusionThreadConfig cfg_;
    MlInferenceEngine::DetectionBus& detections_;
    MlInferenceEngine::MaskBus* masks_;
    LidarBus* lidar_;
    EgoEstimator& ego_;
    SceneBus& toDecision_;
    SceneBus* toRender_;
    SceneBus* toNav_;
    EventLog* log_;
    SceneReconstruction scene_;
    MultiObjectTracker tracker_;
    std::map<std::uint64_t, MaskFrame> maskBySeq_;  // masks waiting for their detections
    LidarFrame latestScan_;
    GroundPlaneModel ground_;
    bool haveGround_ = false;
    bool haveScan_ = false;
    Stats stats_;
};

}  // namespace ar_drive_assist
