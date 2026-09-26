#pragma once
// SceneReconstruction — mask-based camera-LiDAR fusion. See docs/BUILD_GUIDE.md Part 8.3
// (amended 2026-09-25) and docs/decisions.md, "Mask-based LiDAR fusion and startup extrinsic
// check".
//
// ---------------------------------------------------------------------------------------------
// The idea. Every LiDAR point is projected into the camera image. The points that land inside an
// object's YOLOv8m-seg mask belong to that object. The camera supplies WHAT it is and its outline;
// the LiDAR supplies WHERE it is in metres. Masks, not boxes: a pedestrian's box is mostly road
// and the wall behind them, and box-selected points would mix the pedestrian's range with the
// wall's.
//
// The pipeline, and the Part 8.3 rule behind each step:
//   1. Motion compensation (rule 1). A Livox scan fills in over ~100 ms while the car moves, so
//      each point is moved to where it would have been seen at the CAMERA frame's time, using the
//      ego speed and yaw rate. Without it, points smear sideways off objects.
//   2. Ground removal. Points within `groundTolM` of the fitted road plane are road surface,
//      not objects (they belong to GroundPlaneModel, Part 8.1).
//   3. Projection with the calibrated lens model (Part 12.1: pinhole plus OpenCV's k1, k2, p1,
//      p2, k3 distortion). Detections are in the raw camera image, so points are distorted the
//      same way the lens distorts light.
//   4. Depth test (rule 4). The LiDAR sits in a different place from the camera and sees some
//      points the camera cannot (behind a pedestrian's shoulder, say). Only the nearest point per
//      `zBufferCellPx` image cell is visible to the camera; the others cannot be in its masks.
//   5. Assignment through ERODED masks (rule 2). Masks are computed at 1/4 resolution with soft
//      edges, and a small extrinsic error pushes projections outward, so each mask is shrunk by
//      `erodeCells` prototype cells before use.
//   6. Range from the NEAREST DOMINANT depth cluster, never the mean (rule 3). The object's points
//      are split into depth clusters at gaps over `depthGapM`. The nearest cluster holding at
//      least `minClusterFraction` of them is the object; one stray background point cannot move
//      a pedestrian 10 m away.
//   7. Honest "no range" (rule 5). Under `minObjectPoints` points: no LiDAR range. An estimate
//      from MiDaS relative inverse depth, scaled by this frame's LiDAR-ranged objects, may be
//      given, FLAGGED as ESTIMATED. Part 9.3 braking never uses it (usableForBraking()).
//   8. Unexplained obstacles (rule 6). Non-ground points in the ego corridor that no object
//      claimed are clustered; clusters of `minUnknownPoints` or more become UNKNOWN obstacles.
//      The camera adds information, but can never veto what the LiDAR physically measures. This
//      uses ALL points, including those hidden from the camera by the depth test. Points more
//      than `maxObstacleHeightM` above the road (gantries, branches) are ignored.
//   9. Signs (rule 7). Signs come from a box-only detector. Retroreflective points (intensity at
//      least `signMinIntensity`) inside the box give its position, by the same nearest-dominant
//      rule.
//
// Frames. Outputs are in the VEHICLE frame (x forward, y left, z up, metres, origin on the ground
// under the rear axle as in Part 12.2), which is what MultiObjectTracker consumes.
//
// This class is pure geometry: it takes points, masks and calibration as plain values, so every
// rule can be tested with a synthetic scene. Adapting from the bus messages (SceneCloud,
// DetectionFrame, the MaskBus) is the node's job, wired when LidarProcessor exists (Phase 6).
// ---------------------------------------------------------------------------------------------

#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <vector>

#include "ar_drive_assist/inference/Postprocess.h"

namespace ar_drive_assist {

struct CameraModel {  // config/camera_intrinsics.yaml (Part 12.1)
    double fx = 1000, fy = 1000, cx = 960, cy = 540;
    int width = 1920, height = 1080;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;  // OpenCV distortion model

    // Camera-frame point (x right, y down, z forward) -> raw image pixel. False if behind.
    bool project(const Eigen::Vector3d& pc, Eigen::Vector2d& px) const;
};

struct FusionExtrinsics {                // Part 12.2 / 12.2.1 (ExtrinsicMonitor's current state)
    Eigen::Isometry3d cameraFromLidar;   // LiDAR frame -> camera frame
    Eigen::Isometry3d vehicleFromLidar;  // LiDAR frame -> vehicle frame
};

struct LidarPoint {
    Eigen::Vector3f p;     // LiDAR frame, metres
    float intensity = 0;   // Livox reflectivity, 0-255
    std::int64_t tUs = 0;  // capture time, microseconds
};

struct EgoMotion {       // from SensorFusion at the camera frame's time
    double speed = 0;    // m/s
    double yawRate = 0;  // rad/s, CCW positive
};

struct GroundPlane {  // GroundPlaneModel's plane, vehicle frame: n . q + d = 0, |n| = 1
    Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
    double d = 0;
    double distance(const Eigen::Vector3d& q) const { return n.dot(q) + d; }
};

struct FusionDetection {
    Box box;                           // source-frame pixels
    const ObjectMask* mask = nullptr;  // YOLOv8m-seg mask; nullptr for signs (box-only)
    bool isSign = false;
    std::optional<float> relInvDepth;  // MiDaS relative inverse depth, median over the mask
};

struct FusionFrame {
    std::int64_t cameraTUs = 0;
    std::vector<FusionDetection> detections;
    InputMapping maskMapping;  // source -> YOLO input (the letterbox) that the masks were made in
    int maskStride = 4;        // model-input pixels per mask cell
};

enum class RangeSource { LIDAR, ESTIMATED, NONE };

struct FusedObject {
    int detection = -1;  // index into FusionFrame::detections
    int32_t classId = 0;
    RangeSource source = RangeSource::NONE;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();       // vehicle frame, cluster median
    Eigen::Vector3d groundContact = Eigen::Vector3d::Zero();  // position dropped onto the road
    int points = 0;                                           // points in the chosen cluster
    int maskPoints = 0;  // all points selected by the mask (or sign box)
    // points / maskPoints: how cleanly the mask picked out ONE surface. A falling average over
    // many frames is a symptom of extrinsic drift (Part 12.2.1, ExtrinsicMonitor).
    bool usableForBraking() const { return source == RangeSource::LIDAR; }  // Part 9.3
};

struct UnknownObstacle {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();  // vehicle frame, cluster centroid
    Eigen::Vector3d groundContact = Eigen::Vector3d::Zero();
    int points = 0;
};

struct SceneModel {
    std::vector<FusedObject> objects;  // one per detection, in detection order
    std::vector<UnknownObstacle> unknown;
};

struct FusionConfig {
    double groundTolM = 0.15;
    int zBufferCellPx = 4;
    bool depthTest = true;  // rule 4; switched off only by tests showing what it prevents
    int erodeCells = 1;
    double depthGapM = 0.5;
    double minClusterFraction = 0.2;
    int minObjectPoints = 3;
    double corridorHalfWidthM = 1.5;  // ego half-width + margin
    double corridorLengthM = 40.0;
    double unknownClusterRadiusM = 0.5;
    int minUnknownPoints = 5;
    double maxObstacleHeightM = 2.5;  // above the road: gantries and branches cannot hit the car
    float signMinIntensity = 150.0f;
};

// The mask shrunk by `cells` prototype cells (4-neighbour erosion).
ObjectMask erodeMask(const ObjectMask& m, int cells);

class SceneReconstruction {
public:
    explicit SceneReconstruction(FusionConfig cfg = {});

    SceneModel merge(const FusionFrame& frame, const std::vector<LidarPoint>& cloud,
                     const CameraModel& camera, const FusionExtrinsics& ext, const EgoMotion& ego,
                     const GroundPlane& ground) const;

    // Rule 1: a point's vehicle-frame position at time `toUs`, given it was captured at `fromUs`
    // while the car moved with `ego` (constant speed and yaw rate over the scan window).
    static Eigen::Vector3d compensate(const Eigen::Vector3d& qVehicle, std::int64_t fromUs,
                                      std::int64_t toUs, const EgoMotion& ego);

private:
    FusionConfig cfg_;
};

}  // namespace ar_drive_assist
