#pragma once
// RoadSurfaceProjector — the precision, road-locked navigation line. See docs/BUILD_GUIDE.md
// Part 11.4 (amended in Phase 9).
//
// It senses nothing new. It combines:
//   - the route (NavigationEngine, 11.2) and the car's progress along it (MapMatcher, 11.3);
//   - the fused pose (SensorFusion, 8.2);
//   - the LiDAR's measured road surface (GroundPlaneModel, 8.1);
//   - the ego lane boundaries (the lane model, Part 7, in DetectionFrame).
//
// ---------------------------------------------------------------------------------------------
// The steps, per call
//   1. Anchor at the car's progress s0 along the route. Take the route's geometry ahead of s0,
//      every `waypointSpacingM` out to `horizonM`, in the route's own frame at s0 (along a,
//      left b). The map knows the road's shape ahead exactly; what it does not know well is
//      where the CAR is relative to it.
//   2. Place the car relative to the road from the fused pose: lateral offset d from the route
//      centre line, and relative heading delta = car heading - route heading. Waypoint in the
//      vehicle frame = R(-delta) * (a, b - d). GPS puts d wrong by metres, and a car's compass
//      puts delta wrong by degrees (3 deg is 3 m sideways at 60 m), which is why step 3 exists.
//   3. Lane correction (when both ego lane boundaries are detected). The boundaries are
//      back-projected onto the ground, lines are fitted to both between `laneFitMinM` and
//      `laneFitMaxM` ahead, and the lane centre line gives two corrections:
//        heading:  the car's true heading relative to the lane. A POSE error, so it rotates the
//                  whole line.
//        lateral:  where the lane centre is. It puts the line in the car's LANE rather than on
//                  the OSM centre line (which is the middle of the road, or of a carriageway).
//                  It applies along the current road and tapers to zero over `taperAfterTurnM`
//                  after the next maneuver: after a turn the lane on the new road is not known.
//      Both are persistent states changed by at most `lateralCorrectionMaxM` /
//      `headingCorrectionMaxDeg` per frame. That is the guide's cap: ONE bad detection cannot
//      fling the line across lanes, while a real 1.75 m lane offset is still reached in two
//      frames (an absolute 1 m cap never would). Without lanes the correction decays with time
//      constant `correctionDecayS`; it is reset when the car moves onto the next road.
//   4. Height. Within the LiDAR's valid range, the height is the MEASURED surface: the mean of
//      the near-field patch points within `patchRadiusM` (onMeasuredSurface = true). Where the
//      patch has too few points (hidden by a car ahead), the fitted plane is used, marked NOT
//      measured. Beyond range: flat ground at the car's own ground height, marked not measured
//      (Part 11.4 step 3: an accepted, documented precision drop-off).
//   5. Project into the image with the shared calibration (common/Camera.h), the same one
//      SceneReconstruction uses.
//
// Output: each point in the VEHICLE frame (x, y, z) AND in the image, with the pose timestamp it
// belongs to. The renderer needs the 3D points to draw a road-space band and to move the line
// with the car's own motion between this pose and the displayed frame (Part 10.3).
// ---------------------------------------------------------------------------------------------

#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

#include "ar_drive_assist/common/Camera.h"
#include "ar_drive_assist/common/Geo.h"
#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/lidar/GroundPlaneModel.h"
#include "ar_drive_assist/nav/MapMatcher.h"
#include "ar_drive_assist/nav/Route.h"

namespace ar_drive_assist {

struct RoadProjectionConfig {  // config/road_projection.yaml
    double horizonM = 60;
    double waypointSpacingM = 1.5;
    double lateralCorrectionMaxM = 1.0;    // per frame
    double headingCorrectionMaxDeg = 1.0;  // per frame
    double laneFitMinM = 5, laneFitMaxM = 30;
    int laneMinPoints = 4;  // per boundary, inside the fit range
    double laneWidthMinM = 2.3, laneWidthMaxM = 5.0;
    double correctionDecayS = 10;
    double taperAfterTurnM = 10;
    double patchRadiusM = 1.0;
    int patchMinPoints = 3;
};

// Throws std::runtime_error naming the file and key.
RoadProjectionConfig loadRoadProjectionConfig(const std::string& path);

struct ProjectedPoint {
    Eigen::Vector3d vehicle = Eigen::Vector3d::Zero();  // x fwd, y left, z up (m)
    Eigen::Vector2d image = Eigen::Vector2d::Zero();    // raw image pixels
    bool inImage = false;
    double distanceAheadM = 0;  // along the route from the car
    bool onMeasuredSurface = false;
};

struct ProjectedRoute {
    std::vector<ProjectedPoint> polyline;
    std::string nextTurnInstruction;  // e.g. "turn left onto Kenyatta Avenue"; empty if none
    double nextTurnDistanceM = 0;
    std::uint64_t poseTimestampMs = 0;
    bool laneCorrected = false;  // lanes corrected this frame
    double lateralCorrectionM = 0, headingCorrectionDeg = 0;

    RoadProjectedRoute toMessage() const;  // road_projected_route.fbs, for navOverlayBus
};

class RoadSurfaceProjector {
public:
    explicit RoadSurfaceProjector(RoadProjectionConfig cfg = {});
    bool init(const RoadProjectionConfig& cfg);

    ProjectedRoute project(const Route& route, const MapMatcher::MatchedPosition& matched,
                           const GroundPlaneModel& ground, const DetectionFrame& lanes,
                           const VehiclePose& fusedPose, const LocalFrame& frame,
                           const CameraModel& camera, const Eigen::Isometry3d& cameraFromVehicle);

    void reset();  // new route

    // Where an image pixel lands on the ground (vehicle frame), or false if its ray does not hit
    // the ground ahead. Used to back-project lane boundaries; exposed for tests.
    static bool pixelToGround(const Eigen::Vector2d& px, const CameraModel& camera,
                              const Eigen::Isometry3d& cameraFromVehicle,
                              const GroundPlaneModel& ground, Eigen::Vector3d& out);

private:
    RoadProjectionConfig cfg_;
    double lateralCorr_ = 0, headingCorrRad_ = 0;
    std::uint64_t lastMs_ = 0;
    int stepIndex_ = -1;
};

}  // namespace ar_drive_assist
