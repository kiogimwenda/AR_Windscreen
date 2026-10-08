#pragma once
// NavigationThread — Part 5.1's thread 8: route -> map-match -> project onto the road, as one
// sequential pipeline. See docs/BUILD_GUIDE.md Part 11.2-11.4.
//
// Every cycle (10 Hz, the ground model's refresh rate: no point projecting faster than the
// geometry updates):
//   pose      the EKF snapshot (nothing happens before the first GNSS fix)
//   route     to the destination: once, and again whenever the matcher confirms the car has left
//             the route (reroute)
//   match     MapMatcher: the pose snapped onto the road and the progress along the route
//   project   RoadSurfaceProjector: the route laid on the measured road surface and corrected by
//             the detected lane, in the published frames' camera model
//   publish   the ProjectedRoute on navBus, for the renderer
//
// The ground: the newest fitted road surface (LidarProcessor, Part 8.1), carried by the scene
// snapshots; a flat road at the vehicle frame's z = 0 only until the first one arrives.

#include <atomic>
#include <string>

#include "ar_drive_assist/fusion/EgoEstimator.h"
#include "ar_drive_assist/lidar/GroundPlaneModel.h"
#include "ar_drive_assist/nav/MapMatcher.h"
#include "ar_drive_assist/nav/NavigationEngine.h"
#include "ar_drive_assist/system/PipelineMessages.h"

namespace ar_drive_assist {

class EventLog;

struct NavigationThreadConfig {
    std::string osrmData = "data/maps/current/nairobi.osrm";
    double destLat = 0, destLon = 0;
    RoadProjectionConfig projection;
    CameraModel camera;  // the model of the published frames: CameraPipeline::frameModel()
    Eigen::Isometry3d cameraFromVehicle = Eigen::Isometry3d::Identity();
    double periodS = 0.1;
};

class NavigationThread {
public:
    // Throws std::runtime_error if the map cannot be loaded.
    NavigationThread(NavigationThreadConfig cfg, EgoEstimator& ego, SceneBus* scenes, NavBus& out,
                     EventLog* log = nullptr);

    void run(const std::atomic<bool>& stop);

private:
    NavigationThreadConfig cfg_;
    EgoEstimator& ego_;
    SceneBus* scenes_;
    NavBus& out_;
    EventLog* log_;
    NavigationEngine nav_;
    MapMatcher matcher_;
    RoadSurfaceProjector projector_;
};

}  // namespace ar_drive_assist
