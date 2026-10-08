#pragma once
// InjectedObstacle — a test object added to a REPLAY, seen consistently by both sensors. See
// lidar/LidarReplay.h for why a replay needs one.
//
// A box fixed in the WORLD, on the road the car actually drove: when it appears it is placed on
// the recorded trajectory (hub.csv positions) `startGapM` ahead of the car, aligned with the road
// there, and it then stays put (or moves along that trajectory at `speedMps`). Each frame it is
// seen from the car's recorded pose at that moment. The car really drove through that spot, so it
// is a true collision course, through bends too.
//
// Both sensors see it: the LiDAR replay adds its surface points; the camera replay paints its
// silhouette, so it hides whatever is behind it, exactly as a real stopped car would.
// Two simpler versions were physically impossible, and each produced misleading results
// (2026-10-07): LiDAR only (the camera masks of pedestrians BEHIND it claimed its points and it
// broke into fragment tracks), and fixed in the car's frame (in a bend it swung round with the car
// and left the predicted path).

#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ar_drive_assist/common/Camera.h"
#include "ar_drive_assist/scene/SceneReconstruction.h"

namespace ar_drive_assist {

struct InjectedObstacle {
    double appearS = 2.0;     // replay time it appears (s from the start)
    double startGapM = 30.0;  // its rear face, ahead of the vehicle origin (rear axle), then
    double speedMps = 0.0;    // its own speed along the ego heading (0 = stopped in the lane)
    double lengthM = 4.0, widthM = 1.8, heightM = 1.5;
    double lateralM = 0.0;  // offset to the left of the ego centre line
};

class ObstacleModel {
public:
    // `hubCsv`: the recording's hub.csv (positions and headings). Throws std::runtime_error.
    ObstacleModel(InjectedObstacle o, const std::string& hubCsv);

    // Vehicle-from-obstacle at replay time tS (the obstacle frame: origin at the centre of its
    // rear face on the ground, x along the road); none before it appears or once the car has
    // reached it.
    std::optional<Eigen::Isometry3d> poseAt(double tS) const;
    // Surface points (0.1 m spacing) in the LiDAR frame.
    std::vector<LidarPoint> points(double tS, const Eigen::Isometry3d& lidarFromVehicle) const;
    // Paints its silhouette (dark grey) into a frame described by `camera`.
    void paint(cv::Mat& bgr, double tS, const CameraModel& camera,
               const Eigen::Isometry3d& cameraFromVehicle) const;
    const InjectedObstacle& spec() const { return o_; }

private:
    struct Pose {
        double tS, e, n, psi, s;  // time, east, north (m), heading (rad, CCW from east), arc length
    };
    Pose egoAt(double tS) const;
    Pose alongTrack(double s) const;  // the trajectory at arc length s
    InjectedObstacle o_;
    std::vector<Pose> track_;
};

}  // namespace ar_drive_assist
