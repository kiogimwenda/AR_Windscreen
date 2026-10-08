#pragma once
// LidarProcessor — per-scan LiDAR processing (Part 8.1): the fitted road surface, PUBLISHED as
// GroundPlaneModel (never discarded: RoadSurfaceProjector and SceneReconstruction both stand on
// it). The Livox SDK2 capture that feeds it is the live LidarThread (Phase 6); a replay feeds it
// recorded scans (LidarReplay).
//
// ---------------------------------------------------------------------------------------------
// Why a fitted ground, not z = 0
//
// The vehicle frame's z = 0 is the road under the rear axle. The road AHEAD is not on that plane
// whenever the car pitches (braking, load) or the road rises or falls. A 1 deg error puts the road
// 17 cm "up" at 10 m, past SceneReconstruction's 15 cm ground tolerance: road points become an
// obstacle, and because the LiDAR's rings slide along the slope as the car moves, that "obstacle"
// appears to rush towards the car. On the KITTI replay (2026-10-07) exactly this produced a
// LiDAR-only track closing at 25 m/s and a false brake at the first junction.
//
// ---------------------------------------------------------------------------------------------
// The fit (deterministic, so a replay always gives the same answer)
//   candidates  points in the corridor ahead (x from minRangeM to maxRangeM, |y| <= halfWidthM),
//               within searchBandM of the PREVIOUS plane (the first scan: of z = 0): walls,
//               canopies and car roofs are not road candidates
//   RANSAC      `iterations` planes through 3 random candidates (fixed seed); a plane must be
//               within maxTiltDeg of level; the one with most points within inlierTolM wins
//   refine      least squares (principal axis of the inliers' scatter)
//   accept      at least minInliers inliers and the plane within maxHeightAtOriginM of z = 0 at
//               the rear axle; otherwise the previous plane is kept (and `valid` says so)
// The result is a plane: right on a constant slope, an approximation through a crest or a dip.
// nearFieldPatch keeps the measured ground points themselves for consumers that need the real
// surface (RoadSurfaceProjector).
// ---------------------------------------------------------------------------------------------

#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "ar_drive_assist/lidar/GroundPlaneModel.h"
#include "ar_drive_assist/scene/SceneReconstruction.h"

namespace ar_drive_assist {

struct GroundFitConfig {
    double minRangeM = 2.0, maxRangeM = 35.0, halfWidthM = 6.0;
    double searchBandM = 0.6;
    double inlierTolM = 0.08;
    int iterations = 120;
    double maxTiltDeg = 12.0;
    int minInliers = 80;
    double maxHeightAtOriginM = 0.6;
    std::size_t maxPatchPoints = 3000;
    double patchHalfWidthM = 4.0;
};

class LidarProcessor {
public:
    explicit LidarProcessor(GroundFitConfig cfg = {}) : cfg_(cfg) {}

    struct Result {
        GroundPlaneModel ground;
        bool valid = false;  // false: this scan gave no acceptable fit; `ground` is the last one
        int inliers = 0;
    };
    // Fits the road surface of one scan (points in the LiDAR frame). Keeps the plane for the next
    // scan's candidate band.
    Result fitGround(const std::vector<LidarPoint>& scan, const Eigen::Isometry3d& vehicleFromLidar,
                     std::uint64_t timestampMs);

    // The plane as SceneReconstruction's GroundPlane (unit normal pointing up).
    static GroundPlane toGroundPlane(const GroundPlaneModel& g);

private:
    GroundFitConfig cfg_;
    GroundPlaneModel last_;  // z = 0 until the first good fit
    bool haveLast_ = false;
};

}  // namespace ar_drive_assist
