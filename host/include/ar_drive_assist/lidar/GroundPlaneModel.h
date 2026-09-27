#pragma once
// GroundPlaneModel — the fitted road surface, PUBLISHED rather than discarded. See
// docs/BUILD_GUIDE.md Part 8.1. RoadSurfaceProjector (Part 11.4) depends on this being real,
// current geometry: it is what keeps the navigation line on the road through slopes and dips.
//
// The type is defined now (Phase 9 consumes it); LidarProcessor fills it in Phase 6. All in the
// vehicle frame (x forward, y left, z up, metres).

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ar_drive_assist {

struct GroundPlaneModel {
    Eigen::Vector4f planeCoefficients{0, 0, 1, 0};  // ax + by + cz + d = 0
    std::vector<Eigen::Vector3f> nearFieldPatch;    // measured ground points in the drivable
                                                    // corridor, out to LiDAR range
    float maxValidRangeM = 0;                       // beyond this, no measured surface exists
    std::uint64_t timestampMs = 0;

    // Height of the fitted plane at (x, y). A near-vertical "plane" (c ~ 0) is not a road: 0.
    double planeHeightAt(double x, double y) const {
        const double c = planeCoefficients.z();
        if (std::abs(c) < 1e-6) return 0.0;
        return -(planeCoefficients.x() * x + planeCoefficients.y() * y + planeCoefficients.w()) / c;
    }
};

}  // namespace ar_drive_assist
