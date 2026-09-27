// RoadSurfaceProjector tests — see docs/BUILD_GUIDE.md Part 11.4 and RoadSurfaceProjector.h.
//
// Synthetic, hand-computable scenes. The camera sits 1.8 m ahead of the rear axle, 1.3 m up,
// looking straight ahead (f = 1000 px, 1920x1080), unless a test says otherwise. A road point
// (X, 0, 0) then appears at column 960 and row 540 + 1000 * 1.3 / (X - 1.8). Routes are built in
// a local frame at Nairobi CBD, and the fused pose is expressed in that same frame, as
// SensorFusion would give it.

#include <gtest/gtest.h>

#include <cmath>

#include "ar_drive_assist/nav/RoadSurfaceProjector.h"

using namespace ar_drive_assist;

namespace {

constexpr double kPi = 3.14159265358979323846, kDeg = kPi / 180;
const LocalFrame kFrame({-1.2864, 36.8172});

Route routeThrough(const std::vector<std::pair<double, double>>& en) {
    Route r;
    for (auto [e, n] : en) r.geometry.push_back(kFrame.toGeodetic(e, n));
    r.computeCumulative();
    r.valid = true;
    return r;
}

// East along y = 0 from x = -50 to x = 500 (densified every 5 m).
Route straightEast() {
    std::vector<std::pair<double, double>> pts;
    for (double x = -50; x <= 500; x += 5) pts.push_back({x, 0});
    return routeThrough(pts);
}

MapMatcher::MatchedPosition matchedAt(double along) {
    MapMatcher::MatchedPosition m;
    m.valid = true;
    m.distanceAlongRouteM = along;
    return m;
}

VehiclePose poseAt(std::uint64_t ms, double e, double n, double headingDeg) {
    VehiclePose p;
    p.timestamp_ms = ms;
    p.x = e;
    p.y = n;
    p.heading_deg = static_cast<float>(headingDeg);
    p.speed_kph = 36;
    return p;
}

const CameraModel kCam;  // 1920x1080, f = 1000, no distortion
Eigen::Isometry3d camPose(double pitchDeg = 0) {
    return cameraFromVehicle({1.8, 0, 1.3}, 0, pitchDeg, 0);
}

GroundPlaneModel flatGround(double rangeM = 0) {
    GroundPlaneModel g;
    g.maxValidRangeM = static_cast<float>(rangeM);
    return g;
}

// Ego lane boundaries at y = centre +- 1.75 (vehicle frame), as the lane model would report them:
// image pixels of ground points 6-30 m ahead.
DetectionFrame lanesAt(double centre, double yawDeg = 0) {
    DetectionFrame d;
    const double c = std::cos(yawDeg * kDeg), s = std::sin(yawDeg * kDeg);
    for (double side : {+1.75, -1.75}) {
        auto& out = side > 0 ? d.lane_points_left : d.lane_points_right;
        for (double x = 6; x <= 30; x += 2) {
            // A lane line straight ahead of the lane's own frame, rotated by yaw about the car.
            const double lx = x, ly = centre + side;
            const Eigen::Vector3d g(c * lx - s * ly, s * lx + c * ly, 0);
            Eigen::Vector2d px;
            if (!kCam.project(camPose() * g, px)) continue;
            out.push_back(static_cast<float>(px.x()));
            out.push_back(static_cast<float>(px.y()));
        }
    }
    return d;
}

const ProjectedPoint& pointAhead(const ProjectedRoute& r, double metres) {
    for (const auto& p : r.polyline)
        if (std::abs(p.distanceAheadM - metres) < 0.76) return p;
    throw std::runtime_error("no point near " + std::to_string(metres));
}

}  // namespace

// --- Geometry by hand --------------------------------------------------------------------------

TEST(RoadProjector, StraightRoadCarOnCentreline) {
    RoadSurfaceProjector rp;
    const auto r = rp.project(straightEast(), matchedAt(50), flatGround(), {}, poseAt(0, 0, 0, 0),
                              kFrame, kCam, camPose());
    ASSERT_EQ(r.polyline.size(), 41u);  // 0 to 60 m every 1.5 m
    for (const auto& p : r.polyline) {
        EXPECT_NEAR(p.vehicle.x(), p.distanceAheadM, 0.01);
        EXPECT_NEAR(p.vehicle.y(), 0, 0.01);
        EXPECT_NEAR(p.vehicle.z(), 0, 1e-9);
        EXPECT_FALSE(p.onMeasuredSurface) << "no LiDAR range: flat fallback";
    }
    const auto& p = pointAhead(r, 12);  // X = 12: row 540 + 1300 / 10.2 = 667.45
    ASSERT_TRUE(p.inImage);
    EXPECT_NEAR(p.image.x(), 960, 0.05);
    EXPECT_NEAR(p.image.y(), 540 + 1300 / (12 - 1.8), 0.05);
    EXPECT_FALSE(pointAhead(r, 3).inImage) << "under the bonnet: below the image";
}

// The car 1.75 m left of the centre line (left-hand traffic, left lane), heading 2 deg left of
// the road. By hand, a road point 60 m ahead: (a, b - d) = (60, -1.75) rotated by -2 deg gives
// y = -60 sin 2 - 1.75 cos 2 = -3.843, x = 60 cos 2 - 1.75 sin 2 = 59.902.
TEST(RoadProjector, CarOffsetAndYawFromThePose) {
    RoadSurfaceProjector rp;
    const auto r = rp.project(straightEast(), matchedAt(50), flatGround(), {},
                              poseAt(0, 0, 1.75, 2), kFrame, kCam, camPose());
    const auto& p = pointAhead(r, 60);
    EXPECT_NEAR(p.vehicle.x(), 59.902, 0.02);
    EXPECT_NEAR(p.vehicle.y(), -3.843, 0.02);
}

// A left-hand curve of radius 200 m starting at the car: at arc length s the road is at
// (R sin(s/R), R (1 - cos(s/R))). At 60 m: (59.104, 8.933).
TEST(RoadProjector, CurvedRoadFollowsTheArc) {
    std::vector<std::pair<double, double>> pts{{-20, 0}};
    for (double s = 0; s <= 150; s += 2)
        pts.push_back({200 * std::sin(s / 200), 200 * (1 - std::cos(s / 200))});
    const Route r0 = routeThrough(pts);
    RoadSurfaceProjector rp;
    const auto r = rp.project(r0, matchedAt(20), flatGround(), {}, poseAt(0, 0, 0, 0), kFrame, kCam,
                              camPose());
    const auto& p = pointAhead(r, 60);
    EXPECT_NEAR(p.vehicle.x(), 59.104, 0.05);
    EXPECT_NEAR(p.vehicle.y(), 8.933, 0.05);
}

// --- Height: measured surface vs flat fallback -------------------------------------------------

// A 5% uphill measured by the LiDAR out to 30 m (patch z = 0.05 x; the fitted plane is left flat
// to show the PATCH is what is used). Within 30 m the line sits on the slope and is marked
// measured; beyond, it falls back to flat ground at the car's height, marked not measured. On the
// image, the uphill line is drawn higher than a flat-road line would be.
TEST(RoadProjector, NearFieldFollowsTheMeasuredSlope) {
    GroundPlaneModel g = flatGround(30);
    for (double x = 2; x <= 30; x += 0.5)
        for (double y = -3; y <= 3; y += 0.5) g.nearFieldPatch.emplace_back(x, y, 0.05 * x);
    RoadSurfaceProjector rp;
    const auto r = rp.project(straightEast(), matchedAt(50), g, {}, poseAt(0, 0, 0, 0), kFrame,
                              kCam, camPose());
    const auto& near = pointAhead(r, 21);
    EXPECT_TRUE(near.onMeasuredSurface);
    EXPECT_NEAR(near.vehicle.z(), 0.05 * 21, 0.03);
    const auto& far = pointAhead(r, 45);
    EXPECT_FALSE(far.onMeasuredSurface);
    EXPECT_NEAR(far.vehicle.z(), 0, 1e-9);
    const auto flat =
        RoadSurfaceProjector().project(straightEast(), matchedAt(50), flatGround(), {},
                                       poseAt(0, 0, 0, 0), kFrame, kCam, camPose());
    EXPECT_LT(near.image.y(), pointAhead(flat, 21).image.y() - 5) << "drawn higher up the hill";
}

// A car ahead hides the road from 10 to 15 m: there the fitted plane is used, marked NOT measured.
TEST(RoadProjector, PatchGapFallsBackToThePlaneUnmeasured) {
    GroundPlaneModel g = flatGround(30);
    g.planeCoefficients = {0, 0, 1, -0.2f};  // plane z = 0.2
    for (double x = 2; x <= 30; x += 0.5) {
        if (x > 9 && x < 16) continue;
        for (double y = -3; y <= 3; y += 0.5) g.nearFieldPatch.emplace_back(x, y, 0.1);
    }
    RoadSurfaceProjector rp;
    const auto r = rp.project(straightEast(), matchedAt(50), g, {}, poseAt(0, 0, 0, 0), kFrame,
                              kCam, camPose());
    EXPECT_TRUE(pointAhead(r, 6).onMeasuredSurface);
    EXPECT_NEAR(pointAhead(r, 6).vehicle.z(), 0.1, 1e-6);
    EXPECT_FALSE(pointAhead(r, 12).onMeasuredSurface);
    EXPECT_NEAR(pointAhead(r, 12).vehicle.z(), 0.2, 1e-6);
}

// --- Lane correction ---------------------------------------------------------------------------

// The pose is right (car 1.75 m left of the centre line), so the map-only line lies ON the
// centre line (y = -1.75). The lanes show the car centred in its lane. The line must move into
// the lane, at most 1.0 m per frame: -0.75 after one frame, 0 after two.
TEST(RoadProjector, LaneCorrectionMovesTheLineIntoTheLaneAtTheCappedRate) {
    RoadSurfaceProjector rp;
    const auto lanes = lanesAt(0);
    auto frame = [&](std::uint64_t ms, const DetectionFrame& l) {
        return rp.project(straightEast(), matchedAt(50), flatGround(), l, poseAt(ms, 0, 1.75, 0),
                          kFrame, kCam, camPose());
    };
    const auto r0 = RoadSurfaceProjector().project(straightEast(), matchedAt(50), flatGround(), {},
                                                   poseAt(0, 0, 1.75, 0), kFrame, kCam, camPose());
    EXPECT_NEAR(pointAhead(r0, 15).vehicle.y(), -1.75, 0.01);
    const auto r1 = frame(100, lanes);
    EXPECT_TRUE(r1.laneCorrected);
    EXPECT_NEAR(pointAhead(r1, 15).vehicle.y(), -0.75, 0.05);
    const auto r2 = frame(200, lanes);
    EXPECT_NEAR(pointAhead(r2, 15).vehicle.y(), 0.0, 0.05);
    EXPECT_NEAR(r2.lateralCorrectionM, 1.75, 0.05);

    // One bad frame (the lane pair of the NEXT lane over, 8 m away) moves the line by at most the
    // cap, and the next good frame brings it back.
    const auto bad = frame(300, lanesAt(8.0));
    EXPECT_LE(std::abs(pointAhead(bad, 15).vehicle.y() - 0.0), 1.0 + 1e-6);
    frame(400, lanes);
    EXPECT_NEAR(pointAhead(frame(500, lanes), 15).vehicle.y(), 0.0, 0.05);
}

// The compass reads 2 deg left of the truth. Uncorrected, the line swings 2 m sideways at 60 m.
// The lanes (straight ahead of the car) correct the heading at up to 1 deg per frame.
TEST(RoadProjector, LanesCorrectACompassHeadingError) {
    RoadSurfaceProjector rp;
    auto frame = [&](std::uint64_t ms, const DetectionFrame& l) {
        return rp.project(straightEast(), matchedAt(50), flatGround(), l, poseAt(ms, 0, 0, 2),
                          kFrame, kCam, camPose());
    };
    const auto before = frame(0, {});
    EXPECT_NEAR(pointAhead(before, 60).vehicle.y(), -60 * std::sin(2 * kDeg), 0.02);
    for (int k = 1; k <= 3; ++k) frame(100 * k, lanesAt(0));
    const auto after = frame(400, lanesAt(0));
    EXPECT_NEAR(pointAhead(after, 60).vehicle.y(), 0.0, 0.1);
    EXPECT_NEAR(after.headingCorrectionDeg, 2.0, 0.05);
}

// Without lanes, the correction decays (time constant 10 s) instead of snapping back.
TEST(RoadProjector, CorrectionDecaysWithoutLanes) {
    RoadSurfaceProjector rp;
    for (int k = 0; k < 3; ++k)
        rp.project(straightEast(), matchedAt(50), flatGround(), lanesAt(0),
                   poseAt(100 * k, 0, 1.75, 0), kFrame, kCam, camPose());
    const auto r = rp.project(straightEast(), matchedAt(50), flatGround(), {},
                              poseAt(200 + 10000, 0, 1.75, 0), kFrame, kCam, camPose());
    EXPECT_NEAR(r.lateralCorrectionM, 1.75 * std::exp(-1.0), 0.02);
}

// The lateral correction puts the line in the lane ON THE CURRENT ROAD. After a left turn 40 m
// ahead it tapers out over 10 m: the lane on the new road is not known yet.
TEST(RoadProjector, LateralCorrectionTapersAfterTheNextTurn) {
    Route r0 = routeThrough({{-50, 0}, {40, 0}, {40, 200}});
    r0.steps = {{"depart", "", "Moi Avenue", kFrame.toGeodetic(-50, 0), 0, 90, 0},
                {"turn", "left", "Kenyatta Avenue", kFrame.toGeodetic(40, 0), 90, 200, 0}};
    RoadSurfaceProjector rp, plain;
    ProjectedRoute r;
    for (int k = 0; k < 3; ++k)
        r = rp.project(r0, matchedAt(50), flatGround(), lanesAt(0), poseAt(100 * k, 0, 1.75, 0),
                       kFrame, kCam, camPose());
    const auto u = plain.project(r0, matchedAt(50), flatGround(), {}, poseAt(0, 0, 1.75, 0), kFrame,
                                 kCam, camPose());
    EXPECT_NEAR(pointAhead(r, 30).vehicle.y() - pointAhead(u, 30).vehicle.y(), 1.75, 0.05);
    EXPECT_NEAR(pointAhead(r, 45).vehicle.y() - pointAhead(u, 45).vehicle.y(), 1.75 * 0.5, 0.05);
    EXPECT_NEAR(pointAhead(r, 57).vehicle.y() - pointAhead(u, 57).vehicle.y(), 0.0, 0.01);
    EXPECT_EQ(r.nextTurnInstruction, "turn left onto Kenyatta Avenue");
    EXPECT_NEAR(r.nextTurnDistanceM, 40, 0.01);
}

TEST(RoadProjector, CorrectionResetsOnTheNextRoad) {
    Route r0 = routeThrough({{-50, 0}, {40, 0}, {40, 200}});
    r0.steps = {{"depart", "", "A", kFrame.toGeodetic(-50, 0), 0, 90, 0},
                {"turn", "left", "B", kFrame.toGeodetic(40, 0), 90, 200, 0}};
    RoadSurfaceProjector rp;
    for (int k = 0; k < 3; ++k)
        rp.project(r0, matchedAt(50), flatGround(), lanesAt(0), poseAt(100 * k, 0, 1.75, 0), kFrame,
                   kCam, camPose());
    const auto r = rp.project(r0, matchedAt(100), flatGround(), {}, poseAt(400, 40, 10, 90), kFrame,
                              kCam, camPose());
    EXPECT_EQ(r.lateralCorrectionM, 0.0);
}

// --- Plumbing ----------------------------------------------------------------------------------

TEST(RoadProjector, TurnInstructionsSkipNonDecisions) {
    Route r0 = straightEast();
    r0.steps = {{"depart", "", "Moi Avenue", {}, 0, 0, 0},
                {"new name", "straight", "Haile Selassie Avenue", {}, 80, 0, 0},
                {"roundabout", "right", "Uhuru Highway", {}, 150, 0, 0},
                {"arrive", "", "", {}, 300, 0, 0}};
    RoadSurfaceProjector rp;
    auto at = [&](double s) {
        return rp.project(r0, matchedAt(s), flatGround(), {}, poseAt(0, s - 50, 0, 0), kFrame, kCam,
                          camPose());
    };
    EXPECT_EQ(at(50).nextTurnInstruction, "take the roundabout onto Uhuru Highway");
    EXPECT_NEAR(at(50).nextTurnDistanceM, 100, 0.01);
    EXPECT_EQ(at(200).nextTurnInstruction, "arrive at destination");
}

// Ground point -> pixel -> ground, through a distorting lens on a camera pitched 3 deg down.
TEST(RoadProjector, PixelToGroundInvertsProjectionThroughDistortion) {
    CameraModel cam;
    cam.k1 = -0.12;
    cam.k2 = 0.03;
    cam.p1 = 0.001;
    const Eigen::Isometry3d T = camPose(3);
    for (double x = 8; x <= 40; x += 8) {
        for (double y : {-3.0, 0.0, 3.0}) {
            Eigen::Vector2d px;
            ASSERT_TRUE(cam.project(T * Eigen::Vector3d(x, y, 0), px));
            Eigen::Vector3d g;
            ASSERT_TRUE(RoadSurfaceProjector::pixelToGround(px, cam, T, GroundPlaneModel{}, g));
            EXPECT_NEAR(g.x(), x, 0.01);
            EXPECT_NEAR(g.y(), y, 0.01);
        }
    }
}

TEST(RoadProjector, InvalidMatchGivesNoLine) {
    MapMatcher::MatchedPosition m;
    m.valid = false;
    EXPECT_TRUE(RoadSurfaceProjector()
                    .project(straightEast(), m, flatGround(), {}, poseAt(0, 0, 0, 0), kFrame, kCam,
                             camPose())
                    .polyline.empty());
}

TEST(RoadProjector, MessageCarriesVehicleFramePointsAndTimestamp) {
    const auto r = RoadSurfaceProjector().project(straightEast(), matchedAt(50), flatGround(), {},
                                                  poseAt(1234, 0, 0, 0), kFrame, kCam, camPose());
    const auto m = r.toMessage();
    EXPECT_EQ(m.pose_timestamp_ms, 1234u);
    ASSERT_EQ(m.polyline.size(), r.polyline.size());
    EXPECT_NEAR(m.polyline[8]->x_m, 12.0f, 0.01f);
    EXPECT_EQ(m.polyline[8]->in_image, true);
    EXPECT_EQ(m.polyline[0]->in_image, false);
}

TEST(RoadProjector, LoadsTheRealConfigFile) {
    const auto c =
        loadRoadProjectionConfig(std::string(HOST_SOURCE_DIR) + "/config/road_projection.yaml");
    EXPECT_DOUBLE_EQ(c.horizonM, 60);
    EXPECT_DOUBLE_EQ(c.waypointSpacingM, 1.5);
    EXPECT_DOUBLE_EQ(c.lateralCorrectionMaxM, 1.0);
}
