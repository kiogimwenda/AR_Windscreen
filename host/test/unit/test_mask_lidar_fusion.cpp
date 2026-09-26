// Mask-based camera-LiDAR fusion tests — see docs/BUILD_GUIDE.md Part 8.3 and
// SceneReconstruction.h.
//
// The scenes are RAY-CAST, not hand-placed. The LiDAR (on the roof, 0.3 m left of and 0.5 m above
// the camera) sweeps an angular grid, and each ray stops at the first surface it hits, so
// occlusion, parallax between the two viewpoints, and the LiDAR's angular point density are all
// physically right. Masks are made the same way from the camera's viewpoint: a mask cell is set
// when the camera ray through it hits the object first. Soft YOLO mask edges are simulated by
// dilating the mask. Each rule gets a scene that shows the failure it exists to prevent, and,
// where a rule can be switched off, the test shows the failure really happens without it.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "ar_drive_assist/scene/SceneReconstruction.h"

using namespace ar_drive_assist;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180;
const Eigen::Vector3d kCamPos(1.8, 0.0, 1.3);    // vehicle frame (behind the windscreen)
const Eigen::Vector3d kLidarPos(2.0, 0.3, 1.8);  // vehicle frame (roof, offset to the left)
constexpr std::int64_t kCamT = 1'000'000;        // camera frame time, us

struct SceneBox {  // axis-aligned, vehicle frame
    Eigen::Vector3d lo, hi;
    float intensity = 60;
    int keepOneIn = 1;  // dark surfaces: only every n-th ray returns (the rest are absorbed)
};

struct World {
    std::vector<SceneBox> boxes;
    bool groundPlane = true;
};

// Ray/box slab test; returns the entry distance or +inf.
double hitBox(const Eigen::Vector3d& o, const Eigen::Vector3d& d, const SceneBox& b) {
    double t0 = 0, t1 = std::numeric_limits<double>::infinity();
    for (int k = 0; k < 3; ++k) {
        if (std::abs(d(k)) < 1e-12) {
            if (o(k) < b.lo(k) || o(k) > b.hi(k)) return std::numeric_limits<double>::infinity();
            continue;
        }
        double a = (b.lo(k) - o(k)) / d(k), c = (b.hi(k) - o(k)) / d(k);
        if (a > c) std::swap(a, c);
        t0 = std::max(t0, a);
        t1 = std::min(t1, c);
        if (t0 > t1) return std::numeric_limits<double>::infinity();
    }
    return t0 > 1e-9 ? t0 : std::numeric_limits<double>::infinity();
}

// First hit along a ray: returns the box index, -2 for the ground, -1 for nothing (within 70 m).
int castRay(const World& w, const Eigen::Vector3d& o, const Eigen::Vector3d& d, double& tHit) {
    tHit = 70.0;
    int which = -1;
    for (int i = 0; i < static_cast<int>(w.boxes.size()); ++i) {
        const double t = hitBox(o, d, w.boxes[i]);
        if (t < tHit) {
            tHit = t;
            which = i;
        }
    }
    if (w.groundPlane && d.z() < 0) {
        const double t = -o.z() / d.z();
        if (t < tHit) {
            tHit = t;
            which = -2;
        }
    }
    return which;
}

// Camera: x right, y down, z forward.
Eigen::Isometry3d cameraFromVehicle() {
    Eigen::Matrix3d R;
    R << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = -R * kCamPos;
    return T;
}

FusionExtrinsics extrinsics() {
    Eigen::Isometry3d vehicleFromLidar = Eigen::Isometry3d::Identity();
    vehicleFromLidar.translation() = kLidarPos;
    return {cameraFromVehicle() * vehicleFromLidar, vehicleFromLidar};
}

CameraModel camera() {
    return CameraModel{};  // 1920x1080, f = 1000, no distortion
}

// The LiDAR scan: 0.1 deg angular grid over +-25 deg azimuth, -20..+10 deg elevation, swept left
// to right over the 100 ms before the camera frame. `egoSpeed`: the car drives forward during the
// sweep, so earlier columns see the (static) world from further back.
std::vector<LidarPoint> scan(const World& w, double egoSpeed = 0) {
    std::vector<LidarPoint> out;
    const int cols = 500, rows = 300;
    std::vector<int> hitsPerBox(w.boxes.size(), 0);
    for (int c = 0; c < cols; ++c) {
        const std::int64_t t = kCamT - 100'000 + static_cast<std::int64_t>(c) * 100'000 / cols;
        const double back = egoSpeed * (kCamT - t) * 1e-6;  // how far behind the car was then
        World wt = w;
        for (auto& b : wt.boxes) {
            b.lo.x() += back;
            b.hi.x() += back;
        }
        const double az = (25.0 - 0.1 * c) * kDeg;
        for (int r = 0; r < rows; ++r) {
            const double el = (-20.0 + 0.1 * r) * kDeg;
            const Eigen::Vector3d d(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                                    std::sin(el));
            double th;
            const int which = castRay(wt, kLidarPos, d, th);
            if (which == -1) continue;
            float inten = 20;
            if (which >= 0) {
                if (hitsPerBox[which]++ % wt.boxes[which].keepOneIn != 0) continue;  // absorbed
                inten = wt.boxes[which].intensity;
            }
            out.push_back({(d * th).cast<float>(), inten, t});
        }
    }
    return out;
}

Eigen::Vector2d projectV(const Eigen::Vector3d& q) {
    Eigen::Vector2d px;
    const Eigen::Vector3d pc = cameraFromVehicle() * q;
    px = {1000 * pc.x() / pc.z() + 960, 1000 * pc.y() / pc.z() + 540};
    return px;
}

// Camera-view bounding box of a scene box, source pixels.
Box imageBox(const SceneBox& b, int32_t cls, double pad = 0) {
    double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    for (int k = 0; k < 8; ++k) {
        const Eigen::Vector3d q(k & 1 ? b.hi.x() : b.lo.x(), k & 2 ? b.hi.y() : b.lo.y(),
                                k & 4 ? b.hi.z() : b.lo.z());
        const Eigen::Vector2d p = projectV(q);
        x0 = std::min(x0, p.x());
        y0 = std::min(y0, p.y());
        x1 = std::max(x1, p.x());
        y1 = std::max(y1, p.y());
    }
    Box box;
    box.x = static_cast<float>(x0 - pad);
    box.y = static_cast<float>(y0 - pad);
    box.w = static_cast<float>(x1 - x0 + 2 * pad);
    box.h = static_cast<float>(y1 - y0 + 2 * pad);
    box.classId = cls;
    return box;
}

// The camera-view mask of box `k` (cells of 4 px, identity input mapping), grown by `soft` cells
// to imitate YOLO's soft quarter-resolution edges and small extrinsic errors.
ObjectMask renderMask(const World& w, int k, int soft) {
    const Box bb = imageBox(w.boxes[k], 0);
    ObjectMask m;
    m.x = static_cast<int>(bb.x / 4) - soft - 1;
    m.y = static_cast<int>(bb.y / 4) - soft - 1;
    m.w = static_cast<int>(bb.w / 4) + 2 * soft + 4;
    m.h = static_cast<int>(bb.h / 4) + 2 * soft + 4;
    m.bits.assign(static_cast<size_t>(m.w) * m.h, 0);
    const Eigen::Matrix3d Rt = cameraFromVehicle().linear().transpose();
    for (int gy = m.y; gy < m.y + m.h; ++gy) {
        for (int gx = m.x; gx < m.x + m.w; ++gx) {
            const Eigen::Vector3d dc((4 * gx + 2 - 960) / 1000.0, (4 * gy + 2 - 540) / 1000.0, 1);
            double th;
            if (castRay(w, kCamPos, (Rt * dc).normalized(), th) == k)
                m.bits[static_cast<size_t>(gy - m.y) * m.w + (gx - m.x)] = 1;
        }
    }
    for (int it = 0; it < soft; ++it) {
        const ObjectMask in = m;
        for (int gy = m.y; gy < m.y + m.h; ++gy)
            for (int gx = m.x; gx < m.x + m.w; ++gx)
                if (in.atGrid(gx - 1, gy) || in.atGrid(gx + 1, gy) || in.atGrid(gx, gy - 1) ||
                    in.atGrid(gx, gy + 1))
                    m.bits[static_cast<size_t>(gy - m.y) * m.w + (gx - m.x)] = 1;
    }
    return m;
}

FusionFrame frameWith(std::vector<FusionDetection> dets) {
    FusionFrame f;
    f.cameraTUs = kCamT;
    f.detections = std::move(dets);
    return f;  // identity mask mapping, stride 4
}

SceneBox pedestrianAt(double x, double y = 0) {
    return {{x, y - 0.25, 0}, {x + 0.3, y + 0.25, 1.75}};
}
SceneBox wallAt(double x) {
    return {{x, -8, 0}, {x + 0.3, 8, 3}};
}

SceneModel fuse(const World& w, const FusionFrame& f, FusionConfig cfg = {}, double egoSpeed = 0,
                double scanSpeed = 0) {
    return SceneReconstruction(cfg).merge(f, scan(w, scanSpeed), camera(), extrinsics(),
                                          EgoMotion{egoSpeed, 0}, GroundPlane{});
}

}  // namespace

// --- Building blocks, by hand -----------------------------------------------------------------

TEST(MaskLidarFusion, ProjectionHandCases) {
    CameraModel c;
    Eigen::Vector2d px;
    ASSERT_TRUE(c.project({0, 0, 10}, px));
    EXPECT_NEAR(px.x(), 960, 1e-9);
    EXPECT_NEAR(px.y(), 540, 1e-9);
    ASSERT_TRUE(c.project({1, 0.5, 10}, px));
    EXPECT_NEAR(px.x(), 1060, 1e-9);
    EXPECT_NEAR(px.y(), 590, 1e-9);
    c.k1 = 0.1;  // x = 0.1, y = 0: r^2 = 0.01, x_d = 0.1 * 1.001
    ASSERT_TRUE(c.project({1, 0, 10}, px));
    EXPECT_NEAR(px.x(), 960 + 100.1, 1e-9);
    EXPECT_FALSE(c.project({0, 0, -5}, px)) << "behind the camera";
}

TEST(MaskLidarFusion, MotionCompensationHandCases) {
    // Straight at 10 m/s: a point seen 20 m ahead 100 ms before the frame is 19 m ahead at it.
    const auto a = SceneReconstruction::compensate({20, 0, 1}, 0, 100'000, {10, 0});
    EXPECT_NEAR(a.x(), 19, 1e-9);
    EXPECT_NEAR(a.y(), 0, 1e-9);
    EXPECT_NEAR(a.z(), 1, 1e-9);
    // Turning left on the spot at 1 rad/s for 0.1 s: things ahead swing to the right.
    const auto b = SceneReconstruction::compensate({10, 0, 0}, 0, 100'000, {0, 1});
    EXPECT_NEAR(b.x(), 10 * std::cos(0.1), 1e-9);
    EXPECT_NEAR(b.y(), -10 * std::sin(0.1), 1e-9);
}

TEST(MaskLidarFusion, ErosionShrinksByOneCellPerStep) {
    ObjectMask m;
    m.x = 10;
    m.y = 20;
    m.w = m.h = 5;
    m.bits.assign(25, 1);
    const ObjectMask e = erodeMask(m, 1);
    EXPECT_EQ(e.area(), 9);
    EXPECT_TRUE(e.atGrid(12, 22));
    EXPECT_FALSE(e.atGrid(10, 20));
    EXPECT_EQ(erodeMask(m, 2).area(), 1);
}

// --- The rules, on ray-cast scenes ------------------------------------------------------------

// The basic case: a pedestrian 12 m ahead in front of a wall at 25 m, soft mask. Requirement: range
// within 0.1 m of the front face (the LiDAR measures the surface it sees), centred laterally, feet
// on the road. And the pedestrian must NOT reappear as an UNKNOWN obstacle (their points hidden by
// the depth test are still theirs); the wall, which nothing explains, must.
TEST(MaskLidarFusion, PedestrianRangedFromTheirOwnPoints) {
    World w{{pedestrianAt(12), wallAt(25)}};
    const ObjectMask mask = renderMask(w, 0, 1);
    const auto s = fuse(w, frameWith({{imageBox(w.boxes[0], 1), &mask, false, {}}}));
    ASSERT_EQ(s.objects.size(), 1u);
    const FusedObject& o = s.objects[0];
    EXPECT_EQ(o.source, RangeSource::LIDAR);
    EXPECT_TRUE(o.usableForBraking());
    EXPECT_NEAR(o.position.x(), 12.0, 0.1);
    EXPECT_NEAR(o.position.y(), 0.0, 0.15);
    EXPECT_NEAR(o.groundContact.z(), 0.0, 1e-9);
    for (const auto& u : s.unknown) EXPECT_GT(u.position.x(), 20) << "pedestrian duplicated";
    ASSERT_FALSE(s.unknown.empty());
    EXPECT_NEAR(s.unknown[0].position.x(), 25.0, 0.3);
}

// Rule 3: a far stray (40 m) and a near stray (rain, 6 m) inside the mask change nothing.
TEST(MaskLidarFusion, StrayPointsDoNotMoveThePedestrian) {
    World w{{pedestrianAt(12), wallAt(25)}};
    const ObjectMask mask = renderMask(w, 0, 1);
    auto cloud = scan(w);
    const Eigen::Matrix3d Rt = cameraFromVehicle().linear().transpose();
    const Eigen::Vector3d ray =
        (Rt * Eigen::Vector3d(0, (projectV({12, 0, 1}).y() - 540) / 1000, 1)).normalized();
    for (double range : {6.0, 40.0}) {
        const Eigen::Vector3d q = kCamPos + ray * range;
        cloud.push_back({(q - kLidarPos).cast<float>(), 60, kCamT});
    }
    const auto s =
        SceneReconstruction().merge(frameWith({{imageBox(w.boxes[0], 1), &mask, false, {}}}), cloud,
                                    camera(), extrinsics(), {}, GroundPlane{});
    EXPECT_NEAR(s.objects[0].position.x(), 12.0, 0.1);
}

double share(const FusedObject& o) {
    return o.maskPoints ? static_cast<double>(o.points) / o.maskPoints : 0.0;
}

// Rules 2 and 4 are defence in depth behind rule 3. In the two scenes below, rule 3 alone still
// recovers the right range (measured 2026-09-26, and asserted, so a change is noticed). What the
// rules change is how much of what the mask selects is the object itself: the margin rule 3 works
// with, and the share ExtrinsicMonitor watches. Required: at least 80% with the rule, and the
// rule must remove at least half of the contamination. (First draft demanded that the range
// fail without the rule; it did not. Measured clusters are in the progress log.)

// Rule 2: a pedestrian at 15 m seen through the gap between two parked cars at 8 m. The mask's
// soft edge spills two cells onto the cars.
TEST(MaskLidarFusion, ErosionStopsNeighboursLeakingIn) {
    World w{{pedestrianAt(15),
             {{8, 0.14, 0}, {12, 2.0, 1.5}},
             {{8, -2.0, 0}, {12, -0.14, 1.5}},
             wallAt(30)}};
    const ObjectMask mask = renderMask(w, 0, 2);
    const auto f = frameWith({{imageBox(w.boxes[0], 1), &mask, false, {}}});
    FusionConfig eroded, raw;
    eroded.erodeCells = 2;
    raw.erodeCells = 0;
    const FusedObject with = fuse(w, f, eroded).objects[0], without = fuse(w, f, raw).objects[0];
    EXPECT_NEAR(with.position.x(), 15.0, 0.1);
    EXPECT_NEAR(without.position.x(), 15.0, 0.1) << "rule 3 alone no longer copes: investigate";
    EXPECT_GE(share(with), 0.8);
    EXPECT_LT(1 - share(with), 0.5 * (1 - share(without)));
}

// Rule 4: a pedestrian in dark clothing (1 in 7 LiDAR rays returns) 12 m ahead of a wall. From
// the roof the LiDAR sees past the pedestrian's side onto wall the camera cannot see, and those
// points project into the mask.
TEST(MaskLidarFusion, DepthTestRemovesPointsTheCameraCannotSee) {
    SceneBox dark = pedestrianAt(12);
    dark.keepOneIn = 7;
    World w{{dark, wallAt(20)}};
    const ObjectMask mask = renderMask(w, 0, 1);
    const auto f = frameWith({{imageBox(w.boxes[0], 1), &mask, false, {}}});
    FusionConfig off;
    off.depthTest = false;
    const FusedObject with = fuse(w, f).objects[0], without = fuse(w, f, off).objects[0];
    EXPECT_NEAR(with.position.x(), 12.0, 0.15);
    EXPECT_NEAR(without.position.x(), 12.0, 0.15) << "rule 3 alone no longer copes: investigate";
    EXPECT_GE(share(with), 0.8);
    EXPECT_LT(1 - share(with), 0.5 * (1 - share(without)));
}

// Rule 1: the car drives at 15 m/s during the 100 ms sweep. Uncompensated, the early columns see
// the pedestrian up to 1.5 m further away. Compensated with the ego speed, the range is right.
TEST(MaskLidarFusion, MotionCompensationRemovesTheSweepSmear) {
    World w{{pedestrianAt(12), wallAt(25)}};
    const ObjectMask mask = renderMask(w, 0, 1);
    const auto f = frameWith({{imageBox(w.boxes[0], 1), &mask, false, {}}});
    EXPECT_NEAR(fuse(w, f, {}, 15.0, 15.0).objects[0].position.x(), 12.0, 0.15);
    EXPECT_GT(std::abs(fuse(w, f, {}, 0.0, 15.0).objects[0].position.x() - 12.0), 0.5);
}

// Rule 5: a pedestrian 80 m away gets no LiDAR points (beyond range). Without MiDaS: NONE, said
// honestly. With MiDaS inverse depths (rel = a / z + b, here a = 10, b = 0.2) for it and two
// LiDAR-ranged objects: an ESTIMATE near 80 m, never usable for braking.
TEST(MaskLidarFusion, NoRangeIsReportedHonestlyAndEstimatesAreFlagged) {
    World w{{pedestrianAt(12, 2), {{20, -2.5, 0}, {24, -0.8, 1.5}}, pedestrianAt(80, 0)}};
    const ObjectMask m0 = renderMask(w, 0, 1), m1 = renderMask(w, 1, 1), m2 = renderMask(w, 2, 0);
    auto rel = [](double xFront) {
        return static_cast<float>(10.0 / (xFront - kCamPos.x()) + 0.2);
    };
    auto dets = [&](bool withMidas) {
        std::vector<FusionDetection> d{{imageBox(w.boxes[0], 1), &m0, false, {}},
                                       {imageBox(w.boxes[1], 0), &m1, false, {}},
                                       {imageBox(w.boxes[2], 1), &m2, false, {}}};
        if (withMidas) {
            d[0].relInvDepth = rel(12);
            d[1].relInvDepth = rel(20);
            d[2].relInvDepth = rel(80);
        }
        return d;
    };
    const auto plain = fuse(w, frameWith(dets(false)));
    EXPECT_EQ(plain.objects[2].source, RangeSource::NONE);
    EXPECT_FALSE(plain.objects[2].usableForBraking());

    const auto est = fuse(w, frameWith(dets(true)));
    ASSERT_EQ(est.objects[0].source, RangeSource::LIDAR);
    ASSERT_EQ(est.objects[1].source, RangeSource::LIDAR);
    EXPECT_EQ(est.objects[2].source, RangeSource::ESTIMATED);
    EXPECT_FALSE(est.objects[2].usableForBraking());
    EXPECT_NEAR(est.objects[2].position.x(), 80, 4.0);
    EXPECT_NEAR(est.objects[2].position.y(), 0, 0.5);
}

// Rule 6: a fallen branch (no camera detection) in the lane is an UNKNOWN obstacle; the same
// object beside the road, and a gantry 5 m above the road, are not.
TEST(MaskLidarFusion, UnexplainedObstaclesInTheCorridorAreKept) {
    World w{{{{15, -0.6, 0}, {15.6, 0.6, 0.5}},  // in the lane
             {{15, 4.0, 0}, {15.6, 5.0, 0.5}},   // on the verge, outside the corridor
             {{22, -6, 5}, {22.5, 6, 5.6}}}};    // overhead gantry
    const auto s = fuse(w, frameWith({}));
    ASSERT_EQ(s.unknown.size(), 1u);
    EXPECT_NEAR(s.unknown[0].position.x(), 15.0, 0.3);
    EXPECT_NEAR(s.unknown[0].position.y(), 0.0, 0.3);
    EXPECT_NEAR(s.unknown[0].groundContact.z(), 0.0, 1e-9);
}

// Rule 7: a retroreflective sign plate (intensity 220) on a dull pole, in front of a dull wall.
// The box-only sign detection is placed by its bright points at the plate's position.
TEST(MaskLidarFusion, SignPlacedByRetroreflectivePoints) {
    SceneBox plate{{20, 2.7, 2.0}, {20.05, 3.3, 2.6}, 220};
    SceneBox pole{{20.05, 2.95, 0}, {20.15, 3.05, 2.0}, 40};
    World w{{plate, pole, wallAt(30)}};
    FusionDetection sign{imageBox(plate, 3, 3), nullptr, true, {}};
    const auto s = fuse(w, frameWith({sign}));
    const FusedObject& o = s.objects[0];
    ASSERT_EQ(o.source, RangeSource::LIDAR);
    EXPECT_NEAR(o.position.x(), 20.0, 0.1);
    EXPECT_NEAR(o.position.y(), 3.0, 0.15);
    EXPECT_NEAR(o.position.z(), 2.3, 0.15);
}
