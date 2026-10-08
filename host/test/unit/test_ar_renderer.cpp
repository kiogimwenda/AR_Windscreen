// ArRenderer and SignTracker tests — see docs/BUILD_GUIDE.md Part 10.3 and
// docs/architecture/ar-overlay-design.md. No GPU: these check the SCENE (what would be drawn,
// where, in which colour), which is where every display rule is decided.

#include <gtest/gtest.h>

#include <cmath>

#include "ar_drive_assist/render/ArRenderer.h"

using namespace ar_drive_assist;

namespace {

CameraModel cam() {
    CameraModel c;
    c.width = 1920;
    c.height = 1080;
    c.fx = c.fy = 1000;
    c.cx = 960;
    c.cy = 540;
    return c;
}
const Eigen::Isometry3d kCamPose = cameraFromVehicle({1.8, 0, 1.3}, 0, 0, 0);

ArRenderer renderer() {
    return ArRenderer(RendererConfig{}, cam(), kCamPose);
}

// A straight route from 4 m to 60 m, measured to 30 m.
ProjectedRoute straightRoute(std::uint64_t poseMs = 1000) {
    ProjectedRoute r;
    r.poseTimestampMs = poseMs;
    for (double x = 4; x <= 60; x += 2) {
        ProjectedPoint p;
        p.vehicle = {x, 0, 0};
        p.distanceAheadM = x;
        p.onMeasuredSurface = x <= 30;
        r.polyline.push_back(p);
    }
    return r;
}

// Only on a NAMED scene: a pointer into a temporary dangles (AddressSanitizer found one).
const OverlayItem* first(const OverlayScene& s, OverlayKind k) {
    for (const auto& i : s.items)
        if (i.kind == k) return &i;
    return nullptr;
}
std::vector<const OverlayItem*> labels(const OverlayScene& s) {
    std::vector<const OverlayItem*> v;
    for (const auto& i : s.items)
        if (i.kind == OverlayKind::IMAGE_LABEL) v.push_back(&i);
    return v;
}
bool hasLabel(const OverlayScene& s, const std::string& prefix) {
    for (const auto* l : labels(s))
        if (l->text.rfind(prefix, 0) == 0) return true;
    return false;
}

HazardView hazard(int id, int cls, std::uint32_t flags, double risk,
                  std::optional<Eigen::Vector3d> ground, double gap = -1) {
    HazardView h;
    h.trackId = id;
    h.objectClass = cls;
    h.threat.trackId = id;
    h.threat.flags = flags;
    h.threat.risk = risk;
    h.threat.level = RecklessDrivingDetector::levelFor(risk);
    h.threat.inEgoPath = true;
    h.inBrakePath = true;
    h.ground = ground;
    h.gapM = gap;
    h.box = {900, 500, 120, 90};
    return h;
}

SignObservation sign(int cls, std::optional<Eigen::Vector3d> pos = std::nullopt,
                     float conf = 0.8f) {
    SignObservation o;
    o.classId = cls;
    o.confidence = conf;
    o.box = {1400, 400, 40, 40};
    o.position = pos;
    return o;
}

}  // namespace

// --- Navigation --------------------------------------------------------------------------------

// Phase 11 exit criterion: measured-surface points and flat-ground fallback points are drawn
// visibly differently.
TEST(ArRenderer, RouteDistinguishesMeasuredFromFlatGroundPoints) {
    auto r = renderer();
    const ProjectedRoute route = straightRoute(1000);
    RenderInputs in;
    in.frameMs = 1000;
    in.route = &route;
    const auto s = r.build(in);
    const OverlayItem* band = first(s, OverlayKind::ROAD_BAND);
    ASSERT_NE(band, nullptr);
    ASSERT_EQ(band->pointOpacity.size(), route.polyline.size());
    for (std::size_t i = 0; i < route.polyline.size(); ++i)
        EXPECT_FLOAT_EQ(band->pointOpacity[i], route.polyline[i].onMeasuredSurface ? 1.0f : 0.45f);
    EXPECT_TRUE(band->occludedByHazards);
}

// The route is moved by the car's motion between the projector's pose and the frame's capture:
// 15 m/s for 60 ms is 0.9 m, so a point 20 m ahead is drawn 19.1 m ahead.
TEST(ArRenderer, RouteIsMovedToTheFramesCaptureTime) {
    auto r = renderer();
    const ProjectedRoute route = straightRoute(1000);
    RenderInputs in;
    in.frameMs = 1060;
    in.egoSpeedMps = 15;
    in.route = &route;
    const OverlayScene bandScene = r.build(in);
    const OverlayItem* band = first(bandScene, OverlayKind::ROAD_BAND);
    ASSERT_NE(band, nullptr);
    EXPECT_NEAR(band->points[8].x(), 20 - 0.9, 1e-9);  // polyline[8] is x = 20
}

TEST(ArRenderer, EgoShiftTurningByHand) {
    // Turning left at 0.5 rad/s, 10 m/s, for 0.2 s: the car moves along an arc and rotates by
    // 0.1 rad, so a point straight ahead appears rotated to the right.
    const Eigen::Vector3d p = ArRenderer::egoShift({20, 0, 0}, 10, 0.5, 0.2);
    const double dpsi = 0.1, dx = 10 / 0.5 * std::sin(dpsi), dy = 10 / 0.5 * (1 - std::cos(dpsi));
    EXPECT_NEAR(p.x(), std::cos(dpsi) * (20 - dx) + std::sin(dpsi) * (0 - dy), 1e-9);
    EXPECT_NEAR(p.y(), -std::sin(dpsi) * (20 - dx) + std::cos(dpsi) * (0 - dy), 1e-9);
    EXPECT_LT(p.y(), 0);
}

// Route grading by a = v^2 / 2d to a stop target (a collision hazard 20 m ahead). At 15 m/s:
// 0.57 g -> red, pulsing is not used above 0.4 g (solid). At 5 m/s: 0.064 g -> green.
TEST(ArRenderer, RouteGradesByDecelerationToAStopTarget) {
    auto grade = [](double speed) {
        auto r = renderer();
        const ProjectedRoute route = straightRoute();
        RenderInputs in;
        in.frameMs = 1000;
        in.egoSpeedMps = speed;
        in.route = &route;
        in.hazards = {hazard(1, 0, FORWARD_COLLISION, 0.9, Eigen::Vector3d(23.5, 0, 0), 20)};
        return r.build(in);
    };
    const auto fast = grade(15);
    const OverlayItem* band = first(fast, OverlayKind::ROAD_BAND);
    ASSERT_NE(band, nullptr);
    EXPECT_FLOAT_EQ(band->pointColor[4].r, palette::kRed.r);  // x = 12, before the target
    EXPECT_LT(band->pointOpacity[20], 0.3f) << "beyond the barrier the band fades";
    const auto slow = grade(5);
    EXPECT_FLOAT_EQ(first(slow, OverlayKind::ROAD_BAND)->pointColor[4].g, palette::kGreen.g);
}

TEST(ArRenderer, NextTurnLabel) {
    auto r = renderer();
    ProjectedRoute route = straightRoute();
    route.nextTurnInstruction = "turn left onto Kenyatta Avenue";
    route.nextTurnDistanceM = 118;
    RenderInputs in;
    in.frameMs = 1000;
    in.route = &route;
    EXPECT_TRUE(hasLabel(r.build(in), "turn left onto Kenyatta Avenue   120 m"));
}

// --- Hazards -----------------------------------------------------------------------------------

TEST(ArRenderer, OrdinaryTrafficGetsNothing) {
    auto r = renderer();
    RenderInputs in;
    in.frameMs = 1000;
    in.hazards = {hazard(1, 0, 0, 0.1, Eigen::Vector3d(30, 0, 0))};  // LOW, no flags
    EXPECT_TRUE(r.build(in).items.empty());
}

// Forward collision ahead: glow (mask-less: ellipse fallback), barrier at the object, the gap as a
// label, and the silhouette registered as an occluder WITH its depth.
TEST(ArRenderer, ForwardCollisionHazard) {
    auto r = renderer();
    RenderInputs in;
    in.frameMs = 1000;
    in.egoSpeedMps = 10;
    in.hazards = {hazard(4, 0, FORWARD_COLLISION, 0.95, Eigen::Vector3d(23.5, 0.2, 0), 20)};
    const auto s = r.build(in);
    ASSERT_EQ(s.count(OverlayKind::ELLIPSE_GLOW), 1u);
    const OverlayItem* b = first(s, OverlayKind::ROAD_BARRIER);
    ASSERT_NE(b, nullptr);
    EXPECT_NEAR(b->points[0].x(), 23.5, 1e-9);
    EXPECT_TRUE(hasLabel(s, "20 m"));
    ASSERT_EQ(s.occluders.size(), 1u);
    EXPECT_NEAR(s.occluders[0].depthM, 23.5 - 1.8, 1e-9);
    const OverlayItem* g = first(s, OverlayKind::ELLIPSE_GLOW);
    EXPECT_GT(g->style.shimmerHz, 1.5f);
    EXPECT_LE(g->style.shimmerHz, 3.0f);
    EXPECT_FLOAT_EQ(g->style.color.r, palette::forRisk(0.95).r);
}

// In the curved path only (a car parked at the kerb on a corner exit): rule 1 would never brake
// for it, so no barrier and no gap label, which would tell the driver otherwise. The glow stays.
TEST(ArRenderer, NoBarrierForAnObjectRuleOneWouldNotBrakeFor) {
    auto r = renderer();
    RenderInputs in;
    in.frameMs = 1000;
    in.egoSpeedMps = 10;
    HazardView h = hazard(4, 0, FORWARD_COLLISION, 0.95, Eigen::Vector3d(23.5, 1.0, 0), 20);
    h.inBrakePath = false;
    in.hazards = {h};
    const auto s = r.build(in);
    EXPECT_EQ(s.count(OverlayKind::ROAD_BARRIER), 0u);
    EXPECT_FALSE(hasLabel(s, "20 m"));
    EXPECT_EQ(s.count(OverlayKind::ELLIPSE_GLOW), 1u) << "the warning glow stays";
}

TEST(ArRenderer, PedestrianGetsAGroundRingAndAMaskGlow) {
    auto r = renderer();
    RenderInputs in;
    in.frameMs = 1000;
    HazardView h = hazard(5, 1, VULNERABLE_IN_PATH, 0.4, Eigen::Vector3d(15, 1.0, 0));
    PixelMask m;
    m.x = 230;
    m.y = 120;
    m.w = m.h = 10;
    m.bits.assign(100, 1);
    h.mask = m;
    in.hazards = {h};
    const auto s = r.build(in);
    EXPECT_EQ(s.count(OverlayKind::MASK_GLOW), 1u);
    ASSERT_EQ(s.count(OverlayKind::ROAD_RING), 1u);
    EXPECT_NEAR(first(s, OverlayKind::ROAD_RING)->points[0].y(), 1.0, 1e-9);
}

TEST(ArRenderer, TailgatingZoneBetweenBumperAndLead) {
    auto r = renderer();
    RenderInputs in;
    in.frameMs = 1000;
    HazardView h = hazard(6, 0, TAILGATING, 0.4, Eigen::Vector3d(15, 0, 0));
    h.threat.timeGapS = 0.4;  // under half the 1 s minimum: red
    in.hazards = {h};
    const auto s = r.build(in);
    const OverlayItem* z = first(s, OverlayKind::ROAD_ZONE);
    ASSERT_NE(z, nullptr);
    EXPECT_NEAR(z->points[0].x(), 3.5 + 0.5, 1e-9);
    EXPECT_NEAR(z->points[2].x(), 15 - 0.3, 1e-9);
    EXPECT_FLOAT_EQ(z->style.color.r, palette::kRed.r);
}

// No measured position: the glow only, never a road element that might float (rule 6). Off
// screen: a chevron at the edge instead of a glow.
TEST(ArRenderer, DegradesHonestlyWithoutPositionOrOffScreen) {
    auto r = renderer();
    RenderInputs in;
    in.frameMs = 1000;
    in.hazards = {hazard(7, 1, VULNERABLE_IN_PATH | FORWARD_COLLISION, 0.9, std::nullopt, 12)};
    auto s = r.build(in);
    EXPECT_EQ(s.count(OverlayKind::ELLIPSE_GLOW), 1u);
    EXPECT_EQ(s.count(OverlayKind::ROAD_BARRIER), 0u);
    EXPECT_EQ(s.count(OverlayKind::ROAD_RING), 0u);
    HazardView off = hazard(8, 1, VULNERABLE_IN_PATH, 0.6, std::nullopt);
    off.box = {-300, 500, 100, 200};
    in.hazards = {off};
    s = r.build(in);
    EXPECT_EQ(s.count(OverlayKind::IMAGE_CHEVRON), 1u);
    EXPECT_EQ(s.count(OverlayKind::ELLIPSE_GLOW), 0u);
}

// --- Signs -------------------------------------------------------------------------------------

// Rule 2: two frames are not enough; the third confirms.
TEST(ArRenderer, SignsNeedThreeConfidentFrames) {
    auto r = renderer();
    RenderInputs in;
    in.egoSpeedMps = 10;
    for (int k = 0; k < 3; ++k) {
        in.frameMs = 1000 + 33 * k;
        in.signs = {sign(sign_class::kRoundabout)};
        const auto s = r.build(in);
        EXPECT_EQ(hasLabel(s, "ROUNDABOUT"), k == 2) << "frame " << k;
    }
    auto low = renderer();
    for (int k = 0; k < 5; ++k) {
        in.frameMs = 1000 + 33 * k;
        in.signs = {sign(sign_class::kRoundabout, std::nullopt, 0.4f)};
        EXPECT_FALSE(hasLabel(low.build(in), "ROUNDABOUT")) << "below the 0.5 confidence floor";
    }
}

// Value agreement: alternating 30/50 readings of one sign never change the limit; three agreeing
// readings do. The limit then persists after the sign leaves view.
TEST(ArRenderer, SpeedLimitNeedsAgreementAndPersists) {
    auto r = renderer();
    RenderInputs in;
    in.egoSpeedMps = 10;
    in.mapMaxSpeedKph = 80;
    auto frame = [&](int k, std::vector<SignObservation> obs) {
        in.frameMs = 1000 + 33 * k;
        in.signs = std::move(obs);
        return r.build(in);
    };
    const int s30 = sign_class::kSpeedFirst + 2, s50 = sign_class::kSpeedFirst + 4;
    for (int k = 0; k < 6; ++k) frame(k, {sign(k % 2 ? s50 : s30)});
    EXPECT_EQ(*r.signs().limit().kph, 80) << "disputed readings: the map's maxspeed stays";
    for (int k = 6; k < 9; ++k) frame(k, {sign(s50)});
    EXPECT_EQ(*r.signs().limit().kph, 50);
    const auto later = frame(20, {});  // sign out of view
    EXPECT_EQ(*r.signs().limit().kph, 50);
    const OverlayItem* badge = first(later, OverlayKind::IMAGE_BADGE);
    ASSERT_NE(badge, nullptr);
    EXPECT_EQ(badge->text, "50");
}

TEST(ArRenderer, EndOfRestrictionAndRoadChangeReturnToTheMap) {
    auto r = renderer();
    RenderInputs in;
    in.mapMaxSpeedKph = 80;
    const int s50 = sign_class::kSpeedFirst + 4;
    int k = 0;
    auto frame = [&](std::vector<SignObservation> obs, bool roadChanged = false) {
        in.frameMs = 1000 + 33 * k++;
        in.signs = std::move(obs);
        in.roadChanged = roadChanged;
        r.build(in);
    };
    for (int i = 0; i < 3; ++i) frame({sign(s50)});
    ASSERT_EQ(*r.signs().limit().kph, 50);
    for (int i = 0; i < 3; ++i) frame({sign(sign_class::kEndOfRestriction)});
    EXPECT_EQ(*r.signs().limit().kph, 80);
    for (int i = 0; i < 3; ++i) frame({sign(s50)});
    ASSERT_EQ(*r.signs().limit().kph, 50);
    frame({}, true);
    EXPECT_EQ(*r.signs().limit().kph, 80);
}

// Rule 3: Kenya drives on the left; a speed sign measured 6 m to the right is for the opposite
// carriageway.
TEST(ArRenderer, OppositeCarriagewaySpeedSignIsIgnored) {
    auto r = renderer();
    RenderInputs in;
    in.mapMaxSpeedKph = 80;
    for (int k = 0; k < 4; ++k) {
        in.frameMs = 1000 + 33 * k;
        in.signs = {sign(sign_class::kSpeedFirst + 4, Eigen::Vector3d(30, -6, 2))};
        r.build(in);
    }
    EXPECT_EQ(*r.signs().limit().kph, 80);
}

// Speed HUD: limit 50; 50 km/h green, 54 amber (within +10%), 56 red; no speed source: badge only.
TEST(ArRenderer, SpeedHudGrading) {
    auto speedLabel = [](std::optional<double> kph) {
        auto r = renderer();
        RenderInputs in;
        in.mapMaxSpeedKph = 50;
        if (kph) in.egoSpeedMps = *kph / 3.6;
        in.frameMs = 1000;
        const auto s = r.build(in);
        for (const auto* l : labels(s))
            if (l->text.find("km/h") != std::string::npos)
                return std::optional<Rgba>(l->style.color);
        return std::optional<Rgba>();
    };
    EXPECT_FLOAT_EQ(speedLabel(50)->g, palette::kGreen.g);
    EXPECT_FLOAT_EQ(speedLabel(54)->g, palette::kAmber.g);
    EXPECT_FLOAT_EQ(speedLabel(56)->g, palette::kRed.g);
    EXPECT_FALSE(speedLabel(std::nullopt).has_value()) << "never a guessed speed";
}

// A stop sign 40 m ahead: barrier plus countdown. At 15 m/s, a = 225 / (2 * 36.5) = 0.31 g ->
// pulsing (above the comfortable 0.25 g, below the hard 0.4 g). Once the car has stood still
// within reach for 1 s, it clears.
TEST(ArRenderer, StopSignBarrierEscalatesAndClears) {
    auto r = renderer();
    RenderInputs in;
    in.egoSpeedMps = 15;
    OverlayScene s;
    for (int k = 0; k < 3; ++k) {
        in.frameMs = 1000 + 33 * k;
        in.signs = {sign(sign_class::kStop, Eigen::Vector3d(40, 2.5, 2))};
        s = r.build(in);
    }
    const OverlayItem* b = first(s, OverlayKind::ROAD_BARRIER);
    ASSERT_NE(b, nullptr);
    EXPECT_GT(b->style.pulseHz, 0);
    EXPECT_TRUE(hasLabel(s, "STOP  35 m"));
    in.egoSpeedMps = 0;
    for (int k = 0; k < 40; ++k) {  // stopped, the sign 6 m from the rear axle
        in.frameMs = 2000 + 33 * k;
        in.signs = {sign(sign_class::kStop, Eigen::Vector3d(6, 2.5, 2))};
        s = r.build(in);
    }
    EXPECT_EQ(s.count(OverlayKind::ROAD_BARRIER), 0u) << "dissolved after stopping";
}

// Rule 5: at most three sign items, highest priority first; hazards are never decluttered.
TEST(ArRenderer, DeclutterKeepsTheThreeMostImportantSignsAndAllHazards) {
    auto r = renderer();
    RenderInputs in;
    in.egoSpeedMps = 10;
    auto at = [](int cls, double y) {
        SignObservation o = sign(cls);
        o.box = {100 + 60 * y, 300, 40, 40};
        return o;
    };
    OverlayScene s;
    for (int k = 0; k < 3; ++k) {
        in.frameMs = 1000 + 33 * k;
        in.signs = {at(sign_class::kOtherWarning, 0), at(sign_class::kRoundabout, 1),
                    at(sign_class::kNoLeft, 2), at(sign_class::kNoEntry, 3),
                    at(sign_class::kSignals, 4)};
        in.hazards = {hazard(1, 1, VULNERABLE_IN_PATH, 0.5, std::nullopt),
                      hazard(2, 0, SWERVING, 0.4, std::nullopt)};
        s = r.build(in);
    }
    EXPECT_TRUE(hasLabel(s, "NO ENTRY"));
    EXPECT_TRUE(hasLabel(s, "NO LEFT TURN"));
    EXPECT_EQ(labels(s).size(), 3u + 1u) << "three sign banners plus the speed label";
    EXPECT_FALSE(hasLabel(s, "WARNING")) << "lowest priority dropped";
    EXPECT_EQ(s.count(OverlayKind::ELLIPSE_GLOW), 2u) << "hazards are never decluttered";
}

TEST(ArRenderer, NoParkingOnlyAtLowSpeed) {
    auto show = [](double kph) {
        auto r = renderer();
        RenderInputs in;
        in.egoSpeedMps = kph / 3.6;
        OverlayScene s;
        for (int k = 0; k < 3; ++k) {
            in.frameMs = 1000 + 33 * k;
            in.signs = {sign(sign_class::kNoParking)};
            s = r.build(in);
        }
        return hasLabel(s, "NO PARKING");
    };
    EXPECT_TRUE(show(10));
    EXPECT_FALSE(show(50));
}
