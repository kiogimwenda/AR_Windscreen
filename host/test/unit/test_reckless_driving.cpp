// RecklessDrivingDetector tests — see docs/BUILD_GUIDE.md Part 9.2 and RecklessDrivingDetector.h.
//
// Each rule on a synthetic track that should trigger it, and on the look-alike that should not
// (a steady turn is not a swerve; smooth acceleration is not erratic speed; 0.9 s of short gap is
// not sustained tailgating). Tracks come from the real tracker, fed at 10 Hz.

#include <gtest/gtest.h>

#include <cmath>
#include <functional>

#include "ar_drive_assist/safety/RecklessDrivingDetector.h"

using namespace ar_drive_assist;

namespace {

constexpr double kPi = 3.14159265358979323846;

ObjectMeasurement meas(double x, double y, int cls = 0) {
    return {{x, y}, Eigen::Matrix2d::Identity() * 0.1 * 0.1, cls, std::nullopt, true};
}

TrackerConfig realConfig() {
    return loadTrackerConfig(std::string(HOST_SOURCE_DIR) + "/config/motion_prediction.yaml");
}

// Ego parked at the origin facing east unless moved; `where(k)` gives the object's world
// position at step k. Returns the tracker (real motion config) after `steps` updates.
MultiObjectTracker trackOver(int steps, const std::function<Eigen::Vector2d(int)>& where,
                             int cls = 0) {
    MultiObjectTracker t(realConfig());
    for (int k = 0; k < steps; ++k) {
        const Eigen::Vector2d p = where(k);
        t.update({static_cast<std::uint64_t>(100 * k), {}, {meas(p.x(), p.y(), cls)}});
    }
    return t;
}

EgoState parkedEgo(std::uint64_t ms) {
    EgoState e;
    e.timestampMs = ms;
    return e;
}

// Only call on a NAMED vector: a pointer into a temporary dangles (a bug the first version of
// these tests had).
const ThreatAssessment* find(const std::vector<ThreatAssessment>& v, int id) {
    for (const auto& a : v)
        if (a.trackId == id) return &a;
    return nullptr;
}

}  // namespace

TEST(Threat, RiskLevelsFollowTheOverlayDesign) {
    EXPECT_EQ(RecklessDrivingDetector::levelFor(0.0), ThreatLevel::NONE);
    EXPECT_EQ(RecklessDrivingDetector::levelFor(0.2), ThreatLevel::LOW);
    EXPECT_EQ(RecklessDrivingDetector::levelFor(0.4), ThreatLevel::MEDIUM);
    EXPECT_EQ(RecklessDrivingDetector::levelFor(0.75), ThreatLevel::HIGH);
    EXPECT_EQ(RecklessDrivingDetector::levelFor(0.95), ThreatLevel::CRITICAL);
}

// Ego frame by hand: ego at (100, 50) heading north. An object at (100, 70) is 20 m ahead (x = 20,
// y = 0), 16.5 m from the bumper. Ego 10 m/s north, object stopped: closing 10 m/s.
TEST(Threat, EgoFrameByHand) {
    MultiObjectTracker t(realConfig());
    for (int k = 0; k < 5; ++k)
        t.update({static_cast<std::uint64_t>(100 * k), {}, {meas(100, 70)}});
    EgoState ego;
    ego.pose = {100, 50, kPi / 2};
    ego.speedMps = 10;
    ego.timestampMs = 400;
    const auto rel = toEgoFrame(t.tracks(), ego, {});
    ASSERT_EQ(rel.size(), 1u);
    EXPECT_NEAR(rel[0].xM, 20, 0.05);
    EXPECT_NEAR(rel[0].yM, 0, 0.05);
    EXPECT_NEAR(rel[0].gapM, 16.5, 0.05);
    EXPECT_NEAR(rel[0].closingSpeedMps, 10, 0.3);
    EXPECT_TRUE(rel[0].inEgoPath);
    EXPECT_TRUE(rel[0].confirmed);
    EXPECT_TRUE(rel[0].rangeMeasured);
}

// A measurement whose range is only a MiDaS estimate never makes the range "measured".
TEST(Threat, EstimatedRangeIsNotMeasured) {
    MultiObjectTracker t(realConfig());
    for (int k = 0; k < 5; ++k) {
        ObjectMeasurement m = meas(20, 0);
        m.rangeMeasured = false;
        t.update({static_cast<std::uint64_t>(100 * k), {}, {m}});
    }
    EXPECT_FALSE(toEgoFrame(t.tracks(), parkedEgo(400), {})[0].rangeMeasured);
}

TEST(Threat, ForwardCollisionWarningBandAndRisk) {
    // Ego at 15 m/s, stopped car with a bumper gap of 30 m: TTC 2.0 s. With T = 1.8: in the
    // warning band (< 3.6 s), r = (3.6 - 2.0) / 1.8 = 0.889, HIGH.
    MultiObjectTracker t(realConfig());
    for (int k = 0; k < 5; ++k)
        t.update({static_cast<std::uint64_t>(100 * k), {}, {meas(33.5, 0)}});
    EgoState ego = parkedEgo(400);
    ego.speedMps = 15;
    RecklessDrivingDetector det;
    const auto a = det.assess(t.tracks(), ego);
    ASSERT_EQ(a.size(), 1u);
    EXPECT_TRUE(a[0].flags & FORWARD_COLLISION);
    EXPECT_NEAR(a[0].ttcS, 2.0, 0.05);
    EXPECT_NEAR(a[0].risk, (3.6 - 2.0) / 1.8, 0.03);
    EXPECT_EQ(a[0].level, ThreatLevel::HIGH);
}

// Tailgating: ego at 20 m/s behind a lead car at the same speed, 15 m bumper gap: 0.75 s < 1.0 s.
// Not flagged until the short gap has lasted 1 s.
TEST(Threat, TailgatingNeedsToBeSustained) {
    MultiObjectTracker t(realConfig());
    RecklessDrivingDetector det;
    std::vector<bool> flagged;
    for (int k = 0; k < 20; ++k) {
        const double egoX = 2.0 * k;
        t.update({static_cast<std::uint64_t>(100 * k), {egoX, 0, 0}, {meas(18.5, 0)}});
        EgoState ego;
        ego.pose = {egoX, 0, 0};
        ego.speedMps = 20;
        ego.timestampMs = 100 * k;
        const auto a = det.assess(t.tracks(), ego);
        const auto* lead = find(a, 1);
        flagged.push_back(lead && (lead->flags & TAILGATING));
        if (lead && k > 5) {
            EXPECT_NEAR(lead->timeGapS, 0.75, 0.05);
        }
    }
    // Confirmed at k = 2 (third hit); the timer starts there. Not flagged within the next 0.9 s,
    // flagged from 1.0 s on.
    EXPECT_FALSE(flagged[2 + 9]);
    EXPECT_TRUE(flagged[2 + 10]);
    EXPECT_TRUE(flagged.back());
}

// Erratic: surging 10 +- 2.5 m/s with a 3 s period (peak acceleration 5.2 m/s^2, a driver
// stabbing the throttle and brake). Smooth: accelerating steadily from 5 to 15 m/s over the window
// (3.3 m/s^2). Both are within what the vehicle motion model follows.
TEST(Threat, ErraticSpeedButNotSmoothAcceleration) {
    double x = 0;
    auto surging = trackOver(30, [&](int k) {
        x += (10 + 2.5 * std::sin(2 * kPi * 0.1 * k / 3.0)) * 0.1;
        return Eigen::Vector2d(x, 12);
    });
    RecklessDrivingDetector det;
    const auto sAll = det.assess(surging.tracks(), parkedEgo(2900));
    const auto* s = find(sAll, 1);
    ASSERT_NE(s, nullptr);
    EXPECT_TRUE(s->flags & ERRATIC_SPEED);

    double x2 = 0;
    auto smooth = trackOver(30, [&](int k) {
        x2 += (5 + 10.0 * k / 30) * 0.1;
        return Eigen::Vector2d(x2, 12);
    });
    const auto a = RecklessDrivingDetector().assess(smooth.tracks(), parkedEgo(2900));
    const auto* m = find(a, 1);
    EXPECT_TRUE(!m || !(m->flags & ERRATIC_SPEED));
}

// Weaving: 12 m/s east with a 0.8 m lateral wander, period 3 s (peak lateral acceleration
// 3.5 m/s^2). A steady turn of radius 40 m at 12 m/s (3.6 m/s^2) is clearly curved but is not a
// swerve.
TEST(Threat, SwervingButNotASteadyTurn) {
    auto weave = trackOver(30, [](int k) {
        const double t = 0.1 * k;
        return Eigen::Vector2d(12 * t, 12 + 0.8 * std::sin(2 * kPi * t / 3.0));
    });
    const auto wAll = RecklessDrivingDetector().assess(weave.tracks(), parkedEgo(2900));
    const auto* w = find(wAll, 1);
    ASSERT_NE(w, nullptr);
    EXPECT_TRUE(w->flags & SWERVING);

    // An ordinary lane change, 3.5 m over 3 s (the quickest normal one): must NOT be a swerve.
    auto lane = trackOver(30, [](int k) {
        const double t = 0.1 * k;
        return Eigen::Vector2d(12 * t, 12 + 3.5 * (1 - std::cos(kPi * std::min(t, 3.0) / 3.0)) / 2);
    });
    const auto lAll = RecklessDrivingDetector().assess(lane.tracks(), parkedEgo(2900));
    const auto* l = find(lAll, 1);
    EXPECT_TRUE(!l || !(l->flags & SWERVING)) << "a lane change is not a swerve";

    auto turn = trackOver(30, [](int k) {
        const double th = 0.3 * 0.1 * k;
        return Eigen::Vector2d(40 * std::sin(th), 52 - 40 * std::cos(th));
    });
    const auto a = RecklessDrivingDetector().assess(turn.tracks(), parkedEgo(2900));
    const auto* s = find(a, 1);
    EXPECT_TRUE(!s || !(s->flags & SWERVING));
}

TEST(Threat, PedestrianInThePathIsVulnerableAndAtLeastMedium) {
    auto ped = trackOver(10, [](int) { return Eigen::Vector2d(15, 0.5); }, 1);
    const auto a = RecklessDrivingDetector().assess(ped.tracks(), parkedEgo(900));
    ASSERT_EQ(a.size(), 1u);
    EXPECT_TRUE(a[0].flags & VULNERABLE_IN_PATH);
    EXPECT_GE(a[0].risk, 0.4);
    // The same pedestrian on the pavement, 6 m to the side: nothing.
    auto side = trackOver(10, [](int) { return Eigen::Vector2d(15, 6); }, 1);
    EXPECT_TRUE(RecklessDrivingDetector().assess(side.tracks(), parkedEgo(900)).empty());
}

TEST(Threat, CrossingUsesThePredictorRisk) {
    auto ped = trackOver(10, [](int k) { return Eigen::Vector2d(15, -4 + 0.14 * k); }, 1);
    CollisionRisk r;
    r.conflict = true;
    r.probability = 0.6;
    const auto a = RecklessDrivingDetector().assess(ped.tracks(), parkedEgo(900), {{1, r}});
    ASSERT_EQ(a.size(), 1u);
    EXPECT_TRUE(a[0].flags & CROSSING);
    EXPECT_TRUE(a[0].flags & VULNERABLE_IN_PATH) << "crossing into the path";
    EXPECT_NEAR(a[0].risk, 0.6, 1e-9);
}

TEST(Threat, TentativeTracksAreIgnored) {
    MultiObjectTracker t(realConfig());
    t.update({0, {}, {meas(10, 0, 1)}});  // one sighting: tentative
    EXPECT_TRUE(RecklessDrivingDetector().assess(t.tracks(), parkedEgo(0)).empty());
}
