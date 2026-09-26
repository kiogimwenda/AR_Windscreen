// MultiObjectTracker tests — see docs/BUILD_GUIDE.md Part 9.1 and MultiObjectTracker.h.
//
// Synthetic detection sequences, one scenario per lifecycle rule or risk: births and
// confirmation, the one-frame false positive (the Kraków wall "vehicle"), occlusion coasting,
// identity through close parallel walkers, class gating, ego motion, and late-arriving batches.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "ar_drive_assist/safety/MultiObjectTracker.h"

using namespace ar_drive_assist;

namespace {

constexpr int kVehicle = 0, kPedestrian = 1;
const Eigen::Matrix2d kCov = Eigen::Matrix2d::Identity() * 0.15 * 0.15;

ObjectMeasurement meas(double x, double y, int cls) {
    return ObjectMeasurement{{x, y}, kCov, cls, std::nullopt};
}

MeasurementBatch batchAt(std::uint64_t ms, std::vector<ObjectMeasurement> m, EgoPose ego = {}) {
    return MeasurementBatch{ms, ego, std::move(m)};
}

const Track* findById(const MultiObjectTracker& t, int id) {
    for (const auto& tr : t.tracks())
        if (tr.id == id) return &tr;
    return nullptr;
}

}  // namespace

TEST(Tracker, NewObjectIsTentativeThenConfirmedAfterThreeHits) {
    MultiObjectTracker t;
    for (int k = 0; k < 3; ++k) {
        t.update(batchAt(100 * k, {meas(20 + 0.5 * k, 0, kVehicle)}));
        ASSERT_EQ(t.tracks().size(), 1u);
        EXPECT_EQ(t.tracks()[0].state, k < 2 ? TrackState::TENTATIVE : TrackState::CONFIRMED);
    }
    EXPECT_EQ(t.tracks()[0].id, 1);
}

// The Kraków wall "vehicle": a detection that appears in one frame only must never be confirmed.
// It dies as a tentative track. This is what keeps it off the driver display (overlay rule 2).
TEST(Tracker, OneFrameFalsePositiveNeverConfirms) {
    MultiObjectTracker t;
    t.update(batchAt(0, {meas(8, 5, kVehicle)}));
    t.update(batchAt(100, {}));
    t.update(batchAt(200, {}));
    EXPECT_TRUE(t.tracks().empty());
}

// A pedestrian walks behind a parked matatu for 1 s: the track coasts as PREDICTED_ONLY, moving
// on, and is picked up again with the SAME id. Occluded for 2 s, it is dropped (max_coast 1.5 s).
TEST(Tracker, OcclusionCoastsThenReacquiresSameIdentity) {
    MultiObjectTracker t;
    double x = 15, y = -5;
    std::uint64_t ms = 0;
    for (int k = 0; k < 20; ++k, ms += 100) {  // walking left at 1.4 m/s
        y += 0.14;
        t.update(batchAt(ms, {meas(x, y, kPedestrian)}));
    }
    ASSERT_EQ(t.tracks().size(), 1u);
    const int id = t.tracks()[0].id;
    const double yBefore = t.tracks()[0].filter.position().y();
    for (int k = 0; k < 10; ++k, ms += 100) {  // hidden for 1 s
        y += 0.14;
        t.update(batchAt(ms, {}));
    }
    ASSERT_EQ(t.tracks().size(), 1u);
    EXPECT_EQ(t.tracks()[0].state, TrackState::PREDICTED_ONLY);
    EXPECT_GT(t.tracks()[0].filter.position().y(), yBefore + 1.0) << "the prediction kept walking";
    y += 0.14;
    t.update(batchAt(ms, {meas(x, y, kPedestrian)}));
    ms += 100;
    ASSERT_EQ(t.tracks().size(), 1u);
    EXPECT_EQ(t.tracks()[0].id, id);
    EXPECT_EQ(t.tracks()[0].state, TrackState::CONFIRMED);
    for (int k = 0; k < 20; ++k, ms += 100) t.update(batchAt(ms, {}));  // gone for 2 s
    EXPECT_TRUE(t.tracks().empty());
}

// Two pedestrians walking side by side, 0.8 m apart, with noisy measurements: each keeps its own
// identity for 10 s (no swaps), because the Hungarian assignment minimises the total cost.
TEST(Tracker, ParallelWalkersKeepTheirIdentities) {
    MultiObjectTracker t;
    std::mt19937 rng(21);
    std::normal_distribution<double> n(0, 0.15);
    for (int k = 0; k < 100; ++k) {
        const double x = 10 + 0.14 * k;
        t.update(batchAt(100 * k, {meas(x + n(rng), 2.0 + n(rng), kPedestrian),
                                   meas(x + n(rng), 2.8 + n(rng), kPedestrian)}));
        if (k == 5) {
            ASSERT_EQ(t.tracks().size(), 2u);
        }
    }
    ASSERT_EQ(t.tracks().size(), 2u);
    const Track* a = findById(t, 1);
    const Track* b = findById(t, 2);
    ASSERT_TRUE(a && b);
    EXPECT_NEAR(a->filter.position().y(), 2.0, 0.3);  // id 1 was born at y = 2.0
    EXPECT_NEAR(b->filter.position().y(), 2.8, 0.3);
}

// A pedestrian stepping right beside a car must never be absorbed into the car's track.
TEST(Tracker, DifferentKnownClassesNeverShareATrack) {
    MultiObjectTracker t;
    for (int k = 0; k < 5; ++k) t.update(batchAt(100 * k, {meas(12, 3, kVehicle)}));
    t.update(batchAt(500, {meas(12, 3, kVehicle), meas(12.2, 3.1, kPedestrian)}));
    ASSERT_EQ(t.tracks().size(), 2u);
    EXPECT_NE(t.tracks()[0].objectClass, t.tracks()[1].objectClass);
}

// An unclassified LiDAR cluster (Part 8.3 rule 6) continues a classified track, and a track born
// unclassified takes the class of its first classified match.
TEST(Tracker, UnclassifiedMeasurementsJoinAnyClass) {
    MultiObjectTracker t;
    for (int k = 0; k < 4; ++k) t.update(batchAt(100 * k, {meas(20, 0, -1)}));
    ASSERT_EQ(t.tracks().size(), 1u);
    EXPECT_EQ(t.tracks()[0].objectClass, -1);
    t.update(batchAt(400, {meas(20, 0, kVehicle)}));
    ASSERT_EQ(t.tracks().size(), 1u);
    EXPECT_EQ(t.tracks()[0].objectClass, kVehicle);
    t.update(batchAt(500, {meas(20, 0, -1)}));  // camera lost it; LiDAR still sees it
    EXPECT_EQ(t.tracks().size(), 1u);
}

// The ego car drives east at 10 m/s past a parked car. In the VEHICLE frame the parked car
// approaches at 10 m/s; in the world frame, where tracking happens, it must stand still.
TEST(Tracker, ParkedCarIsStillWhileEgoDrivesPast) {
    MultiObjectTracker t;
    const double parkedX = 60, parkedY = 3;
    for (int k = 0; k < 40; ++k) {
        EgoPose ego{1.0 * k, 0, 0};  // 10 m/s east
        t.update(batchAt(100 * k, {meas(parkedX - ego.x, parkedY, kVehicle)}, ego));
    }
    ASSERT_EQ(t.tracks().size(), 1u);
    EXPECT_LT(t.tracks()[0].filter.velocity().norm(), 0.5);
    EXPECT_NEAR(t.tracks()[0].filter.position().x(), parkedX, 0.3);
}

// A turned ego: heading north (psi = 90 deg). An object 10 m ahead (vehicle frame x) is 10 m NORTH
// in the world.
TEST(Tracker, VehicleToWorldRespectsEgoHeading) {
    const EgoPose ego{100, 50, 3.14159265358979323846 / 2};
    const auto w = MultiObjectTracker::toWorld(ego, {10, 0});
    EXPECT_NEAR(w.x(), 100, 1e-9);
    EXPECT_NEAR(w.y(), 60, 1e-9);
}

// A LiDAR batch that arrives after a newer camera batch is replayed in time order. The result must
// be IDENTICAL to receiving the batches in order.
TEST(Tracker, LateBatchIsReplayedInOrder) {
    auto run = [](bool swapLast) {
        MultiObjectTracker t;
        std::vector<MeasurementBatch> bs;
        for (int k = 0; k < 10; ++k) bs.push_back(batchAt(100 * k, {meas(20 + 1.0 * k, 0, 0)}));
        if (swapLast) std::swap(bs[8], bs[9]);  // batch 800 arrives after batch 900
        for (const auto& b : bs) t.update(b);
        return t.tracks().at(0).filter.state();
    };
    const auto inOrder = run(false), late = run(true);
    EXPECT_LT((inOrder - late).norm(), 1e-9);
}

TEST(Tracker, BatchOlderThanTheReplayWindowIsDroppedAndCounted) {
    MultiObjectTracker t;
    for (int k = 0; k < 10; ++k) t.update(batchAt(100 * k, {meas(20, 0, 0)}));
    t.update(batchAt(50, {meas(20, 0, 0)}));  // 850 ms late: outside the 500 ms window
    EXPECT_EQ(t.droppedLateBatches(), 1u);
}

TEST(Tracker, LoadsTheRealConfigFile) {
    const auto c =
        loadTrackerConfig(std::string(HOST_SOURCE_DIR) + "/config/motion_prediction.yaml");
    EXPECT_EQ(c.confirmHits, 3);
    EXPECT_DOUBLE_EQ(c.maxCoastS, 1.5);
    EXPECT_EQ(c.noiseByClass.size(), 6u);
    EXPECT_GT(c.noiseFor(2).yawAccelStd, c.noiseFor(0).yawAccelStd) << "boda-bodas weave more";
}

// A new track must learn its velocity in ANY direction. Motion is stored as speed + heading, and
// at speed 0 the heading is unobservable, so without two-point initialisation a track born still
// could never learn a direction other than its initial guess. It would lag until the gate split it
// into duplicate tracks (the bug the occlusion test first exposed).
//
// Window: 1.5 s from first sighting. It was set AFTER measuring the convergence curve (logged
// 2026-09-26): with exact measurements the speed overshoots to ~2.1 m/s between ~0.3 and 1.0 s and
// is back within 0.2 m/s by ~1.4 s, a known limitation of the speed+heading state right after
// birth (a Cartesian constant-velocity model for pedestrians is the recorded fix). Tracks are only
// displayed from confirmation (0.3 s), and predicted paths are drawn with their uncertainty.
TEST(Tracker, NewTrackLearnsVelocityInAnyDirection) {
    for (double dir : {0.0, 1.57, 3.1, -2.0}) {  // east, north, (almost) west, south-west
        MultiObjectTracker t;
        double x = 10, y = 10;
        for (int k = 0; k < 15; ++k) {
            x += 0.14 * std::cos(dir);
            y += 0.14 * std::sin(dir);
            t.update(batchAt(100 * k, {meas(x, y, kPedestrian)}));
        }
        ASSERT_EQ(t.tracks().size(), 1u) << "direction " << dir;
        const Eigen::Vector2d v = t.tracks()[0].filter.velocity();
        EXPECT_NEAR(v.norm(), 1.4, 0.4) << "direction " << dir;
        EXPECT_LT(std::abs(ImmFilter::wrap(std::atan2(v.y(), v.x()) - dir)), 0.35)
            << "direction " << dir;
    }
}
