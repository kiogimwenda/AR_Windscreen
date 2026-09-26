// RoadStatus tests — see docs/BUILD_GUIDE.md Part 11.2.1 and RoadStatus.h.
//
// The rules of the three-layer road status, without OSRM: sensor closures (and their expiry),
// provider-neutral live records (and their staleness), precedence (the sensors win), road
// observations and their persistence, and the segment-speed file. That OSRM really routes around
// a closure is tested in host/test/integration/test_osrm_navigation.cpp (label osrm).

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "ar_drive_assist/nav/RoadStatus.h"

using namespace ar_drive_assist;

namespace {

constexpr std::uint64_t kMin = 60 * 1000, kHour = 60 * kMin;
const SegmentKey kSeg{101, 102};

std::string tmpPath(const std::string& name) {
    return ::testing::TempDir() + name;
}

}  // namespace

TEST(RoadStatus, SensorClosureIsImpassableThenExpiresAfterItsTtl) {
    RoadStatus rs;
    rs.closeSegment(kSeg, 0);
    auto o = rs.activeOverrides(1 * kHour);
    ASSERT_EQ(o.size(), 1u);
    EXPECT_EQ(o[0].speedKph, 0.0);
    EXPECT_EQ(o[0].source, "sensor");
    EXPECT_TRUE(rs.activeOverrides(2 * kHour).empty()) << "2 h TTL";
}

TEST(RoadStatus, SeeingAClosureAgainExtendsIt) {
    RoadStatus rs;
    rs.closeSegment(kSeg, 0);
    rs.closeSegment(kSeg, 90 * kMin);
    EXPECT_EQ(rs.activeOverrides(3 * kHour).size(), 1u);
    EXPECT_TRUE(rs.activeOverrides(90 * kMin + 2 * kHour).empty());
}

// "The map never overrules the sensors": a live feed saying the road flows at 50 km/h does not
// reopen a segment the car itself found blocked.
TEST(RoadStatus, SensorClosureBeatsALiveRecordThatSaysTheRoadIsOpen) {
    RoadStatus rs;
    rs.closeSegment(kSeg, 0);
    rs.setLive(kSeg, 50, 5 * kMin, "tomtom");
    const auto o = rs.activeOverrides(6 * kMin);
    ASSERT_EQ(o.size(), 1u);
    EXPECT_EQ(o[0].speedKph, 0.0);
    EXPECT_EQ(o[0].source, "sensor");
}

// Live data older than 15 min is dropped, so the segment falls back to its base-map speed.
TEST(RoadStatus, StaleLiveRecordFallsBackToBaseWeights) {
    RoadStatus rs;
    rs.setLive(kSeg, 12, 0, "tomtom");
    EXPECT_EQ(rs.activeOverrides(15 * kMin).size(), 1u);
    EXPECT_TRUE(rs.activeOverrides(15 * kMin + 1).empty());
}

TEST(RoadStatus, NewerLiveRecordWinsAndOlderArrivalsAreIgnored) {
    RoadStatus rs;
    rs.setLive(kSeg, 30, 10 * kMin, "tomtom");
    rs.setLive(kSeg, 5, 8 * kMin, "tomtom");  // arrives late, but older
    EXPECT_EQ(rs.activeOverrides(11 * kMin).at(0).speedKph, 30);
    rs.setLive(kSeg, 0, 12 * kMin, "tomtom");  // an incident closes it
    EXPECT_EQ(rs.activeOverrides(13 * kMin).at(0).speedKph, 0);
}

// No provider connected (no signal): no live records, nothing overridden, nothing breaks.
TEST(RoadStatus, NoLiveProviderMeansNoOverrides) {
    RoadStatus rs;
    EXPECT_TRUE(rs.activeOverrides(0).empty());
}

// Route segment i is OSM node pair (osmNodes[i], osmNodes[i+1]); closing 150-350 m along a route
// of 100 m segments closes segments 1, 2 and 3.
TEST(RoadStatus, CloseRouteAheadPicksTheSegmentsByArcLength) {
    Route r;
    const LocalFrame f({-1.2864, 36.8172});
    for (int i = 0; i <= 6; ++i) {
        r.geometry.push_back(f.toGeodetic(100.0 * i, 0));
        r.osmNodes.push_back(1000 + i);
    }
    r.computeCumulative();
    RoadStatus one;
    EXPECT_EQ(one.closeRouteAhead(r, 150, 200, 0, false), 3);
    const auto o = one.activeOverrides(1);
    ASSERT_EQ(o.size(), 3u);
    EXPECT_EQ(o[0].segment, SegmentKey(1001, 1002));
    EXPECT_EQ(o[2].segment, SegmentKey(1003, 1004));
    RoadStatus both;
    EXPECT_EQ(both.closeRouteAhead(r, 150, 200, 0, true), 6);
    EXPECT_EQ(both.activeOverrides(1).size(), 6u);
}

TEST(RoadStatus, SegmentSpeedFileIsWhatOsrmCustomizeReads) {
    RoadStatus rs;
    rs.closeSegment({5, 6}, 0);
    rs.setLive({7, 8}, 23.6, 0, "tomtom");
    const std::string p = tmpPath("speeds.csv");
    ASSERT_TRUE(rs.writeSegmentSpeedCsv(p, 1));
    std::ifstream f(p);
    std::string a, b;
    std::getline(f, a);
    std::getline(f, b);
    EXPECT_EQ(a, "5,6,0");
    EXPECT_EQ(b, "7,8,24");
}

// Observations of the same kind within 10 m are one pothole seen again; a different kind, or
// 30 m away, is a different one.
TEST(RoadStatus, ObservationsMergeByKindAndDistance) {
    RoadStatus rs;
    const LocalFrame f({-1.2864, 36.8172});
    rs.observe(ObservationType::POTHOLE, f.toGeodetic(0, 0), 0);
    rs.observe(ObservationType::POTHOLE, f.toGeodetic(4, 0), kHour);
    rs.observe(ObservationType::BUMP, f.toGeodetic(2, 0), kHour);
    rs.observe(ObservationType::POTHOLE, f.toGeodetic(30, 0), kHour);
    ASSERT_EQ(rs.observations().size(), 3u);
    const RoadObservation& p = rs.observations()[0];
    EXPECT_EQ(p.sightings, 2);
    EXPECT_EQ(p.lastSeenMs, kHour);
    double e, n;
    f.toLocal(p.position, e, n);
    EXPECT_NEAR(e, 2.0, 0.01) << "position is the mean of the sightings";
}

TEST(RoadStatus, ObservationsSurviveARestartAndDamagedFilesAreRejectedWhole) {
    RoadStatus rs;
    rs.observe(ObservationType::POTHOLE, {-1.28, 36.81}, 5);
    rs.observe(ObservationType::BLOCKAGE, {-1.29, 36.82}, 7);
    const std::string p = tmpPath("observations.csv");
    ASSERT_TRUE(rs.saveObservations(p));
    RoadStatus back;
    ASSERT_TRUE(back.loadObservations(p));
    ASSERT_EQ(back.observations().size(), 2u);
    EXPECT_EQ(back.observations()[1].type, ObservationType::BLOCKAGE);
    EXPECT_NEAR(back.observations()[0].position.lat, -1.28, 1e-9);
    {
        std::ofstream f(p, std::ios::app);
        f << "garbage line\n";
    }
    RoadStatus damaged;
    damaged.observe(ObservationType::BUMP, {0, 0}, 1);
    EXPECT_FALSE(damaged.loadObservations(p));
    EXPECT_EQ(damaged.observations().size(), 1u) << "existing observations kept";
}
