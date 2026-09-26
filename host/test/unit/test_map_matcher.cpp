// MapMatcher and route-geometry tests — see docs/BUILD_GUIDE.md Part 11.3 and MapMatcher.h.
//
// CI has no OSRM, so the OSRM Match call is replaced by a FAKE backend that snaps the newest trace
// point onto a known road polyline (optionally with scripted errors). Everything MapMatcher itself
// decides is exercised: throttling, the trace window, the route-projection window, progress that
// never decreases, dead reckoning between matches, and off-route / wrong-way confirmation.
// OSRM's own matching and routing are tested against a small real OSM file in
// host/test/integration/test_osrm_navigation.cpp (label `osrm`).

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <random>

#include "ar_drive_assist/nav/MapMatcher.h"

using namespace ar_drive_assist;

namespace {

const GeoPoint kOrigin{-1.2864, 36.8172};  // Nairobi CBD
const LocalFrame kFrame(kOrigin);

GeoPoint at(double east, double north) {
    return kFrame.toGeodetic(east, north);
}

Route routeThrough(std::vector<std::pair<double, double>> en) {
    Route r;
    for (auto [e, n] : en) r.geometry.push_back(at(e, n));
    r.computeCumulative();
    r.valid = true;
    return r;
}

// 500 m east, then 500 m north.
Route lRoute() {
    return routeThrough({{0, 0}, {500, 0}, {500, 500}});
}

VehiclePose pose(std::uint64_t ms, double east, double north, double headingDeg, double kph) {
    VehiclePose p;
    p.timestamp_ms = ms;
    p.x = east;
    p.y = north;
    p.heading_deg = static_cast<float>(headingDeg);
    p.speed_kph = static_cast<float>(kph);
    return p;
}

// Snaps the newest trace point onto `road`, then applies `perturb` (call index, snapped point).
struct FakeBackend : MatchBackend {
    Route road;
    std::function<BackendMatch(std::size_t, BackendMatch)> perturb;
    std::vector<std::vector<TracePoint>>* traces = nullptr;
    std::size_t calls = 0;
    BackendMatch matchLast(const std::vector<TracePoint>& trace) override {
        if (traces) traces->push_back(trace);
        const RouteProjection pr = projectOntoRoute(road, trace.back().p);
        BackendMatch m{true, road.pointAt(pr.alongM), 0.9};
        return perturb ? perturb(calls++, m) : (++calls, m);
    }
};

MapMatcher matcherOn(const Route& road,
                     std::function<BackendMatch(std::size_t, BackendMatch)> p = {},
                     std::vector<std::vector<TracePoint>>* traces = nullptr) {
    MapMatcher mm;
    auto b = std::make_unique<FakeBackend>();
    b->road = road;
    b->perturb = std::move(p);
    b->traces = traces;
    mm.setBackend(std::move(b));
    return mm;
}

}  // namespace

// --- Geodesy and route geometry ---------------------------------------------------------------

TEST(Geo, HaversineAndHeadingReferenceValues) {
    // One degree of arc on the 6371 km mean sphere: 111.195 km.
    EXPECT_NEAR(geo::distanceM({0, 0}, {1, 0}), 111194.9, 0.5);
    EXPECT_NEAR(geo::headingDeg(kOrigin, at(100, 0)), 0.0, 0.01);   // east
    EXPECT_NEAR(geo::headingDeg(kOrigin, at(0, 100)), 90.0, 0.01);  // north
    EXPECT_NEAR(std::abs(geo::headingDeg(kOrigin, at(-100, 0))), 180.0, 0.01);
    EXPECT_NEAR(geo::headingDeg(kOrigin, at(0, -100)), -90.0, 0.01);  // south
}

TEST(Geo, LocalFrameRoundTripIsExactAcrossTheMapArea) {
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-100000, 100000);
    for (int i = 0; i < 200; ++i) {
        const double e = u(rng), n = u(rng);
        double e2, n2;
        kFrame.toLocal(kFrame.toGeodetic(e, n), e2, n2);
        EXPECT_NEAR(e2, e, 1e-6);
        EXPECT_NEAR(n2, n, 1e-6);
    }
}

TEST(Route, LengthAndPointAt) {
    const Route r = lRoute();
    EXPECT_NEAR(r.lengthM(), 1000.0, 0.01);
    double e, n;
    kFrame.toLocal(r.pointAt(250), e, n);
    EXPECT_NEAR(e, 250, 0.01);
    EXPECT_NEAR(n, 0, 0.01);
    kFrame.toLocal(r.pointAt(750), e, n);
    EXPECT_NEAR(e, 500, 0.01);
    EXPECT_NEAR(n, 250, 0.01);
    kFrame.toLocal(r.pointAt(5000), e, n);  // clamped to the destination
    EXPECT_NEAR(n, 500, 0.01);
}

TEST(Route, ProjectionGivesProgressSignedOffsetAndHeading) {
    const Route r = lRoute();
    auto p = projectOntoRoute(r, at(100, 3));
    EXPECT_NEAR(p.alongM, 100, 0.01);
    EXPECT_NEAR(p.offsetM, 3, 0.01) << "north of an eastbound road is LEFT";
    EXPECT_NEAR(p.headingDeg, 0, 0.01);
    p = projectOntoRoute(r, at(100, -3));
    EXPECT_NEAR(p.offsetM, -3, 0.01);
    p = projectOntoRoute(r, at(497, 300));  // 3 m west of the northbound leg: LEFT
    EXPECT_NEAR(p.alongM, 800, 0.01);
    EXPECT_NEAR(p.offsetM, 3, 0.01);
    EXPECT_NEAR(p.headingDeg, 90, 0.01);
    p = projectOntoRoute(r, at(520, -15));  // outside the corner: distance to the corner itself
    EXPECT_NEAR(p.alongM, 500, 0.01);
    EXPECT_NEAR(std::abs(p.offsetM), std::hypot(20, 15), 0.01);
}

// A route that goes out and comes back along the same street (U-turn at the end). A position on
// the street matches BOTH passes; the search window around the previous progress picks the
// right one.
TEST(Route, WindowedProjectionPicksTheRightPassOfADoubledBackRoute) {
    const Route r = routeThrough({{0, 0}, {500, 0}, {500, 4}, {0, 4}});
    const GeoPoint p = at(200, 2);
    EXPECT_NEAR(projectOntoRoute(r, p, 0, 0).alongM, 200, 0.01);  // outbound pass only
    const RouteProjection back = projectOntoRoute(r, p, 2, 2);    // return pass only
    EXPECT_NEAR(back.alongM, 504 + 300, 0.01);
    EXPECT_NEAR(std::abs(back.headingDeg), 180, 0.01);
}

// --- MapMatcher --------------------------------------------------------------------------------

// Poses at 50 Hz for 3 s: at most one backend call per 100 ms (Part 11.3: 5-10 Hz).
TEST(MapMatcher, ThrottlesBackendCallsToTenHertz) {
    const Route r = lRoute();
    MapMatcher mm = matcherOn(r);
    for (int k = 0; k < 150; ++k) mm.match(pose(20 * k, 0.2 * k, 0, 0, 36), kFrame, r);
    EXPECT_GE(mm.backendCalls(), 29u);
    EXPECT_LE(mm.backendCalls(), 31u);
}

// OSRM's Match timestamps are whole seconds and must increase: trace points at least 1 s apart,
// at most 8, the newest always the latest pose.
TEST(MapMatcher, TraceWindowIsSpacedAndBounded) {
    const Route r = lRoute();
    std::vector<std::vector<TracePoint>> traces;
    MapMatcher mm = matcherOn(r, {}, &traces);
    for (int k = 0; k < 600; ++k) mm.match(pose(20 * k, 0.2 * k, 0, 0, 36), kFrame, r);
    ASSERT_FALSE(traces.empty());
    for (const auto& tr : traces) {
        ASSERT_LE(tr.size(), 8u);
        for (std::size_t i = 1; i < tr.size(); ++i) {
            EXPECT_GE(tr[i].timestampMs, tr[i - 1].timestampMs + 1000);
        }
    }
    EXPECT_EQ(traces.back().size(), 8u);
    EXPECT_GE(traces.back().back().timestampMs, 20u * 599 - 100);
}

// The exit criterion's core: along a drive, with a backend whose snapped position jitters +-4 m
// along the road (GPS noise that survives matching), progress NEVER decreases and ends within
// 5 m of the truth.
TEST(MapMatcher, ProgressIsMonotonicUnderJitterAndEndsAccurate) {
    const Route r = lRoute();
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> jitter(-4, 4);
    MapMatcher mm = matcherOn(r, [&](std::size_t, BackendMatch m) {
        const double along = projectOntoRoute(r, m.p).alongM + jitter(rng);
        m.p = r.pointAt(along);
        return m;
    });
    double prev = -1, truth = 0;
    for (int k = 0; k < 50 * 60; ++k) {  // 60 s at 50 Hz, 10 m/s: 600 m, round the corner
        truth = 0.2 * k;
        const GeoPoint g = r.pointAt(truth);
        double e, n;
        kFrame.toLocal(g, e, n);
        const auto m = mm.match(pose(20 * k, e, n, truth < 500 ? 0 : 90, 36), kFrame, r);
        ASSERT_TRUE(m.valid);
        EXPECT_GE(m.distanceAlongRouteM, prev) << "at k=" << k;
        prev = m.distanceAlongRouteM;
        EXPECT_TRUE(m.onRoute);
    }
    EXPECT_NEAR(prev, truth, 5.0);
}

// Between matches, progress advances with the fused speed instead of freezing.
TEST(MapMatcher, DeadReckonsBetweenMatches) {
    const Route r = lRoute();
    MapMatcher mm = matcherOn(r);
    auto a = mm.match(pose(0, 100, 0, 0, 72), kFrame, r);  // 20 m/s
    ASSERT_TRUE(a.fromMatch);
    auto b = mm.match(pose(50, 101, 0, 0, 72), kFrame, r);  // 50 ms later: no backend call
    EXPECT_FALSE(b.fromMatch);
    EXPECT_NEAR(b.distanceAlongRouteM - a.distanceAlongRouteM, 1.0, 1e-6);
    EXPECT_EQ(mm.backendCalls(), 1u);
}

// The car turns off onto a side road. One match 40 m off (a glitch) must not reroute; three in a
// row must.
TEST(MapMatcher, OffRouteNeedsConsecutiveConfirmation) {
    const Route r = lRoute();
    const Route network = routeThrough({{0, 0}, {300, 0}, {300, -500}});  // turns south at 300 m
    MapMatcher glitchy = matcherOn(r, [&](std::size_t i, BackendMatch m) {
        if (i == 5) m.p = at(250, -40);
        return m;
    });
    for (int k = 0; k < 20; ++k) {
        const auto m = glitchy.match(pose(200 * k, 100 + 5 * k, 0, 0, 90), kFrame, r);
        EXPECT_TRUE(m.onRoute) << "k=" << k;
    }
    MapMatcher mm = matcherOn(network);
    bool rerouted = false;
    int matchesOff = 0;
    for (int k = 0; k < 40; ++k) {  // along the side road, 10 m per 200 ms
        const double s = 250 + 10.0 * k;
        const double e = std::min(s, 300.0), n = -std::max(0.0, s - 300);
        const auto m = mm.match(pose(200 * k, e, n, s < 300 ? 0 : -90, 180), kFrame, r);
        if (std::abs(m.lateralOffsetM) > 25) ++matchesOff;
        if (!m.onRoute) {
            rerouted = true;
            EXPECT_GE(matchesOff, 3);
            break;
        }
    }
    EXPECT_TRUE(rerouted);
}

// Driving against the route (U-turned on the route's own road): wrong way, then reroute.
TEST(MapMatcher, WrongWayOnTheRouteTriggersReroute) {
    const Route r = lRoute();
    MapMatcher mm = matcherOn(r);
    bool rerouted = false;
    for (int k = 0; k < 10 && !rerouted; ++k) {
        rerouted = !mm.match(pose(200 * k, 400 - 3 * k, 0, 180, 54), kFrame, r).onRoute;
    }
    EXPECT_TRUE(rerouted);
}

// No match (e.g. a car park the map does not have): invalid, and still throttled.
TEST(MapMatcher, UnmatchedIsInvalidAndStillThrottled) {
    const Route r = lRoute();
    MapMatcher mm = matcherOn(r, [](std::size_t, BackendMatch m) {
        m.matched = false;
        return m;
    });
    for (int k = 0; k < 100; ++k) {
        EXPECT_FALSE(mm.match(pose(20 * k, 0.2 * k, 0, 0, 36), kFrame, r).valid);
    }
    EXPECT_LE(mm.backendCalls(), 21u);
}

TEST(MapMatcher, LaneHintComesFromTheCurrentStepAndDefaultsToOne) {
    Route r = lRoute();
    r.steps = {{"depart", "", "Moi Avenue", at(0, 0), 0, 500, 0},
               {"turn", "left", "Kenyatta Avenue", at(500, 0), 500, 500, 3}};
    MapMatcher mm = matcherOn(r);
    for (int k = 0; k <= 40; ++k) {  // 20 m/s, a pose every 0.5 s: 800 m, round the corner
        const double along = 20.0 * k;
        double e, n;
        kFrame.toLocal(r.pointAt(along), e, n);
        const auto m = mm.match(pose(500 * k, e, n, along < 500 ? 0 : 90, 72), kFrame, r);
        EXPECT_EQ(m.laneCountHint, along < 500 ? 1 : 3) << "along " << along;
    }
}
