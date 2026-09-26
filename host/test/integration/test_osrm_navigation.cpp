// OSRM routing and map-matching integration tests — see docs/BUILD_GUIDE.md Part 11.2 / 11.3.
// Label `osrm`: needs the OSRM tools on PATH (Phase 0 builds them). Not run in CI.
//
// Part 1, a road network with KNOWN geometry. The test writes a small OSM file near Nairobi CBD
// and builds it with the real osrm-extract / partition / customize (MLD):
//
//        D ---------- Service Road ----------- F      y = 15 m (a service road beside the main
//        road) |                                     | A ----------- Main Road --- B ------- C y =
//        0
//                                    |
//                                Side Road
//                                    |
//                                    G                 (500, -600)
//
//   A = (0, 0), B = (500, 0), C = (1000, 0), D = (0, 15), F = (1000, 15), local metres east/north.
//
// Part 2, the real Nairobi map (data/maps/current, scripts/refresh_osm.sh). Skipped when the map
// has not been built on this machine.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <set>

#include "ar_drive_assist/nav/MapMatcher.h"
#include "ar_drive_assist/nav/NavigationEngine.h"
#include "ar_drive_assist/nav/RoadNetworkUpdater.h"

using namespace ar_drive_assist;
namespace fs = std::filesystem;

namespace {

const GeoPoint kOrigin{-1.2864, 36.8172};
const LocalFrame kFrame(kOrigin);

GeoPoint at(double e, double n) {
    return kFrame.toGeodetic(e, n);
}
void local(const GeoPoint& g, double& e, double& n) {
    kFrame.toLocal(g, e, n);
}

VehiclePose pose(std::uint64_t ms, double e, double n, double headingDeg, double kph) {
    VehiclePose p;
    p.timestamp_ms = ms;
    p.x = e;
    p.y = n;
    p.heading_deg = static_cast<float>(headingDeg);
    p.speed_kph = static_cast<float>(kph);
    return p;
}

// Writes the fixture network. Roads share their junction nodes (one node per position), and each
// road has a node every <= 50 m so matching has real geometry to work with.
void writeFixture(const fs::path& osm) {
    std::ofstream f(osm);
    f.precision(10);
    f << "<?xml version='1.0' encoding='UTF-8'?>\n<osm version='0.6' generator='ar-test'>\n";
    long id = 1;
    std::map<std::pair<long, long>, long> byPos;  // decimetre position -> node id
    auto nodeAt = [&](double e, double n) {
        const auto key = std::make_pair(std::lround(e * 10), std::lround(n * 10));
        const auto it = byPos.find(key);
        if (it != byPos.end()) return it->second;
        const GeoPoint g = at(e, n);
        f << "<node id='" << id << "' version='1' lat='" << g.lat << "' lon='" << g.lon << "'/>\n";
        byPos[key] = id;
        return id++;
    };
    struct Way {
        std::vector<long> nodes;
        std::string tags;
    };
    std::vector<Way> ways;
    auto road = [&](std::vector<std::pair<double, double>> pts, std::string tags) {
        Way w{{}, std::move(tags)};
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
            const auto [e0, n0] = pts[i];
            const auto [e1, n1] = pts[i + 1];
            const int k =
                std::max(1, static_cast<int>(std::ceil(std::hypot(e1 - e0, n1 - n0) / 50)));
            for (int j = (i == 0 ? 0 : 1); j <= k; ++j) {
                w.nodes.push_back(nodeAt(e0 + (e1 - e0) * j / k, n0 + (n1 - n0) * j / k));
            }
        }
        ways.push_back(w);
    };
    road({{0, 0}, {500, 0}, {1000, 0}},
         "<tag k='highway' v='primary'/><tag k='name' v='Main Road'/>");
    road({{0, 15}, {1000, 15}},
         "<tag k='highway' v='residential'/><tag k='name' v='Service Road'/>");
    road({{0, 0}, {0, 15}}, "<tag k='highway' v='residential'/><tag k='name' v='West Link'/>");
    road({{1000, 0}, {1000, 15}},
         "<tag k='highway' v='residential'/><tag k='name' v='East Link'/>");
    road({{500, 0}, {500, -600}}, "<tag k='highway' v='secondary'/><tag k='name' v='Side Road'/>");
    long wid = 1;
    for (const Way& w : ways) {
        f << "<way id='" << wid++ << "' version='1'>";
        for (long n : w.nodes) f << "<nd ref='" << n << "'/>";
        f << w.tags << "</way>\n";
    }
    f << "</osm>\n";
}

class OsrmFixture : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (std::system("command -v osrm-extract > /dev/null 2>&1") != 0) return;
        dir() = fs::temp_directory_path() / ("ar_nav_fixture_" + std::to_string(::getpid()));
        fs::create_directories(dir());
        const fs::path osm = dir() / "fixture.osm";
        writeFixture(osm);
        const std::string base = (dir() / "fixture").string();
        const std::string profile = "/usr/local/share/osrm/profiles/car.lua";
        const std::string cmd = "osrm-extract -p " + profile + " " + osm.string() + " > " +
                                (dir() / "build.log").string() + " 2>&1 && osrm-partition " + base +
                                ".osrm >> " + (dir() / "build.log").string() +
                                " 2>&1 && osrm-customize " + base + ".osrm >> " +
                                (dir() / "build.log").string() + " 2>&1";
        built() = std::system(cmd.c_str()) == 0;
    }
    static void TearDownTestSuite() {
        if (!dir().empty()) fs::remove_all(dir());
    }
    void SetUp() override {
        if (!built())
            GTEST_SKIP() << "OSRM tools missing or fixture build failed (" << dir() << ")";
        ASSERT_TRUE(nav.init((dir() / "fixture.osrm").string())) << nav.lastError();
    }
    static fs::path& dir() {
        static fs::path d;
        return d;
    }
    static bool& built() {
        static bool b = false;
        return b;
    }
    NavigationEngine nav;
};

}  // namespace

// Main Road end to end: ~1000 m, on the named road, starting and ending where asked.
TEST_F(OsrmFixture, RouteAlongMainRoad) {
    const Route r = nav.route(at(20, 1), at(980, 1));
    ASSERT_TRUE(r.valid) << nav.lastError();
    EXPECT_NEAR(r.lengthM(), 960, 3);
    double e, n;
    local(r.geometry.front(), e, n);
    EXPECT_NEAR(e, 20, 1);
    EXPECT_NEAR(n, 0, 1);
    ASSERT_GE(r.steps.size(), 2u);
    EXPECT_EQ(r.steps.front().maneuver, "depart");
    EXPECT_EQ(r.steps.front().roadName, "Main Road");
    EXPECT_EQ(r.steps.back().maneuver, "arrive");
    EXPECT_FALSE(r.osmNodes.empty());
}

// East along Main Road, then south down Side Road: a RIGHT turn at B, 500 m along.
TEST_F(OsrmFixture, TurnOntoSideRoad) {
    const Route r = nav.route(at(0, 0), at(500, -600));
    ASSERT_TRUE(r.valid) << nav.lastError();
    EXPECT_NEAR(r.lengthM(), 1100, 3);
    bool found = false;
    for (const RouteStep& s : r.steps) {
        if (s.roadName == "Side Road" && s.maneuver != "arrive") {
            found = true;
            EXPECT_EQ(s.modifier, "right");
            EXPECT_NEAR(s.alongM, 500, 3);
        }
    }
    EXPECT_TRUE(found);
}

// A car at (250, 0) heading WEST wants to reach C in the EAST. Without the departure constraint,
// the route starts east (a U-turn the driver cannot make on a busy road); with it, the route
// departs west and loops round via the Service Road.
TEST_F(OsrmFixture, DepartsInTheDirectionOfTravel) {
    const Route free = nav.route(at(250, 0), at(1000, 0));
    ASSERT_TRUE(free.valid);
    EXPECT_NEAR(free.lengthM(), 750, 3);
    const double west = 180;
    const Route r = nav.route(at(250, 0), at(1000, 0), &west);
    ASSERT_TRUE(r.valid) << nav.lastError();
    EXPECT_GT(r.lengthM(), 1200);
    EXPECT_NEAR(std::abs(projectOntoRoute(r, r.geometry.front()).headingDeg), 180, 5);
}

// The Service Road runs 15 m from the Main Road. A car drives the Service Road; its fused
// position has 5 m (1 sigma) of noise, so a single fix is often nearer the WRONG road. Matching
// the trace (MapMatcher + OSRM) must keep it on the Service Road: required at least 95% of
// matches after the first 3 s. The single-fix nearest-road rate is measured alongside, to show
// what the trace buys.
TEST_F(OsrmFixture, TraceKeepsTheCarOnTheParallelServiceRoad) {
    MapMatcher mm;
    ASSERT_TRUE(mm.init((dir() / "fixture.osrm").string()));
    const Route service = nav.route(at(0, 15), at(1000, 15));
    ASSERT_TRUE(service.valid);
    std::mt19937 rng(5);
    std::normal_distribution<double> noise(0, 5);
    int matches = 0, onService = 0, naiveWrong = 0;
    for (int k = 0; k < 900; ++k) {  // 90 s at 10 Hz, 10 m/s: 900 m
        const double e = 50 + 1.0 * k * 0.95, ne = noise(rng), nn = noise(rng);
        const auto m = mm.match(pose(100 * k, e + ne, 15 + nn, 0, 36), kFrame, service);
        if (std::abs(15 + nn - 15) > std::abs(15 + nn - 0)) ++naiveWrong;  // nearer Main Road
        if (k < 30 || !m.fromMatch || !m.valid) continue;
        ++matches;
        double me, mn;
        local({m.latitude, m.longitude}, me, mn);
        if (std::abs(mn - 15) < 3) ++onService;
    }
    ASSERT_GT(matches, 500);
    const double rate = static_cast<double>(onService) / matches;
    std::printf(
        "[ info ] trace match on Service Road: %.1f%% (%d/%d); single-fix nearest road "
        "wrong: %.1f%%\n",
        100 * rate, onService, matches, 100.0 * naiveWrong / 900);
    EXPECT_GE(rate, 0.95);
}

// Full pipeline: route along Main Road, drive it with noisy fixes through the OSRM-backed
// MapMatcher. Progress never decreases, stays on route, and ends within 10 m of the truth.
TEST_F(OsrmFixture, ProgressAlongARouteWithRealMatching) {
    MapMatcher mm;
    ASSERT_TRUE(mm.init((dir() / "fixture.osrm").string()));
    const Route r = nav.route(at(0, 0), at(500, -600));
    ASSERT_TRUE(r.valid);
    std::mt19937 rng(9);
    std::normal_distribution<double> noise(0, 3);
    double prev = -1, truth = 0;
    for (int k = 0; k < 100 * 10; ++k) {  // 100 s at 10 Hz, 10 m/s: 1000 m of the 1100
        truth = 1.0 * k;
        double e, n;
        local(r.pointAt(truth), e, n);
        const double heading = truth < 500 ? 0 : -90;
        const auto m =
            mm.match(pose(100 * k, e + noise(rng), n + noise(rng), heading, 36), kFrame, r);
        if (!m.valid) continue;
        EXPECT_GE(m.distanceAlongRouteM, prev) << "k=" << k;
        prev = m.distanceAlongRouteM;
        EXPECT_TRUE(m.onRoute) << "k=" << k;
    }
    EXPECT_NEAR(prev, truth, 10);
}

// Layer 3 end to end (Part 11.2.1): the car finds Main Road blocked 600-700 m along its route.
// The closure goes into RoadStatus, RoadNetworkUpdater re-weights a copy of the map, and the
// reloaded engine routes round via the Service Road (15 + 1000 + 15 = 1030 m instead of 1000).
// After the closure's 2 h TTL the base map is used again. A live record 20 min old closing the
// road is stale and changes nothing.
TEST_F(OsrmFixture, ClosedRoadIsAvoidedThenReopensAfterItsTtl) {
    const std::string base = (dir() / "fixture.osrm").string();
    const Route before = nav.route(at(0, 0), at(1000, 0));
    ASSERT_TRUE(before.valid);
    EXPECT_NEAR(before.lengthM(), 1000, 3);

    RoadStatus rs;
    ASSERT_GT(rs.closeRouteAhead(before, 600, 100, 0, true), 0);
    RoadNetworkUpdater up(base, (dir() / "work").string());
    const std::string closedMap = up.apply(rs, 1000);
    ASSERT_FALSE(closedMap.empty()) << up.lastError();
    ASSERT_NE(closedMap, base);
    NavigationEngine closedNav;
    ASSERT_TRUE(closedNav.init(closedMap)) << closedNav.lastError();
    const Route around = closedNav.route(at(0, 0), at(1000, 0));
    ASSERT_TRUE(around.valid) << closedNav.lastError();
    EXPECT_NEAR(around.lengthM(), 1030, 5);
    double e, n;
    local(around.pointAt(500), e, n);
    EXPECT_NEAR(n, 15, 1) << "halfway along, the detour is on the Service Road";

    const std::uint64_t later = 3ull * 3600 * 1000;  // past the 2 h TTL
    EXPECT_EQ(up.apply(rs, later), base);

    RoadStatus staleLive;
    for (std::size_t i = 0; i + 1 < before.osmNodes.size(); ++i) {
        staleLive.setLive({before.osmNodes[i], before.osmNodes[i + 1]}, 0, 0, "test");
    }
    EXPECT_EQ(up.apply(staleLive, 20 * 60 * 1000), base);
}

// --- The real Nairobi map ---------------------------------------------------------------------

namespace {
std::string nairobiMap() {
    const fs::path p = fs::path(AR_MAPS_DIR) / "current" / "nairobi.osrm";
    return fs::exists(p.string() + ".fileIndex") || fs::exists(p.string() + ".ebg") ? p.string()
                                                                                    : "";
}
}  // namespace

// CBD (Kencom) to Thika town: the smoke-test route, now through NavigationEngine. Road distance
// is about 45 km; required 35-60 km, a full geometry and turn steps naming Thika Road.
TEST(NairobiMap, CbdToThika) {
    const std::string map = nairobiMap();
    if (map.empty()) GTEST_SKIP() << "no Nairobi map; run host/scripts/refresh_osm.sh";
    NavigationEngine nav;
    ASSERT_TRUE(nav.init(map)) << nav.lastError();
    const Route r = nav.route({-1.2864, 36.8252}, {-1.0333, 37.0693});
    ASSERT_TRUE(r.valid) << nav.lastError();
    std::printf("[ info ] CBD -> Thika: %.1f km, %.0f min, %zu points, %zu steps\n",
                r.lengthM() / 1000, r.durationS / 60, r.geometry.size(), r.steps.size());
    EXPECT_GT(r.lengthM(), 35000);
    EXPECT_LT(r.lengthM(), 60000);
    bool thikaRoad = false;
    for (const RouteStep& s : r.steps)
        thikaRoad = thikaRoad || s.roadName.find("Thika") != std::string::npos;
    EXPECT_TRUE(thikaRoad);
}

// A synthetic drive along the first 5 km of that route (sampled from its geometry at 10 Hz,
// 15 m/s, 4 m GPS-like noise) through the OSRM MapMatcher on the real map. This exercises real
// Nairobi road geometry; a RECORDED drive is still required for the Phase 8 exit criterion.
TEST(NairobiMap, SyntheticDriveStaysMatchedOnRealRoads) {
    const std::string map = nairobiMap();
    if (map.empty()) GTEST_SKIP() << "no Nairobi map; run host/scripts/refresh_osm.sh";
    NavigationEngine nav;
    ASSERT_TRUE(nav.init(map));
    const Route r = nav.route({-1.2864, 36.8252}, {-1.0333, 37.0693});
    ASSERT_TRUE(r.valid);
    MapMatcher mm;
    ASSERT_TRUE(mm.init(map));
    const LocalFrame frame(r.geometry.front());
    std::mt19937 rng(11);
    std::normal_distribution<double> noise(0, 4);
    double prev = -1, truth = 0;
    int offRoute = 0, n = 0;
    for (int k = 0; truth < 5000; ++k) {
        truth = 1.5 * k;
        const GeoPoint g = r.pointAt(truth);
        double e, no;
        frame.toLocal(g, e, no);
        const double h = projectOntoRoute(r, g).headingDeg;
        const auto m = mm.match(pose(100 * k, e + noise(rng), no + noise(rng), h, 54), frame, r);
        if (!m.valid) continue;
        ++n;
        EXPECT_GE(m.distanceAlongRouteM, prev);
        prev = m.distanceAlongRouteM;
        offRoute += !m.onRoute;
    }
    std::printf(
        "[ info ] 5 km synthetic drive: %d positions, %d off-route, final progress error "
        "%.1f m\n",
        n, offRoute, prev - truth);
    EXPECT_EQ(offRoute, 0);
    EXPECT_NEAR(prev, truth, 15);
}

// A closure on the real map: the car finds the CBD -> Thika route blocked 2.0-2.2 km in. The
// re-weighted map must route round it (no segment of the new route is a closed segment) and the
// whole closure-to-reroute cycle is timed, since Part 11.2.1 promises a reroute within seconds.
TEST(NairobiMap, ClosureOnTheRealMapIsRoutedAround) {
    const std::string map = nairobiMap();
    if (map.empty()) GTEST_SKIP() << "no Nairobi map; run host/scripts/refresh_osm.sh";
    NavigationEngine nav;
    ASSERT_TRUE(nav.init(map));
    const GeoPoint cbd{-1.2864, 36.8252}, thika{-1.0333, 37.0693};
    const Route before = nav.route(cbd, thika);
    ASSERT_TRUE(before.valid);
    RoadStatus rs;
    ASSERT_GT(rs.closeRouteAhead(before, 2000, 200, 0, true), 0);
    const auto t0 = std::chrono::steady_clock::now();
    RoadNetworkUpdater up(
        map, (fs::temp_directory_path() / ("ar_nav_nbo_" + std::to_string(::getpid()))).string());
    const std::string closedMap = up.apply(rs, 1000);
    ASSERT_FALSE(closedMap.empty()) << up.lastError();
    NavigationEngine rerouted;
    ASSERT_TRUE(rerouted.init(closedMap));
    const Route after = rerouted.route(cbd, thika);
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    ASSERT_TRUE(after.valid) << rerouted.lastError();
    std::set<SegmentKey> closed;
    for (const auto& o : rs.activeOverrides(1000)) closed.insert(o.segment);
    int used = 0;
    for (std::size_t i = 0; i + 1 < after.osmNodes.size(); ++i) {
        used += closed.count({after.osmNodes[i], after.osmNodes[i + 1]});
    }
    std::printf(
        "[ info ] closure -> reroute: %.1f s; route %.1f km -> %.1f km; closed segments "
        "used: %d\n",
        secs, before.lengthM() / 1000, after.lengthM() / 1000, used);
    EXPECT_EQ(used, 0);
    EXPECT_LT(secs, 10.0);
    fs::remove_all(fs::path(closedMap).parent_path().parent_path());
}
