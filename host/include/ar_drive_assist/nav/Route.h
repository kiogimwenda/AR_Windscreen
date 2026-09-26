#pragma once
// Route — what NavigationEngine returns (Part 11.2), and the geometry MapMatcher (11.3) and
// RoadSurfaceProjector (11.4) need from it. Pure data and geometry: no OSRM here, so the
// geometry is unit-tested in CI.
//
// A route is its FULL road geometry, an ordered polyline of latitude/longitude points along the
// road network, plus the turn-by-turn steps. The overlay needs the geometry: text instructions
// alone cannot place a line on the road.
//
// Progress along the route is arc length along that polyline (cumulativeM). Projecting a position
// onto the route gives:
//   - alongM:  how far along the route it is (the navigation line starts here);
//   - offsetM: signed distance from the route's centre line, LEFT positive (a lane is ~3.5 m;
//              tens of metres means off the route);
//   - heading: the route segment's direction, CCW from east (VehiclePose's convention).
// Projection is done in a local metric frame centred on each segment, which is exact to
// millimetres over road-segment lengths.

#include <cstdint>
#include <string>
#include <vector>

#include "ar_drive_assist/common/Geo.h"

namespace ar_drive_assist {

struct RouteStep {
    std::string maneuver;  // OSRM maneuver type: "depart", "turn", "roundabout", "arrive", ...
    std::string modifier;  // "left", "slight right", "straight", ... (may be empty)
    std::string roadName;  // name of the road this step continues on (may be empty)
    GeoPoint location;     // where the maneuver happens
    double alongM = 0;     // route distance of the maneuver from the start
    double distanceM = 0;  // length of this step
    int laneCount = 0;     // lanes at the maneuver's intersection, 0 if OSM has no lane data
};

struct Route {
    bool valid = false;
    std::vector<GeoPoint> geometry;       // full-resolution polyline, start -> destination
    std::vector<double> cumulativeM;      // arc length at each geometry point (0 at start)
    std::vector<std::uint64_t> osmNodes;  // OSM node ids along the route (for closures, 11.2.1)
    std::vector<RouteStep> steps;
    double distanceM = 0, durationS = 0;  // OSRM's totals

    // Fills cumulativeM from geometry. Call after setting geometry.
    void computeCumulative();
    double lengthM() const { return cumulativeM.empty() ? 0.0 : cumulativeM.back(); }
    // The point `alongM` metres along the route (clamped to its ends).
    GeoPoint pointAt(double alongM) const;
};

struct RouteProjection {
    double alongM = 0;
    double offsetM = 0;       // signed, LEFT of the direction of travel positive
    double headingDeg = 0;    // of the segment, CCW from east
    std::size_t segment = 0;  // index i of segment [i, i+1]
    bool valid = false;
};

// Nearest point of the route to `p`, searching segments [firstSegment, lastSegment] (clamped).
// Searching a window around the previous result, not the whole route, stops a route that doubles
// back on itself (a U-turn, a flyover above the road) from snapping to the wrong pass.
RouteProjection projectOntoRoute(const Route& route, const GeoPoint& p,
                                 std::size_t firstSegment = 0,
                                 std::size_t lastSegment = static_cast<std::size_t>(-1));

}  // namespace ar_drive_assist
