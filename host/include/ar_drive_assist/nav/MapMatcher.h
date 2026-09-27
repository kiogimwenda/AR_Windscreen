#pragma once
// MapMatcher — snaps the fused pose onto the road network and measures progress along the route.
// See docs/BUILD_GUIDE.md Part 11.3.
//
// ---------------------------------------------------------------------------------------------
// Why: the fused GPS/EKF position (Part 8.2) is good to a few metres. That is not enough to know
// which road the car is on where roads run close together (a service road beside Thika Road, a
// flyover), let alone how far along the route it is. OSRM's Match service fits a short TRACE of
// recent positions onto the road graph with a hidden-Markov model. A trace, not one fix, is what
// makes it reliable: a single fix 8 m from two parallel roads is ambiguous; a trace of how the car
// has been moving is not.
//
// The loop, per call (the caller calls at pose rate; the matcher decides when to do real work):
//   1. Throttle. At most one Match per `minIntervalMs` (default 100 ms, 10 Hz). Road identity and
//      progress change far slower than vehicle dynamics. Between matches, progress is advanced
//      by the fused speed (dead reckoning along the route), so the overlay does not stutter.
//   2. Trace window. The last `traceLength` poses, at least `traceSpacingMs` apart (OSRM's
//      timestamps are whole seconds and must increase). The newest pose replaces the newest
//      trace point until it is a full spacing later.
//   3. Match the trace, via a MatchBackend (OSRM in the car; a fake in unit tests, so all the
//      logic below is tested in CI without OSRM).
//   4. Project the matched point onto the route, searching only a window around the previous
//      progress, so a route that passes the same place twice (a U-turn, a flyover) is not
//      snapped to the wrong pass.
//   5. Progress never decreases. Backward jitter from GPS noise is clamped. A real reversal is
//      caught as "wrong way" instead (step 6).
//   6. Off route: the matched road is more than `offRouteM` from the route, OR the car is heading
//      against the route (more than 120 deg), for `offRouteConfirm` matches in a row. One noisy
//      match never triggers a reroute.
//
// Frames: the pose is in SensorFusion's local metres. It is turned back into latitude/longitude
// with the SAME LocalFrame SensorFusion used (SensorFusion::frame()), so no second approximation
// is introduced. Headings are degrees CCW from east, as in VehiclePose.
// ---------------------------------------------------------------------------------------------

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ar_drive_assist/common/Geo.h"
#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/nav/Route.h"

namespace ar_drive_assist {

struct TracePoint {
    GeoPoint p;
    std::uint64_t timestampMs = 0;
    double radiusM = 15;  // search radius around the fix
};

struct BackendMatch {
    bool matched = false;
    GeoPoint p;             // the newest trace point, snapped onto the road graph
    double confidence = 0;  // OSRM's matching confidence, 0-1
};

// Matches a trace onto the road graph and returns where its NEWEST point was snapped.
class MatchBackend {
public:
    virtual ~MatchBackend() = default;
    virtual BackendMatch matchLast(const std::vector<TracePoint>& trace) = 0;
};

struct MapMatcherConfig {
    std::uint64_t minIntervalMs = 100;  // 10 Hz at most (Part 11.3: 5-10 Hz)
    std::uint64_t traceSpacingMs = 1000;
    std::size_t traceLength = 8;
    // A gap longer than this (no poses: logger paused, GPS lost, tunnel) starts a fresh trace.
    // Old points would make OSRM join positions minutes apart as one continuous drive: on Ian's
    // recorded drive, the first fix after a 10-minute gap snapped 41 m off (2026-09-27).
    std::uint64_t traceGapResetMs = 10000;
    double gpsRadiusM = 15;  // ~3-6 sigma of the fused position (Part 8.2: 2.5 m GPS)
    double offRouteM = 25;
    double wrongWayDeg = 120;
    int offRouteConfirm = 3;
    double searchBehindM = 50, searchAheadM = 300;
};

class MapMatcher {
public:
    struct MatchedPosition {
        double latitude = 0, longitude = 0;  // snapped onto the road graph, not the raw fix
        double distanceAlongRouteM = 0;      // arc-length progress along the current Route
        int laneCountHint = 1;     // from OSM lane data at the current step where present; a hint
        float roadHeadingDeg = 0;  // CCW from east (VehiclePose convention)
        bool valid = false;
        bool onRoute = true;        // false once off route has been confirmed: reroute
        double lateralOffsetM = 0;  // from the route centre line, left positive
        bool fromMatch = false;     // false: dead-reckoned between matches
    };

    explicit MapMatcher(MapMatcherConfig cfg = {});

    bool init(const std::string& osrmDataPath);  // OSRM backend, same data as NavigationEngine
    void setBackend(std::unique_ptr<MatchBackend> backend);

    MatchedPosition match(const VehiclePose& fusedPose, const LocalFrame& frame,
                          const Route& currentRoute);
    void reset();  // call when a new route is set

    std::uint64_t backendCalls() const { return backendCalls_; }
    const std::vector<TracePoint>& trace() const { return trace_; }

private:
    MapMatcherConfig cfg_;
    std::unique_ptr<MatchBackend> backend_;
    std::vector<TracePoint> committed_;  // at least traceSpacingMs apart
    std::vector<TracePoint> trace_;      // the last query sent
    MatchedPosition last_;
    bool haveProgress_ = false;
    std::uint64_t lastCallMs_ = 0, lastPoseMs_ = 0;
    int offRouteCount_ = 0;
    GeoPoint prevMatched_;
    bool havePrevMatched_ = false;
    std::uint64_t backendCalls_ = 0;
};

}  // namespace ar_drive_assist
