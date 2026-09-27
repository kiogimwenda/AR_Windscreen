#include "ar_drive_assist/nav/MapMatcher.h"

#include <algorithm>
#include <cmath>

namespace ar_drive_assist {
namespace {

double angleDiffDeg(double a, double b) {
    double d = std::fmod(a - b, 360.0);
    if (d > 180) d -= 360;
    if (d <= -180) d += 360;
    return std::abs(d);
}

// Segment index containing arc length `along` (clamped).
std::size_t segmentAt(const Route& r, double along) {
    if (r.cumulativeM.size() < 2) return 0;
    const auto it = std::upper_bound(r.cumulativeM.begin(), r.cumulativeM.end(), along);
    const std::size_t i =
        it == r.cumulativeM.begin() ? 0 : static_cast<std::size_t>(it - r.cumulativeM.begin()) - 1;
    return std::min(i, r.cumulativeM.size() - 2);
}

int laneHint(const Route& r, double along) {
    int lanes = 0;
    for (const RouteStep& s : r.steps) {
        if (s.alongM > along) break;
        lanes = s.laneCount;
    }
    return lanes > 0 ? lanes : 1;
}

}  // namespace

MapMatcher::MapMatcher(MapMatcherConfig cfg) : cfg_(cfg) {}

void MapMatcher::setBackend(std::unique_ptr<MatchBackend> backend) {
    backend_ = std::move(backend);
}

void MapMatcher::reset() {
    trace_.clear();
    committed_.clear();
    last_ = {};
    haveProgress_ = false;
    lastCallMs_ = lastPoseMs_ = 0;
    offRouteCount_ = 0;
    havePrevMatched_ = false;
}

MapMatcher::MatchedPosition MapMatcher::match(const VehiclePose& pose, const LocalFrame& frame,
                                              const Route& route) {
    const std::uint64_t t = pose.timestamp_ms;
    const GeoPoint fix = frame.toGeodetic(pose.x, pose.y);

    // 1. Throttle: at most one backend call per minIntervalMs, whatever the last result was (an
    // unmatched stretch must not turn into a Match per pose). Between matches, a valid on-route
    // position is dead-reckoned along the route with the fused speed.
    if (backendCalls_ > 0 && t < lastCallMs_ + cfg_.minIntervalMs) {
        if (!last_.valid) return last_;
        MatchedPosition p = last_;
        p.fromMatch = false;
        if (p.onRoute && t > lastPoseMs_) {
            const double speed = std::max(0.0, pose.speed_kph / 3.6);
            p.distanceAlongRouteM = std::min(
                route.lengthM(), last_.distanceAlongRouteM + speed * (t - lastPoseMs_) / 1000.0);
            const GeoPoint q = route.pointAt(p.distanceAlongRouteM);
            p.latitude = q.lat;
            p.longitude = q.lon;
        }
        last_ = p;
        lastPoseMs_ = t;
        return p;
    }

    // 2. Trace window. COMMITTED points are at least traceSpacingMs apart (a new one every
    // spacing). The query is the committed points with the newest one replaced by the current
    // pose, unless the current pose is itself due to be committed. (Replacing the stored point
    // each time instead would keep moving its timestamp forward, so it would never become a full
    // spacing old and the trace would never grow past one point: found by
    // TraceWindowIsSpacedAndBounded.)
    const TracePoint tp{fix, t, cfg_.gpsRadiusM};
    if (!committed_.empty() && t > committed_.back().timestampMs + cfg_.traceGapResetMs) {
        committed_.clear();  // after a long gap the old trace says nothing about this position
    }
    if (committed_.empty() || t >= committed_.back().timestampMs + cfg_.traceSpacingMs) {
        committed_.push_back(tp);
        while (committed_.size() > cfg_.traceLength) committed_.erase(committed_.begin());
        trace_ = committed_;
    } else {
        trace_ = committed_;
        trace_.back() = tp;
    }

    // 3. Match.
    lastCallMs_ = lastPoseMs_ = t;
    if (!backend_) return last_ = MatchedPosition{};
    ++backendCalls_;
    const BackendMatch m = backend_->matchLast(trace_);
    MatchedPosition out;
    if (!m.matched) {
        out.valid = false;
        out.distanceAlongRouteM = last_.distanceAlongRouteM;
        return last_ = out;
    }
    out.valid = true;
    out.fromMatch = true;
    out.latitude = m.p.lat;
    out.longitude = m.p.lon;

    // 4. Project onto the route, in a window around the previous progress.
    std::size_t first = 0, lastSeg = static_cast<std::size_t>(-1);
    if (haveProgress_) {
        first = segmentAt(route, last_.distanceAlongRouteM - cfg_.searchBehindM);
        lastSeg = segmentAt(route, last_.distanceAlongRouteM + cfg_.searchAheadM);
    }
    const RouteProjection proj = projectOntoRoute(route, m.p, first, lastSeg);

    // Heading of travel from successive matched points (when they are far enough apart).
    double travelHeading = pose.heading_deg;
    if (havePrevMatched_ && geo::distanceM(prevMatched_, m.p) > 2.0) {
        travelHeading = geo::headingDeg(prevMatched_, m.p);
    }
    prevMatched_ = m.p;
    havePrevMatched_ = true;

    if (!proj.valid) {
        out.onRoute = false;
        out.roadHeadingDeg = static_cast<float>(travelHeading);
        return last_ = out;
    }

    // 5. Progress never decreases.
    const double prevAlong = haveProgress_ ? last_.distanceAlongRouteM : 0.0;
    out.distanceAlongRouteM = haveProgress_ ? std::max(prevAlong, proj.alongM) : proj.alongM;
    out.lateralOffsetM = proj.offsetM;

    // 6. Off route: too far from the route, or driving against it, several matches in a row.
    const bool far = std::abs(proj.offsetM) > cfg_.offRouteM;
    const bool wrongWay =
        angleDiffDeg(pose.heading_deg, proj.headingDeg) > cfg_.wrongWayDeg && pose.speed_kph > 5.0f;
    offRouteCount_ = (far || wrongWay) ? offRouteCount_ + 1 : 0;
    out.onRoute = offRouteCount_ < cfg_.offRouteConfirm;
    out.roadHeadingDeg = static_cast<float>(out.onRoute ? proj.headingDeg : travelHeading);
    out.laneCountHint = laneHint(route, out.distanceAlongRouteM);
    haveProgress_ = true;
    return last_ = out;
}

}  // namespace ar_drive_assist
