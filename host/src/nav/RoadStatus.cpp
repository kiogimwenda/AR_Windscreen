#include "ar_drive_assist/nav/RoadStatus.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace ar_drive_assist {

RoadStatus::RoadStatus(RoadStatusConfig cfg) : cfg_(cfg) {}

void RoadStatus::closeSegment(const SegmentKey& seg, std::uint64_t nowMs) {
    auto& until = closedUntil_[seg];
    until = std::max(until, nowMs + cfg_.closureTtlMs);  // seen again: the closure is extended
}

int RoadStatus::closeRouteAhead(const Route& route, double alongM, double lengthM,
                                std::uint64_t nowMs, bool bothDirections) {
    // Route segment i runs geometry[i] -> geometry[i+1], which is OSM node osmNodes[i] ->
    // osmNodes[i+1] (NavigationEngine returns one node id per geometry point).
    if (route.osmNodes.size() != route.geometry.size() ||
        route.cumulativeM.size() != route.geometry.size())
        return 0;
    int n = 0;
    for (std::size_t i = 0; i + 1 < route.osmNodes.size(); ++i) {
        const bool overlaps =
            route.cumulativeM[i + 1] > alongM && route.cumulativeM[i] < alongM + lengthM;
        if (!overlaps || route.osmNodes[i] == route.osmNodes[i + 1]) continue;
        closeSegment({route.osmNodes[i], route.osmNodes[i + 1]}, nowMs);
        ++n;
        if (bothDirections) {
            closeSegment({route.osmNodes[i + 1], route.osmNodes[i]}, nowMs);
            ++n;
        }
    }
    return n;
}

void RoadStatus::observe(ObservationType type, const GeoPoint& p, std::uint64_t nowMs) {
    for (RoadObservation& o : observations_) {
        if (o.type == type && geo::distanceM(o.position, p) <= cfg_.mergeRadiusM) {
            // Running mean of the position: repeated sightings refine where it is.
            const double w = 1.0 / (o.sightings + 1);
            o.position.lat += w * (p.lat - o.position.lat);
            o.position.lon += w * (p.lon - o.position.lon);
            o.lastSeenMs = std::max(o.lastSeenMs, nowMs);
            ++o.sightings;
            return;
        }
    }
    observations_.push_back({type, p, nowMs, nowMs, 1});
}

void RoadStatus::setLive(const SegmentKey& seg, double speedKph, std::uint64_t observedMs,
                         const std::string& source) {
    auto it = live_.find(seg);
    if (it != live_.end() && it->second.observedMs > observedMs) return;  // keep the newer record
    live_[seg] = {std::max(0.0, speedKph), observedMs, source};
}

std::vector<SegmentOverride> RoadStatus::activeOverrides(std::uint64_t nowMs) const {
    std::map<SegmentKey, SegmentOverride> out;
    for (const auto& [seg, l] : live_) {
        if (nowMs > l.observedMs + cfg_.liveMaxAgeMs) continue;  // stale: base weights apply
        out[seg] = {seg, l.speedKph, l.source};
    }
    for (const auto& [seg, until] : closedUntil_) {
        if (nowMs >= until) continue;
        out[seg] = {seg, 0.0, "sensor"};  // the sensors win over any live record
    }
    std::vector<SegmentOverride> v;
    for (const auto& [seg, o] : out) v.push_back(o);
    return v;
}

bool RoadStatus::writeSegmentSpeedCsv(const std::string& path, std::uint64_t nowMs) const {
    std::ofstream f(path);
    if (!f) return false;
    for (const SegmentOverride& o : activeOverrides(nowMs)) {
        f << o.segment.first << ',' << o.segment.second << ','
          << static_cast<long>(o.speedKph + 0.5) << '\n';
    }
    return static_cast<bool>(f);
}

bool RoadStatus::saveObservations(const std::string& path) const {
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp);
        if (!f) return false;
        f.precision(10);
        for (const RoadObservation& o : observations_) {
            f << static_cast<int>(o.type) << ',' << o.position.lat << ',' << o.position.lon << ','
              << o.firstSeenMs << ',' << o.lastSeenMs << ',' << o.sightings << '\n';
        }
        if (!f) return false;
    }
    return std::rename(tmp.c_str(), path.c_str()) == 0;  // atomic: never a half-written file
}

bool RoadStatus::loadObservations(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    std::vector<RoadObservation> loaded;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream s(line);
        RoadObservation o;
        int type = 0;
        char c1, c2, c3, c4, c5;
        if (!(s >> type >> c1 >> o.position.lat >> c2 >> o.position.lon >> c3 >> o.firstSeenMs >>
              c4 >> o.lastSeenMs >> c5 >> o.sightings) ||
            type < 0 || type > static_cast<int>(ObservationType::CLOSURE)) {
            return false;  // a damaged file is rejected whole, not half-loaded
        }
        o.type = static_cast<ObservationType>(type);
        loaded.push_back(o);
    }
    observations_ = std::move(loaded);
    return true;
}

}  // namespace ar_drive_assist
