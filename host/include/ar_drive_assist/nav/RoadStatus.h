#pragma once
// RoadStatus — what the car knows about the state of the roads beyond the base map. See
// docs/BUILD_GUIDE.md Part 11.2.1 ("Road status: three layers, and the map never overrules the
// sensors").
//
// ---------------------------------------------------------------------------------------------
// What it holds
//   Layer 3 (own sensors):
//     - local CLOSURES: road segments the car itself found blocked (a confirmed barrier or
//       blockage across the lane, or a no-entry sign on the route's next edge). They expire after
//       `closureTtlMs` (default 2 h) unless seen again.
//     - road OBSERVATIONS: potholes, bumps, blockages, with position, first/last seen and the
//       number of sightings, merged within `mergeRadiusM`. Kept across trips (save/load), so
//       tomorrow's route can warn about today's pothole.
//   Layer 2 (live network, when a provider is connected):
//     - provider-neutral SEGMENT records {from node, to node, speed or closed, source, observed
//     at}.
//       A provider adapter (e.g. TomTom, after its gates pass) fills these; nothing here is
//       provider-specific. Records older than `liveMaxAgeMs` (15 min) are STALE and ignored, so
//       the segment falls back to its base-map speed.
//
// What it produces
//   activeOverrides(now): the segment speeds OSRM should route with right now, one per directed
//   segment. Precedence: a sensor closure beats any live record (the map never overrules the
//   sensors, even to REOPEN a road the car found closed). A live closure is speed 0. Speed 0 makes
//   an OSRM segment impassable.
//   writeSegmentSpeedCsv(): those overrides in osrm-customize's --segment-speed-file format
//   ("from_osm_id,to_osm_id,speed_kmh"), for RoadNetworkUpdater.
//
// Segments are DIRECTED OSM node pairs, as OSRM's segment-speed file expects. A barrier across
// the whole road closes both directions; a no-entry sign closes one.
// ---------------------------------------------------------------------------------------------

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "ar_drive_assist/common/Geo.h"
#include "ar_drive_assist/nav/Route.h"

namespace ar_drive_assist {

using SegmentKey = std::pair<std::uint64_t, std::uint64_t>;  // directed: (from OSM node, to)

struct SegmentOverride {
    SegmentKey segment;
    double speedKph = 0;  // 0 = closed
    std::string source;   // "sensor", or the live provider's name
};

enum class ObservationType { POTHOLE, BUMP, BLOCKAGE, CLOSURE };

struct RoadObservation {
    ObservationType type = ObservationType::POTHOLE;
    GeoPoint position;
    std::uint64_t firstSeenMs = 0, lastSeenMs = 0;
    int sightings = 0;
};

struct RoadStatusConfig {
    std::uint64_t closureTtlMs = 2ull * 3600 * 1000;
    std::uint64_t liveMaxAgeMs = 15ull * 60 * 1000;
    double mergeRadiusM = 10;
};

class RoadStatus {
public:
    explicit RoadStatus(RoadStatusConfig cfg = {});

    // --- Layer 3: own sensors
    void closeSegment(const SegmentKey& seg, std::uint64_t nowMs);
    // Closes the route's segments from `alongM` for `lengthM` metres (the blocked stretch
    // ahead), in the direction of travel, and in reverse too when `bothDirections`. Returns how
    // many directed segments were closed.
    int closeRouteAhead(const Route& route, double alongM, double lengthM, std::uint64_t nowMs,
                        bool bothDirections);
    void observe(ObservationType type, const GeoPoint& p, std::uint64_t nowMs);
    const std::vector<RoadObservation>& observations() const { return observations_; }

    // --- Layer 2: live network (provider-neutral)
    void setLive(const SegmentKey& seg, double speedKph, std::uint64_t observedMs,
                 const std::string& source);

    // --- Output
    std::vector<SegmentOverride> activeOverrides(std::uint64_t nowMs) const;
    bool writeSegmentSpeedCsv(const std::string& path, std::uint64_t nowMs) const;

    // Observations persist across trips (CSV: type,lat,lon,first_ms,last_ms,sightings).
    bool saveObservations(const std::string& path) const;
    bool loadObservations(const std::string& path);

private:
    struct Live {
        double speedKph;
        std::uint64_t observedMs;
        std::string source;
    };
    RoadStatusConfig cfg_;
    std::map<SegmentKey, std::uint64_t> closedUntil_;  // sensor closures
    std::map<SegmentKey, Live> live_;
    std::vector<RoadObservation> observations_;
};

}  // namespace ar_drive_assist
