#pragma once
// RoadNetworkUpdater — turns RoadStatus's overrides into routable OSRM data. See
// docs/BUILD_GUIDE.md Part 11.2.1.
//
// Every update starts from a PRISTINE copy of the base map (data/maps/current), then runs
// osrm-customize --segment-speed-file with the COMPLETE current override set. Correct by
// construction: a closure that has expired or been lifted cannot linger because an earlier update
// wrote it into the files. (osrm-customize rewrites its outputs in place, so the base map is never
// customised directly, and a hard-linked copy would corrupt it.) Measured on the Nairobi map
// (2026-09-26): customize takes ~1 s, so a sensor closure can reroute within seconds.
//
// apply() returns the `.osrm` base path to (re)initialise NavigationEngine and MapMatcher with:
// the base map itself when there is nothing to override, or a fresh re-weighted copy. The two
// newest copies are kept (one may still be in use while the next is built). Runs the OSRM tools as
// processes, off the real-time path.

#include <cstdint>
#include <string>

#include "ar_drive_assist/nav/RoadStatus.h"

namespace ar_drive_assist {

class RoadNetworkUpdater {
public:
    // baseOsrm: e.g. data/maps/current/nairobi.osrm. workRoot: where re-weighted copies go.
    RoadNetworkUpdater(std::string baseOsrm, std::string workRoot);

    std::string apply(const RoadStatus& status, std::uint64_t nowMs);
    const std::string& lastError() const { return lastError_; }

private:
    std::string baseOsrm_, workRoot_, lastError_;
    unsigned counter_ = 0;
};

}  // namespace ar_drive_assist
