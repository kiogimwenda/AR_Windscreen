#pragma once
// NavigationEngine — routing on the offline Nairobi map with OSRM (MLD). See
// docs/BUILD_GUIDE.md Part 11.2 and 11.2.1.
//
// routeTo() returns a Route (Route.h): the FULL road geometry, the turn-by-turn steps, and the OSM
// node ids along it (so a road the car finds closed can be marked closed, Part 11.2.1 layer 3).
// MapMatcher and RoadSurfaceProjector need the geometry; turn text alone cannot place a line on
// the road.
//
// Departure direction. A route from a moving car must start in the direction the car is already
// going. Without that, OSRM may route from the far side of a dual carriageway and begin with a
// U-turn the driver cannot make. When the car is moving (> 2 m/s), the start is constrained to
// road segments within +-45 deg of its heading (OSRM "bearings").
//
// Data: the `.osrm` base path, normally data/maps/current/nairobi.osrm (scripts/refresh_osm.sh).

#include <memory>
#include <string>

#include "ar_drive_assist/common/Geo.h"
#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/nav/Route.h"

namespace osrm {
class OSRM;  // src/nav/OsrmEngine.h; the OSRM headers stay out of public headers
}

namespace ar_drive_assist {

class NavigationEngine {
public:
    NavigationEngine();
    ~NavigationEngine();

    bool init(const std::string& osrmDataPath);  // false (and lastError()) if the data is missing

    // From the fused pose (local metres, converted with SensorFusion's frame) to a destination.
    Route routeTo(double destLat, double destLon, const VehiclePose& currentPose,
                  const LocalFrame& frame);
    // From an explicit start. `headingDeg` (CCW from east) constrains the departure when given.
    Route route(const GeoPoint& from, const GeoPoint& to, const double* headingDeg = nullptr);

    const std::string& lastError() const { return lastError_; }

private:
    std::shared_ptr<osrm::OSRM> osrm_;
    std::string lastError_;
};

}  // namespace ar_drive_assist
