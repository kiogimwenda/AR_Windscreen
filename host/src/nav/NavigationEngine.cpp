#include "ar_drive_assist/nav/NavigationEngine.h"

#include <cmath>
#include <osrm/route_parameters.hpp>

#include "OsrmEngine.h"

namespace ar_drive_assist {

using namespace osrm_detail;

NavigationEngine::NavigationEngine() = default;
NavigationEngine::~NavigationEngine() = default;

bool NavigationEngine::init(const std::string& osrmDataPath) {
    try {
        osrm_ = open(osrmDataPath);
        return true;
    } catch (const std::exception& e) {
        lastError_ = e.what();
        osrm_.reset();
        return false;
    }
}

Route NavigationEngine::routeTo(double destLat, double destLon, const VehiclePose& pose,
                                const LocalFrame& frame) {
    const GeoPoint from = frame.toGeodetic(pose.x, pose.y);
    if (pose.speed_kph > 7.2f) {  // > 2 m/s: depart in the direction of travel
        const double h = pose.heading_deg;
        return route(from, {destLat, destLon}, &h);
    }
    return route(from, {destLat, destLon});
}

Route NavigationEngine::route(const GeoPoint& from, const GeoPoint& to, const double* headingDeg) {
    Route r;
    if (!osrm_) {
        lastError_ = "NavigationEngine not initialised";
        return r;
    }
    osrm::RouteParameters params;
    params.coordinates = {coord(from), coord(to)};
    params.steps = true;
    params.annotations = true;
    params.annotations_type = osrm::RouteParameters::AnnotationsType::Nodes;
    params.geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
    params.overview = osrm::RouteParameters::OverviewType::Full;
    if (headingDeg) {
        // OSRM bearings are compass degrees (clockwise from north), 0-360.
        const double compass = std::fmod(std::fmod(90.0 - *headingDeg, 360.0) + 360.0, 360.0);
        params.bearings = {
            osrm::engine::Bearing{static_cast<short>(std::lround(compass) % 360), 45},
            std::nullopt};
    }

    osrm::engine::api::ResultT result = osrm::json::Object();
    const auto status = osrm_->Route(params, result);
    const auto& json = std::get<osrm::json::Object>(result);
    try {
        if (status != osrm::Status::Ok || str(at(json, "code")) != "Ok") {
            lastError_ = has(json, "message") ? str(at(json, "message")) : "OSRM route failed";
            return r;
        }
        const auto& route0 = obj(arr(at(json, "routes")).values.at(0));
        r.distanceM = num(at(route0, "distance"));
        r.durationS = num(at(route0, "duration"));
        for (const auto& c : arr(at(obj(at(route0, "geometry")), "coordinates")).values) {
            r.geometry.push_back(lonLat(c));
        }
        r.computeCumulative();

        double stepStart = 0;
        for (const auto& legV : arr(at(route0, "legs")).values) {
            const auto& leg = obj(legV);
            if (has(leg, "annotation")) {
                for (const auto& n : arr(at(obj(at(leg, "annotation")), "nodes")).values) {
                    r.osmNodes.push_back(static_cast<std::uint64_t>(num(n)));
                }
            }
            for (const auto& stepV : arr(at(leg, "steps")).values) {
                const auto& step = obj(stepV);
                const auto& man = obj(at(step, "maneuver"));
                RouteStep s;
                s.maneuver = str(at(man, "type"));
                if (has(man, "modifier")) s.modifier = str(at(man, "modifier"));
                if (has(step, "name")) s.roadName = str(at(step, "name"));
                s.location = lonLat(at(man, "location"));
                s.distanceM = num(at(step, "distance"));
                // OSRM's step distances are its own; the maneuver's position along OUR geometry
                // is found by projecting it, so steps and MapMatcher progress share one scale.
                s.alongM = projectOntoRoute(r, s.location).alongM;
                if (s.alongM < stepStart) s.alongM = stepStart;
                stepStart = s.alongM;
                if (has(step, "intersections")) {
                    const auto& ints = arr(at(step, "intersections")).values;
                    if (!ints.empty() && has(obj(ints[0]), "lanes")) {
                        s.laneCount =
                            static_cast<int>(arr(at(obj(ints[0]), "lanes")).values.size());
                    }
                }
                r.steps.push_back(s);
            }
        }
        r.valid = r.geometry.size() >= 2;
    } catch (const std::exception& e) {
        lastError_ = e.what();
        r = Route{};
    }
    return r;
}

}  // namespace ar_drive_assist
