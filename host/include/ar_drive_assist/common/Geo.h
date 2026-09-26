#pragma once
// Geo — latitude/longitude helpers and the local east/north metric frame. Header-only.
//
// Two coordinate systems meet at navigation:
//   - SensorFusion (Part 8.2) and everything downstream of it work in LOCAL METRES: east/north
//     around the first GPS fix. Kalman filters and tracking need a flat metric plane.
//   - OSRM (Part 11.2/11.3) and OpenStreetMap work in LATITUDE/LONGITUDE.
// LocalFrame converts between them. Both directions must be the EXACT inverse of each other, and
// there is exactly one implementation, used by SensorFusion and by MapMatcher: two different
// approximations would disagree by metres, which is a lane.
//
// Accuracy. The frame is a local tangent plane on the WGS-84 ellipsoid around its origin:
//     north = M(lat0) * dlat,   east = N(lat0) * cos(lat0) * dlon
// with M the meridional and N the prime-vertical radius of curvature at the origin's latitude.
// Using one sphere radius for both directions instead is off by up to 0.67% north-south at
// Nairobi's latitude (M = 6335 km there, not 6378 km): 7 m per km, and GPS-derived motion would
// disagree with OBD speed. Over the Nairobi map area (100 km radius), the remaining error far
// from the origin is a slowly varying scale of order 0.05%. It does not matter here: conversions
// are always round-tripped through the same frame (so they cancel), and distances near the car
// are off by 5 mm at 10 m. Long distances between two lat/lon points use the haversine formula
// (distanceM, +-0.5%), which is only used for smoke tests and summaries.

#include <algorithm>
#include <cmath>

namespace ar_drive_assist {

struct GeoPoint {
    double lat = 0;  // degrees, north positive
    double lon = 0;  // degrees, east positive
};

namespace geo {

constexpr double kEarthRadiusM = 6371000.0;    // mean radius, for great-circle distances
constexpr double kWgs84A = 6378137.0;          // WGS-84 equatorial radius
constexpr double kWgs84E2 = 6.69437999014e-3;  // WGS-84 first eccentricity squared
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

// Great-circle distance in metres.
inline double distanceM(const GeoPoint& a, const GeoPoint& b) {
    const double dLat = (b.lat - a.lat) * kDeg, dLon = (b.lon - a.lon) * kDeg;
    const double h =
        std::sin(dLat / 2) * std::sin(dLat / 2) +
        std::cos(a.lat * kDeg) * std::cos(b.lat * kDeg) * std::sin(dLon / 2) * std::sin(dLon / 2);
    return 2 * kEarthRadiusM * std::asin(std::sqrt(std::min(1.0, h)));
}

// Heading from a to b in degrees COUNTER-CLOCKWISE FROM EAST, in (-180, 180]: the same convention
// as VehiclePose.heading_deg (vehicle_pose.fbs). Compass bearing = 90 - this.
inline double headingDeg(const GeoPoint& a, const GeoPoint& b) {
    const double lat1 = a.lat * kDeg, lat2 = b.lat * kDeg, dLon = (b.lon - a.lon) * kDeg;
    const double north =
        std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
    const double east = std::sin(dLon) * std::cos(lat2);
    return std::atan2(north, east) / kDeg;
}

}  // namespace geo

class LocalFrame {
public:
    LocalFrame() = default;
    explicit LocalFrame(const GeoPoint& origin) : origin_(origin) {
        const double s = std::sin(origin.lat * geo::kDeg);
        const double w = 1.0 - geo::kWgs84E2 * s * s;
        const double n = geo::kWgs84A / std::sqrt(w);  // prime vertical
        northPerDeg_ = geo::kWgs84A * (1 - geo::kWgs84E2) / (w * std::sqrt(w)) * geo::kDeg;  // M
        eastPerDeg_ = n * std::cos(origin.lat * geo::kDeg) * geo::kDeg;
    }

    const GeoPoint& origin() const { return origin_; }

    // Returns {east, north} metres.
    void toLocal(const GeoPoint& p, double& east, double& north) const {
        east = eastPerDeg_ * (p.lon - origin_.lon);
        north = northPerDeg_ * (p.lat - origin_.lat);
    }
    GeoPoint toGeodetic(double east, double north) const {
        return {origin_.lat + north / northPerDeg_, origin_.lon + east / eastPerDeg_};
    }

    // Metres between two nearby points (a road segment), in this frame's tangent plane.
    double localDistanceM(const GeoPoint& a, const GeoPoint& b) const {
        double ax, ay, bx, by;
        toLocal(a, ax, ay);
        toLocal(b, bx, by);
        return std::hypot(bx - ax, by - ay);
    }

private:
    GeoPoint origin_;
    double northPerDeg_ = geo::kWgs84A * geo::kDeg, eastPerDeg_ = geo::kWgs84A * geo::kDeg;
};

}  // namespace ar_drive_assist
