#include "ar_drive_assist/nav/Route.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ar_drive_assist {

void Route::computeCumulative() {
    cumulativeM.assign(geometry.size(), 0.0);
    for (std::size_t i = 1; i < geometry.size(); ++i) {
        // Tangent-plane length at the segment's start: the same metric projectOntoRoute uses, so
        // progress and offsets agree exactly.
        cumulativeM[i] = cumulativeM[i - 1] +
                         LocalFrame(geometry[i - 1]).localDistanceM(geometry[i - 1], geometry[i]);
    }
}

GeoPoint Route::pointAt(double alongM) const {
    if (geometry.empty()) return {};
    if (alongM <= 0 || geometry.size() == 1) return geometry.front();
    if (alongM >= lengthM()) return geometry.back();
    const auto it = std::upper_bound(cumulativeM.begin(), cumulativeM.end(), alongM);
    const std::size_t i = static_cast<std::size_t>(it - cumulativeM.begin()) - 1;
    const double seg = cumulativeM[i + 1] - cumulativeM[i];
    const double f = seg > 0 ? (alongM - cumulativeM[i]) / seg : 0.0;
    // Linear in lat/lon is exact to millimetres over a road segment's length.
    return {geometry[i].lat + f * (geometry[i + 1].lat - geometry[i].lat),
            geometry[i].lon + f * (geometry[i + 1].lon - geometry[i].lon)};
}

RouteProjection projectOntoRoute(const Route& route, const GeoPoint& p, std::size_t firstSegment,
                                 std::size_t lastSegment) {
    RouteProjection best;
    if (route.geometry.size() < 2 || route.cumulativeM.size() != route.geometry.size()) return best;
    const std::size_t nSeg = route.geometry.size() - 1;
    lastSegment = std::min(lastSegment, nSeg - 1);
    firstSegment = std::min(firstSegment, lastSegment);
    double bestD2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = firstSegment; i <= lastSegment; ++i) {
        // Local metric frame at the segment's start.
        const LocalFrame f(route.geometry[i]);
        double bx, by, px, py;
        f.toLocal(route.geometry[i + 1], bx, by);
        f.toLocal(p, px, py);
        const double len2 = bx * bx + by * by;
        const double t = len2 > 0 ? std::clamp((px * bx + py * by) / len2, 0.0, 1.0) : 0.0;
        const double dx = px - t * bx, dy = py - t * by;
        const double d2 = dx * dx + dy * dy;
        if (d2 < bestD2) {
            bestD2 = d2;
            best.valid = true;
            best.segment = i;
            best.alongM =
                route.cumulativeM[i] + t * (route.cumulativeM[i + 1] - route.cumulativeM[i]);
            // Distance to the nearest point of the SEGMENT (not its infinite line, which would
            // understate it past a corner), signed by the cross product: LEFT positive.
            best.offsetM = std::copysign(std::sqrt(d2), bx * py - by * px);
            best.headingDeg = std::atan2(by, bx) / geo::kDeg;
        }
    }
    return best;
}

}  // namespace ar_drive_assist
