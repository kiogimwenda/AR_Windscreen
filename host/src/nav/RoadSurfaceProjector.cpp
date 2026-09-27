#include "ar_drive_assist/nav/RoadSurfaceProjector.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace ar_drive_assist {
namespace {

constexpr double kDeg = geo::kDeg;

double wrapPi(double a) {
    a = std::fmod(a + geo::kPi, 2 * geo::kPi);
    if (a < 0) a += 2 * geo::kPi;
    return a - geo::kPi;
}

// Least-squares line y = c0 + c1 x. False if fewer than minPoints or the x spread is too short to
// define a direction (under 5 m).
bool fitLine(const std::vector<Eigen::Vector2d>& pts, int minPoints, double& c0, double& c1) {
    if (static_cast<int>(pts.size()) < minPoints) return false;
    double sx = 0, sy = 0, sxx = 0, sxy = 0, xmin = 1e9, xmax = -1e9;
    for (const auto& p : pts) {
        sx += p.x();
        sy += p.y();
        sxx += p.x() * p.x();
        sxy += p.x() * p.y();
        xmin = std::min(xmin, p.x());
        xmax = std::max(xmax, p.x());
    }
    const double n = static_cast<double>(pts.size()), den = n * sxx - sx * sx;
    if (xmax - xmin < 5.0 || std::abs(den) < 1e-9) return false;
    c1 = (n * sxy - sx * sy) / den;
    c0 = (sy - c1 * sx) / n;
    return true;
}

// Steps that are not decisions for the driver (a road changing its name, OSRM notifications).
bool isDecision(const RouteStep& s) {
    return s.maneuver != "depart" && s.maneuver != "new name" && s.maneuver != "notification";
}

std::string describe(const RouteStep& s) {
    if (s.maneuver == "arrive") return "arrive at destination";
    std::string verb;
    if (s.maneuver == "roundabout" || s.maneuver == "rotary") {
        verb = "take the roundabout";
    } else if (s.maneuver == "merge") {
        verb = "merge" + (s.modifier.empty() ? "" : " " + s.modifier);
    } else if (s.maneuver == "fork" || s.maneuver == "off ramp" || s.maneuver == "on ramp") {
        verb = "keep " + (s.modifier.empty() ? std::string("ahead") : s.modifier);
    } else if (s.modifier == "straight") {
        verb = "continue straight";
    } else if (s.modifier == "uturn") {
        verb = "make a U-turn";
    } else {
        verb = "turn" + (s.modifier.empty() ? "" : " " + s.modifier);
    }
    return s.roadName.empty() ? verb : verb + " onto " + s.roadName;
}

}  // namespace

RoadProjectionConfig loadRoadProjectionConfig(const std::string& path) {
    YAML::Node n;
    try {
        n = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("config: cannot load " + path + ": " + e.what());
    }
    auto req = [&](const std::string& key) -> double {
        if (!n[key]) throw std::runtime_error("config: " + path + ": '" + key + "' missing");
        return n[key].as<double>();
    };
    auto opt = [&](const std::string& key, double def) {
        return n[key] ? n[key].as<double>() : def;
    };
    RoadProjectionConfig c;
    c.horizonM = req("horizon_m");
    c.waypointSpacingM = req("waypoint_spacing_m");
    c.lateralCorrectionMaxM = req("lateral_correction_max_m");
    c.headingCorrectionMaxDeg = opt("heading_correction_max_deg", c.headingCorrectionMaxDeg);
    c.laneFitMinM = opt("lane_fit_min_m", c.laneFitMinM);
    c.laneFitMaxM = opt("lane_fit_max_m", c.laneFitMaxM);
    c.correctionDecayS = opt("correction_decay_s", c.correctionDecayS);
    c.taperAfterTurnM = opt("taper_after_turn_m", c.taperAfterTurnM);
    c.patchRadiusM = opt("patch_radius_m", c.patchRadiusM);
    if (c.horizonM <= 0 || c.waypointSpacingM <= 0 || c.lateralCorrectionMaxM < 0 ||
        c.headingCorrectionMaxDeg < 0) {
        throw std::runtime_error("config: " + path +
                                 ": non-positive horizon/spacing or negative cap");
    }
    return c;
}

RoadProjectedRoute ProjectedRoute::toMessage() const {
    RoadProjectedRoute m;
    m.timestamp_ms = poseTimestampMs;
    m.pose_timestamp_ms = poseTimestampMs;
    m.next_turn_instruction = nextTurnInstruction;
    m.next_turn_distance_m = static_cast<float>(nextTurnDistanceM);
    m.lane_corrected = laneCorrected;
    for (const ProjectedPoint& p : polyline) {
        auto o = std::make_unique<schema::RoadOverlayPointT>();
        o->image_x = static_cast<float>(p.image.x());
        o->image_y = static_cast<float>(p.image.y());
        o->world_distance_m = static_cast<float>(p.distanceAheadM);
        o->on_measured_surface = p.onMeasuredSurface;
        o->x_m = static_cast<float>(p.vehicle.x());
        o->y_m = static_cast<float>(p.vehicle.y());
        o->z_m = static_cast<float>(p.vehicle.z());
        o->in_image = p.inImage;
        m.polyline.push_back(std::move(o));
    }
    return m;
}

RoadSurfaceProjector::RoadSurfaceProjector(RoadProjectionConfig cfg) : cfg_(cfg) {}

bool RoadSurfaceProjector::init(const RoadProjectionConfig& cfg) {
    cfg_ = cfg;
    reset();
    return true;
}

void RoadSurfaceProjector::reset() {
    lateralCorr_ = headingCorrRad_ = 0;
    lastMs_ = 0;
    stepIndex_ = -1;
}

bool RoadSurfaceProjector::pixelToGround(const Eigen::Vector2d& px, const CameraModel& camera,
                                         const Eigen::Isometry3d& cameraFromVehicle,
                                         const GroundPlaneModel& ground, Eigen::Vector3d& out) {
    const Eigen::Isometry3d vehicleFromCamera = cameraFromVehicle.inverse();
    const Eigen::Vector3d origin = vehicleFromCamera.translation();
    const Eigen::Vector3d dir = vehicleFromCamera.linear() * camera.unproject(px);
    const Eigen::Vector3d n = ground.planeCoefficients.head<3>().cast<double>();
    const double d = ground.planeCoefficients.w();
    const double denom = n.dot(dir);
    if (std::abs(denom) < 1e-9) return false;
    const double t = -(n.dot(origin) + d) / denom;
    if (t <= 0) return false;  // the ray points above the horizon
    out = origin + t * dir;
    return out.x() > 0;
}

ProjectedRoute RoadSurfaceProjector::project(const Route& route,
                                             const MapMatcher::MatchedPosition& matched,
                                             const GroundPlaneModel& ground,
                                             const DetectionFrame& lanes, const VehiclePose& pose,
                                             const LocalFrame& frame, const CameraModel& camera,
                                             const Eigen::Isometry3d& cameraFromVehicle) {
    ProjectedRoute out;
    out.poseTimestampMs = pose.timestamp_ms;
    if (!route.valid || !matched.valid || route.geometry.size() < 2) return out;

    // 1. Route frame at the car's progress s0: origin P0, tangent psiR (CCW from east).
    const double len = route.lengthM();
    const double s0 = std::clamp(matched.distanceAlongRouteM, 0.0, len);
    const GeoPoint p0 = route.pointAt(s0);
    const LocalFrame rf(p0);
    double te, tn;
    if (s0 + 1.0 <= len) {
        rf.toLocal(route.pointAt(s0 + 1.0), te, tn);
    } else {
        rf.toLocal(route.pointAt(s0 - 1.0), te, tn);
        te = -te;
        tn = -tn;
    }
    const double psiR = std::atan2(tn, te), c = std::cos(psiR), s = std::sin(psiR);

    // 2. The car relative to the road, from the fused pose: lateral offset d (left of the centre
    // line positive) and relative heading delta.
    double ve, vn;
    rf.toLocal(frame.toGeodetic(pose.x, pose.y), ve, vn);
    const double d = -s * ve + c * vn;
    const double delta = wrapPi(pose.heading_deg * kDeg - psiR);

    // Current step (the road the car is on) and the next decision point.
    int stepNow = -1;
    const RouteStep* next = nullptr;
    for (std::size_t i = 0; i < route.steps.size(); ++i) {
        if (route.steps[i].alongM <= s0 + 1e-6) stepNow = static_cast<int>(i);
        if (!next && route.steps[i].alongM > s0 + 1.0 && isDecision(route.steps[i])) {
            next = &route.steps[i];
        }
    }
    const double sTurn = next ? next->alongM : len + cfg_.taperAfterTurnM + 1;
    if (next) {
        out.nextTurnInstruction = describe(*next);
        out.nextTurnDistanceM = next->alongM - s0;
    }

    // Waypoints, uncorrected (map + fused pose): in the vehicle frame, horizontal only for now.
    struct W {
        double s;
        Eigen::Vector2d xy;
    };
    std::vector<W> ws;
    const double cd = std::cos(delta), sd = std::sin(delta);
    for (double k = 0;; k += cfg_.waypointSpacingM) {
        const double sk = std::min(s0 + k, len);
        double e, n;
        rf.toLocal(route.pointAt(sk), e, n);
        const double a = c * e + s * n, b = -s * e + c * n - d;   // route frame, relative to car
        ws.push_back({sk, {cd * a + sd * b, -sd * a + cd * b}});  // rotate by -delta
        if (sk >= len || k >= cfg_.horizonM) break;
    }

    // 3. Lane correction.
    if (stepIndex_ >= 0 && stepNow != stepIndex_) {
        lateralCorr_ = headingCorrRad_ = 0;  // a new road: its lane offset is not known yet
    }
    stepIndex_ = stepNow;
    const double dt =
        lastMs_ && pose.timestamp_ms > lastMs_ ? (pose.timestamp_ms - lastMs_) / 1000.0 : 0.0;
    lastMs_ = pose.timestamp_ms;

    auto toGround = [&](const std::vector<float>& flat) {
        std::vector<Eigen::Vector2d> pts;
        for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
            Eigen::Vector3d g;
            if (pixelToGround({flat[i], flat[i + 1]}, camera, cameraFromVehicle, ground, g) &&
                g.x() >= cfg_.laneFitMinM && g.x() <= cfg_.laneFitMaxM) {
                pts.push_back(g.head<2>());
            }
        }
        return pts;
    };
    double l0, l1, r0, r1;
    bool lanesOk = fitLine(toGround(lanes.lane_points_left), cfg_.laneMinPoints, l0, l1) &&
                   fitLine(toGround(lanes.lane_points_right), cfg_.laneMinPoints, r0, r1);
    if (lanesOk) {
        const double width = (l0 + 10 * l1) - (r0 + 10 * r1);  // at 10 m ahead
        lanesOk = width >= cfg_.laneWidthMinM && width <= cfg_.laneWidthMaxM;
    }
    std::vector<Eigen::Vector2d> nearRoute;  // the current road's line within the fit range
    for (const W& w : ws) {
        if (w.s < sTurn && w.xy.x() >= cfg_.laneFitMinM && w.xy.x() <= cfg_.laneFitMaxM)
            nearRoute.push_back(w.xy);
    }
    double q0, q1;
    if (lanesOk && fitLine(nearRoute, 2, q0, q1)) {
        const double m0 = (l0 + r0) / 2, m1 = (l1 + r1) / 2;  // lane centre line
        const double headingTarget = std::atan(m1) - std::atan(q1);
        // Lateral target: where the lane centre is, relative to the route line once rotated.
        const double ch = std::cos(headingTarget), sh = std::sin(headingTarget);
        std::vector<Eigen::Vector2d> rotated;
        for (const auto& p : nearRoute)
            rotated.push_back({ch * p.x() - sh * p.y(), sh * p.x() + ch * p.y()});
        double q0r = q0, q1r = q1;
        fitLine(rotated, 2, q0r, q1r);
        const double lateralTarget = m0 - q0r;
        const double hCap = cfg_.headingCorrectionMaxDeg * kDeg;
        headingCorrRad_ += std::clamp(headingTarget - headingCorrRad_, -hCap, hCap);
        lateralCorr_ += std::clamp(lateralTarget - lateralCorr_, -cfg_.lateralCorrectionMaxM,
                                   cfg_.lateralCorrectionMaxM);
        out.laneCorrected = true;
    } else if (dt > 0 && cfg_.correctionDecayS > 0) {
        const double f = std::exp(-dt / cfg_.correctionDecayS);
        headingCorrRad_ *= f;
        lateralCorr_ *= f;
    }
    out.lateralCorrectionM = lateralCorr_;
    out.headingCorrectionDeg = headingCorrRad_ / kDeg;

    // Ground patch lookup: 2-D grid with cells of the search radius.
    const double cell = cfg_.patchRadiusM;
    // Cell indices are negative behind or to the right of the car. Left-shifting a negative value
    // is undefined behaviour before C++20 (UBSan found it), so the halves are packed as unsigned.
    auto key = [&](double x, double y) {
        const auto ix = static_cast<std::uint32_t>(static_cast<std::int32_t>(std::floor(x / cell)));
        const auto iy = static_cast<std::uint32_t>(static_cast<std::int32_t>(std::floor(y / cell)));
        return (static_cast<std::uint64_t>(ix) << 32) | iy;
    };
    std::unordered_map<std::uint64_t, std::vector<const Eigen::Vector3f*>> grid;
    for (const auto& p : ground.nearFieldPatch) grid[key(p.x(), p.y())].push_back(&p);
    const double flatZ = ground.planeHeightAt(0, 0);

    const double ch = std::cos(headingCorrRad_), sh = std::sin(headingCorrRad_);
    for (const W& w : ws) {
        ProjectedPoint p;
        p.distanceAheadM = w.s - s0;
        // Heading correction rotates everything (a pose error); the lateral correction applies
        // along the current road and tapers out after the next decision point.
        Eigen::Vector2d xy(ch * w.xy.x() - sh * w.xy.y(), sh * w.xy.x() + ch * w.xy.y());
        const double taper =
            w.s <= sTurn
                ? 1.0
                : std::max(0.0, 1.0 - (w.s - sTurn) / std::max(1e-9, cfg_.taperAfterTurnM));
        xy.y() += lateralCorr_ * taper;

        // 4. Height.
        double z = flatZ;
        if (std::hypot(xy.x(), xy.y()) <= ground.maxValidRangeM) {
            z = ground.planeHeightAt(xy.x(), xy.y());
            double sum = 0;
            int count = 0;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    const auto it = grid.find(key(xy.x() + dx * cell, xy.y() + dy * cell));
                    if (it == grid.end()) continue;
                    for (const Eigen::Vector3f* g : it->second) {
                        if (std::hypot(g->x() - xy.x(), g->y() - xy.y()) <= cfg_.patchRadiusM) {
                            sum += g->z();
                            ++count;
                        }
                    }
                }
            }
            if (count >= cfg_.patchMinPoints) {
                z = sum / count;
                p.onMeasuredSurface = true;
            }
        }
        p.vehicle = {xy.x(), xy.y(), z};

        // 5. Into the image.
        p.inImage = camera.project(cameraFromVehicle * p.vehicle, p.image);
        out.polyline.push_back(p);
    }
    return out;
}

}  // namespace ar_drive_assist
