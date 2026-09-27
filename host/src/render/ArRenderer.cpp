#include "ar_drive_assist/render/ArRenderer.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>

namespace ar_drive_assist {
namespace {

constexpr double kG = 9.80665;
constexpr int kLayerZone = 0, kLayerRoute = 1, kLayerBarrier = 2, kLayerRing = 3, kLayerGlow = 4,
              kLayerLabel = 5, kLayerHud = 6;

std::string metres(double d) {
    const long r = std::lround(d >= 20 ? d / 5.0 : d) * (d >= 20 ? 5 : 1);  // 5 m steps far away
    return std::to_string(r) + " m";
}

// Colour severity: 0 green, 1 amber, 2 red.
Rgba severityColor(int s) {
    return s >= 2 ? palette::kRed : s == 1 ? palette::kAmber : palette::kGreen;
}

}  // namespace

ArRenderer::ArRenderer(RendererConfig cfg, CameraModel camera, Eigen::Isometry3d cameraFromVehicle)
    : cfg_(cfg), camera_(camera), cameraFromVehicle_(cameraFromVehicle) {}

Eigen::Vector3d ArRenderer::egoShift(const Eigen::Vector3d& p, double v, double w, double dt) {
    const double dpsi = w * dt;
    Eigen::Vector2d d;
    if (std::abs(w) > 1e-6) {
        d = {v / w * std::sin(dpsi), v / w * (1 - std::cos(dpsi))};
    } else {
        d = {v * dt, 0};
    }
    const Eigen::Vector2d rel = p.head<2>() - d;
    const double c = std::cos(-dpsi), s = std::sin(-dpsi);
    return {c * rel.x() - s * rel.y(), s * rel.x() + c * rel.y(), p.z()};
}

OverlayScene ArRenderer::build(const RenderInputs& in) {
    signs_.update(in.frameMs, in.signs, in.egoSpeedMps, in.yawRateRadPerS, in.mapMaxSpeedKph,
                  in.roadChanged);
    OverlayScene sc;
    sc.timestampMs = in.frameMs;
    const double W = camera_.width, H = camera_.height;
    const bool haveSpeed = in.egoSpeedMps.has_value();
    const double v = in.egoSpeedMps.value_or(0.0);
    auto project = [&](const Eigen::Vector3d& p, Eigen::Vector2d& px) {
        return camera_.project(cameraFromVehicle_ * p, px);
    };
    auto label = [&](const std::string& text, const Eigen::Vector2d& at, const Rgba& c, float size,
                     int layer, int source) {
        OverlayItem it;
        it.kind = OverlayKind::IMAGE_LABEL;
        it.text = text;
        it.imagePos = at;
        it.sizePx = size;
        it.style.color = c;
        it.style.layer = layer;
        it.sourceId = source;
        sc.items.push_back(it);
    };
    auto barrier = [&](double x, double y, const Rgba& c, float pulse, int source) {
        OverlayItem it;
        it.kind = OverlayKind::ROAD_BARRIER;
        it.points = {{x, y + cfg_.laneHalfWidthM, 0}, {x, y - cfg_.laneHalfWidthM, 0}};
        it.heightM = 1.1f;
        it.style = {c, 0.85f, 0, 0, pulse, kLayerBarrier};
        it.occludedByHazards = true;
        it.sourceId = source;
        sc.items.push_back(it);
    };
    auto decel = [&](double d, double vEnd = 0) {  // a = (v^2 - v_end^2) / 2d, in g
        return d > 0.5 && v > vEnd ? (v * v - vEnd * vEnd) / (2 * d) / kG : (v > vEnd ? 99.0 : 0.0);
    };
    auto severity = [&](double aG) { return aG > cfg_.comfortG ? 2 : aG > cfg_.easeOffG ? 1 : 0; };

    // --- Stop targets and speed approach, for the route grading -----------------------------
    std::optional<double> stopGap;  // bumper gap to the nearest stop target
    for (const TrackedSign& s : signs_.signs()) {
        if (s.classId != sign_class::kStop || !s.confirmed || s.cleared || !s.position) continue;
        const double gap = s.position->x() - cfg_.frontBumperM;
        if (gap > 0 && (!stopGap || gap < *stopGap)) stopGap = gap;
    }
    for (const HazardView& h : in.hazards) {
        if ((h.threat.flags & FORWARD_COLLISION) && h.threat.inEgoPath && h.gapM > 0 &&
            (!stopGap || h.gapM < *stopGap))
            stopGap = h.gapM;
    }
    std::optional<std::pair<double, double>> slowTo;  // (gap, limit m/s) of an upcoming lower limit
    for (const TrackedSign& s : signs_.signs()) {
        if (!s.confirmed || !s.agreedSpeedKph || !s.position) continue;
        const double gap = s.position->x() - cfg_.frontBumperM, vl = *s.agreedSpeedKph / 3.6;
        if (gap > 0 && vl < v && (!slowTo || gap < slowTo->first)) slowTo = std::make_pair(gap, vl);
    }
    const SpeedLimit lim = signs_.limit();
    int speedSeverity = 0;
    if (haveSpeed && lim.kph) {
        const double kph = v * 3.6;
        speedSeverity = kph <= *lim.kph ? 0 : kph <= *lim.kph * 1.1 ? 1 : 2;
    }

    // --- Route band -------------------------------------------------------------------------
    if (in.route && !in.route->polyline.empty()) {
        const double dt =
            (static_cast<double>(in.frameMs) - static_cast<double>(in.route->poseTimestampMs)) /
            1000.0;
        OverlayItem band;
        band.kind = OverlayKind::ROAD_BAND;
        band.widthM = static_cast<float>(cfg_.routeWidthM);
        band.style = {palette::kGreen, 0.9f, 8, 0, 0, kLayerRoute};
        band.occludedByHazards = true;
        const double aStop = stopGap ? decel(*stopGap) : 0.0;
        if (stopGap && aStop > cfg_.comfortG && aStop <= cfg_.hardG) band.style.pulseHz = 1.5f;
        for (const ProjectedPoint& p : in.route->polyline) {
            const Eigen::Vector3d q = egoShift(p.vehicle, v, in.yawRateRadPerS, dt);
            band.points.push_back(q);
            float alpha = p.onMeasuredSurface ? 1.0f : cfg_.farOpacity;
            int sev = speedSeverity;
            const double gapHere = q.x() - cfg_.frontBumperM;
            if (stopGap) {
                if (gapHere <= *stopGap)
                    sev = std::max(sev, severity(aStop));
                else
                    alpha *= 0.25f;  // beyond the barrier the band fades out
            }
            if (slowTo && gapHere <= slowTo->first)
                sev = std::max(sev, severity(decel(slowTo->first, slowTo->second)));
            band.pointOpacity.push_back(alpha);
            band.pointColor.push_back(severityColor(sev));
        }
        sc.items.push_back(band);
        if (!in.route->nextTurnInstruction.empty()) {
            label(in.route->nextTurnInstruction + "   " + metres(in.route->nextTurnDistanceM),
                  {W / 2, H - 90}, palette::kWhite, 40, kLayerHud, -1);
        }
    }

    // --- Hazards (never decluttered) ----------------------------------------------------------
    for (const HazardView& h : in.hazards) {
        const ThreatAssessment& t = h.threat;
        const std::uint32_t hazardFlags =
            kRecklessFlags | VULNERABLE_IN_PATH | FORWARD_COLLISION | CROSSING | SUDDEN_BRAKING;
        if (t.level < ThreatLevel::MEDIUM && !(t.flags & hazardFlags)) continue;  // rule 8
        const double r = std::max(t.risk, 0.4);
        const Rgba col = palette::forRisk(r);
        const Eigen::Vector2d centre(h.box[0] + h.box[2] / 2, h.box[1] + h.box[3] / 2);
        const bool onScreen =
            h.box[0] + h.box[2] > 0 && h.box[0] < W && h.box[1] + h.box[3] > 0 && h.box[1] < H;
        if (onScreen) {
            OverlayItem glow;
            glow.kind = h.mask ? OverlayKind::MASK_GLOW : OverlayKind::ELLIPSE_GLOW;
            if (h.mask) glow.mask = *h.mask;
            glow.box = h.box;
            glow.style = {col, 1.0f,      10, static_cast<float>(std::min(3.0, 0.5 + 1.5 * r)),
                          0,   kLayerGlow};
            glow.sourceId = h.trackId;
            sc.items.push_back(glow);
            const double depth =
                h.depthM > 0
                    ? h.depthM
                    : (h.ground ? h.ground->x() - cameraFromVehicle_.inverse().translation().x()
                                : 0);
            if (depth > 0)
                sc.occluders.push_back({h.mask ? *h.mask : ellipseMaskFor(h.box), depth});
        } else {
            OverlayItem chev;
            chev.kind = OverlayKind::IMAGE_CHEVRON;
            chev.imagePos = centre;
            chev.sizePx = 64;
            chev.style = {col, 1.0f, 0, 0, 1.5f, kLayerHud};
            chev.sourceId = h.trackId;
            sc.items.push_back(chev);
        }
        if (!h.ground) continue;  // rule 6: no road element without a measured position
        const Eigen::Vector3d g = *h.ground;
        if ((t.flags & FORWARD_COLLISION) && t.inEgoPath) {
            const double a = decel(h.gapM);
            barrier(g.x(), g.y(), col, a > cfg_.comfortG ? 1.5f : 0.0f, h.trackId);
            Eigen::Vector2d px;
            if (h.gapM > 0 && project({g.x(), g.y(), 1.5}, px))
                label(metres(h.gapM), px, col, 40, kLayerLabel, h.trackId);
        }
        if (t.flags & VULNERABLE_IN_PATH) {
            OverlayItem ring;
            ring.kind = OverlayKind::ROAD_RING;
            ring.points = {g};
            ring.widthM = 0.6f;
            ring.style = {col, 0.95f, 0, 0, 1.0f, kLayerRing};
            ring.occludedByHazards = true;
            ring.sourceId = h.trackId;
            sc.items.push_back(ring);
        }
        if (t.flags & TAILGATING) {
            const double x0 = cfg_.frontBumperM + 0.5, x1 = g.x() - 0.3;
            if (x1 > x0) {
                OverlayItem zone;
                zone.kind = OverlayKind::ROAD_ZONE;
                zone.points = {{x0, 1.0, 0}, {x0, -1.0, 0}, {x1, -1.0, 0}, {x1, 1.0, 0}};
                const bool severe = t.timeGapS >= 0 && t.timeGapS < 0.5 * cfg_.tailgatingMinGapS;
                zone.style = {severe ? palette::kRed : palette::kAmber, 0.6f, 0, 0, 0, kLayerZone};
                zone.occludedByHazards = true;
                zone.sourceId = h.trackId;
                sc.items.push_back(zone);
            }
        }
        if ((t.flags & (SWERVING | ERRATIC_SPEED)) && in.lanes) {
            // The lane line on the neighbour's side glows red ("don't drift this way", §3.6).
            const auto& px = g.y() > 0 ? in.lanes->lane_points_left : in.lanes->lane_points_right;
            OverlayItem line;
            line.kind = OverlayKind::ROAD_BAND;
            line.widthM = 0.15f;
            line.style = {palette::kRed, 0.9f, 8, 0, 1.0f, kLayerRoute};
            line.occludedByHazards = true;
            line.sourceId = h.trackId;
            for (std::size_t i = 0; i + 1 < px.size(); i += 2) {
                Eigen::Vector3d q;
                if (RoadSurfaceProjector::pixelToGround({px[i], px[i + 1]}, camera_,
                                                        cameraFromVehicle_, GroundPlaneModel{}, q))
                    line.points.push_back(q);
            }
            std::sort(
                line.points.begin(), line.points.end(),
                [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.x() < b.x(); });
            if (line.points.size() >= 2) sc.items.push_back(line);
        }
    }

    // --- Signs: at most maxSignItems, by priority ------------------------------------------------
    struct Candidate {
        int priority;
        std::function<void(int)> emit;  // argument: banner slot
    };
    std::vector<Candidate> cands;
    int bannerSlot = 0;
    auto banner = [&](const std::string& text, const Rgba& c, int source) {
        return [&, text, c, source](int) {
            label(text, {W - 300, 90 + 80.0 * bannerSlot++}, c, 36, kLayerLabel, source);
        };
    };
    for (const TrackedSign& s : signs_.signs()) {
        if (!s.confirmed || s.cleared || s.lastSeenMs + 1500 < in.frameMs) continue;
        const std::optional<double> gap =
            s.position ? std::optional<double>(s.position->x() - cfg_.frontBumperM) : std::nullopt;
        const std::string dist = gap && *gap > 0 ? "  " + metres(*gap) : "";
        const bool placeable = gap && *gap > 0;
        switch (s.classId) {
            case sign_class::kStop:
                if (placeable) {
                    const double a = decel(*gap);
                    cands.push_back({0, [&, s, gap, a](int) {
                                         barrier(s.position->x(), 0, palette::kRed,
                                                 a > cfg_.comfortG && a <= cfg_.hardG ? 1.5f : 0.0f,
                                                 s.id);
                                         if (a > cfg_.hardG) sc.items.back().style.opacity = 1.0f;
                                         Eigen::Vector2d px;
                                         if (project({s.position->x(), 0, 1.5}, px))
                                             label("STOP  " + metres(*gap), px, palette::kRed, 40,
                                                   kLayerLabel, s.id);
                                     }});
                } else {
                    cands.push_back({0, banner("STOP", palette::kRed, s.id)});
                }
                break;
            case sign_class::kNoEntry:
                if (placeable && std::abs(s.position->y()) < cfg_.laneHalfWidthM + 1.0) {
                    cands.push_back(
                        {0, [&, s](int) {
                             barrier(s.position->x(), s.position->y(), palette::kRed, 0, s.id);
                             Eigen::Vector2d px;
                             if (project({s.position->x(), s.position->y(), 1.5}, px))
                                 label("NO ENTRY", px, palette::kRed, 40, kLayerLabel, s.id);
                         }});
                } else {
                    cands.push_back({0, banner("NO ENTRY" + dist, palette::kRed, s.id)});
                }
                break;
            case sign_class::kGiveWay:
                if (placeable) {
                    cands.push_back({1, [&, s](int) {
                                         OverlayItem m;
                                         m.kind = OverlayKind::ROAD_MARKER;
                                         m.points = {{s.position->x(), 0, 0}};
                                         m.widthM = static_cast<float>(2 * cfg_.laneHalfWidthM);
                                         m.style = {palette::kAmber, 0.9f, 0, 0, 0, kLayerBarrier};
                                         m.occludedByHazards = true;
                                         m.sourceId = s.id;
                                         sc.items.push_back(m);
                                     }});
                } else {
                    cands.push_back({1, banner("GIVE WAY", palette::kAmber, s.id)});
                }
                break;
            case sign_class::kNoLeft:
                cands.push_back({2, banner("NO LEFT TURN" + dist, palette::kRed, s.id)});
                break;
            case sign_class::kNoRight:
                cands.push_back({2, banner("NO RIGHT TURN" + dist, palette::kRed, s.id)});
                break;
            case sign_class::kNoUTurn:
                cands.push_back({2, banner("NO U-TURN" + dist, palette::kRed, s.id)});
                break;
            case sign_class::kNoOvertaking:
                cands.push_back({2, banner("NO OVERTAKING", palette::kRed, s.id)});
                break;
            case sign_class::kHump:
                if (placeable) {
                    cands.push_back({3, [&, s, gap](int) {
                                         OverlayItem m;
                                         m.kind = OverlayKind::ROAD_MARKER;
                                         m.points = {{s.position->x(), 0, 0}};
                                         m.widthM = static_cast<float>(2 * cfg_.laneHalfWidthM);
                                         m.style = {palette::kAmber, 0.9f, 0, 0, 0, kLayerBarrier};
                                         m.occludedByHazards = true;
                                         m.sourceId = s.id;
                                         sc.items.push_back(m);
                                         Eigen::Vector2d px;
                                         if (project({s.position->x(), 0, 0.8}, px))
                                             label("HUMP  " + metres(*gap), px, palette::kAmber, 36,
                                                   kLayerLabel, s.id);
                                     }});
                } else {
                    cands.push_back({3, banner("HUMP AHEAD", palette::kAmber, s.id)});
                }
                break;
            case sign_class::kPedestrianCrossing:
                cands.push_back({3, banner("PEDESTRIAN CROSSING" + dist, palette::kWhite, s.id)});
                break;
            case sign_class::kSchool:
                cands.push_back({3, banner("SCHOOL ZONE (advice)" + dist, palette::kSchool, s.id)});
                break;
            case sign_class::kRoundabout:
                cands.push_back({3, banner("ROUNDABOUT" + dist, palette::kWhite, s.id)});
                break;
            case sign_class::kSignals:
                cands.push_back({3, banner("TRAFFIC SIGNALS" + dist, palette::kWhite, s.id)});
                break;
            case sign_class::kKeepLeftRight:
                cands.push_back({3, banner("KEEP LEFT / RIGHT" + dist, palette::kWhite, s.id)});
                break;
            case sign_class::kNoParking:
                if (haveSpeed && v < cfg_.noParkingMaxSpeedMps)
                    cands.push_back({4, banner("NO PARKING", palette::kWhite, s.id)});
                break;
            case sign_class::kOtherWarning:
                cands.push_back({4, banner("WARNING" + dist, palette::kAmber, s.id)});
                break;
            case sign_class::kOtherRegulatory:
                cands.push_back({4, banner("REGULATORY SIGN" + dist, palette::kWhite, s.id)});
                break;
            default:
                break;  // speed limits and end of restriction: the HUD
        }
    }
    std::stable_sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        return a.priority < b.priority;
    });
    for (int i = 0; i < std::min<int>(cfg_.maxSignItems, static_cast<int>(cands.size())); ++i)
        cands[i].emit(i);

    // --- HUD ------------------------------------------------------------------------------------
    if (lim.kph) {
        OverlayItem badge;
        badge.kind = OverlayKind::IMAGE_BADGE;
        badge.text = std::to_string(*lim.kph);
        badge.imagePos = {140, 140};
        badge.sizePx = 120;
        badge.style.layer = kLayerHud;
        sc.items.push_back(badge);
    }
    if (haveSpeed) {
        std::ostringstream t;
        t << std::lround(v * 3.6) << " km/h";
        label(t.str(), {lim.kph ? 330.0 : 140.0, 140},
              lim.kph ? severityColor(speedSeverity) : palette::kWhite, 44, kLayerHud, -1);
    }
    return sc;
}

}  // namespace ar_drive_assist
