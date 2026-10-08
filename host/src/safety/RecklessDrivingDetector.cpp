#include "ar_drive_assist/safety/RecklessDrivingDetector.h"

#include <algorithm>
#include <cmath>

namespace ar_drive_assist {
namespace {

bool isVehicleLike(int32_t cls) {
    return cls == 0 || cls == 2 || cls < 0;  // vehicle, cyclist/boda-boda, unclassified
}
bool isVulnerable(int32_t cls) {
    return cls == 1 || cls == 2 || cls == 4;  // pedestrian, cyclist, obstacle/animal
}

// Standard deviation of y after removing a least-squares polynomial in x of the given degree
// (1 = line, 2 = parabola). Solved with Eigen's QR, which stays well conditioned when x spans
// hundreds of metres.
double detrendedStd(const std::vector<double>& x, const std::vector<double>& y, int degree) {
    const int n = static_cast<int>(x.size());
    if (n <= degree + 1) return 0.0;
    const double x0 = x.front();
    Eigen::MatrixXd A(n, degree + 1);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) {
        double p = 1;
        for (int k = 0; k <= degree; ++k, p *= (x[i] - x0)) A(i, k) = p;
        b(i) = y[i];
    }
    const Eigen::VectorXd coef = A.colPivHouseholderQr().solve(b);
    const Eigen::VectorXd r = b - A * coef;
    return std::sqrt(r.squaredNorm() / (n - degree - 1));  // unbiased: dof = n - parameters
}

}  // namespace

std::vector<EgoRelativeTrack> toEgoFrame(const std::vector<Track>& tracks, const EgoState& ego,
                                         const EgoFrameConfig& cfg) {
    const double c = std::cos(ego.pose.psi), s = std::sin(ego.pose.psi);
    const Eigen::Vector2d egoVel(ego.speedMps * c, ego.speedMps * s);
    const double w = ego.yawRateRadPerS;
    const double kappa = w / std::max(ego.speedMps, 1.0);  // path curvature (1/m)
    std::vector<EgoRelativeTrack> out;
    for (const Track& t : tracks) {
        EgoRelativeTrack e;
        e.track = &t;
        const Eigen::Vector2d r = t.filter.position() - Eigen::Vector2d(ego.pose.x, ego.pose.y);
        const Eigen::Vector2d v = t.filter.velocity() - egoVel;
        e.xM = c * r.x() + s * r.y();
        e.yM = -s * r.x() + c * r.y();
        // Rate of change of x in the car's (rotating) frame: the relative velocity along the
        // car's axis, plus w * y because the frame itself turns.
        const double xDot = c * v.x() + s * v.y() + w * e.yM;
        e.gapM = e.xM - cfg.frontBumperFromRearAxleM;
        e.closingSpeedMps = -xDot;
        e.inEgoPath =
            e.gapM > 0 && std::abs(e.yM - 0.5 * kappa * e.xM * e.xM) <= cfg.egoPathHalfWidthM;
        e.inStraightPath = e.gapM > 0 && std::abs(e.yM) <= cfg.egoPathHalfWidthM;
        e.measuredRunMs = t.measuredRunSinceMs > 0 && t.measuredRunSinceMs <= ego.timestampMs
                              ? ego.timestampMs - t.measuredRunSinceMs
                              : 0;
        e.confirmed = t.state == TrackState::CONFIRMED;
        e.lastMeasuredMs = t.lastMeasuredRangeMs;
        e.rangeMeasured = t.lastMeasuredRangeMs > 0 && t.lastMeasuredRangeMs <= ego.timestampMs &&
                          ego.timestampMs - t.lastMeasuredRangeMs <= cfg.measuredRangeMaxAgeMs;
        out.push_back(e);
    }
    return out;
}

RecklessDrivingDetector::RecklessDrivingDetector(RecklessConfig cfg) : cfg_(cfg) {}

ThreatLevel RecklessDrivingDetector::levelFor(double r) {
    if (r >= 0.9) return ThreatLevel::CRITICAL;
    if (r >= 0.7) return ThreatLevel::HIGH;
    if (r >= 0.4) return ThreatLevel::MEDIUM;
    if (r > 0) return ThreatLevel::LOW;
    return ThreatLevel::NONE;
}

std::vector<ThreatAssessment> RecklessDrivingDetector::assess(
    const std::vector<Track>& tracks, const EgoState& ego,
    const std::map<int, CollisionRisk>& risks) {
    const auto rel = toEgoFrame(tracks, ego, cfg_.egoFrame);
    const double T = cfg_.ttcBrakeThresholdS;

    // The lead vehicle: the nearest confirmed vehicle-like track in the ego path.
    const EgoRelativeTrack* lead = nullptr;
    for (const auto& e : rel) {
        if (e.inEgoPath && e.confirmed && isVehicleLike(e.track->objectClass) &&
            (!lead || e.gapM < lead->gapM))
            lead = &e;
    }
    // Tailgating timer: only the current lead can be tailgated.
    for (auto it = tailgatingSinceMs_.begin(); it != tailgatingSinceMs_.end();) {
        it = (!lead || it->first != lead->track->id) ? tailgatingSinceMs_.erase(it) : std::next(it);
    }

    std::vector<ThreatAssessment> out;
    for (const auto& e : rel) {
        const Track& t = *e.track;
        if (t.state == TrackState::TENTATIVE) continue;  // not shown, not warned about (rule 2)
        ThreatAssessment a;
        a.trackId = t.id;
        a.inEgoPath = e.inEgoPath;

        if (e.inEgoPath && e.closingSpeedMps > cfg_.minClosingSpeedMps) {
            a.ttcS = e.gapM / e.closingSpeedMps;
            if (a.ttcS < 2 * T) a.flags |= FORWARD_COLLISION;
        }
        if (&e == lead && ego.speedMps >= cfg_.tailgatingMinEgoSpeedMps) {
            a.timeGapS = e.gapM / ego.speedMps;
            if (a.timeGapS < cfg_.tailgatingMinGapS) {
                const auto [it, fresh] = tailgatingSinceMs_.try_emplace(t.id, ego.timestampMs);
                if (!fresh && ego.timestampMs - it->second >= cfg_.tailgatingSustainS * 1000) {
                    a.flags |= TAILGATING;
                }
            } else {
                tailgatingSinceMs_.erase(t.id);
            }
        } else if (&e == lead) {
            tailgatingSinceMs_.erase(t.id);
        }

        // History-based rules, on the tracker's filtered states.
        if (isVehicleLike(t.objectClass) && t.history.size() >= cfg_.minHistory) {
            std::vector<double> ts, vs;
            for (const auto& [ms, st] : t.history) {
                ts.push_back(ms / 1000.0);
                vs.push_back(st(3));
            }
            if (detrendedStd(ts, vs, 1) > cfg_.erraticSpeedStdMps) a.flags |= ERRATIC_SPEED;

            const Eigen::Vector2d p0 = t.history.front().second.head<2>();
            const Eigen::Vector2d d = t.history.back().second.head<2>() - p0;
            if (d.norm() > 5.0) {  // needs real travel to define a direction
                const Eigen::Vector2d u = d.normalized(), n(-u.y(), u.x());
                std::vector<double> along, across;
                for (const auto& [ms, st] : t.history) {
                    const Eigen::Vector2d q = st.head<2>() - p0;
                    along.push_back(q.dot(u));
                    across.push_back(q.dot(n));
                }
                if (detrendedStd(along, across, 2) > cfg_.swerveLateralStdM) a.flags |= SWERVING;
            }
        }
        if (e.inEgoPath && isVehicleLike(t.objectClass) && t.filter.state()(3) > 2.0 &&
            t.filter.modelProbabilities()[static_cast<int>(MotionModel::STOP)] >
                cfg_.suddenBrakingStopProb) {
            a.flags |= SUDDEN_BRAKING;
        }
        double collisionProb = 0;
        if (const auto it = risks.find(t.id); it != risks.end()) {
            collisionProb = it->second.probability;
            if (it->second.conflict && collisionProb >= cfg_.crossingProbability)
                a.flags |= CROSSING;
        }
        if (isVulnerable(t.objectClass) && (e.inEgoPath || (a.flags & CROSSING)))
            a.flags |= VULNERABLE_IN_PATH;

        const double rTtc = a.ttcS > 0 ? std::clamp((2 * T - a.ttcS) / T, 0.0, 1.0) : 0.0;
        const double rFlag =
            (a.flags & (kRecklessFlags | VULNERABLE_IN_PATH | SUDDEN_BRAKING)) ? 0.4 : 0.0;
        a.risk = std::max({rTtc, collisionProb, rFlag});
        a.level = levelFor(a.risk);
        if (a.level != ThreatLevel::NONE || a.flags) out.push_back(a);
    }
    return out;
}

}  // namespace ar_drive_assist
