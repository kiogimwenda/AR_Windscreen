#include "ar_drive_assist/render/SignTracker.h"

#include <algorithm>
#include <cmath>

namespace ar_drive_assist {
namespace {

double iou(const Eigen::Vector4d& a, const Eigen::Vector4d& b) {
    const double x0 = std::max(a[0], b[0]), y0 = std::max(a[1], b[1]);
    const double x1 = std::min(a[0] + a[2], b[0] + b[2]), y1 = std::min(a[1] + a[3], b[1] + b[3]);
    const double inter = std::max(0.0, x1 - x0) * std::max(0.0, y1 - y0);
    const double uni = a[2] * a[3] + b[2] * b[3] - inter;
    return uni > 0 ? inter / uni : 0.0;
}

bool sameKind(int a, int b) {
    return a == b || (sign_class::isSpeed(a) && sign_class::isSpeed(b));
}

}  // namespace

SignTracker::SignTracker(SignTrackerConfig cfg) : cfg_(cfg) {}

void SignTracker::update(std::uint64_t nowMs, const std::vector<SignObservation>& obs,
                         std::optional<double> egoSpeedMps, double yawRate,
                         std::optional<int> mapMaxSpeedKph, bool roadChanged) {
    // Carry stored positions forward with the car's motion since the last frame.
    const double dt = lastMs_ && nowMs > lastMs_ ? (nowMs - lastMs_) / 1000.0 : 0.0;
    lastMs_ = nowMs;
    const double v = egoSpeedMps.value_or(0.0), dpsi = yawRate * dt;
    for (TrackedSign& s : signs_) {
        if (!s.position) continue;
        const Eigen::Vector2d rel = s.position->head<2>() - Eigen::Vector2d(v * dt, 0);
        const double c = std::cos(-dpsi), sn = std::sin(-dpsi);
        s.position = Eigen::Vector3d(c * rel.x() - sn * rel.y(), sn * rel.x() + c * rel.y(),
                                     s.position->z());
    }

    // Associate observations with signs (greedy, best match first).
    std::vector<bool> seen(signs_.size(), false);
    for (const SignObservation& o : obs) {
        int best = -1;
        double bestScore = 0;
        for (std::size_t i = 0; i < signs_.size(); ++i) {
            if (seen[i] || !sameKind(signs_[i].classId, o.classId)) continue;
            double score = iou(signs_[i].box, o.box);
            if (score < cfg_.iouMatch) score = 0;
            if (o.position && signs_[i].position &&
                (*o.position - *signs_[i].position).head<2>().norm() < cfg_.matchDistanceM)
                score = std::max(score, 0.5);
            if (score > bestScore) {
                bestScore = score;
                best = static_cast<int>(i);
            }
        }
        if (best < 0) {
            TrackedSign s;
            s.id = nextId_++;
            signs_.push_back(s);
            seen.push_back(false);
            best = static_cast<int>(signs_.size()) - 1;
        }
        TrackedSign& s = signs_[best];
        seen[best] = true;
        s.classId = o.classId;
        s.box = o.box;
        if (o.position) s.position = o.position;
        s.lastSeenMs = nowMs;
        s.hits = o.confidence >= cfg_.minConfidence ? s.hits + 1 : 0;
        if (s.hits >= cfg_.minHits) s.confirmed = true;
        if (sign_class::isSpeed(o.classId) && o.confidence >= cfg_.minConfidence) {
            s.speedReadings.push_back(sign_class::speedValueKph(o.classId));
            while (static_cast<int>(s.speedReadings.size()) > cfg_.agreeFrames)
                s.speedReadings.pop_front();
            if (static_cast<int>(s.speedReadings.size()) == cfg_.agreeFrames &&
                std::all_of(s.speedReadings.begin(), s.speedReadings.end(),
                            [&](int x) { return x == s.speedReadings.front(); })) {
                s.agreedSpeedKph = s.speedReadings.front();
            }
        }
    }
    // A sign not seen this frame loses its consecutive count; it is forgotten after forgetMs.
    for (std::size_t i = 0; i < signs_.size(); ++i)
        if (!seen[i]) signs_[i].hits = 0;
    signs_.erase(
        std::remove_if(signs_.begin(), signs_.end(),
                       [&](const TrackedSign& s) { return nowMs - s.lastSeenMs > cfg_.forgetMs; }),
        signs_.end());

    // Speed limit.
    const auto fromMap = [&] {
        limit_ =
            mapMaxSpeedKph ? SpeedLimit{mapMaxSpeedKph, SpeedLimit::Source::MAP} : SpeedLimit{};
    };
    if (roadChanged || (limit_.source != SpeedLimit::Source::SIGN)) fromMap();
    for (const TrackedSign& s : signs_) {
        if (!s.confirmed || s.lastSeenMs != nowMs) continue;
        if (s.position && s.position->y() < -cfg_.oppositeSideM) continue;  // opposite carriageway
        if (sign_class::isSpeed(s.classId) && s.agreedSpeedKph) {
            limit_ = {s.agreedSpeedKph, SpeedLimit::Source::SIGN};
        } else if (s.classId == sign_class::kEndOfRestriction) {
            fromMap();
        }
    }

    // Stop signs clear once the car has stood still near them.
    const bool still = egoSpeedMps && *egoSpeedMps < cfg_.stationaryMps;
    if (!still)
        stationarySinceMs_ = 0;
    else if (!stationarySinceMs_)
        stationarySinceMs_ = nowMs;
    for (TrackedSign& s : signs_) {
        if (s.classId != sign_class::kStop || !s.position || !still) continue;
        if (s.position->head<2>().norm() < cfg_.stopClearM &&
            nowMs - stationarySinceMs_ >= cfg_.stopClearS * 1000)
            s.cleared = true;
    }
}

}  // namespace ar_drive_assist
