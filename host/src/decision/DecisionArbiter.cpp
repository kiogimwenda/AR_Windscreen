#include "ar_drive_assist/decision/DecisionArbiter.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace ar_drive_assist {
namespace {

constexpr double kG = 9.80665;  // m/s^2 per g

ActuationRequest request(std::uint64_t nowMs, RequestType type, std::uint8_t intensity,
                         ReasonCode reason) {
    ActuationRequest r;
    r.timestamp_ms = nowMs;
    r.type = type;
    r.intensity = intensity;
    r.reason_code = static_cast<std::uint8_t>(reason);
    return r;
}

}  // namespace

std::string Decision::context() const {
    std::ostringstream s;
    s.precision(3);
    s << "rule=" << rule << " armed=" << (armed ? 1 : 0) << " target=" << targetTrackId
      << " ttc_s=" << ttcS << " gap_m=" << gapM << " closing_mps=" << closingSpeedMps;
    if (targetTrackId >= 0 && ttcS >= 0)
        s << " y_m=" << lateralM << " class=" << targetClass << " run_ms=" << measuredRunMs;
    if (brakeQualifies && rule != 1) s << " confirming_ms=" << confirmingMs;
    for (const auto& w : warnings) s << " warn=\"" << w << '"';
    return s.str();
}

DecisionArbiter::DecisionArbiter(const DecisionThresholds& thresholds, EventLog* log)
    : th_(thresholds), log_(log) {}

std::uint8_t DecisionArbiter::clampedBrakeIntensity() const {
    return std::min(
        {th_.brakeRequestIntensity, th_.brakeActuatorMaxIntensity, kHubMaxSafeBrakeIntensity});
}

bool DecisionArbiter::armed(const ArbiterInput& in) const {
    const HubState& h = in.hub;
    const bool fresh = h.haveReport && h.reportMs <= in.nowMs &&
                       static_cast<double>(in.nowMs - h.reportMs) <= th_.hubStateMaxAgeMs;
    return fresh && h.killSwitchEngaged == 0 && h.haveAck && h.actuatorFaultCode == 0 &&
           in.eventLogHealthy;
}

Decision DecisionArbiter::evaluate(const ArbiterInput& in) const {
    return evaluate(in, true);
}

Decision DecisionArbiter::evaluate(const ArbiterInput& in, bool allowBrake) const {
    Decision d;
    d.armed = armed(in);
    const bool pedalPressed = in.hub.brakePedalActive == 1;

    // Rule 1: the nearest qualifying in-lane object, by time to collision on CURRENT estimates.
    const EgoRelativeTrack* target = nullptr;
    double targetTtc = 0;
    for (const auto& o : in.objects) {
        if (!o.track || !o.confirmed || !o.rangeMeasured || !o.inEgoPath) continue;
        // A ghost (a re-associated track) has no long measured run; a far corner-exit overlap
        // is in the curved path only (EgoRelativeTrack). Neither is a brake target.
        if (!o.inStraightPath || static_cast<double>(o.measuredRunMs) < th_.brakeMinMeasuredTrackMs)
            continue;
        if (!std::isfinite(o.gapM) || !std::isfinite(o.closingSpeedMps) || o.gapM <= 0) continue;
        if (o.closingSpeedMps <= th_.minClosingSpeedMps) continue;
        const double ttc = o.gapM / o.closingSpeedMps;
        if (!std::isfinite(ttc)) continue;
        if (!target || ttc < targetTtc) {
            target = &o;
            targetTtc = ttc;
        }
    }
    if (target) {
        d.targetTrackId = target->track->id;
        d.ttcS = targetTtc;
        d.gapM = target->gapM;
        d.closingSpeedMps = target->closingSpeedMps;
        d.lateralM = target->yM;
        d.targetClass = target->track->objectClass;
        d.measuredRunMs = target->measuredRunMs;
        d.targetMeasuredMs = target->lastMeasuredMs;
    }
    d.brakeQualifies = target && targetTtc < th_.ttcBrakeThresholdS && !pedalPressed && d.armed;
    if (d.brakeQualifies && allowBrake) {
        d.rule = 1;
        d.request = request(in.nowMs, RequestType::RequestType_BRAKE, clampedBrakeIntensity(),
                            ReasonCode::FCW_TTC);
        return d;
    }

    // Rule 2: the car itself is braking hard.
    const double decel = -in.ego.longitudinalAccelMps2;
    if (std::isfinite(decel) && decel > th_.hardBrakeDecelG * kG) {
        d.rule = 2;
        d.request =
            request(in.nowMs, RequestType::RequestType_HAZARDS, 0, ReasonCode::HARD_BRAKE_DETECTED);
        return d;
    }

    // Rule 3: warnings only.
    for (const ThreatAssessment& t : in.threats) {
        if (t.inEgoPath && (t.level == ThreatLevel::HIGH || t.level == ThreatLevel::CRITICAL)) {
            std::ostringstream w;
            w << "track " << t.trackId << (t.level == ThreatLevel::CRITICAL ? " CRITICAL" : " HIGH")
              << " flags=0x" << std::hex << t.flags;
            d.warnings.push_back(w.str());
            if (d.targetTrackId < 0) d.targetTrackId = t.trackId;
        }
    }
    d.rule = d.warnings.empty() ? 4 : 3;
    d.request =
        request(in.nowMs, RequestType::RequestType_NONE, 0, ReasonCode::NONE);  // rules 3 and 4
    return d;
}

Decision DecisionArbiter::step(const ArbiterInput& in) {
    Decision d = evaluate(in);
    // Onset confirmation: braking STARTS only once rule 1 has qualified on the same target on
    // every cycle since it first did, and the LiDAR has measured that target brakeConfirmMs or
    // more after that first cycle. A measurement, not a prediction, confirms: on a cycle without
    // a LiDAR scan the tracker only predicts (MeasurementBatch::observed), and a prediction
    // drifting into the path must not complete the confirmation (KITTI replay with jitter: a
    // kerbside cyclist's track did). Once braking, it continues while rule 1 qualifies at all.
    if (!d.brakeQualifies) {
        pendingTarget_ = -1;
        braking_ = false;
    } else if (!braking_) {
        if (d.targetTrackId != pendingTarget_ || in.nowMs < pendingSinceMs_) {
            pendingTarget_ = d.targetTrackId;
            pendingSinceMs_ = in.nowMs;
        }
        d.confirmingMs = in.nowMs - pendingSinceMs_;
        const bool measuredLater =
            d.targetMeasuredMs >= pendingSinceMs_ &&
            static_cast<double>(d.targetMeasuredMs - pendingSinceMs_) >= th_.brakeConfirmMs;
        if (measuredLater) {
            braking_ = true;
        } else {
            const std::uint64_t confirming = d.confirmingMs;
            d = evaluate(in, false);  // rules 2-4 meanwhile
            d.confirmingMs = confirming;
        }
    }
    const bool active = d.request.type != RequestType::RequestType_NONE;
    if (log_ && (active || lastWasActive_)) log_->logActuationRequest(d.request, d.context());
    lastWasActive_ = active;
    return d;
}

}  // namespace ar_drive_assist
