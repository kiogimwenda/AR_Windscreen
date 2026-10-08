// DecisionThread — see include/ar_drive_assist/decision/DecisionThread.h.
#include "ar_drive_assist/decision/DecisionThread.h"

#include <chrono>
#include <map>
#include <thread>

#include "ar_drive_assist/system/EventLog.h"
#include "ar_drive_assist/vehicle/VehicleInterface.h"

namespace ar_drive_assist {

namespace {

// One ego frame for both the detector and the arbiter, from the same configuration files.
EgoFrameConfig egoFrameFrom(const DecisionThresholds& th, const VehicleParams& v) {
    EgoFrameConfig e;
    e.frontBumperFromRearAxleM = v.frontBumperFromRearAxleM;
    e.egoPathHalfWidthM = th.egoPathHalfWidthM;
    return e;
}

RecklessConfig recklessFrom(const DecisionThresholds& th, const EgoFrameConfig& ef) {
    RecklessConfig r;
    r.egoFrame = ef;
    r.ttcBrakeThresholdS = th.ttcBrakeThresholdS;
    r.tailgatingMinGapS = th.tailgatingMinGapS;
    r.minClosingSpeedMps = th.minClosingSpeedMps;
    return r;
}

}  // namespace

DecisionThread::DecisionThread(DecisionThreadConfig cfg, SceneBus& scenes,
                               VehicleInterface* vehicle, DecisionBus* toRender, EventLog* log)
    : cfg_(std::move(cfg)),
      scenes_(scenes),
      vehicle_(vehicle),
      toRender_(toRender),
      log_(log),
      predictor_(cfg_.predictor),
      detector_(recklessFrom(cfg_.thresholds, egoFrameFrom(cfg_.thresholds, cfg_.vehicle))),
      arbiter_(cfg_.thresholds, log),
      egoFrame_(egoFrameFrom(cfg_.thresholds, cfg_.vehicle)) {}

DecisionSnapshot DecisionThread::process(const SceneSnapshot& scene, const HubState& hub,
                                         std::uint64_t nowMs) {
    ++stats_.cycles;
    ArbiterInput in;
    in.nowMs = nowMs;
    in.ego = scene.ego;
    in.hub = hub;
    in.eventLogHealthy = log_ ? log_->healthy() : true;

    std::map<int, CollisionRisk> risks;
    if (scene.egoValid) {
        in.objects = toEgoFrame(scene.tracks, scene.ego, egoFrame_);
        for (const Track& t : scene.tracks)
            if (t.state == TrackState::CONFIRMED)
                risks[t.id] = predictor_.risk(t, scene.egoX, scene.egoP);
        in.threats = detector_.assess(scene.tracks, scene.ego, risks);
    }
    DecisionSnapshot out;
    out.timestampMs = nowMs;
    out.decision = arbiter_.step(in);
    if (out.decision.request.type == schema::RequestType_BRAKE) ++stats_.brakeRequests;
    stats_.warnings += out.decision.warnings.size();

    for (const EgoRelativeTrack& o : in.objects) {
        TrackView v;
        v.trackId = o.track->id;
        v.objectClass = o.track->objectClass;
        v.posVehicle = {o.xM, o.yM};
        v.gapM = o.gapM;
        v.closingSpeedMps = o.closingSpeedMps;
        v.inEgoPath = o.inEgoPath;
        v.confirmed = o.confirmed;
        v.rangeMeasured = o.rangeMeasured;
        v.inStraightPath = o.inStraightPath;
        v.measuredRunMs = o.measuredRunMs;
        v.state = static_cast<int>(o.track->state);
        for (const ThreatAssessment& t : in.threats)
            if (t.trackId == v.trackId) v.threat = t;
        out.tracks.push_back(v);
    }
    return out;
}

void DecisionThread::run(const std::atomic<bool>& stop) {
    while (!stop.load()) {
        SceneSnapshot scene;
        if (!scenes_.popLatest(scene)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const HubState hub = vehicle_ ? vehicle_->hubState() : HubState{};
        DecisionSnapshot d = process(scene, hub, VehicleInterface::hostNowMs());
        if (vehicle_) vehicle_->sendActuationRequest(d.decision.request);
        if (toRender_) toRender_->push(std::move(d));
    }
}

}  // namespace ar_drive_assist
