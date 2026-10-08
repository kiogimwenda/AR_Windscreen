#pragma once
// DecisionThread — Part 5.1's thread 5: tracks -> threats -> DecisionArbiter -> the hub. See
// docs/BUILD_GUIDE.md Part 9.2, 9.3.
//
// For every SceneSnapshot from FusionThread:
//   ego frame   each track seen from the car (gap from the bumper, closing speed, in the path?),
//               computed ONCE so the warning and the brake decision cannot disagree
//   risk        MotionPredictor: collision probability of each confirmed track against the ego's
//               own predicted motion (the EKF's state and covariance)
//   assess      RecklessDrivingDetector: threat level and flags per track
//   arbitrate   DecisionArbiter::step(): the one place a request is decided, with the hub's state
//               (armed? kill switch? pedal?) as VehicleInterface last heard it
//   send        VehicleInterface::sendActuationRequest(), EVERY cycle, "none" included: the hub
//               link turns silence into a zeroed command after 150 ms (VehicleInterface.h), so a
//               stalled decision thread releases rather than holds
//   publish     a DecisionSnapshot to the renderer (warnings, per-track threat)
//
// No scene, no decision: if FusionThread stops, nothing is renewed and the hub link and the
// hub's own watchdog release (Part 3.3).

#include <atomic>
#include <cstdint>

#include "ar_drive_assist/system/PipelineMessages.h"

namespace ar_drive_assist {

class EventLog;
class VehicleInterface;

struct DecisionThreadConfig {
    DecisionThresholds thresholds;
    VehicleParams vehicle;
    PredictorConfig predictor;
};

class DecisionThread {
public:
    DecisionThread(DecisionThreadConfig cfg, SceneBus& scenes, VehicleInterface* vehicle,
                   DecisionBus* toRender = nullptr, EventLog* log = nullptr);

    void run(const std::atomic<bool>& stop);

    // One cycle, synchronously: the decision for this scene and hub state (tests call it).
    DecisionSnapshot process(const SceneSnapshot& scene, const HubState& hub, std::uint64_t nowMs);

    struct Stats {
        std::uint64_t cycles = 0, brakeRequests = 0, warnings = 0;
    };
    Stats stats() const { return stats_; }

private:
    DecisionThreadConfig cfg_;
    SceneBus& scenes_;
    VehicleInterface* vehicle_;
    DecisionBus* toRender_;
    EventLog* log_;
    MotionPredictor predictor_;
    RecklessDrivingDetector detector_;
    DecisionArbiter arbiter_;
    EgoFrameConfig egoFrame_;
    Stats stats_;
};

}  // namespace ar_drive_assist
