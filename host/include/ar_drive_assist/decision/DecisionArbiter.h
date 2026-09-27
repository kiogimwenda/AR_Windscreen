#pragma once
// DecisionArbiter — the ONLY subsystem permitted to produce an ActuationRequest. See
// docs/BUILD_GUIDE.md Part 9.3 and Part 0's non-negotiable safety properties.
//
// ---------------------------------------------------------------------------------------------
// Four rules, in priority order; the first that matches wins; one request per cycle.
//
//   1. BRAKE (reason FCW_TTC). Time to collision to the nearest in-lane object is below
//      `ttc_brake_threshold_s`, AND the driver's brake pedal is not reported pressed (0 or
//      unknown), AND the system is ARMED. Time to collision = the bumper-to-object gap / the
//      closing speed, both from the tracker's CURRENT estimate. Never from predicted positions,
//      closest approach, collision probabilities or model probabilities (amended 2026-09-25):
//      braking on a forecast risks phantom braking, which is itself a hazard. An object may trigger
//      it only if ALL hold:
//        - it is CONFIRMED (not tentative, not coasting on prediction while unseen);
//        - its range was MEASURED by the LiDAR within the last 200 ms (never a MiDaS estimate);
//        - it is in the ego path, ahead of the bumper, closing faster than `min_closing_speed_mps`;
//        - every number involved is finite (a NaN or infinity never brakes).
//      ARMED means: a hub report no older than `hub_state_max_age_ms`, kill switch not engaged,
//      no actuator fault in the hub's last AckStatus, AND the EventLog still able to record.
//      Unknown is not armed. A system that cannot record actuation evidence must not actuate.
//   2. HAZARDS (reason HARD_BRAKE_DETECTED): the IMU shows deceleration above `hard_brake_decel_g`.
//   3. A HIGH/CRITICAL reckless-driving or hazard assessment in the ego path: a WARNING for the
//      renderer and warning tone. The request stays NONE: reckless-driving detection never brakes
//      or signals on its own.
//   4. Otherwise a zeroed request {type NONE, intensity 0, reason NONE}.
//
// The brake intensity is the configured `brake_request_intensity`, clamped to the configured
// `brake_actuator_max_intensity` AND to the hub's own compiled ceiling
// (kHubMaxSafeBrakeIntensity). So even a thresholds struct built wrongly in code cannot request
// more. The hub clamps again independently (Part 4.4, the second ceiling).
//
// step() writes every non-NONE request, and the first NONE after one (the release), to the
// EventLog WITH its evaluation context (rule, TTC, gap, closing speed, target, armed state) BEFORE
// the request is handed on. That record is the actuation evidence for the report and viva.
// ---------------------------------------------------------------------------------------------

#include <cstdint>
#include <string>
#include <vector>

#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/safety/RecklessDrivingDetector.h"
#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {

enum class ReasonCode : std::uint8_t { NONE = 0, FCW_TTC = 1, HARD_BRAKE_DETECTED = 2 };

// The hub's state as last reported (SensorReport at 50 Hz, AckStatus per command).
struct HubState {
    bool haveReport = false;
    std::uint64_t reportMs = 0;            // host time the last SensorReport arrived
    std::uint8_t brakePedalActive = 0xFF;  // 0/1, 0xFF = unknown (SensorReport.obdBrakePedalActive)
    std::uint8_t killSwitchEngaged = 1;    // 1 = actuator power cut; assumed cut until reported
    bool haveAck = false;
    std::uint8_t actuatorFaultCode = 0;  // from the last AckStatus; 0 = none
};

struct ArbiterInput {
    std::uint64_t nowMs = 0;
    EgoState ego;
    std::vector<EgoRelativeTrack> objects;  // toEgoFrame(tracks, ego, ...)
    std::vector<ThreatAssessment> threats;  // RecklessDrivingDetector::assess(...)
    HubState hub;
    bool eventLogHealthy = true;
};

struct Decision {
    ActuationRequest request;  // what goes to VehicleInterface
    int rule = 4;              // which Part 9.3 rule fired (1-4)
    bool armed = false;
    int targetTrackId = -1;  // rule 1/3: the object concerned
    double ttcS = -1, gapM = -1, closingSpeedMps = 0;
    std::vector<std::string> warnings;  // rule 3: for the renderer and the warning tone
    std::string context() const;        // one line for the EventLog
};

class DecisionArbiter {
public:
    DecisionArbiter(const DecisionThresholds& thresholds, EventLog* log = nullptr);

    // Pure: the decision for one cycle.
    Decision evaluate(const ArbiterInput& in) const;
    // evaluate() + the evidence trail. Use this in the running system.
    Decision step(const ArbiterInput& in);

    bool armed(const ArbiterInput& in) const;
    // The only way a brake intensity is computed.
    std::uint8_t clampedBrakeIntensity() const;

private:
    DecisionThresholds th_;
    EventLog* log_;
    bool lastWasActive_ = false;
};

}  // namespace ar_drive_assist
