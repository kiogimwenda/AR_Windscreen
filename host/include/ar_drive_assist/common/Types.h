#pragma once
// Shared plain types used across subsystem boundaries — see docs/BUILD_GUIDE.md Part 3.5 and
// Part 5.
//
// The messages that travel on the Part 5.3 bus are FlatBuffers object-API structs generated from
// common/schemas/*.fbs (see cmake/FlatBufferSchemas.cmake for why the bus carries those rather
// than serialized buffers). They are aliased here under the names the guide uses, so subsystem
// code says ActuationRequest, not schema::ActuationRequestT.
//
// Config is a plain struct filled by SystemManager::loadConfig(). It only holds the sections a
// built subsystem actually reads. Later phases add their own section together with the code that
// consumes it, so no field sits here unvalidated and unused.

#include <array>
#include <cstdint>
#include <string>

#include "ar_drive_assist/common/schemas/actuation_request_generated.h"
#include "ar_drive_assist/common/schemas/detections_generated.h"
#include "ar_drive_assist/common/schemas/road_projected_route_generated.h"
#include "ar_drive_assist/common/schemas/vehicle_pose_generated.h"

namespace ar_drive_assist {

using ActuationRequest = schema::ActuationRequestT;
using RequestType = schema::RequestType;
using DetectionFrame = schema::DetectionFrameT;
using VehiclePose = schema::VehiclePoseT;
using RoadProjectedRoute = schema::RoadProjectedRouteT;

// config/vehicle_params.yaml (Appendix A)
struct VehicleParams {
    double wheelbaseM = 0.0;
    double cameraHeightM = 0.0;
    std::string obdPidBrakeActive;  // empty = the vehicle does not expose it
    std::string serialDevice;
    int serialBaud = 0;
    // Added in Phase 10. Tracked positions are measured from the rear axle (the vehicle frame's
    // origin, Part 12.2), but time-to-collision must use the gap from the FRONT BUMPER; without
    // this every range reads several metres long and braking comes late.
    double frontBumperFromRearAxleM = 0.0;
    double halfWidthM = 0.0;
    // Added 2026-10-06. The BNO085's mounting in the windscreen pod: [roll, pitch, yaw] in the
    // vehicle frame, degrees (fusion/ImuMount, BUILD_GUIDE 12.2.2). Identity until calibrated.
    std::array<double, 3> imuMountRpyDeg{0.0, 0.0, 0.0};
};

// The hub's own brake ceiling, BrakeActuatorDriver::kMaxSafeIntensity (firmware, Part 4.4): the
// documented starting value, finalised by the Part 13.3 bench test. The host's configured ceiling
// may never exceed it. test_decision_arbiter.cpp cross-checks this against the firmware header
// once the constant exists there (Phase 12).
constexpr std::uint8_t kHubMaxSafeBrakeIntensity = 90;

// config/decision_thresholds.yaml (Part 12.5)
struct DecisionThresholds {
    double ttcBrakeThresholdS = 0.0;
    double hardBrakeDecelG = 0.0;
    double tailgatingMinGapS = 0.0;
    std::uint8_t brakeActuatorMaxIntensity = 0;
    // Added in Phase 10 (Part 9.3):
    std::uint8_t brakeRequestIntensity = 0;  // the "<tuned value>" of rule 1; <= the ceiling
    double egoPathHalfWidthM = 0.0;          // an object is "in lane" within this of the ego path
    double minClosingSpeedMps = 0.0;         // below this, no time-to-collision is computed
    double hubStateMaxAgeMs = 0.0;           // older hub reports mean "not known to be armed"
};

struct Config {
    VehicleParams vehicle;
    DecisionThresholds decision;
};

}  // namespace ar_drive_assist
