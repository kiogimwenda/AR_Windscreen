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
};

// config/decision_thresholds.yaml (Part 12.5)
struct DecisionThresholds {
    double ttcBrakeThresholdS = 0.0;
    double hardBrakeDecelG = 0.0;
    double tailgatingMinGapS = 0.0;
    std::uint8_t brakeActuatorMaxIntensity = 0;
};

struct Config {
    VehicleParams vehicle;
    DecisionThresholds decision;
};

}  // namespace ar_drive_assist
