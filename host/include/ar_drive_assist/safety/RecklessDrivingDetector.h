#pragma once
// RecklessDrivingDetector — rule-based hazard and reckless-driving classification. See
// docs/BUILD_GUIDE.md Part 9.2, and docs/architecture/ar-overlay-design.md §3 for how its output
// is shown.
//
// ---------------------------------------------------------------------------------------------
// Its output INFORMS: the AR overlay (hazard glow, following-distance zone, lane-line glow) and a
// warning tone. It never brakes or signals (Part 9.3 rule 3). Braking belongs to
// DecisionArbiter's own forward-collision check alone.
//
// Rules, per confirmed track (each a flag; several can hold at once):
//   TAILGATING         the EGO car's time gap to the vehicle directly ahead (bumper-to-object
//                      range / ego speed) below `tailgatingMinGapS`, sustained for
//                      `tailgatingSustainS`. The zone painted on the road (overlay §3.5).
//   ERRATIC_SPEED      the vehicle's speed over its history varies more than a steady change of
//                      speed explains: the standard deviation of speed AFTER removing a straight-
//                      line trend. A car accelerating smoothly away from lights is not erratic;
//                      one surging and braking is.
//   SWERVING           lateral wander of the vehicle's own path: positions are expressed along
//                      and across its mean direction of travel, a parabola in the along-track
//                      coordinate is removed (so a steady turn is not a swerve), and the residual
//                      spread is tested. From the tracker's filtered state (the L-shape reference
//                      point, Part 9.1), never a raw centroid.
//   SUDDEN_BRAKING     a vehicle in the ego path whose IMM STOP-model probability is high.
//   CROSSING           the motion predictor's closest-approach conflict with a collision
//                      probability above `crossingProbability` (crossing traffic, cut-ins).
//   FORWARD_COLLISION  in the ego path with time-to-collision below 2x the brake threshold (the
//                      WARNING band; the brake decision is the arbiter's).
//   VULNERABLE_IN_PATH a pedestrian, cyclist or obstacle/animal in the ego path, or crossing into
//                      it.
//
// Risk r in [0, 1] (overlay §3.1), the larger of:
//   - time to collision: 1 at or below the brake threshold T, falling linearly to 0 at 2T;
//   - the predicted collision probability;
//   - 0.4 when any reckless flag, VULNERABLE_IN_PATH or SUDDEN_BRAKING holds (a lead car braking
//     hard is at least an amber warning; overlay §3.1 lists reckless flags and vulnerable users).
// Level: CRITICAL r >= 0.9, HIGH >= 0.7, MEDIUM >= 0.4, LOW > 0, else NONE.
// ---------------------------------------------------------------------------------------------

#include <cstdint>
#include <map>
#include <vector>

#include "ar_drive_assist/safety/MotionPredictor.h"
#include "ar_drive_assist/safety/MultiObjectTracker.h"

namespace ar_drive_assist {

// The car itself, in the tracker's world frame, from SensorFusion.
struct EgoState {
    EgoPose pose;  // world x, y (m), psi (rad, CCW from east)
    double speedMps = 0;
    double yawRateRadPerS = 0;
    double longitudinalAccelMps2 = 0;  // IMU, forward positive (braking is negative)
    std::uint64_t timestampMs = 0;
};

// A track as seen from the car. Computed once, used by both this detector and the arbiter, so a
// warning and a brake decision can never disagree about what is in the lane.
struct EgoRelativeTrack {
    const Track* track = nullptr;
    double xM = 0, yM = 0;       // vehicle frame (x forward, y left), from the rear axle
    double gapM = 0;             // x minus the front-bumper distance: the physical gap ahead
    double closingSpeedMps = 0;  // positive when the gap is shrinking
    bool inEgoPath = false;      // within the ego path corridor (curvature from the yaw rate)
    // Also within the corridor of the STRAIGHT path ahead. Rule 1 brakes only on an object in
    // both: close in the two agree, so a real target in a bend still brakes; far out, a
    // constant-turn extrapolation on a corner exit sweeps across parked cars at the kerb (KITTI
    // replay, 2026-10-07), and that overlap alone is a warning (rule 3), not a brake.
    bool inStraightPath = false;
    std::uint64_t measuredRunMs = 0;   // length of the track's unbroken LiDAR-measured run (ms)
    bool confirmed = false;            // CONFIRMED, not tentative or coasting on prediction
    bool rangeMeasured = false;        // last LiDAR-measured range within `measuredRangeMaxAgeMs`
    std::uint64_t lastMeasuredMs = 0;  // when that range was measured (0 = never)
};

struct EgoFrameConfig {
    double frontBumperFromRearAxleM = 3.5;
    double egoPathHalfWidthM = 1.2;
    std::uint64_t measuredRangeMaxAgeMs = 200;  // two LiDAR frames
};

// Ego path: an arc of curvature kappa = yaw rate / speed; at distance x ahead it lies
// kappa x^2 / 2 to the side. An object is in the path when |y - kappa x^2 / 2| is within the
// corridor half-width and it is ahead of the bumper.
std::vector<EgoRelativeTrack> toEgoFrame(const std::vector<Track>& tracks, const EgoState& ego,
                                         const EgoFrameConfig& cfg);

enum class ThreatLevel : std::uint8_t { NONE, LOW, MEDIUM, HIGH, CRITICAL };

enum ThreatFlag : std::uint32_t {
    TAILGATING = 1u << 0,
    ERRATIC_SPEED = 1u << 1,
    SWERVING = 1u << 2,
    SUDDEN_BRAKING = 1u << 3,
    CROSSING = 1u << 4,
    FORWARD_COLLISION = 1u << 5,
    VULNERABLE_IN_PATH = 1u << 6,
};
constexpr std::uint32_t kRecklessFlags = TAILGATING | ERRATIC_SPEED | SWERVING;

struct ThreatAssessment {
    int trackId = 0;
    std::uint32_t flags = 0;
    ThreatLevel level = ThreatLevel::NONE;
    double risk = 0;  // r in [0, 1]
    bool inEgoPath = false;
    double ttcS = -1;      // -1 when not closing
    double timeGapS = -1;  // ego time gap to it when it is the lead vehicle, else -1
};

struct RecklessConfig {
    EgoFrameConfig egoFrame;
    double ttcBrakeThresholdS = 1.8;  // T of the risk ramp (decision_thresholds.yaml)
    double minClosingSpeedMps = 0.5;
    double tailgatingMinGapS = 1.0;  // decision_thresholds.yaml
    double tailgatingSustainS = 1.0;
    double tailgatingMinEgoSpeedMps = 3.0;  // queueing at walking pace is not tailgating
    std::size_t minHistory = 15;            // of the tracker's 30-state history
    double erraticSpeedStdMps = 1.5;
    // Chosen from a discrimination table (Phase 10 progress log, 2026-09-27), true paths, 3 s
    // window:
    //   weaves A 0.5-0.8 m, period 2-3 s: 0.28-0.45 m;  lane changes 3-6 s: <= 0.155 m;
    //   steady turns R 25-80 m: <= 0.068 m.
    // LIMIT: a slow weave (period ~4 s) scores 0.175 m, the same as a quick lane change (0.155 m):
    // within the tracker's 30-state (3 s) history the two cannot be told apart, so slow weaves are
    // not flagged. A longer history is the recorded fix.
    double swerveLateralStdM = 0.25;
    double suddenBrakingStopProb = 0.6;
    double crossingProbability = 0.3;
};

class RecklessDrivingDetector {
public:
    explicit RecklessDrivingDetector(RecklessConfig cfg = {});

    // `risks`: MotionPredictor::risk per track id, where computed (optional).
    std::vector<ThreatAssessment> assess(const std::vector<Track>& tracks, const EgoState& ego,
                                         const std::map<int, CollisionRisk>& risks = {});

    static ThreatLevel levelFor(double risk);

private:
    RecklessConfig cfg_;
    std::map<int, std::uint64_t> tailgatingSinceMs_;  // lead track id -> when the gap went short
};

}  // namespace ar_drive_assist
