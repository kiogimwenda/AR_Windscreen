#pragma once
// SignTracker — turns per-frame road-sign detections into what the display may show. See
// docs/architecture/ar-overlay-design.md rules 2, 3, 7 and §2 (sign behaviours).
//
// ---------------------------------------------------------------------------------------------
//   Confirmation (rule 2): a sign is shown only after `minHits` consecutive frames, each at
//     confidence >= `minConfidence`. One-frame flickers never reach the screen.
//   Association: a detection continues a sign when their image boxes overlap (IoU >= `iouMatch`)
//     or their measured ground positions are within `matchDistanceM`. All SPEED classes associate
//     with one another, so a sign read as 30 in one frame and 50 in the next stays ONE sign whose
//     value is disputed, rather than becoming two signs.
//   Speed value agreement (§2, added 2026-09-26): a speed sign's value is accepted only when its
//     last `agreeFrames` readings all agree. If they disagree, the displayed limit stays as it was
//     (or the map's maxspeed): never a guess. Evidence: the detector read a 30 as 50 at 0.57 in the
//     Kenyan audit, and a wrong limit is worse than a missing one.
//   Which signs apply (rule 3): Kenya drives on the LEFT. A sign measured more than
//     `oppositeSideM` to the right is for the opposite carriageway, and never sets our limit.
//   Speed limit persistence: the accepted limit holds after the sign leaves view, until the next
//     accepted speed sign, an end-of-restriction sign (back to the map's maxspeed), or a change of
//     road (map matching; back to the map's maxspeed). With no sign, the map's maxspeed is used.
//   Stop signs: a stop target until the car has been stationary for `stopClearS` within
//     `stopClearM` of it; then it is cleared (the barrier dissolves).
//   Positions are in the vehicle frame and are carried forward with the car's own motion between
//   sightings, so a sign that went out of view stays where it is on the road.
// ---------------------------------------------------------------------------------------------

#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace ar_drive_assist {

namespace sign_class {
constexpr int kStop = 0, kGiveWay = 1, kNoEntry = 2, kNoLeft = 3, kNoRight = 4, kNoUTurn = 5,
              kNoOvertaking = 6, kSpeedFirst = 7, kSpeedLast = 18, kEndOfRestriction = 19,
              kPedestrianCrossing = 20, kSchool = 21, kHump = 22, kRoundabout = 23, kSignals = 24,
              kKeepLeftRight = 25, kNoParking = 26, kOtherWarning = 27, kOtherRegulatory = 28;
inline bool isSpeed(int c) {
    return c >= kSpeedFirst && c <= kSpeedLast;
}
inline int speedValueKph(int c) {
    return isSpeed(c) ? (c - kSpeedFirst + 1) * 10 : 0;
}
}  // namespace sign_class

struct SignObservation {
    int classId = 0;
    float confidence = 0;
    Eigen::Vector4d box = Eigen::Vector4d::Zero();  // image pixels: x, y, w, h
    std::optional<Eigen::Vector3d> position;        // vehicle frame (Part 8.3 rule 7), if ranged
};

struct TrackedSign {
    int id = 0;
    int classId = 0;  // the latest class (for speed signs: the latest reading)
    int hits = 0;     // consecutive confident frames
    bool confirmed = false;
    Eigen::Vector4d box = Eigen::Vector4d::Zero();
    std::optional<Eigen::Vector3d> position;
    std::uint64_t lastSeenMs = 0;
    std::deque<int> speedReadings;  // recent speed values (km/h) of this sign
    std::optional<int> agreedSpeedKph;
    bool cleared = false;  // stop sign: the car has stopped at it
};

struct SpeedLimit {
    std::optional<int> kph;
    enum class Source { NONE, SIGN, MAP } source = Source::NONE;
};

struct SignTrackerConfig {
    int minHits = 3;
    float minConfidence = 0.5f;
    double iouMatch = 0.3;
    double matchDistanceM = 3.0;
    std::uint64_t forgetMs = 1500;
    int agreeFrames = 3;
    double oppositeSideM = 4.0;
    // stopClearM is from the rear axle (the vehicle frame's origin): about 6 m past the bumper.
    double stopClearS = 1.0, stopClearM = 10.0, stationaryMps = 0.5;
};

class SignTracker {
public:
    explicit SignTracker(SignTrackerConfig cfg = {});

    // One camera frame. `egoSpeedMps`/`yawRate`: the car's motion since the previous call, used to
    // carry sign positions forward. `mapMaxSpeedKph`: OSM maxspeed of the matched road, if known.
    void update(std::uint64_t nowMs, const std::vector<SignObservation>& observations,
                std::optional<double> egoSpeedMps, double yawRateRadPerS,
                std::optional<int> mapMaxSpeedKph, bool roadChanged);

    const std::vector<TrackedSign>& signs() const { return signs_; }
    const SpeedLimit& limit() const { return limit_; }

private:
    SignTrackerConfig cfg_;
    std::vector<TrackedSign> signs_;
    SpeedLimit limit_;
    int nextId_ = 1;
    std::uint64_t lastMs_ = 0;
    std::uint64_t stationarySinceMs_ = 0;
};

}  // namespace ar_drive_assist
