#pragma once
// MultiObjectTracker — keeps a persistent identity and motion estimate for every object around the
// car. See docs/BUILD_GUIDE.md Part 9.1 (amended 2026-09-25).
//
// The loop is the original design's (Appendix Q.7), kept because it is sound:
//     predict every track -> Mahalanobis cost matrix -> Hungarian assignment -> gate -> update
//     -> start tracks for unmatched measurements -> retire stale tracks -> history
// What changed is how tracks move: each is an ImmFilter (ImmFilter.h) with real uncertainty, so
// the Mahalanobis distance uses the track's actual predicted covariance.
//
// ---------------------------------------------------------------------------------------------
// Frames. Measurements arrive in the VEHICLE frame (x forward, y left, metres), as the perception
// side produces them. They are converted to the world frame with the ego pose (SensorFusion) and
// tracked there. In the world frame a parked car has zero velocity; tracked in the vehicle frame,
// every parked car would appear to approach at the ego speed.
//
// Track lifecycle (docs/architecture/ar-overlay-design.md rule 2 relies on it):
//   TENTATIVE       new, not yet shown. Needs `confirmHits` consecutive associations. A
//                   single-frame false positive (the Kraków "vehicle" on a wall) dies here.
//   CONFIRMED       being measured.
//   PREDICTED_ONLY  lost (occluded?), coasting on prediction for at most `maxCoastS`. It is
//                   displayed as a prediction, never as a sighting.
//
// Classes. A measurement never updates a track of a DIFFERENT known class: a pedestrian must not
// be merged into the car beside them. An unclassified measurement (a LiDAR cluster no camera mask
// explains, Part 8.3 rule 6) may update any track, and a track first seen unclassified takes the
// class of its first classified match.
//
// Late measurements. The LiDAR path is slower than the camera path, so a batch can arrive after a
// later one was processed. The tracker keeps a snapshot before each batch for `replayWindowMs`,
// rewinds to the snapshot before the late batch's time, and replays everything in time order.
// The result is identical to in-order processing. Batches older than the window are dropped and
// counted.
// ---------------------------------------------------------------------------------------------

#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "ar_drive_assist/safety/ImmFilter.h"

namespace ar_drive_assist {

enum class TrackState { TENTATIVE, CONFIRMED, PREDICTED_ONLY };

struct ObjectMeasurement {
    Eigen::Vector2d posVehicle;  // metres, vehicle frame (x forward, y left)
    Eigen::Matrix2d cov;         // covariance of posVehicle
    int32_t objectClass = -1;    // ObjectClass (Postprocess.h), or -1 = unclassified (LiDAR only)
    std::optional<double> headingVehicle;  // rad, from an L-shape fit, modulo pi
};

struct EgoPose {  // from SensorFusion: world position (m) and heading (rad, CCW from east)
    double x = 0, y = 0, psi = 0;
};

struct MeasurementBatch {
    std::uint64_t timestampMs = 0;
    EgoPose ego;
    std::vector<ObjectMeasurement> measurements;
};

struct TrackerConfig {
    double gateChi2 = 13.8;  // Mahalanobis^2 gate: chi-square, 2 dof, 99.9%
    int confirmHits = 3;
    int tentativeMaxMisses = 1;
    double maxCoastS = 1.5;
    std::uint64_t replayWindowMs = 500;
    std::size_t historyLength = 30;               // Part 9.1: the classifier's 30-frame history
    std::map<int32_t, MotionNoise> noiseByClass;  // ObjectClass -> noise; -1 = unclassified
    MotionNoise noiseFor(int32_t cls) const;
};

// Reads config/motion_prediction.yaml. Throws std::runtime_error naming the file and key.
TrackerConfig loadTrackerConfig(const std::string& path);

struct Track {
    int id = 0;
    int32_t objectClass = -1;
    TrackState state = TrackState::TENTATIVE;
    ImmFilter filter;
    int hits = 0, misses = 0;
    std::uint64_t lastUpdateMs = 0, lastTimeMs = 0;
    std::deque<std::pair<std::uint64_t, ImmFilter::State>> history;
    Eigen::Matrix2d birthCov = Eigen::Matrix2d::Zero();  // first measurement's covariance
};

class MultiObjectTracker {
public:
    explicit MultiObjectTracker(TrackerConfig cfg = {});

    void update(const MeasurementBatch& batch);
    const std::vector<Track>& tracks() const { return tracks_; }
    std::uint64_t droppedLateBatches() const { return droppedLate_; }

    // Vehicle frame <-> world frame for a given ego pose.
    static Eigen::Vector2d toWorld(const EgoPose& ego, const Eigen::Vector2d& pVehicle);
    static Eigen::Matrix2d rotation(double psi);

private:
    void process(const MeasurementBatch& batch);  // one batch, strictly in time order

    struct Snapshot {
        std::uint64_t timestampMs;
        std::vector<Track> tracks;
        int nextId;
        MeasurementBatch batch;  // the batch that was applied on top of this snapshot
    };

    TrackerConfig cfg_;
    std::vector<Track> tracks_;
    int nextId_ = 1;
    std::deque<Snapshot> replay_;  // time-ordered, within replayWindowMs of the newest batch
    std::uint64_t droppedLate_ = 0;
};

}  // namespace ar_drive_assist
