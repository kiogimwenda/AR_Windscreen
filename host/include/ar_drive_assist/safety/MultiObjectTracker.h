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
// Starting a track (two-point initiation). A brand-new track has seen one position and has NO
// velocity: the filter's speed prior is about an arbitrary heading, so its predicted-position
// covariance is small and lopsided. Gating the second sighting with it rejected any object that
// moved more than ~0.5 m per frame: a car at 20 m/s (2 m per LiDAR frame) started a new track
// every frame and was never confirmed (found by the Phase 10 arbiter tests). Until the velocity
// is known, the second sighting is gated instead by the class's fastest plausible speed, in ANY
// direction: S = P_pos + R + (v_max dt / 3)^2 I. After two-point initialisation, normal gating.
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
    // True when the position comes from LiDAR points (SceneReconstruction RangeSource::LIDAR).
    // False for a MiDaS-scaled ESTIMATE: such a measurement may move the track, but it never
    // counts as a measured range for braking (Part 8.3 rule 5, Part 9.3 rule 1).
    bool rangeMeasured = true;
};

struct EgoPose {  // from SensorFusion: world position (m) and heading (rad, CCW from east)
    double x = 0, y = 0, psi = 0;
};

struct MeasurementBatch {
    std::uint64_t timestampMs = 0;
    EgoPose ego;
    std::vector<ObjectMeasurement> measurements;
    // False: nothing was LOOKED AT this cycle (the fusion had no usable LiDAR scan for the frame),
    // so an empty batch is no evidence that anything is gone. The tracks are only predicted: no
    // miss, no change of state, the measured run unbroken; a track not updated for maxCoastS
    // still retires. (KITTI replay with timing jitter, 2026-10-07: one frame in three without a
    // scan broke every track's measured run, and a stopped car in the lane was never braked for.)
    bool observed = true;
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
    // Fastest plausible speed per class (m/s). Gates a track's SECOND sighting, before its
    // velocity is known (see process(), step 3). Missing classes use kDefaultMaxSpeedMps.
    std::map<int32_t, double> maxSpeedByClass;
    double maxSpeedFor(int32_t cls) const;
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
    std::uint64_t lastMeasuredRangeMs = 0;  // last update from a LiDAR-measured range (0 = never)
    bool velocityKnown = false;             // set by two-point initialisation (second sighting)
    // Start of the current UNBROKEN run of updates from LiDAR-measured ranges (0 = none). A miss,
    // or an update from an estimate, ends the run. DecisionArbiter rule 1 brakes only on a run of
    // brake_min_measured_track_ms: a track that coasts and is then re-associated to some other
    // cluster (a ghost) starts again from zero (KITTI replay, 2026-10-07). Tolerating one missed
    // frame was tried and rejected: it let a chain of fragments (born at 28 m, "closing" at
    // 31 m/s in a city street) through to a false brake; the strict rule costs at most one
    // 200 ms run after a missed association in a crowded scene.
    std::uint64_t measuredRunSinceMs = 0;
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
