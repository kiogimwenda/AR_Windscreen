#pragma once
// ExtrinsicMonitor — startup verification and bounded refinement of the LiDAR-camera extrinsics,
// and continuous monitoring. See docs/BUILD_GUIDE.md Part 12.2.1.
//
// ---------------------------------------------------------------------------------------------
// The score: edge alignment (Levinson & Thrun's idea). Where the LiDAR's range jumps (the near side
// of an object's outline against what is behind it), the camera image almost always has an edge.
// Each frame:
//   - the image's edges (Canny), and per pixel the distance to the nearest edge, turned into a
//     closeness map exp(-d^2 / 2 sigma^2) (1 on an edge, ~0 a few sigma away);
//   - the scan's points, brought to the frame's time (SceneReconstruction::compensate) and
//     projected with the current extrinsics; a "depth edge" is a pair: a visible point and the
//     visible point in an adjacent image cell more than depthJumpM behind it. The outline lies
//     between them, so the pair is scored at the midpoint of their projections (scoring the
//     near points alone biased the score: they all sit inside the outline).
// A candidate transform's score is the mean closeness at its projections of the pairs' midpoints,
// closeness taken relative to its local mean (so dense texture earns nothing by itself).
// It needs no objects in view: buildings, poles, parked cars and trees all have outlines.
// (The guide's second score, mask agreement, is not used: it depends on the detector's masks
// being right, and the edges were enough on the KITTI drive. See the progress log.)
//
// The state machine (Part 12.2.1):
//   UNVERIFIED  until `verifyFrames` frames of good evidence have arrived.
//   VERIFIED    the baseline is the best within the search bounds: no candidate beat it by
//               `minGain` on both halves of the evidence, and a wider look (wideScale x the
//               bounds, coarse) found no clearly better alignment outside them either.
//   REFINED     a correction (|rotation| <= maxRotDeg, |translation| <= maxTransM per axis) beat
//               the baseline by `minGain` on the frames it was fitted to AND on the others (held
//               out), and is not pressed against the bounds.
//   DEGRADED    a clearly better alignment lies beyond the bounds (the wider look found it on the
//               coarse map, and it held after polishing on the fine map, on the fit AND held-out
//               frames: the real error is larger than a refinement may fix),
//               or no usable evidence for `maxUnverifiedFrames` frames. Latched for the session:
//               the baseline is kept, a FAULT is logged, the driver is told to recalibrate, and
//               fusion stops using camera masks for ranging (FusionThread).
// Noise control: each axis of a fitted correction is kept only as far as the HELD-OUT frames
// support it (pulled back to the smallest fraction of itself that keeps the held-out score within
// 1 %), so a weakly constrained axis (roll, on a wide short image) cannot drift to a bound on
// noise. Errors beyond wideScale x the bounds are out of the score's reach and are not detected
// here; Part 12.2.2's knock check (the IMU's tilt at rest) covers a mount knocked that far.
// Every check repeats every `recheckFrames` frames of evidence on recent frames, from the
// CURRENT transform, so a knocked mount turns into DEGRADED within one window (at the bounds) or
// two (the peak gone). The search runs coarse to fine (closeness maps at coarseScale x sigma, then
// sigma), so it finds the peak from up to about the bound away. Each session starts
// from the baseline again: nothing accumulates across sessions.
//
// Evidence: a frame counts only with at least `minEdgePoints` depth edges, spread over 3 of 4
// columns of the image (yaw, roll) and 2 of 3 bands of its height (pitch), and a refinement needs
// the window to have depth edges both near
// (< 15 m) and far (> 25 m), since near points pin translation and far points rotation. A
// featureless scene (an empty road, a wall) never produces REFINED.
//
// Never actuation-critical: braking depends on LiDAR ranges in the vehicle frame (Part 9.3), not
// on this. Not thread-safe except current(); feed it from one thread.

#include <cstdint>
#include <deque>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

#include "ar_drive_assist/common/Camera.h"
#include "ar_drive_assist/lidar/LidarFrame.h"
#include "ar_drive_assist/scene/SceneReconstruction.h"

namespace ar_drive_assist {

class EventLog;

enum class ExtrinsicStatus { UNVERIFIED, VERIFIED, REFINED, DEGRADED };
const char* toString(ExtrinsicStatus s);

struct ExtrinsicState {
    Eigen::Isometry3d cameraFromLidar = Eigen::Isometry3d::Identity();
    ExtrinsicStatus status = ExtrinsicStatus::UNVERIFIED;
    double score = 0;  // the current transform's score on the last window (0-1)
    std::string reason;
};

struct ExtrinsicMonitorConfig {
    // The refinement's bounds, per axis. Translation is NOT refined by default (0): the score
    // hardly changes with a few centimetres (flat on KITTI, where a degree of rotation moved it
    // 10-50 %), so a translation "refinement" would only follow noise; the board calibration's
    // (Part 12.2) stands. Part 12.2.1 allows ~3 cm; set it to try.
    double maxRotDeg = 1.0, maxTransM = 0.0;
    double boundFraction = 0.9;  // a correction beyond this fraction of a bound is "at the bound"
    double minGain = 0.03;       // relative score gain to accept a correction
    double edgeSigmaPx = 2.0;    // closeness falloff, at the image's resolution
    int localWindowPx = 15;      // closeness is relative to its mean over this half-window (min)
    double coarseScale = 4.0;    // the coarse map's sigma, x edgeSigmaPx: the search's reach
    double wideScale = 3.0;      // "beyond the bounds" is looked for this many bounds out
    double depthJumpM = 1.0;     // a depth edge: a neighbour this much nearer or farther
    int cellPx = 4;              // the projection's z-buffer cell
    int edgeRadiusCells = 1;     // how far to look for that neighbour
    int minEdgePoints = 150;     // per frame, to count as evidence
    int verifyFrames = 12;       // evidence frames for the first check
    int recheckFrames = 24;      // evidence frames per later check (half fit, half held out)
    int maxUnverifiedFrames = 600;  // frames offered without reaching a verdict -> DEGRADED
    double maxRangeM = 60.0;
    // Evidence only while the car is not turning: an error in the scan's timing (a LiDAR sweep
    // stamped with one time; a replay's pairing) is, during a turn, a rotation of the whole cloud,
    // which a refinement would take for misalignment (KITTI replayed in real time: 0.97 deg of
    // "yaw correction" from a 0.1 deg truth). At 3 deg/s, 100 ms of timing error is 0.3 deg.
    double maxYawRateDegS = 3.0;
};

class ExtrinsicMonitor {
public:
    // `vehicleFromLidar` is used only to bring points to the frame's time (the vehicle's motion).
    ExtrinsicMonitor(const Eigen::Isometry3d& baselineCameraFromLidar,
                     const Eigen::Isometry3d& vehicleFromLidar, const CameraModel& camera,
                     ExtrinsicMonitorConfig cfg = {}, EventLog* log = nullptr);

    // One camera frame (undistorted, as `camera` describes it) and the scan nearest it.
    void addFrame(const cv::Mat& image, std::int64_t frameTimeUs,
                  const std::vector<LidarPoint>& scan, const EgoMotion& ego);

    ExtrinsicState current() const;  // any thread

    // --- exposed for the tests ---
    struct Evidence {
        cv::Mat closeness[2];  // CV_32F, image size: [0] coarse, [1] fine
        // Depth edges: (near point, the point behind it across the outline), LiDAR frame, at the
        // frame's time.
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
        bool usable = false;
        double nearFraction = 0, farFraction = 0;
    };
    Evidence prepare(const cv::Mat& image, std::int64_t frameTimeUs,
                     const std::vector<LidarPoint>& scan, const EgoMotion& ego) const;
    double score(const std::vector<const Evidence*>& frames, const Eigen::Isometry3d& T,
                 int level = 1) const;  // level 0 coarse, 1 fine
    // The best correction of the baseline within the bounds (x boundScale), starting from T: a
    // grid over the rotations, then coordinate search on the coarse map and (fine) the fine one.
    Eigen::Isometry3d search(const std::vector<const Evidence*>& frames, const Eigen::Isometry3d& T,
                             double& bestScore, bool& atBound, double boundScale = 1.0,
                             bool fine = true, bool grid = true) const;
    std::uint64_t evidenceFrames() const { return evidenceCount_; }

private:
    void check();
    void setState(ExtrinsicStatus s, const Eigen::Isometry3d& T, double score,
                  const std::string& reason);

    Eigen::Isometry3d baseline_, vehicleFromLidar_;
    CameraModel camera_;
    ExtrinsicMonitorConfig cfg_;
    EventLog* log_;

    std::deque<Evidence> window_;
    std::uint64_t offered_ = 0, evidenceCount_ = 0, sinceCheck_ = 0;
    Eigen::Isometry3d currentT_;

    mutable std::mutex m_;
    ExtrinsicState state_;
};

}  // namespace ar_drive_assist
