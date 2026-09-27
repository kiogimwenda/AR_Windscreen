#pragma once
// ArRenderer — decides WHAT the driver sees; builds the OverlayScene each frame and draws nothing
// itself. See docs/BUILD_GUIDE.md Part 10.3 and docs/architecture/ar-overlay-design.md.
//
// ---------------------------------------------------------------------------------------------
// build() is pure scene logic (plus the SignTracker's memory), so every behaviour below is
// unit-tested without a GPU. The sink draws the result.
//
// Navigation (Part 11.4, design §2 speed grading):
//   - The route band from RoadSurfaceProjector's vehicle-frame points. Points on the MEASURED road
//     surface are drawn at full strength; flat-ground fallback points (beyond LiDAR range) at
//     `farOpacity`: the honest cue that precision falls off with distance (Part 10.3).
//   - Moved by the car's own motion between the projector's pose and this video frame's capture
//     time (Part 10.3's latency compensation, read correctly for video see-through: the target is
//     the displayed frame's capture time, see decisions.md, Phase 9).
//   - Colour: speed vs the limit (green; amber up to +10%; red beyond), and, up to a STOP TARGET
//     (a stop sign, or a forward-collision hazard), the deceleration needed to stop there,
//     a = v^2 / 2d: amber above `easeOffG`, red and pulsing above `comfortG`. The same for an
//     upcoming LOWER speed limit, with a = (v^2 - v_limit^2) / 2d. Beyond a stop target the band
//     fades: the barrier ends it.
//   - The next turn, as a label.
// Hazards (§3; never decluttered, rule 5). Only tracks with a hazard flag or a risk level of at
// least MEDIUM; ordinary traffic gets nothing (rule 8).
//   - The glow on the object's mask (or a soft ellipse in its box without one, rule 6), coloured
//     by risk r, shimmering faster as r rises (at most 3 Hz). The silhouette and its depth are
//     registered as an occluder (rule 4).
//   - Forward collision in the ego path: a barrier across the lane at the object, with the gap.
//   - Pedestrian, cyclist or animal in or entering the path: a ring on the ground at its feet.
//   - Tailgating: the following-distance zone between the bumper and the lead vehicle.
//   - Swerving / erratic neighbour: the lane line on its side glows red.
//   - No measured position: glow only, never a road element that might float (rule 6). Off the
//     screen: a chevron at the edge, pointing towards it.
// Signs (§2, via SignTracker). At most `maxSignItems` at once, by priority: stop / no entry >
// speed > turn restrictions > advisories > information (rule 5). Road-placed where a measured
// position exists; otherwise, and for behaviours needing junction geometry from the map (no-turn
// barriers, roundabout exits), a banner (rule 6).
// HUD: the speed-limit badge beside the actual speed, graded; with no speed source, the limit only
// and no grading (never a guessed speed).
// ---------------------------------------------------------------------------------------------

#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "ar_drive_assist/common/Camera.h"
#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/nav/RoadSurfaceProjector.h"
#include "ar_drive_assist/render/OverlayScene.h"
#include "ar_drive_assist/render/SignTracker.h"
#include "ar_drive_assist/safety/RecklessDrivingDetector.h"

namespace ar_drive_assist {

// One tracked object as the renderer needs it.
struct HazardView {
    int trackId = 0;
    int32_t objectClass = -1;
    ThreatAssessment threat;
    std::optional<Eigen::Vector3d> ground;  // vehicle frame, on the road at its base, if measured
    double gapM = -1;                       // bumper gap, if in the path
    double depthM = 0;                      // camera depth, for occlusion
    std::optional<PixelMask> mask;          // segmentation mask (camera pixels)
    Eigen::Vector4d box = Eigen::Vector4d::Zero();  // camera pixels x, y, w, h
};

struct RenderInputs {
    std::uint64_t frameMs = 0;          // capture time of the video frame being displayed
    std::optional<double> egoSpeedMps;  // OBD (GPS fallback); none = no speed source
    double yawRateRadPerS = 0;
    const ProjectedRoute* route = nullptr;
    std::vector<HazardView> hazards;
    const DetectionFrame* lanes = nullptr;  // ego lane boundaries (image pixels)
    std::vector<SignObservation> signs;
    std::optional<int> mapMaxSpeedKph;
    bool roadChanged = false;
};

struct RendererConfig {
    double frontBumperM = 3.5;
    double laneHalfWidthM = 1.6;
    double routeWidthM = 1.0;
    float farOpacity = 0.45f;
    double easeOffG = 0.15, comfortG = 0.25, hardG = 0.4;
    double tailgatingMinGapS = 1.0;
    int maxSignItems = 3;
    double noParkingMaxSpeedMps = 20 / 3.6;
};

class ArRenderer {
public:
    ArRenderer(RendererConfig cfg, CameraModel camera, Eigen::Isometry3d cameraFromVehicle);

    OverlayScene build(const RenderInputs& in);

    const SignTracker& signs() const { return signs_; }

    // A vehicle-frame point at time t0, as seen from the car at t0 + dt (constant speed and yaw
    // rate). Used to move the route from the projector's pose time to the frame's capture time.
    static Eigen::Vector3d egoShift(const Eigen::Vector3d& p, double speedMps, double yawRate,
                                    double dt);

private:
    RendererConfig cfg_;
    CameraModel camera_;
    Eigen::Isometry3d cameraFromVehicle_;
    SignTracker signs_;
};

}  // namespace ar_drive_assist
