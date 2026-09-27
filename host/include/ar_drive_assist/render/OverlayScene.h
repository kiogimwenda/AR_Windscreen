#pragma once
// OverlayScene — WHAT the driver display draws, as data. See docs/BUILD_GUIDE.md Part 10.1
// (amended 2026-09-25: GPU overlays) and docs/architecture/ar-overlay-design.md.
//
// ArRenderer builds one of these per frame; a DisplaySink draws it on top of the video. Keeping it
// as plain data means the decisions (what to show, where, in which colour) are unit-tested without
// a GPU, and a future ProjectorSink can draw the same scene through different optics.
//
// Every item has its geometry in ONE of three spaces:
//   ROAD   metres, vehicle frame (x forward, y left, z up, origin on the ground under the rear
//          axle). The GPU projects these in the vertex shader with the SAME camera model the
//          perception side uses (common/Camera.h), so a barrier lands where the detection says
//          the object is. Route band, barriers, rings, zones, lane-line glows, hump markers.
//   IMAGE  camera pixels. Badges, labels, icons, the screen-edge fallback chevron.
//   MASK   a tracked object's segmentation mask (camera pixels): the hazard glow along its outline.
//
// And a style: colour, opacity, glow, shimmer/pulse rates (animated by a time uniform on the GPU),
// and a layer (drawn back to front).
//
// Rule 4 of the overlay design, "never hide a hazard": items with `occludedByHazards` (all
// road-space items) are masked by every hazard's silhouette WHERE THEY LIE BEHIND IT, so a barrier
// beyond a pedestrian appears behind them, never painted across them. Hazard masks, with their
// distances, are listed in `occluders`.

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

namespace ar_drive_assist {

struct Rgba {
    float r = 1, g = 1, b = 1, a = 1;
};

// The overlay design's one colour language (§0): green fine, amber caution, red danger; white is
// neutral information only.
namespace palette {
constexpr Rgba kGreen{0.20f, 0.90f, 0.40f, 1};
constexpr Rgba kAmber{1.00f, 0.72f, 0.10f, 1};
constexpr Rgba kRed{1.00f, 0.18f, 0.15f, 1};
constexpr Rgba kWhite{1, 1, 1, 1};
constexpr Rgba kSchool{1.00f, 0.90f, 0.10f, 1};
// Continuous amber (r = 0.4) -> red (r = 1) for the hazard risk level (§3.1).
Rgba forRisk(double r);
}  // namespace palette

struct OverlayStyle {
    Rgba color;
    float opacity = 1.0f;
    float glowPx = 0.0f;     // soft halo width around the shape, in output pixels
    float shimmerHz = 0.0f;  // moving light band (hazard glow); 0 = none. Capped at 3 Hz (§3.2)
    float pulseHz = 0.0f;    // whole-item brightness pulse; 0 = none. Capped at 3 Hz
    int layer = 0;           // draw order, low first
};

enum class OverlayKind : std::uint8_t {
    ROAD_BAND,      // road: a polyline with a width, lying on the road (route, lane-line glow)
    ROAD_BARRIER,   // road: a vertical wall standing on the road, across `points[0]`-`points[1]`
    ROAD_RING,      // road: a ring on the ground at `points[0]`, radius `widthM`
    ROAD_ZONE,      // road: a filled region on the road (polygon `points`, e.g. tailgating gap)
    ROAD_MARKER,    // road: a chevron/hump marker lying across the lane at `points[0]`
    IMAGE_LABEL,    // image: text in a rounded badge at `imagePos`
    IMAGE_BADGE,    // image: a sign-shaped badge (speed limit) at `imagePos`
    IMAGE_CHEVRON,  // image: screen-edge chevron pointing towards `imagePos` (off-screen hazard)
    MASK_GLOW,      // mask: shimmering rim glow along an object's outline
    ELLIPSE_GLOW,   // image: glow fallback without a mask: soft ellipse inside `box` (rule 6)
};

// A binary mask in camera pixels: `bits` covers the rectangle (x, y, w, h) at `cellPx` camera
// pixels per cell (YOLOv8-seg masks are quarter resolution: 4).
struct PixelMask {
    int x = 0, y = 0, w = 0, h = 0;  // in cells
    int cellPx = 4;
    std::vector<std::uint8_t> bits;  // w * h, row-major, 1 = object
    bool at(int cx, int cy) const {
        if (cx < x || cy < y || cx >= x + w || cy >= y + h) return false;
        return bits[static_cast<std::size_t>(cy - y) * w + (cx - x)] != 0;
    }
};

// A hazard's silhouette with its distance (camera depth, metres). Road items are hidden only where
// they lie BEHIND the hazard: a barrier in front of a car stays visible (first version masked by
// silhouette alone and hid a barrier at 28 m behind the car at 30 m it was warning about).
struct HazardOccluder {
    PixelMask mask;
    double depthM = 0;
};

struct OverlayItem {
    OverlayKind kind = OverlayKind::ROAD_BAND;
    OverlayStyle style;
    // Road space (vehicle frame, metres). Bands: centre line; barrier: its two ends on the ground;
    // ring/marker: centre; zone: polygon outline.
    std::vector<Eigen::Vector3d> points;
    std::vector<float>
        pointOpacity;              // per point (bands): e.g. far-field flat-ground points fainter
    std::vector<Rgba> pointColor;  // per point (bands): speed grading along the route line
    float widthM = 0.5f;           // band width / ring radius / marker width
    float heightM = 1.0f;          // barrier height
    // Image space.
    Eigen::Vector2d imagePos = Eigen::Vector2d::Zero();
    Eigen::Vector4d box = Eigen::Vector4d::Zero();  // x, y, w, h (ellipse fallback)
    std::string text;
    float sizePx = 48;
    // Mask space.
    PixelMask mask;
    bool occludedByHazards = false;  // rule 4
    int sourceId = -1;               // track or sign id, for tests and debugging
};

// A soft ellipse inside `box` (camera pixels x, y, w, h): the glow fallback without a mask.
PixelMask ellipseMaskFor(const Eigen::Vector4d& box);

struct OverlayScene {
    std::uint64_t timestampMs = 0;          // the camera frame this scene belongs to
    std::vector<OverlayItem> items;         // drawn by layer, then in order
    std::vector<HazardOccluder> occluders;  // hazard silhouettes that road items must not cover
    // Items of a kind, for tests.
    std::size_t count(OverlayKind k) const {
        std::size_t n = 0;
        for (const auto& i : items) n += i.kind == k;
        return n;
    }
};

}  // namespace ar_drive_assist
