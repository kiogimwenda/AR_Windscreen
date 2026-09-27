#include "ar_drive_assist/render/OverlayScene.h"

#include <algorithm>

namespace ar_drive_assist::palette {

Rgba forRisk(double r) {
    // Overlay design §3.1: amber at r = 0.4, red at r = 1, continuously between.
    const float f = static_cast<float>(std::clamp((r - 0.4) / 0.6, 0.0, 1.0));
    return {kAmber.r + f * (kRed.r - kAmber.r), kAmber.g + f * (kRed.g - kAmber.g),
            kAmber.b + f * (kRed.b - kAmber.b), 1};
}

}  // namespace ar_drive_assist::palette

namespace ar_drive_assist {

// Rule 6's glow fallback without a segmentation mask: a soft ellipse a little inside the box
// (boxes are loose around objects). Quarter-resolution cells, like YOLOv8-seg masks.
PixelMask ellipseMaskFor(const Eigen::Vector4d& box) {
    PixelMask m;
    m.cellPx = 4;
    m.x = static_cast<int>(box[0] / 4);
    m.y = static_cast<int>(box[1] / 4);
    m.w = std::max(1, static_cast<int>(box[2] / 4));
    m.h = std::max(1, static_cast<int>(box[3] / 4));
    m.bits.assign(static_cast<std::size_t>(m.w) * m.h, 0);
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x) {
            const double u = (x + 0.5) / m.w * 2 - 1, v = (y + 0.5) / m.h * 2 - 1;
            if (u * u / 0.8 + v * v / 0.9 <= 1) m.bits[static_cast<std::size_t>(y) * m.w + x] = 1;
        }
    return m;
}

}  // namespace ar_drive_assist
