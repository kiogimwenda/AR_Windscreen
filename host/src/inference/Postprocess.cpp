#include "ar_drive_assist/inference/Postprocess.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ar_drive_assist {

int32_t mapCocoClass(int coco) {
    // COCO class indices as used by ultralytics (0-based).
    switch (coco) {
        case 0:  // person
            return static_cast<int32_t>(ObjectClass::Pedestrian);
        case 1:  // bicycle
        case 3:  // motorcycle: a two-wheeler with an exposed rider (boda-boda), grouped with
                 // bicycles
            return static_cast<int32_t>(ObjectClass::Cyclist);
        case 2:  // car
        case 5:  // bus
        case 7:  // truck
            return static_cast<int32_t>(ObjectClass::Vehicle);
        case 9:   // traffic light
        case 11:  // stop sign
            return static_cast<int32_t>(ObjectClass::Sign);
        // COCO has no generic "obstacle". Large animals are the obstacles COCO CAN see, and on
        // Kenyan roads livestock and wildlife are real hazards.
        case 16:  // dog
        case 17:  // horse
        case 18:  // sheep
        case 19:  // cow
        case 20:  // elephant
        case 21:  // bear
        case 22:  // zebra
        case 23:  // giraffe
            return static_cast<int32_t>(ObjectClass::Obstacle);
        default:
            return -1;
    }
}

InputMapping InputMapping::letterbox(int srcW, int srcH, int dstW, int dstH) {
    const float s = std::min(static_cast<float>(dstW) / srcW, static_cast<float>(dstH) / srcH);
    InputMapping m;
    m.scaleX = m.scaleY = s;
    m.offsetX = (dstW - srcW * s) / 2.0f;
    m.offsetY = (dstH - srcH * s) / 2.0f;
    return m;
}

float iou(const Box& a, const Box& b) {
    const float ix = std::max(0.0f, std::min(a.x + a.w, b.x + b.w) - std::max(a.x, b.x));
    const float iy = std::max(0.0f, std::min(a.y + a.h, b.y + b.h) - std::max(a.y, b.y));
    const float inter = ix * iy;
    const float uni = a.w * a.h + b.w * b.h - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

std::vector<Box> decodeYolo(const float* out, int numClasses, int numCandidates,
                            const InputMapping& mapping, int srcW, int srcH,
                            const YoloParams& params) {
    std::vector<Box> candidates;
    for (int i = 0; i < numCandidates; ++i) {
        // Best class for this candidate AFTER mapping: several COCO classes feed one ObjectClass,
        // and only mapped classes count.
        float best = params.confThreshold;
        int32_t bestClass = -1;
        for (int c = 0; c < numClasses; ++c) {
            const float score = out[(4 + c) * numCandidates + i];
            if (score < best) continue;
            const int32_t mapped = mapCocoClass(c);
            if (mapped < 0) continue;
            best = score;
            bestClass = mapped;
        }
        if (bestClass < 0) continue;

        const float cx = out[0 * numCandidates + i], cy = out[1 * numCandidates + i];
        const float w = out[2 * numCandidates + i], h = out[3 * numCandidates + i];
        float x0 = mapping.toSourceX(cx - w / 2), y0 = mapping.toSourceY(cy - h / 2);
        float x1 = mapping.toSourceX(cx + w / 2), y1 = mapping.toSourceY(cy + h / 2);
        // Clip: a box that runs into the letterbox padding would otherwise extend off-frame.
        x0 = std::clamp(x0, 0.0f, static_cast<float>(srcW));
        x1 = std::clamp(x1, 0.0f, static_cast<float>(srcW));
        y0 = std::clamp(y0, 0.0f, static_cast<float>(srcH));
        y1 = std::clamp(y1, 0.0f, static_cast<float>(srcH));
        if (x1 <= x0 || y1 <= y0) continue;
        candidates.push_back({x0, y0, x1 - x0, y1 - y0, bestClass, best, i});
    }

    // Non-maximum suppression, per class. The detector fires on the same object from several
    // neighbouring candidates. Keep the most confident box, drop the same-class boxes that
    // overlap it by more than iouThreshold, and repeat. Different classes never suppress each
    // other, so a pedestrian in front of a car survives. Nothing here ever removes a pedestrian
    // because it overlaps another class: a missed pedestrian is the unsafe direction.
    std::sort(candidates.begin(), candidates.end(),
              [](const Box& a, const Box& b) { return a.confidence > b.confidence; });
    std::vector<Box> kept;
    std::vector<bool> removed(candidates.size(), false);
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (removed[i]) continue;
        kept.push_back(candidates[i]);
        if (static_cast<int>(kept.size()) >= params.maxDetections) break;
        for (std::size_t j = i + 1; j < candidates.size(); ++j) {
            if (!removed[j] && candidates[j].classId == candidates[i].classId &&
                iou(candidates[i], candidates[j]) > params.iouThreshold) {
                removed[j] = true;
            }
        }
    }
    return kept;
}

int ObjectMask::area() const {
    int n = 0;
    for (uint8_t b : bits) n += b;
    return n;
}

bool ObjectMask::atGrid(int gx, int gy) const {
    if (gx < x || gy < y || gx >= x + w || gy >= y + h) return false;
    return bits[static_cast<size_t>(gy - y) * w + (gx - x)] != 0;
}

void maskCoefficients(const float* out, int numClasses, int numCoeffs, int numCandidates,
                      const Box& box, float* coeffsOut) {
    for (int k = 0; k < numCoeffs; ++k) {
        coeffsOut[k] = out[static_cast<size_t>(4 + numClasses + k) * numCandidates + box.candidate];
    }
}

ObjectMask decodeMask(const float* protos, int numCoeffs, int protoW, int protoH,
                      const float* coeffs, const Box& box, const InputMapping& mapping,
                      int stride) {
    // Box corners: source -> model input -> prototype grid. Prototype pixel (gx, gy) covers model
    // pixels [gx*stride, (gx+1)*stride). A grid pixel belongs to the crop if its index lies inside
    // the box's grid-space extent, the same rule as ultralytics' crop_mask.
    const float gx0 = (box.x * mapping.scaleX + mapping.offsetX) / stride;
    const float gy0 = (box.y * mapping.scaleY + mapping.offsetY) / stride;
    const float gx1 = ((box.x + box.w) * mapping.scaleX + mapping.offsetX) / stride;
    const float gy1 = ((box.y + box.h) * mapping.scaleY + mapping.offsetY) / stride;

    ObjectMask m;
    m.x = std::clamp(static_cast<int>(std::floor(gx0)), 0, protoW);
    m.y = std::clamp(static_cast<int>(std::floor(gy0)), 0, protoH);
    const int xe = std::clamp(static_cast<int>(std::ceil(gx1)), 0, protoW);
    const int ye = std::clamp(static_cast<int>(std::ceil(gy1)), 0, protoH);
    m.w = std::max(0, xe - m.x);
    m.h = std::max(0, ye - m.y);
    m.bits.assign(static_cast<size_t>(m.w) * m.h, 0);

    const size_t plane = static_cast<size_t>(protoW) * protoH;
    for (int gy = m.y; gy < ye; ++gy) {
        for (int gx = m.x; gx < xe; ++gx) {
            const size_t px = static_cast<size_t>(gy) * protoW + gx;
            float v = 0.0f;
            for (int k = 0; k < numCoeffs; ++k) v += coeffs[k] * protos[k * plane + px];
            // sigmoid(v) > 0.5  <=>  v > 0 (see the header)
            m.bits[static_cast<size_t>(gy - m.y) * m.w + (gx - m.x)] = v > 0.0f ? 1 : 0;
        }
    }
    return m;
}

bool maskContains(const ObjectMask& mask, const InputMapping& mapping, int stride, float srcX,
                  float srcY) {
    const float gx = (srcX * mapping.scaleX + mapping.offsetX) / stride;
    const float gy = (srcY * mapping.scaleY + mapping.offsetY) / stride;
    return mask.atGrid(static_cast<int>(std::floor(gx)), static_cast<int>(std::floor(gy)));
}

namespace {

// Soft arg-max over a window around the hard arg-max, as pred2coords does. `at(g)` is the logit
// of grid cell g. Returns the refined cell position (in cells, + 0.5 as upstream).
template <typename At>
float refinedPosition(At at, int numGrid, int localWidth) {
    int best = 0;
    for (int g = 1; g < numGrid; ++g) {
        if (at(g) > at(best)) best = g;
    }
    const int lo = std::max(0, best - localWidth);
    const int hi = std::min(numGrid - 1, best + localWidth);
    // Softmax, subtracting the max first: exp() of a large logit overflows float otherwise.
    float maxLogit = at(lo);
    for (int g = lo + 1; g <= hi; ++g) maxLogit = std::max(maxLogit, at(g));
    float sum = 0.0f, weighted = 0.0f;
    for (int g = lo; g <= hi; ++g) {
        const float e = std::exp(at(g) - maxLogit);
        sum += e;
        weighted += e * static_cast<float>(g);
    }
    return weighted / sum + 0.5f;
}

}  // namespace

Lanes decodeUfld(const float* locRow, const float* locCol, const float* existRow,
                 const float* existCol, const LaneBand& band, const UfldParams& p) {
    Lanes lanes;
    const int L = p.numLanes;

    // Tensor indexing, all batch 1:
    //   loc_row[g][k][lane]   = locRow[(g * numClsRow + k) * L + lane]
    //   exist_row[e][k][lane] = existRow[(e * numClsRow + k) * L + lane], e: 0 = absent, 1 =
    //   present
    // and the same with numClsCol for the column tensors.
    auto rowPresent = [&](int k, int lane) {
        return existRow[(1 * p.numClsRow + k) * L + lane] >
               existRow[(0 * p.numClsRow + k) * L + lane];
    };
    auto colPresent = [&](int k, int lane) {
        return existCol[(1 * p.numClsCol + k) * L + lane] >
               existCol[(0 * p.numClsCol + k) * L + lane];
    };

    // Ego-lane boundaries (1, 2): one x per row anchor. The lane counts only if present at MORE
    // than half the anchors (pred2coords' rule), which suppresses fragmentary false lanes.
    for (int lane : {1, 2}) {
        int present = 0;
        for (int k = 0; k < p.numClsRow; ++k) present += rowPresent(k, lane);
        if (present <= p.numClsRow / 2) continue;
        for (int k = 0; k < p.numClsRow; ++k) {
            if (!rowPresent(k, lane)) continue;
            const float cell =
                refinedPosition([&](int g) { return locRow[(g * p.numClsRow + k) * L + lane]; },
                                p.numGridRow, p.localWidth);
            const float rowAnchor =
                p.rowAnchorStart + (1.0f - p.rowAnchorStart) * k / (p.numClsRow - 1);
            lanes[lane].push_back(
                {band.x + cell / (p.numGridRow - 1) * band.w, band.y + rowAnchor * band.h});
        }
    }

    // Outer lanes (0, 3): one y per column anchor, and a looser presence rule (> 1/4 of the
    // anchors), matching pred2coords.
    for (int lane : {0, 3}) {
        int present = 0;
        for (int k = 0; k < p.numClsCol; ++k) present += colPresent(k, lane);
        if (present <= p.numClsCol / 4) continue;
        for (int k = 0; k < p.numClsCol; ++k) {
            if (!colPresent(k, lane)) continue;
            const float cell =
                refinedPosition([&](int g) { return locCol[(g * p.numClsCol + k) * L + lane]; },
                                p.numGridCol, p.localWidth);
            const float colAnchor = static_cast<float>(k) / (p.numClsCol - 1);
            lanes[lane].push_back(
                {band.x + colAnchor * band.w, band.y + cell / (p.numGridCol - 1) * band.h});
        }
    }
    return lanes;
}

}  // namespace ar_drive_assist
