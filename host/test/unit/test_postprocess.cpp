// Model-output decoding tests — see docs/BUILD_GUIDE.md Part 7.4, and Postprocess.h.
//
// Every test builds a synthetic output tensor whose correct decoding is known by hand, so these
// run without a GPU or any model. Agreement with the real models on real frames is checked
// separately, visually, with the Phase 5 debug viewer.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ar_drive_assist/inference/Postprocess.h"

using namespace ar_drive_assist;

namespace {

constexpr int kClasses = 80;
constexpr int kCoco_person = 0, kCoco_bicycle = 1, kCoco_car = 2, kCoco_motorcycle = 3,
              kCoco_truck = 7, kCoco_cow = 19, kCoco_cup = 41;

// A (4 + 80, n) YOLOv8 output tensor, channel-major, all zeros until candidates are set.
struct YoloTensor {
    int n;
    std::vector<float> data;
    explicit YoloTensor(int n_) : n(n_), data(static_cast<size_t>((4 + kClasses) * n_), 0.0f) {}
    void set(int i, float cx, float cy, float w, float h, int cocoClass, float score) {
        data[0 * n + i] = cx;
        data[1 * n + i] = cy;
        data[2 * n + i] = w;
        data[3 * n + i] = h;
        data[(4 + cocoClass) * n + i] = score;
    }
};

const InputMapping kIdentity{};

}  // namespace

// --- class mapping ---------------------------------------------------------------------------

TEST(MapCocoClass, MapsRoadRelevantClasses) {
    EXPECT_EQ(mapCocoClass(kCoco_person), int32_t(ObjectClass::Pedestrian));
    EXPECT_EQ(mapCocoClass(kCoco_bicycle), int32_t(ObjectClass::Cyclist));
    EXPECT_EQ(mapCocoClass(kCoco_motorcycle), int32_t(ObjectClass::Cyclist));
    EXPECT_EQ(mapCocoClass(kCoco_car), int32_t(ObjectClass::Vehicle));
    EXPECT_EQ(mapCocoClass(5), int32_t(ObjectClass::Vehicle));  // bus
    EXPECT_EQ(mapCocoClass(kCoco_truck), int32_t(ObjectClass::Vehicle));
    EXPECT_EQ(mapCocoClass(9), int32_t(ObjectClass::Sign));   // traffic light
    EXPECT_EQ(mapCocoClass(11), int32_t(ObjectClass::Sign));  // stop sign
    for (int animal = 16; animal <= 23; ++animal) {
        EXPECT_EQ(mapCocoClass(animal), int32_t(ObjectClass::Obstacle)) << animal;
    }
    EXPECT_EQ(mapCocoClass(kCoco_cup), -1);
    EXPECT_EQ(mapCocoClass(14), -1);  // bird: too small and too mobile to be a braking obstacle
}

// --- letterbox -------------------------------------------------------------------------------

// The production case: a 2560x1440 camera frame into YOLO's 1280x736 input. Scale 0.5 gives
// 1280x720, centred, with 8 px of padding top and bottom.
TEST(InputMapping, LetterboxOfProductionFrame) {
    const auto m = InputMapping::letterbox(2560, 1440, 1280, 736);
    EXPECT_FLOAT_EQ(m.scaleX, 0.5f);
    EXPECT_FLOAT_EQ(m.scaleY, 0.5f);
    EXPECT_FLOAT_EQ(m.offsetX, 0.0f);
    EXPECT_FLOAT_EQ(m.offsetY, 8.0f);
    EXPECT_FLOAT_EQ(m.toSourceX(640.0f), 1280.0f);
    EXPECT_FLOAT_EQ(m.toSourceY(8.0f), 0.0f);  // top of the image content, not of the padding
    EXPECT_FLOAT_EQ(m.toSourceY(728.0f), 1440.0f);
}

// The 1080p fallback that Phase 4 may choose must map correctly too.
TEST(InputMapping, LetterboxOf1080pFrame) {
    const auto m = InputMapping::letterbox(1920, 1080, 1280, 736);
    EXPECT_FLOAT_EQ(m.scaleX, 1280.0f / 1920.0f);
    EXPECT_NEAR(m.offsetY, (736.0f - 720.0f) / 2.0f, 1e-4f);
}

// --- YOLO decoding -------------------------------------------------------------------------------

TEST(DecodeYolo, DecodesOneBoxIntoSourcePixels) {
    YoloTensor t(4);
    t.set(2, /*cx*/ 640, /*cy*/ 368, /*w*/ 100, /*h*/ 200, kCoco_person, 0.9f);
    const auto m = InputMapping::letterbox(2560, 1440, 1280, 736);
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, m, 2560, 1440);
    ASSERT_EQ(boxes.size(), 1u);
    // centre (640, 368) in model pixels -> (1280, 720) in the source; size doubles.
    EXPECT_FLOAT_EQ(boxes[0].x, 1280.0f - 100.0f);
    EXPECT_FLOAT_EQ(boxes[0].y, 720.0f - 200.0f);
    EXPECT_FLOAT_EQ(boxes[0].w, 200.0f);
    EXPECT_FLOAT_EQ(boxes[0].h, 400.0f);
    EXPECT_EQ(boxes[0].classId, int32_t(ObjectClass::Pedestrian));
    EXPECT_FLOAT_EQ(boxes[0].confidence, 0.9f);
}

TEST(DecodeYolo, DropsBelowThresholdAndUnmappedClasses) {
    YoloTensor t(3);
    t.set(0, 100, 100, 50, 50, kCoco_car, 0.2f);   // below 0.25
    t.set(1, 300, 300, 50, 50, kCoco_cup, 0.99f);  // confident, but not a road class
    t.set(2, 500, 500, 50, 50, kCoco_car, 0.3f);   // kept
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, kIdentity, 1000, 1000);
    ASSERT_EQ(boxes.size(), 1u);
    EXPECT_FLOAT_EQ(boxes[0].confidence, 0.3f);
}

// An unmapped class with a higher score must not hide a mapped class on the same candidate.
TEST(DecodeYolo, BestMappedClassWinsOverHigherUnmappedClass) {
    YoloTensor t(1);
    t.set(0, 100, 100, 50, 50, kCoco_cup, 0.95f);
    t.data[(4 + kCoco_car) * t.n + 0] = 0.6f;
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, kIdentity, 1000, 1000);
    ASSERT_EQ(boxes.size(), 1u);
    EXPECT_EQ(boxes[0].classId, int32_t(ObjectClass::Vehicle));
    EXPECT_FLOAT_EQ(boxes[0].confidence, 0.6f);
}

TEST(DecodeYolo, NmsKeepsMostConfidentOfOverlappingSameClassBoxes) {
    YoloTensor t(3);
    t.set(0, 100, 100, 100, 100, kCoco_car, 0.7f);
    t.set(1, 105, 102, 100, 100, kCoco_car, 0.9f);  // same object, stronger
    t.set(2, 600, 600, 100, 100, kCoco_car, 0.8f);  // a different car
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, kIdentity, 1000, 1000);
    ASSERT_EQ(boxes.size(), 2u);
    EXPECT_FLOAT_EQ(boxes[0].confidence, 0.9f);
    EXPECT_FLOAT_EQ(boxes[1].confidence, 0.8f);
}

// "car" and "truck" on the same vehicle are two COCO classes but one ObjectClass. Mapping before
// NMS makes them one Vehicle box, not two.
TEST(DecodeYolo, CarAndTruckOnSameVehicleCollapseToOneBox) {
    YoloTensor t(2);
    t.set(0, 200, 200, 120, 80, kCoco_car, 0.6f);
    t.set(1, 202, 201, 118, 80, kCoco_truck, 0.55f);
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, kIdentity, 1000, 1000);
    ASSERT_EQ(boxes.size(), 1u);
    EXPECT_EQ(boxes[0].classId, int32_t(ObjectClass::Vehicle));
}

// The safety rule in Postprocess.cpp: a pedestrian is never suppressed by another class's box,
// including a cyclist or vehicle box it overlaps heavily.
TEST(DecodeYolo, PedestrianOverlappingOtherClassesIsNeverSuppressed) {
    YoloTensor t(3);
    t.set(0, 300, 300, 60, 160, kCoco_person, 0.4f);
    t.set(1, 300, 310, 70, 140, kCoco_motorcycle, 0.95f);
    t.set(2, 300, 300, 300, 200, kCoco_car, 0.99f);
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, kIdentity, 1000, 1000);
    ASSERT_EQ(boxes.size(), 3u);
    int pedestrians = 0;
    for (const auto& b : boxes) pedestrians += b.classId == int32_t(ObjectClass::Pedestrian);
    EXPECT_EQ(pedestrians, 1);
}

TEST(DecodeYolo, ClipsBoxesRunningIntoThePadding) {
    YoloTensor t(1);
    // Centre near the top edge of the image content, box extending into the 8 px padding.
    t.set(0, 640, 20, 100, 60, kCoco_car, 0.9f);
    const auto m = InputMapping::letterbox(2560, 1440, 1280, 736);
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, m, 2560, 1440);
    ASSERT_EQ(boxes.size(), 1u);
    EXPECT_FLOAT_EQ(boxes[0].y, 0.0f);
    EXPECT_FLOAT_EQ(boxes[0].h, (20 + 30 - 8) * 2.0f);
}

TEST(DecodeYolo, RespectsMaxDetections) {
    YoloTensor t(10);
    for (int i = 0; i < 10; ++i) t.set(i, 50.0f + 90 * i, 50, 40, 40, kCoco_car, 0.5f + 0.01f * i);
    YoloParams p;
    p.maxDetections = 3;
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, kIdentity, 1000, 1000, p);
    ASSERT_EQ(boxes.size(), 3u);
    EXPECT_FLOAT_EQ(boxes[0].confidence, 0.59f);  // the three most confident survive
}

TEST(Iou, KnownValues) {
    const Box a{0, 0, 10, 10}, b{5, 0, 10, 10}, c{20, 20, 5, 5};
    EXPECT_FLOAT_EQ(iou(a, a), 1.0f);
    EXPECT_FLOAT_EQ(iou(a, b), 50.0f / 150.0f);
    EXPECT_FLOAT_EQ(iou(a, c), 0.0f);
}

// --- UFLDv2 lane decoding ------------------------------------------------------------------------

namespace {

// UFLD tensors for the CULane-R18 geometry, with every lane absent until told otherwise.
struct UfldTensors {
    UfldParams p;
    std::vector<float> locRow, locCol, existRow, existCol;
    UfldTensors()
        : locRow(size_t(p.numGridRow * p.numClsRow * p.numLanes), 0.0f),
          locCol(size_t(p.numGridCol * p.numClsCol * p.numLanes), 0.0f),
          existRow(size_t(2 * p.numClsRow * p.numLanes), 0.0f),
          existCol(size_t(2 * p.numClsCol * p.numLanes), 0.0f) {
        for (int k = 0; k < p.numClsRow; ++k)
            for (int l = 0; l < p.numLanes; ++l)
                existRow[(0 * p.numClsRow + k) * p.numLanes + l] = 1;
        for (int k = 0; k < p.numClsCol; ++k)
            for (int l = 0; l < p.numLanes; ++l)
                existCol[(0 * p.numClsCol + k) * p.numLanes + l] = 1;
    }
    // Row-anchor lane `lane` present at anchor k, peaked at grid cell g (neighbours equal).
    void rowLane(int lane, int k, int g) {
        existRow[(1 * p.numClsRow + k) * p.numLanes + lane] = 2;
        locRow[(g * p.numClsRow + k) * p.numLanes + lane] = 20.0f;
    }
    void colLane(int lane, int k, int g) {
        existCol[(1 * p.numClsCol + k) * p.numLanes + lane] = 2;
        locCol[(g * p.numClsCol + k) * p.numLanes + lane] = 20.0f;
    }
    Lanes decode(const LaneBand& band) const {
        return decodeUfld(locRow.data(), locCol.data(), existRow.data(), existCol.data(), band, p);
    }
};

const LaneBand kUnitBand{0, 0, 199, 1};  // so x == refined cell / 199 * 199 == cell

}  // namespace

TEST(DecodeUfld, NothingPresentGivesNoLanes) {
    UfldTensors t;
    for (const auto& lane : t.decode(kUnitBand)) EXPECT_TRUE(lane.empty());
}

// A sharp peak at cell g, with equal (zero) neighbours, refines to g + 0.5 almost exactly: the
// symmetric neighbours cancel, and the peak's softmax weight is ~1 - 2e-9.
TEST(DecodeUfld, EgoLaneAtEveryAnchorDecodesPeakPositionAndAnchorRows) {
    UfldTensors t;
    for (int k = 0; k < t.p.numClsRow; ++k) t.rowLane(1, k, 50);
    const LaneBand band{100, 200, 1990, 1000};
    const auto lanes = t.decode(band);
    ASSERT_EQ(lanes[1].size(), size_t(t.p.numClsRow));
    EXPECT_TRUE(lanes[0].empty() && lanes[2].empty() && lanes[3].empty());

    // x = band.x + (50 + 0.5) / 199 * band.w
    EXPECT_NEAR(lanes[1][0].x, 100 + 50.5f / 199 * 1990, 1e-2);
    // row anchors run linspace(0.42, 1) down the band
    EXPECT_NEAR(lanes[1].front().y, 200 + 0.42f * 1000, 1e-3);
    EXPECT_NEAR(lanes[1].back().y, 200 + 1.0f * 1000, 1e-3);
}

// pred2coords' presence rule: a row-anchor lane needs MORE than half the anchors (> 36 of 72).
TEST(DecodeUfld, RowLaneNeedsMoreThanHalfTheAnchors) {
    UfldTensors exactlyHalf, oneMore;
    for (int k = 0; k < 36; ++k) exactlyHalf.rowLane(2, k, 10);
    for (int k = 0; k < 37; ++k) oneMore.rowLane(2, k, 10);
    EXPECT_TRUE(exactlyHalf.decode(kUnitBand)[2].empty());
    EXPECT_EQ(oneMore.decode(kUnitBand)[2].size(), 37u);
}

// Column-anchor lanes (0, 3) use a looser rule: more than a quarter (> 20 of 81).
TEST(DecodeUfld, ColumnLaneNeedsMoreThanAQuarterOfAnchors) {
    UfldTensors quarter, oneMore;
    for (int k = 0; k < 20; ++k) quarter.colLane(3, k, 40);
    for (int k = 0; k < 21; ++k) oneMore.colLane(3, k, 40);
    EXPECT_TRUE(quarter.decode(kUnitBand)[3].empty());
    ASSERT_EQ(oneMore.decode(kUnitBand)[3].size(), 21u);

    // Column lanes swap the roles: x comes from the anchor, y from the refined cell.
    const LaneBand band{0, 0, 800, 99};
    const auto lane = oneMore.decode(band)[3];
    EXPECT_NEAR(lane[0].x, 0.0f, 1e-4);                     // anchor 0 of linspace(0, 1, 81)
    EXPECT_NEAR(lane[20].x, 20.0f / 80.0f * 800.0f, 1e-3);  // anchor 20
    EXPECT_NEAR(lane[0].y, (40 + 0.5f) / 99 * 99, 1e-2);
}

// The soft arg-max: with the neighbour above the peak also strong, the refined position moves
// towards it. Exactly as far as the softmax weights say.
TEST(DecodeUfld, SoftArgmaxRefinesTowardsStrongNeighbour) {
    UfldTensors t;
    for (int k = 0; k < t.p.numClsRow; ++k) {
        t.rowLane(1, k, 100);
        t.locRow[(101 * t.p.numClsRow + k) * t.p.numLanes + 1] = 20.0f;  // tie with the peak
    }
    const auto lanes = t.decode(kUnitBand);
    ASSERT_FALSE(lanes[1].empty());
    // Window {99, 100, 101} with logits {0, 20, 20}: weights ~{0, .5, .5} -> 100.5, then +0.5.
    EXPECT_NEAR(lanes[1][0].x, 101.0f, 1e-3);
}

// Large logits must not overflow exp() and produce NaN.
TEST(DecodeUfld, LargeLogitsStayFinite) {
    UfldTensors t;
    for (int k = 0; k < t.p.numClsRow; ++k) {
        t.rowLane(1, k, 5);
        t.locRow[(5 * t.p.numClsRow + k) * t.p.numLanes + 1] = 500.0f;
    }
    for (const auto& pt : t.decode(kUnitBand)[1]) {
        ASSERT_TRUE(std::isfinite(pt.x));
    }
}

// --- YOLOv8-seg masks
// ------------------------------------------------------------------------------

namespace {

// Prototypes with K planes of W x H, all zero until set. Plane layout matches output1:
// protos[k * H * W + y * W + x].
struct Protos {
    int K, W, H;
    std::vector<float> data;
    Protos(int k, int w, int h) : K(k), W(w), H(h), data(size_t(k) * w * h, 0.0f) {}
    void fill(int k, int x0, int y0, int x1, int y1, float v) {  // [x0,x1) x [y0,y1)
        for (int y = y0; y < y1; ++y)
            for (int x = x0; x < x1; ++x) data[size_t(k) * W * H + size_t(y) * W + x] = v;
    }
};

// A mapping where model pixels equal source pixels, so grid = source / 4.
const InputMapping kUnit{};

}  // namespace

TEST(MaskCoefficients, ReadsTheCandidatesCoefficientsAfterTheClassScores) {
    constexpr int nC = 3, nK = 4, N = 5;
    std::vector<float> out(size_t(4 + nC + nK) * N, 0.0f);
    for (int k = 0; k < nK; ++k) out[size_t(4 + nC + k) * N + 2] = 10.0f + k;  // candidate 2
    Box b;
    b.candidate = 2;
    float c[nK];
    maskCoefficients(out.data(), nC, nK, N, b, c);
    for (int k = 0; k < nK; ++k) EXPECT_FLOAT_EQ(c[k], 10.0f + k);
}

// decodeYolo must record which candidate each kept box came from, including after NMS reordering
// by confidence. Otherwise masks get attached to the wrong objects.
TEST(DecodeYolo, KeptBoxesRememberTheirCandidateIndex) {
    YoloTensor t(4);
    t.set(1, 100, 100, 50, 50, kCoco_car, 0.6f);
    t.set(3, 600, 600, 50, 50, kCoco_person, 0.9f);
    const auto boxes = decodeYolo(t.data.data(), kClasses, t.n, kIdentity, 1000, 1000);
    ASSERT_EQ(boxes.size(), 2u);
    EXPECT_EQ(boxes[0].candidate, 3);  // most confident first
    EXPECT_EQ(boxes[1].candidate, 1);
}

TEST(DecodeMask, PositiveProtoRegionInsideBoxBecomesTheMask) {
    Protos p(2, 40, 30);
    p.fill(0, 5, 5, 15, 12, 2.0f);  // object region, grid pixels
    p.fill(0, 0, 0, 40, 5, -1.0f);  // elsewhere negative (e.g. background)
    const float coeffs[2] = {1.0f, 0.0f};
    Box b{/*x*/ 16, /*y*/ 16, /*w*/ 48, /*h*/ 40};  // source px -> grid [4,16) x [4,14)
    const auto m = decodeMask(p.data.data(), 2, p.W, p.H, coeffs, b, kUnit);
    EXPECT_EQ(m.x, 4);
    EXPECT_EQ(m.y, 4);
    EXPECT_EQ(m.w, 12);
    EXPECT_EQ(m.h, 10);
    EXPECT_EQ(m.area(), 10 * 7);  // the 10x7 positive region, fully inside the crop
    EXPECT_TRUE(m.atGrid(5, 5));
    EXPECT_FALSE(m.atGrid(4, 5));  // inside the crop, but the proto is 0 there: v = 0, not > 0
}

// Crop: a positive prototype region that extends beyond the box must NOT leak into the mask.
// Neighbouring objects share prototypes, so this is what keeps masks per object.
TEST(DecodeMask, MaskIsCroppedToTheBox) {
    Protos p(1, 40, 30);
    p.fill(0, 0, 0, 40, 30, 1.0f);  // positive everywhere
    const float coeffs[1] = {1.0f};
    Box b{40, 40, 20, 20};  // grid [10,15) x [10,15)
    const auto m = decodeMask(p.data.data(), 1, p.W, p.H, coeffs, b, kUnit);
    EXPECT_EQ(m.area(), 5 * 5);
    EXPECT_FALSE(m.atGrid(9, 12));
    EXPECT_FALSE(m.atGrid(15, 12));
}

// The weighted sum, not any single prototype, decides: two planes that cancel give no mask; a
// negative coefficient inverts a plane.
TEST(DecodeMask, CoefficientsCombinePrototypesLinearly) {
    Protos p(2, 20, 20);
    p.fill(0, 0, 0, 20, 20, 1.0f);
    p.fill(1, 0, 0, 20, 20, 1.0f);
    p.fill(1, 0, 0, 10, 20, 3.0f);  // left half stronger on plane 1
    Box b{0, 0, 80, 80};            // whole grid
    const float cancel[2] = {1.0f, -1.0f};
    const auto m = decodeMask(p.data.data(), 2, p.W, p.H, cancel, b, kUnit);
    // right half: 1 - 1 = 0 (not > 0); left half: 1 - 3 = -2. Nothing is object.
    EXPECT_EQ(m.area(), 0);
    const float invertLeft[2] = {2.5f, -1.0f};  // right: 2.5-1 > 0; left: 2.5-3 < 0
    const auto m2 = decodeMask(p.data.data(), 2, p.W, p.H, invertLeft, b, kUnit);
    EXPECT_EQ(m2.area(), 10 * 20);
    EXPECT_TRUE(m2.atGrid(15, 5));
    EXPECT_FALSE(m2.atGrid(5, 5));
}

// The production geometry: a 2560x1440 frame letterboxed to 1280x736 (scale 0.5, 8 px bands), a
// 4 px prototype stride, so a 320x184 grid. A source box must land on the right grid cells, and
// maskContains must answer in source pixels. This is how Part 8.3 queries LiDAR points.
TEST(DecodeMask, ProductionLetterboxGeometryAndSourceQuery) {
    const auto map = InputMapping::letterbox(2560, 1440, 1280, 736);
    Protos p(1, 320, 184);
    p.fill(0, 0, 0, 320, 184, 1.0f);
    const float coeffs[1] = {1.0f};
    // Source box x [800,1200), y [400,1000): model x [400,600), y [208,508): grid x [100,150),
    // y [52,127).
    Box b{800, 400, 400, 600};
    const auto m = decodeMask(p.data.data(), 1, p.W, p.H, coeffs, b, map);
    EXPECT_EQ(m.x, 100);
    EXPECT_EQ(m.y, 52);
    EXPECT_EQ(m.w, 50);
    EXPECT_EQ(m.h, 75);
    EXPECT_TRUE(maskContains(m, map, 4, 1000.0f, 700.0f));    // box centre
    EXPECT_FALSE(maskContains(m, map, 4, 1300.0f, 700.0f));   // right of the box
    EXPECT_FALSE(maskContains(m, map, 4, 1000.0f, 1100.0f));  // below the box
}

TEST(DecodeMask, BoxOutsideTheGridGivesAnEmptyMaskNotACrash) {
    Protos p(1, 10, 10);
    const float coeffs[1] = {1.0f};
    Box b{500, 500, 20, 20};  // grid [125,..): entirely off the 10x10 grid
    const auto m = decodeMask(p.data.data(), 1, p.W, p.H, coeffs, b, kUnit);
    EXPECT_EQ(m.w * m.h, 0);
    EXPECT_EQ(m.area(), 0);
    EXPECT_FALSE(m.atGrid(0, 0));
}
