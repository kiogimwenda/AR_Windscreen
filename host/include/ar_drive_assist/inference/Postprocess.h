#pragma once
// Model-output decoding: raw TensorRT output tensors -> boxes and lane points in SOURCE-FRAME
// pixels. See docs/BUILD_GUIDE.md Part 7.1, Part 7.4.
//
// Deliberately free of OpenCV, CUDA and TensorRT: plain float arrays in, plain structs out. That
// keeps it in the GPU-free unit suite (Part 13.1), so CI checks the decoding logic that every
// downstream decision depends on.
//
// ---------------------------------------------------------------------------------------------
// Coordinate frames
//
// Each model sees a resized (and for YOLO letterboxed, for UFLD band-cropped) copy of the camera
// frame. Every decoder takes an InputMapping describing that transform and returns coordinates
// in the ORIGINAL frame, so fusion and rendering never need to know what size a model ran at.
// ---------------------------------------------------------------------------------------------

#include <array>
#include <cstdint>
#include <vector>

namespace ar_drive_assist {

// Part 7.1's object classes. COCO's 80 classes are mapped onto these (see mapCocoClass).
enum class ObjectClass : int32_t {
    Vehicle = 0,
    Pedestrian = 1,
    Cyclist = 2,
    Sign = 3,
    Obstacle = 4
};

// Returns the ObjectClass for a COCO class index, or -1 for classes this system ignores (cups,
// laptops, ...).
int32_t mapCocoClass(int cocoClass);

// Model-input pixel = source pixel * scale + offset, per axis.
struct InputMapping {
    float scaleX = 1.0f, scaleY = 1.0f;
    float offsetX = 0.0f, offsetY = 0.0f;
    float toSourceX(float modelX) const { return (modelX - offsetX) / scaleX; }
    float toSourceY(float modelY) const { return (modelY - offsetY) / scaleY; }

    // Aspect-preserving fit into (dstW x dstH), centred, padding the rest: how YOLO sees a frame.
    static InputMapping letterbox(int srcW, int srcH, int dstW, int dstH);
};

struct Box {
    float x = 0, y = 0, w = 0, h = 0;  // top-left + size, source-frame pixels
    int32_t classId = 0;               // ObjectClass
    float confidence = 0;
    int32_t candidate = -1;  // index of the model-output candidate it came from (mask coefficients)
};

struct YoloParams {
    float confThreshold = 0.25f;  // ultralytics' default
    float iouThreshold = 0.45f;   // NMS overlap above which the weaker same-class box is removed
    int maxDetections = 300;
};

// Decodes YOLOv8's (1, 4 + numClasses, numCandidates) output: channel-major, so candidate i's
// value for channel c is out[c * numCandidates + i]. Channels 0-3 are box centre x, centre y,
// width and height in model-input pixels. The rest are per-class scores, already sigmoid-ed by
// the exported head. Classes are mapped to ObjectClass BEFORE non-maximum suppression, so a
// vehicle scored as both "car" and "truck" collapses to one Vehicle box. Boxes are clipped to the
// source frame.
std::vector<Box> decodeYolo(const float* out, int numClasses, int numCandidates,
                            const InputMapping& mapping, int srcW, int srcH,
                            const YoloParams& params = {});

// Intersection-over-union of two boxes (0 when disjoint).
float iou(const Box& a, const Box& b);

// --- YOLOv8-seg masks ---------------------------------------------------------------------------
//
// A segmentation model does not output one mask image per object. It outputs K (= 32) PROTOTYPE
// masks for the whole image, at 1/stride (= 1/4) of the input resolution, and K coefficients per
// candidate, stored right after the class scores in output0. An object's mask is
//
//     mask(x, y) = sigmoid( sum_k coeff_k * proto_k(x, y) ) > 0.5,  cropped to the object's box.
//
// sigmoid(v) > 0.5 is exactly v > 0, so the decoder thresholds the weighted sum and never evaluates
// an exponential. This is the YOLACT method, and ultralytics' own decoding does the same.

// A binary mask on the prototype grid, stored only inside the object's box (the crop).
struct ObjectMask {
    int x = 0, y = 0, w = 0, h = 0;  // crop rectangle, prototype-grid pixels
    std::vector<uint8_t> bits;       // w * h, row-major, 1 = object
    int area() const;                // number of object pixels (grid pixels)
    bool atGrid(int gx, int gy) const;
};

// Reads the K mask coefficients of `box` from a YOLOv8-seg output0 laid out as
// (1, 4 + numClasses + numCoeffs, numCandidates), channel-major like decodeYolo's input.
void maskCoefficients(const float* out, int numClasses, int numCoeffs, int numCandidates,
                      const Box& box, float* coeffsOut);

// Decodes one object's mask. `protos` is output1, laid out as (1, numCoeffs, protoH, protoW).
// `mapping` is the source -> model-input mapping used for the frame (the letterbox), and
// `stride` is model-input pixels per prototype pixel (4 for YOLOv8-seg).
ObjectMask decodeMask(const float* protos, int numCoeffs, int protoW, int protoH,
                      const float* coeffs, const Box& box, const InputMapping& mapping,
                      int stride = 4);

// Whether a SOURCE-frame point lies on the object. This is the query Part 8.3's mask-based
// LiDAR fusion makes for every projected point.
bool maskContains(const ObjectMask& mask, const InputMapping& mapping, int stride, float srcX,
                  float srcY);

// --- UFLDv2 lanes ------------------------------------------------------------------------------
struct LanePoint {
    float x = 0, y = 0;  // source-frame pixels
};

// CULane ResNet-18 defaults (configs/culane_res18.py in the UFLDv2 repository).
struct UfldParams {
    int numGridRow = 200, numClsRow = 72;  // loc_row: (1, numGridRow, numClsRow, numLanes)
    int numGridCol = 100, numClsCol = 81;  // loc_col: (1, numGridCol, numClsCol, numLanes)
    int numLanes = 4;
    float rowAnchorStart = 0.42f;  // row anchors: linspace(0.42, 1, numClsRow) of band height
    int localWidth = 1;            // soft-argmax window half-width
};

// The band of the source frame that the lane model saw (source pixels), before resizing.
struct LaneBand {
    float x = 0, y = 0, w = 0, h = 0;
};

// Lanes indexed as UFLDv2 outputs them: 0 = outer left, 1 = ego-lane left, 2 = ego-lane right,
// 3 = outer right. Lanes 1 and 2 come from row anchors, lanes 0 and 3 from column anchors.
// A lane the model does not consider present is left empty.
using Lanes = std::array<std::vector<LanePoint>, 4>;

// A port of UFLDv2's demo.py pred2coords: per anchor, the arg-max grid cell refined by a softmax-
// weighted mean over +-localWidth neighbours. One deliberate difference: coordinates stay float
// rather than being truncated to int.
Lanes decodeUfld(const float* locRow, const float* locCol, const float* existRow,
                 const float* existCol, const LaneBand& band, const UfldParams& params = {});

}  // namespace ar_drive_assist
