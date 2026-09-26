#pragma once
// MlInferenceEngine — runs YOLOv8m, UFLDv2 and MiDaS on every frame, concurrently, and publishes
// the results. See docs/BUILD_GUIDE.md Part 5.1 (InferenceThread), Part 7.4.
//
// ---------------------------------------------------------------------------------------------
// One frame, step by step
//
//   upload     the frame goes to the GPU once, on the upload stream
//   event      a CUDA event is recorded on the upload stream after the upload. Each model's
//              stream is told to wait for it (cudaStreamWaitEvent). The GPU then will not start
//              pre-processing until the upload has landed, and the CPU never blocks to enforce
//              that. The alternative, a CPU-side sync after the upload, would stall this thread
//              for the whole transfer.
//   pre-process + enqueue, per model, each on its own stream: three independent queues
//   sync       wait for all three, then decode on the CPU
//   publish    a DetectionFrame on detectionBus, and the depth map on depthBus
//
// ---------------------------------------------------------------------------------------------
// The lane band
//
// UFLDv2 was trained on CULane images: 1640x590 (2.78:1), with the horizon roughly 40% of the way
// down. It sees a band of that shape, spanning the frame's full width and placed so the horizon
// sits where it did in training. `laneBandTopFraction` is the band's top edge as a fraction of
// frame height. It depends on how the camera is mounted (pitch and height), so it is
// configuration, set during calibration (Part 12). Following upstream, the model sees only the
// bottom 60% of that band (crop_ratio 0.6).
//
// ---------------------------------------------------------------------------------------------
// Depth
//
// MiDaS outputs RELATIVE inverse depth: larger = nearer, with no unit and an arbitrary per-frame
// scale. It is useful for "which of these is closer", not for metres. Metric range near the car
// comes from the LiDAR (Part 8). The map is published on its own bus, and
// DetectionFrame::depth_map_ref carries the frame's sequence number so a consumer can pair the
// two (Part 3.5: the map is a handle, never inlined).
// ---------------------------------------------------------------------------------------------

#include <cuda_runtime_api.h>

#include <atomic>
#include <cstdint>
#include <opencv2/core/cuda.hpp>
#include <string>
#include <vector>

#include "ar_drive_assist/camera/CameraFrame.h"
#include "ar_drive_assist/common/RingBuffer.h"
#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/inference/Postprocess.h"
#include "ar_drive_assist/inference/Preprocess.h"
#include "ar_drive_assist/inference/TrtEngine.h"

namespace ar_drive_assist {

class EventLog;

struct InferenceConfig {
    // YOLOv8m-seg (Part 7.1, amended 2026-09-25). A box-only YOLOv8m engine still works: the
    // engine is recognised by its outputs (two = segmentation), and then no masks are produced.
    std::string yoloEngine = "host/models/engines/yolov8m_seg.engine";
    std::string ufldEngine = "host/models/engines/ufld.engine";
    std::string midasEngine = "host/models/engines/midas.engine";
    YoloParams yolo;
    UfldParams ufld;
    // See "The lane band". 0.32 put the horizon where UFLD expects it on the Kraków test footage
    // (data/README.md), where the horizon sits ~58% down the frame. That is a DIFFERENT camera
    // mount from the project's, so Part 12 calibration must re-tune this for the real one.
    float laneBandTopFraction = 0.32f;
    // Road-sign detector (the fourth model; docs/experiments/2026-09-26-sign-detector.md). Empty =
    // no sign detection. Its classes are its own 29 (kSignClassNames), so no COCO mapping.
    std::string signEngine = "host/models/engines/signs.engine";
    YoloParams signs{0.25f, 0.45f, 100, YoloParams::Classes::Identity};
};

// Object masks for one frame: masks[i] belongs to DetectionFrame::boxes[i] of the frame with the
// same seq (DetectionFrame::mask_ref). Grid pixels map to source pixels through `mapping` and
// `stride` (see Postprocess.h, maskContains).
struct MaskFrame {
    std::uint64_t seq = 0;
    std::uint64_t timestampMs = 0;
    std::vector<ObjectMask> masks;
    InputMapping mapping;
    int stride = 4;
};

struct DepthFrame {
    std::uint64_t seq = 0;
    std::uint64_t timestampMs = 0;
    cv::Mat inverseDepth;  // CV_32FC1, model resolution, relative (larger = nearer)
    InputMapping mapping;  // source frame -> depth map pixels
};

class MlInferenceEngine {
public:
    using FrameBus = RingBuffer<CameraFrame, 4>;
    using DetectionBus = RingBuffer<DetectionFrame, 8>;
    using DepthBus = RingBuffer<DepthFrame, 4>;
    using MaskBus = RingBuffer<MaskFrame, 4>;

    // Loads all three engines. Throws std::runtime_error if any is missing or wrong, so a broken
    // model set fails at startup rather than inside the pipeline.
    MlInferenceEngine(const InferenceConfig& cfg, FrameBus& frames, DetectionBus& detections,
                      DepthBus* depth = nullptr, EventLog* log = nullptr, MaskBus* masks = nullptr);
    ~MlInferenceEngine();

    // SystemManager contract (SystemManager.h): pop the newest frame, process, publish.
    void run(const std::atomic<bool>& stop);

    struct Result {
        DetectionFrame detections;  // what goes on the bus
        std::vector<Box> boxes;     // the same boxes, as plain structs, for tools
        std::vector<Box> signs;     // sign detections; classId indexes kSignClassNames
        MaskFrame masks;            // one mask per box (empty list for a box-only engine)
        Lanes lanes;                // all four UFLD lanes (the bus carries only 1 and 2)
        DepthFrame depth;
        double gpuMs = 0;  // upload -> all three models synced (the Part 7.1 FPS budget)
        double totalMs = 0;
    };
    // Processes one frame synchronously. Used by run(), and directly by tools and tests.
    Result process(const CameraFrame& frame);

    // Where the lane band falls in a frame of the given size (source pixels).
    LaneBand laneBand(int frameW, int frameH) const;

private:
    InferenceConfig cfg_;
    FrameBus& frames_;
    DetectionBus& detections_;
    DepthBus* depth_;
    MaskBus* masks_;
    bool segmentation_ = false;  // the YOLO engine has a prototype output (YOLOv8-seg)
    int numMaskCoeffs_ = 0;
    EventLog* log_;

    TrtEngine yolo_, ufld_, midas_, signs_;
    Preprocessor yoloPre_, ufldPre_, midasPre_, signsPre_;
    bool hasSigns_ = false;
    cv::cuda::Stream uploadStream_;
    cv::cuda::GpuMat gpuFrame_;
    cudaEvent_t uploaded_ = nullptr;
};

}  // namespace ar_drive_assist
