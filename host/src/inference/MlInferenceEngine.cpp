#include "ar_drive_assist/inference/MlInferenceEngine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <stdexcept>
#include <thread>

#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {
namespace {

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

void check(cudaError_t err, const char* what) {
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("MlInferenceEngine: ") + what + ": " +
                                 cudaGetErrorString(err));
    }
}

// CULane's frame shape (1640x590) and UFLD's crop_ratio (the model sees the bottom 60% of a
// resized frame), from configs/culane_res18.py in the UFLDv2 repository.
constexpr float kCulaneAspect = 590.0f / 1640.0f;
constexpr float kUfldCropRatio = 0.6f;

// Engine outputs are looked up by name: TensorRT does not promise to keep the order the exporter
// declared them in.
const TensorInfo& output(const TrtEngine& e, const std::string& name) {
    for (const TensorInfo& t : e.outputs()) {
        if (t.name == name) return t;
    }
    throw std::runtime_error("MlInferenceEngine: engine has no output '" + name + "'");
}

const float* outputData(const TrtEngine& e, const std::string& name) {
    for (std::size_t i = 0; i < e.outputs().size(); ++i) {
        if (e.outputs()[i].name == name) return e.hostOutput(i);
    }
    throw std::runtime_error("MlInferenceEngine: engine has no output '" + name + "'");
}

}  // namespace

MlInferenceEngine::MlInferenceEngine(const InferenceConfig& cfg, FrameBus& frames,
                                     DetectionBus& detections, DepthBus* depth, EventLog* log,
                                     MaskBus* masks)
    : cfg_(cfg),
      frames_(frames),
      detections_(detections),
      depth_(depth),
      masks_(masks),
      log_(log),
      yoloPre_(PreprocessSpec{}),
      ufldPre_(PreprocessSpec{}),
      midasPre_(PreprocessSpec{}),
      signsPre_(PreprocessSpec{}) {
    yolo_.load(cfg_.yoloEngine);
    // A YOLOv8-seg engine has a second output: the prototype masks, (1, K, H/4, W/4).
    segmentation_ = yolo_.outputs().size() == 2;
    if (segmentation_) numMaskCoeffs_ = static_cast<int>(output(yolo_, "output1").shape[1]);
    ufld_.load(cfg_.ufldEngine);
    midas_.load(cfg_.midasEngine);
    hasSigns_ = !cfg_.signEngine.empty();
    if (hasSigns_) signs_.load(cfg_.signEngine);

    // Each engine's input shape (N, C, H, W) decides its pre-processing size, so a rebuilt engine
    // at another resolution needs no code change here.
    const auto hw = [](const TrtEngine& e) {
        return std::pair<int, int>(static_cast<int>(e.input().shape[3]),
                                   static_cast<int>(e.input().shape[2]));
    };
    auto [yw, yh] = hw(yolo_);
    auto [uw, uh] = hw(ufld_);
    auto [mw, mh] = hw(midas_);
    yoloPre_ = Preprocessor(PreprocessSpec::yolo(yw, yh));
    ufldPre_ = Preprocessor(PreprocessSpec::imagenet(uw, uh));
    midasPre_ = Preprocessor(PreprocessSpec::imagenet(mw, mh));
    if (hasSigns_) {
        auto [sw, sh] = hw(signs_);
        signsPre_ = Preprocessor(PreprocessSpec::yolo(sw, sh));  // same letterbox as training
    }

    // cudaEventDisableTiming: this event is only for ordering, and skipping timestamps makes
    // recording and waiting on it cheaper.
    check(cudaEventCreateWithFlags(&uploaded_, cudaEventDisableTiming), "cudaEventCreate");
    if (log_) {
        log_->logGeneral("inference: engines loaded (yolo " + std::to_string(yw) + "x" +
                         std::to_string(yh) + ", ufld " + std::to_string(uw) + "x" +
                         std::to_string(uh) + ", midas " + std::to_string(mw) + "x" +
                         std::to_string(mh) + ")");
    }
}

MlInferenceEngine::~MlInferenceEngine() {
    if (uploaded_) cudaEventDestroy(uploaded_);
}

LaneBand MlInferenceEngine::laneBand(int frameW, int frameH) const {
    LaneBand b;
    b.x = 0;
    b.w = static_cast<float>(frameW);
    b.h = std::min(static_cast<float>(frameH), frameW * kCulaneAspect);
    // Clamp so the band never runs off the bottom of the frame.
    b.y = std::clamp(cfg_.laneBandTopFraction * frameH, 0.0f, frameH - b.h);
    return b;
}

MlInferenceEngine::Result MlInferenceEngine::process(const CameraFrame& frame) {
    const auto t0 = Clock::now();
    if (frame.bgr.empty() || frame.bgr.type() != CV_8UC3) {
        throw std::invalid_argument("MlInferenceEngine: frame must be non-empty CV_8UC3");
    }
    const int W = frame.bgr.cols, H = frame.bgr.rows;

    // The lane model sees the bottom 60% of the band (UFLD's crop), while the decoder maps
    // against the full band, as upstream does.
    const LaneBand band = laneBand(W, H);
    const float cropTop = band.y + (1.0f - kUfldCropRatio) * band.h;
    ufldPre_.setRoi(cv::Rect(0, static_cast<int>(std::lround(cropTop)), W,
                             static_cast<int>(std::lround(band.y + band.h - cropTop))));

    // Upload once, then make every model stream wait on the upload, on the GPU (see header).
    gpuFrame_.upload(frame.bgr, uploadStream_);
    check(cudaEventRecord(uploaded_, cv::cuda::StreamAccessor::getStream(uploadStream_)),
          "cudaEventRecord");

    struct Job {
        TrtEngine& engine;
        Preprocessor& pre;
        InputMapping mapping;
    };
    // jobs[0..2] are always present; jobs[3], the sign detector, is optional.
    std::vector<Job> jobs = {{yolo_, yoloPre_, {}}, {ufld_, ufldPre_, {}}, {midas_, midasPre_, {}}};
    if (hasSigns_) jobs.push_back({signs_, signsPre_, {}});
    for (Job& j : jobs) {
        check(cudaStreamWaitEvent(j.engine.stream(), uploaded_, 0), "cudaStreamWaitEvent");
        cv::cuda::Stream s = cv::cuda::StreamAccessor::wrapStream(j.engine.stream());
        j.mapping = j.pre.run(gpuFrame_, j.engine.deviceInput(), s);
        j.engine.enqueue();
    }
    for (Job& j : jobs) j.engine.sync();

    Result r;
    r.gpuMs = msSince(t0);

    // --- YOLO --- output0 is (1, 4 + classes [+ K mask coefficients], candidates)
    const TensorInfo& yOut = output(yolo_, "output0");
    const float* yData = outputData(yolo_, "output0");
    const int numClasses = static_cast<int>(yOut.shape[1]) - 4 - numMaskCoeffs_;
    const int numCandidates = static_cast<int>(yOut.shape[2]);
    r.boxes = decodeYolo(yData, numClasses, numCandidates, jobs[0].mapping, W, H, cfg_.yolo);

    // --- masks (YOLOv8-seg only): one per kept box, decoded from the shared prototypes ---
    r.masks.seq = frame.seq;
    r.masks.timestampMs = frame.timestampMs;
    r.masks.mapping = jobs[0].mapping;
    if (segmentation_) {
        const TensorInfo& pOut = output(yolo_, "output1");  // (1, K, protoH, protoW)
        const int protoH = static_cast<int>(pOut.shape[2]),
                  protoW = static_cast<int>(pOut.shape[3]);
        // Model-input pixels per prototype pixel (4 for YOLOv8-seg), from the engine's own shapes.
        r.masks.stride = static_cast<int>(yolo_.input().shape[3]) / protoW;
        std::vector<float> coeffs(static_cast<size_t>(numMaskCoeffs_));
        for (const Box& b : r.boxes) {
            maskCoefficients(yData, numClasses, numMaskCoeffs_, numCandidates, b, coeffs.data());
            r.masks.masks.push_back(decodeMask(outputData(yolo_, "output1"), numMaskCoeffs_, protoW,
                                               protoH, coeffs.data(), b, jobs[0].mapping,
                                               r.masks.stride));
        }
    }

    // --- signs --- (1, 4 + 29, candidates), classes used as-is
    if (hasSigns_) {
        const TensorInfo& sOut = output(signs_, "output0");
        r.signs = decodeYolo(outputData(signs_, "output0"), static_cast<int>(sOut.shape[1]) - 4,
                             static_cast<int>(sOut.shape[2]), jobs[3].mapping, W, H, cfg_.signs);
    }

    // --- UFLD --- outputs are looked up by name: TensorRT does not promise to keep the order
    // the exporter declared them in.
    auto ufldOut = [&](const char* name) -> const float* {
        for (std::size_t i = 0; i < ufld_.outputs().size(); ++i) {
            if (ufld_.outputs()[i].name == name) return ufld_.hostOutput(i);
        }
        throw std::runtime_error(std::string("MlInferenceEngine: ufld output missing: ") + name);
    };
    r.lanes = decodeUfld(ufldOut("loc_row"), ufldOut("loc_col"), ufldOut("exist_row"),
                         ufldOut("exist_col"), band, cfg_.ufld);

    // --- MiDaS --- (1, H, W) relative inverse depth, copied out of the engine's pinned buffer,
    // which the next frame will overwrite.
    const TensorInfo& dOut = midas_.outputs().at(0);
    r.depth.seq = frame.seq;
    r.depth.timestampMs = frame.timestampMs;
    r.depth.mapping = jobs[2].mapping;
    r.depth.inverseDepth = cv::Mat(static_cast<int>(dOut.shape[1]), static_cast<int>(dOut.shape[2]),
                                   CV_32FC1, const_cast<float*>(midas_.hostOutput(0)))
                               .clone();

    // --- the bus message ---
    r.detections.timestamp_ms = frame.timestampMs;
    r.detections.depth_map_ref = frame.seq;
    r.detections.mask_ref = frame.seq;
    for (const Box& b : r.boxes) {
        auto bb = std::make_unique<schema::BoundingBoxT>();
        bb->x = b.x;
        bb->y = b.y;
        bb->w = b.w;
        bb->h = b.h;
        bb->class_id = b.classId;
        bb->confidence = b.confidence;
        bb->track_id = -1;  // assigned by MultiObjectTracker (Phase 7)
        r.detections.boxes.push_back(std::move(bb));
    }
    for (const Box& b : r.signs) {
        auto bb = std::make_unique<schema::BoundingBoxT>();
        bb->x = b.x;
        bb->y = b.y;
        bb->w = b.w;
        bb->h = b.h;
        bb->class_id = b.classId;  // kSignClassNames index
        bb->confidence = b.confidence;
        bb->track_id = -1;
        r.detections.signs.push_back(std::move(bb));
    }
    for (const LanePoint& p : r.lanes[1]) {
        r.detections.lane_points_left.push_back(p.x);
        r.detections.lane_points_left.push_back(p.y);
    }
    for (const LanePoint& p : r.lanes[2]) {
        r.detections.lane_points_right.push_back(p.x);
        r.detections.lane_points_right.push_back(p.y);
    }
    r.totalMs = msSince(t0);
    return r;
}

void MlInferenceEngine::run(const std::atomic<bool>& stop) {
    std::uint64_t processed = 0, skipped = 0, busFull = 0;
    double totalMs = 0;
    auto lastReport = Clock::now();

    while (!stop.load()) {
        CameraFrame frame;
        std::size_t discarded = 0;
        // popLatest: always work on the newest frame. Older ones are stale by definition
        // (Part 5.3's drop-oldest policy, realised on the consumer side, see RingBuffer.h).
        if (!frames_.popLatest(frame, &discarded)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        skipped += discarded;

        Result r = process(frame);
        totalMs += r.totalMs;
        ++processed;

        if (depth_ && !depth_->push(std::move(r.depth))) ++busFull;
        if (masks_ && segmentation_ && !masks_->push(std::move(r.masks))) ++busFull;
        if (!detections_.push(std::move(r.detections))) ++busFull;

        if (log_ && msSince(lastReport) >= 5000.0) {
            log_->logGeneral("inference: " + std::to_string(processed) + " frames, mean " +
                             std::to_string(totalMs / processed) + " ms, skipped " +
                             std::to_string(skipped) + ", output bus full " +
                             std::to_string(busFull));
            lastReport = Clock::now();
        }
    }
}

}  // namespace ar_drive_assist
