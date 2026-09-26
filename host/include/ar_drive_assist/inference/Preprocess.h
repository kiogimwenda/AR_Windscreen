#pragma once
// GPU pre-processing: camera frame (BGR, 8-bit, on the GPU) -> one model's input tensor. See
// docs/BUILD_GUIDE.md Part 7.4.
//
// ---------------------------------------------------------------------------------------------
// What every network here expects, and why each step exists
//
//   1. region   YOLO and MiDaS see the whole frame. UFLD sees a CULane-shaped horizontal band
//               (see MlInferenceEngine.h): it was trained on 2.78:1 images, and squashing a 16:9
//               frame into that shape would distort the road geometry it relies on.
//   2. resize   to the fixed size the engine was built for. YOLO's resize is a LETTERBOX: it
//               preserves aspect ratio and pads the leftover rows with grey (114), as in its
//               training. The others are stretched to their input size.
//   3. BGR->RGB OpenCV frames are BGR; all three models were trained on RGB.
//   4. scale    8-bit 0..255 -> float 0..1.
//   5. normalise per channel (x - mean) / std. UFLD and MiDaS use the ImageNet statistics
//               their backbones were pre-trained with. YOLOv8 uses none (mean 0, std 1).
//   6. NCHW     OpenCV stores pixels interleaved (RGBRGB..., "HWC"). The networks want each
//               channel as its own plane (RRR...GGG...BBB..., "CHW"). cuda::split writes the three
//               planes straight into the engine's input buffer, so there is no extra copy.
//
// Every step is queued on the caller's stream, the engine's own, so pre-processing for the three
// models overlaps just as their inference does.
// ---------------------------------------------------------------------------------------------

#include <opencv2/core/cuda.hpp>

#include "ar_drive_assist/inference/Postprocess.h"

namespace ar_drive_assist {

struct PreprocessSpec {
    enum class Fit { Letterbox, Stretch };

    int dstW = 0, dstH = 0;
    Fit fit = Fit::Stretch;
    cv::Rect roi;               // region of the source frame; empty = whole frame
    float padValue = 0.0f;      // letterbox padding, in 0..1 units before normalisation
    float mean[3] = {0, 0, 0};  // RGB order
    float std[3] = {1, 1, 1};

    static PreprocessSpec yolo(int w, int h);                         // letterbox, pad 114/255
    static PreprocessSpec imagenet(int w, int h, cv::Rect roi = {});  // stretch, ImageNet stats
};

class Preprocessor {
public:
    explicit Preprocessor(PreprocessSpec spec) : spec_(spec) {}

    // Writes spec.dstW x spec.dstH, 3-channel, FP32, RGB, NCHW into `dst` (device memory), queued
    // on `stream`. Returns the source -> model-input mapping, which the decoders use to map model
    // outputs back into source pixels.
    InputMapping run(const cv::cuda::GpuMat& bgr8, float* dst, cv::cuda::Stream& stream);

    const PreprocessSpec& spec() const { return spec_; }
    // The lane band depends on the frame size, which is only known once frames arrive.
    void setRoi(const cv::Rect& roi) { spec_.roi = roi; }

private:
    PreprocessSpec spec_;
    // Scratch buffers, allocated on first use and reused every frame: a cudaMalloc per frame
    // would cost more than the resize itself.
    cv::cuda::GpuMat resized_, canvas_, rgb_, asFloat_;
};

}  // namespace ar_drive_assist
