#include "ar_drive_assist/inference/Preprocess.h"

#include <cmath>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <stdexcept>
#include <vector>

namespace ar_drive_assist {

PreprocessSpec PreprocessSpec::yolo(int w, int h) {
    PreprocessSpec s;
    s.dstW = w;
    s.dstH = h;
    s.fit = Fit::Letterbox;
    s.padValue = 114.0f / 255.0f;  // ultralytics' letterbox grey
    return s;
}

PreprocessSpec PreprocessSpec::imagenet(int w, int h, cv::Rect roi) {
    PreprocessSpec s;
    s.dstW = w;
    s.dstH = h;
    s.fit = Fit::Stretch;
    s.roi = roi;
    const float mean[3] = {0.485f, 0.456f, 0.406f}, std[3] = {0.229f, 0.224f, 0.225f};
    for (int c = 0; c < 3; ++c) {
        s.mean[c] = mean[c];
        s.std[c] = std[c];
    }
    return s;
}

InputMapping Preprocessor::run(const cv::cuda::GpuMat& bgr8, float* dst, cv::cuda::Stream& stream) {
    if (bgr8.type() != CV_8UC3) throw std::invalid_argument("Preprocessor: expected CV_8UC3");
    const cv::Rect frame(0, 0, bgr8.cols, bgr8.rows);
    const cv::Rect roi = spec_.roi.empty() ? frame : spec_.roi;
    if ((roi & frame) != roi) throw std::invalid_argument("Preprocessor: roi outside frame");
    const cv::cuda::GpuMat src = bgr8(roi);  // a view: no copy

    InputMapping m;
    canvas_.create(spec_.dstH, spec_.dstW, CV_8UC3);
    if (spec_.fit == PreprocessSpec::Fit::Letterbox) {
        const double s = std::min(double(spec_.dstW) / roi.width, double(spec_.dstH) / roi.height);
        const int w = static_cast<int>(std::round(roi.width * s));
        const int h = static_cast<int>(std::round(roi.height * s));
        const int ox = (spec_.dstW - w) / 2, oy = (spec_.dstH - h) / 2;
        const int pad = static_cast<int>(std::round(spec_.padValue * 255.0f));
        canvas_.setTo(cv::Scalar(pad, pad, pad), stream);
        cv::cuda::resize(src, resized_, cv::Size(w, h), 0, 0, cv::INTER_LINEAR, stream);
        resized_.copyTo(canvas_(cv::Rect(ox, oy, w, h)), stream);
        m.scaleX = static_cast<float>(w) / roi.width;
        m.scaleY = static_cast<float>(h) / roi.height;
        m.offsetX = ox - roi.x * m.scaleX;
        m.offsetY = oy - roi.y * m.scaleY;
    } else {
        cv::cuda::resize(src, canvas_, cv::Size(spec_.dstW, spec_.dstH), 0, 0, cv::INTER_LINEAR,
                         stream);
        m.scaleX = static_cast<float>(spec_.dstW) / roi.width;
        m.scaleY = static_cast<float>(spec_.dstH) / roi.height;
        m.offsetX = -roi.x * m.scaleX;
        m.offsetY = -roi.y * m.scaleY;
    }

    cv::cuda::cvtColor(canvas_, rgb_, cv::COLOR_BGR2RGB, 0, stream);
    rgb_.convertTo(asFloat_, CV_32FC3, 1.0 / 255.0, stream);
    cv::cuda::subtract(asFloat_, cv::Scalar(spec_.mean[0], spec_.mean[1], spec_.mean[2]), asFloat_,
                       cv::noArray(), -1, stream);
    cv::cuda::divide(asFloat_, cv::Scalar(spec_.std[0], spec_.std[1], spec_.std[2]), asFloat_, 1,
                     -1, stream);

    // Three single-channel GpuMat HEADERS over consecutive planes of the engine's input buffer.
    // split() writes each channel into its plane, which is the HWC -> CHW conversion.
    const size_t plane = static_cast<size_t>(spec_.dstW) * spec_.dstH;
    std::vector<cv::cuda::GpuMat> planes;
    for (int c = 0; c < 3; ++c) {
        planes.emplace_back(spec_.dstH, spec_.dstW, CV_32FC1, dst + c * plane,
                            spec_.dstW * sizeof(float));
    }
    cv::cuda::split(asFloat_, planes, stream);
    return m;
}

}  // namespace ar_drive_assist
