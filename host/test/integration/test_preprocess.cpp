// GPU pre-processing tests — see Preprocess.h. Needs a GPU (ctest label `gpu`, not in CI).
//
// The inputs are chosen so the correct output can be computed by hand. That catches the classic
// silent mistakes, which a comparison against a second implementation of the same idea would
// share: swapped R and B, normalisation applied twice or not at all, planes written interleaved,
// padding on the wrong rows.

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <vector>

#include "ar_drive_assist/inference/Preprocess.h"

using namespace ar_drive_assist;

namespace {

// Runs a Preprocessor on a host frame and returns the three output planes on the host.
struct Result {
    std::vector<float> data;
    int w, h;
    InputMapping mapping;
    float at(int c, int y, int x) const { return data[(size_t(c) * h + y) * w + x]; }
};

Result run(const cv::Mat& frame, const PreprocessSpec& spec) {
    cv::cuda::GpuMat gpu(frame);
    float* dst = nullptr;
    cudaMalloc(reinterpret_cast<void**>(&dst), size_t(3) * spec.dstW * spec.dstH * sizeof(float));
    cv::cuda::Stream stream;
    Preprocessor p(spec);
    Result r{std::vector<float>(size_t(3) * spec.dstW * spec.dstH), spec.dstW, spec.dstH, {}};
    r.mapping = p.run(gpu, dst, stream);
    stream.waitForCompletion();
    cudaMemcpy(r.data.data(), dst, r.data.size() * sizeof(float), cudaMemcpyDeviceToHost);
    cudaFree(dst);
    return r;
}

}  // namespace

// Pure red in OpenCV's BGR order is (0, 0, 255). After BGR->RGB, /255 and ImageNet normalisation,
// every pixel must be exactly ((1-.485)/.229, (0-.456)/.224, (0-.406)/.225), in planes R, G, B.
TEST(Preprocess, ChannelOrderScalingAndNormalisation) {
    const cv::Mat red(1440, 2560, CV_8UC3, cv::Scalar(0, 0, 255));
    const auto r = run(red, PreprocessSpec::imagenet(448, 256));
    const float want[3] = {(1 - 0.485f) / 0.229f, (0 - 0.456f) / 0.224f, (0 - 0.406f) / 0.225f};
    for (int c = 0; c < 3; ++c) {
        for (int y : {0, 128, 255}) {
            for (int x : {0, 224, 447}) {
                ASSERT_NEAR(r.at(c, y, x), want[c], 1e-5) << "plane " << c << " @" << x << "," << y;
            }
        }
    }
}

// The production YOLO case: 2560x1440 -> 1280x736. The content is 1280x720 with 8 grey rows above
// and below. YOLO has no normalisation, so white content is exactly 1.0 and the padding exactly
// 114/255.
TEST(Preprocess, YoloLetterboxPaddingAndMapping) {
    const cv::Mat white(1440, 2560, CV_8UC3, cv::Scalar(255, 255, 255));
    const auto r = run(white, PreprocessSpec::yolo(1280, 736));
    EXPECT_FLOAT_EQ(r.mapping.scaleX, 0.5f);
    EXPECT_FLOAT_EQ(r.mapping.offsetY, 8.0f);
    for (int c = 0; c < 3; ++c) {
        for (int y : {0, 7, 728, 735}) EXPECT_NEAR(r.at(c, y, 640), 114.0f / 255.0f, 1e-6) << y;
        for (int y : {8, 368, 727}) EXPECT_NEAR(r.at(c, y, 640), 1.0f, 1e-6) << y;
    }
}

// A white square in a black frame must appear where the returned mapping says it will: this ties
// the pixels written to the mapping the decoders use to go back.
TEST(Preprocess, FeatureLandsWhereTheMappingSays) {
    cv::Mat frame(1440, 2560, CV_8UC3, cv::Scalar(0, 0, 0));
    frame(cv::Rect(1000, 600, 40, 40)).setTo(cv::Scalar(255, 255, 255));
    const auto r = run(frame, PreprocessSpec::yolo(1280, 736));
    const int mx = static_cast<int>(std::lround(r.mapping.scaleX * 1020 + r.mapping.offsetX));
    const int my = static_cast<int>(std::lround(r.mapping.scaleY * 620 + r.mapping.offsetY));
    EXPECT_NEAR(r.at(0, my, mx), 1.0f, 1e-6);       // centre of the square
    EXPECT_NEAR(r.at(0, my, mx + 30), 0.0f, 1e-6);  // well outside it
}

// A region of interest selects only that part of the frame, and the mapping accounts for its
// offset: this is how the lane model's band is cut out.
TEST(Preprocess, RoiSelectsRegionAndOffsetsMapping) {
    cv::Mat frame(1440, 2560, CV_8UC3, cv::Scalar(0, 0, 255));        // red
    frame(cv::Rect(0, 720, 2560, 720)).setTo(cv::Scalar(255, 0, 0));  // bottom half blue
    const cv::Rect bottom(0, 720, 2560, 720);
    const auto r = run(frame, PreprocessSpec::imagenet(1600, 320, bottom));
    const float blueB = (1 - 0.406f) / 0.225f, blueR = (0 - 0.485f) / 0.229f;
    EXPECT_NEAR(r.at(2, 0, 0), blueB, 1e-5);  // first row is already blue: the red half is excluded
    EXPECT_NEAR(r.at(0, 319, 1599), blueR, 1e-5);
    EXPECT_FLOAT_EQ(r.mapping.toSourceY(0.0f), 720.0f);
}

TEST(Preprocess, RejectsRoiOutsideFrameAndWrongType) {
    const cv::Mat frame(100, 100, CV_8UC3, cv::Scalar::all(0));
    EXPECT_THROW(run(frame, PreprocessSpec::imagenet(32, 32, cv::Rect(50, 50, 100, 100))),
                 std::invalid_argument);
    const cv::Mat gray(100, 100, CV_8UC1, cv::Scalar::all(0));
    EXPECT_THROW(run(gray, PreprocessSpec::imagenet(32, 32)), std::invalid_argument);
}
