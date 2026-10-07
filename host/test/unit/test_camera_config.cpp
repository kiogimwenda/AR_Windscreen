// CameraConfig: the GStreamer pipeline strings, the YAML loading, the calibration file, and the
// frame-rate measurement (Part 6.2). The capture itself is tested with OpenCV and GStreamer in
// host/test/integration/test_camera_pipeline.cpp (label camera).
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

#include "ar_drive_assist/camera/CameraConfig.h"

using namespace ar_drive_assist;

namespace {

std::string writeTemp(const std::string& name, const std::string& text) {
    const std::string path = ::testing::TempDir() + name;
    std::ofstream(path) << text;
    return path;
}

bool contains(const std::string& s, const std::string& part) {
    return s.find(part) != std::string::npos;
}

}  // namespace

TEST(CameraConfig, SourceKinds) {
    EXPECT_EQ(cameraSourceKind("/dev/video0"), CameraSourceKind::Device);
    EXPECT_EQ(cameraSourceKind("data/drive.mp4"), CameraSourceKind::File);
    EXPECT_EQ(cameraSourceKind("gst:videotestsrc ! appsink"), CameraSourceKind::Pipeline);
}

TEST(CameraConfig, MjpgPipelineAsksForExactlyTheFormatSizeAndRate) {
    CameraConfig c;
    c.source = "/dev/video2";
    const std::string p = cameraGstPipeline(c);
    EXPECT_TRUE(contains(p, "v4l2src device=/dev/video2"));
    EXPECT_TRUE(contains(p, "image/jpeg,width=2560,height=1440,framerate=30/1 ! jpegdec"));
    EXPECT_TRUE(contains(p, "video/x-raw,format=BGR"));
    EXPECT_TRUE(contains(p, "appsink drop=true max-buffers=1 sync=false"));
}

TEST(CameraConfig, YuyvPipelineAndTheFallbackSize) {
    CameraConfig c;
    c.format = "YUYV";
    c.width = 1920;
    c.height = 1080;
    c.fps = 25;
    const std::string p = cameraGstPipeline(c);
    EXPECT_TRUE(contains(p, "video/x-raw,format=YUY2,width=1920,height=1080,framerate=25/1"));
    EXPECT_FALSE(contains(p, "jpegdec"));
}

TEST(CameraConfig, PipelineSourceIsUsedVerbatimAndBadInputsThrow) {
    CameraConfig c;
    c.source = "gst:videotestsrc ! appsink";
    EXPECT_EQ(cameraGstPipeline(c), "videotestsrc ! appsink");
    c.source = "/dev/video0";
    c.format = "H264";
    EXPECT_THROW(cameraGstPipeline(c), std::invalid_argument);
    c.source = "drive.mp4";
    EXPECT_THROW(cameraGstPipeline(c), std::invalid_argument);
}

TEST(CameraConfig, RealConfigFileLoads) {
    // The file the system starts with: a bad edit fails CI, not the car.
    const CameraConfig c = loadCameraConfig(std::string(HOST_SOURCE_DIR) + "/config/camera.yaml");
    EXPECT_EQ(c.width, 2560);
    EXPECT_EQ(c.height, 1440);
    EXPECT_EQ(c.format, "MJPG");
    EXPECT_GT(c.stallTimeoutS, 0);
}

TEST(CameraConfig, MalformedValuesAreRefusedByName) {
    const std::string bad = writeTemp("cam_bad.yaml", "width: wide\n");
    try {
        loadCameraConfig(bad);
        FAIL() << "accepted a non-numeric width";
    } catch (const std::runtime_error& e) {
        EXPECT_TRUE(contains(e.what(), "width"));
    }
    EXPECT_THROW(loadCameraConfig(writeTemp("cam_fps.yaml", "fps: 0\n")), std::runtime_error);
    EXPECT_THROW(loadCameraConfig(writeTemp("cam_fmt.yaml", "format: H264\n")), std::runtime_error);
    EXPECT_THROW(loadCameraConfig(::testing::TempDir() + "does_not_exist.yaml"),
                 std::runtime_error);
}

TEST(CameraIntrinsics, TheTemplateMeansNotCalibrated) {
    const CameraIntrinsics in =
        loadCameraIntrinsics(std::string(HOST_SOURCE_DIR) + "/config/camera_intrinsics.yaml");
    EXPECT_FALSE(in.calibrated);
}

TEST(CameraIntrinsics, ACalibrationIsReadIntoTheSharedCameraModel) {
    const CameraIntrinsics in = loadCameraIntrinsics(
        writeTemp("intr.yaml",
                  "image_width: 1920\nimage_height: 1080\n"
                  "camera_matrix: [1400, 0, 955, 0, 1395, 545, 0, 0, 1]\n"
                  "distortion_coefficients: [-0.31, 0.12, 0.001, -0.002, -0.02]\n"
                  "reprojection_error_px: 0.31\n"));
    ASSERT_TRUE(in.calibrated);
    EXPECT_DOUBLE_EQ(in.model.fx, 1400);
    EXPECT_DOUBLE_EQ(in.model.fy, 1395);
    EXPECT_DOUBLE_EQ(in.model.cx, 955);
    EXPECT_DOUBLE_EQ(in.model.cy, 545);
    EXPECT_DOUBLE_EQ(in.model.k1, -0.31);
    EXPECT_DOUBLE_EQ(in.model.p2, -0.002);
    EXPECT_DOUBLE_EQ(in.model.k3, -0.02);
    EXPECT_EQ(in.model.width, 1920);
    EXPECT_EQ(in.model.height, 1080);
    EXPECT_DOUBLE_EQ(in.reprojectionErrorPx, 0.31);
}

TEST(CameraIntrinsics, NonsenseCalibrationsAreRefused) {
    const std::string size = "image_width: 1920\nimage_height: 1080\n";
    const std::string d5 = "distortion_coefficients: [0, 0, 0, 0, 0]\n";
    EXPECT_THROW(loadCameraIntrinsics(
                     writeTemp("k8.yaml", size + "camera_matrix: [1, 0, 1, 0, 1, 1, 0, 0]\n" + d5)),
                 std::runtime_error);
    EXPECT_THROW(loadCameraIntrinsics(writeTemp(
                     "d4.yaml", size + "camera_matrix: [1400, 0, 955, 0, 1395, 545, 0, 0, 1]\n"
                                       "distortion_coefficients: [0, 0, 0, 0]\n")),
                 std::runtime_error);
    EXPECT_THROW(loadCameraIntrinsics(writeTemp(
                     "f0.yaml", size + "camera_matrix: [0, 0, 955, 0, 1395, 545, 0, 0, 1]\n" + d5)),
                 std::runtime_error);
    // principal point outside the image: a calibration for a different resolution
    EXPECT_THROW(
        loadCameraIntrinsics(writeTemp(
            "cx.yaml", size + "camera_matrix: [1400, 0, 2000, 0, 1395, 545, 0, 0, 1]\n" + d5)),
        std::runtime_error);
}

TEST(FrameRateMonitor, ASteadyCameraMeasuresItsRate) {
    FrameRateMonitor m(30);
    for (int i = 0; i < 300; ++i) m.add(static_cast<std::uint64_t>(1000 + i * 1000.0 / 30));
    EXPECT_NEAR(m.fps(), 30, 0.2);
    EXPECT_EQ(m.lateFrames(), 0u);
    EXPECT_NEAR(m.maxGapMs(), 33.3, 1.0);
    EXPECT_EQ(m.frames(), 300u);
}

TEST(FrameRateMonitor, ALostFrameShowsAsALateFrameEvenWhenTheRateLooksFine) {
    FrameRateMonitor m(30);
    std::uint64_t t = 0;
    for (int i = 0; i < 300; ++i) {
        t += (i == 150) ? 67 : 33;  // one frame lost on the way: a double gap
        m.add(t);
    }
    EXPECT_GT(m.fps(), 29.0);       // the average barely moves...
    EXPECT_EQ(m.lateFrames(), 1u);  // ...but the loss is counted
    EXPECT_NEAR(m.maxGapMs(), 67, 0.5);
    m.resetCounters();
    EXPECT_EQ(m.lateFrames(), 0u);
}

TEST(FrameRateMonitor, TheRateIsMeasuredOverTheWindowOnly) {
    FrameRateMonitor m(30, 2.0);  // 2 s window
    std::uint64_t t = 0;
    for (int i = 0; i < 300; ++i) m.add(t += 33);  // 10 s at 30 fps
    for (int i = 0; i < 60; ++i) m.add(t += 66);   // then 4 s at 15 fps (2K over usbipd, say)
    EXPECT_NEAR(m.fps(), 15.15, 0.3);
}
