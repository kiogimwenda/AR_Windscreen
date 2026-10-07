// CameraPipeline through real OpenCV and GStreamer, with no camera: GStreamer's videotestsrc
// stands in for the device (Phase 4's exit check on the real camera is build/host/camera_check).
// Label `camera`:  ctest --test-dir build -L camera
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ar_drive_assist/camera/CameraPipeline.h"

using namespace ar_drive_assist;
using Clock = std::chrono::steady_clock;

namespace {

const std::string kTemplate = std::string(HOST_SOURCE_DIR) + "/config/camera_intrinsics.yaml";

// A live test source: frames arrive at `fps` in real time, like a camera's.
std::string liveSource(int w, int h, int fps, const std::string& extra = "") {
    return "gst:videotestsrc is-live=true pattern=ball ! video/x-raw,width=" + std::to_string(w) +
           ",height=" + std::to_string(h) + ",framerate=" + std::to_string(fps) + "/1" + extra +
           " ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false";
}

CameraConfig config(const std::string& source, int w, int h, int fps = 30) {
    CameraConfig c;
    c.source = source;
    c.width = w;
    c.height = h;
    c.fps = fps;
    c.intrinsicsPath = kTemplate;
    return c;
}

std::string writeIntrinsics(const std::string& name, const CameraModel& m) {
    const std::string path = ::testing::TempDir() + name;
    std::ofstream(path) << "image_width: " << m.width << "\nimage_height: " << m.height
                        << "\ncamera_matrix: [" << m.fx << ", 0, " << m.cx << ", 0, " << m.fy
                        << ", " << m.cy << ", 0, 0, 1]\ndistortion_coefficients: [" << m.k1 << ", "
                        << m.k2 << ", " << m.p1 << ", " << m.p2 << ", " << m.k3
                        << "]\nreprojection_error_px: 0.2\n";
    return path;
}

double msSince(Clock::time_point t) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

}  // namespace

TEST(CameraPipeline, PublishesEveryFrameInOrderAtTheSourceRate) {
    CameraPipeline::FrameBus bus;
    CameraPipeline cam(config(liveSource(640, 360, 30), 640, 360), bus);
    std::atomic<bool> stop{false};
    std::thread t([&] { cam.run(stop); });

    std::vector<CameraFrame> got;
    const auto start = Clock::now();
    while (msSince(start) < 2000) {
        CameraFrame f;
        if (bus.pop(f))
            got.push_back(f);
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    stop = true;
    t.join();

    ASSERT_GE(got.size(), 50u);  // ~60 in 2 s
    FrameRateMonitor m(30);
    for (std::size_t i = 0; i < got.size(); ++i) {
        EXPECT_EQ(got[i].bgr.size(), cv::Size(640, 360));
        EXPECT_EQ(got[i].bgr.type(), CV_8UC3);
        if (i > 0) {
            EXPECT_EQ(got[i].seq, got[i - 1].seq + 1) << "a frame went missing";
            EXPECT_GE(got[i].timestampMs, got[i - 1].timestampMs);
        }
        m.add(got[i].timestampMs);
    }
    EXPECT_NEAR(m.fps(), 30, 2.0);
    EXPECT_EQ(cam.stats().busDrops, 0u);
}

TEST(CameraPipeline, APublishedFrameKeepsItsPixelsAfterTheNextCapture) {
    // The header's "Why a new Mat": the inference thread may still be reading frame N when frame
    // N+1 is captured. Frame N must not change under it.
    CameraPipeline::FrameBus bus;
    CameraPipeline cam(config(liveSource(320, 240, 30), 320, 240), bus);
    CameraFrame first, second;
    ASSERT_TRUE(cam.read(first));
    const cv::Mat copy = first.bgr.clone();
    for (int i = 0; i < 5; ++i) ASSERT_TRUE(cam.read(second));  // the ball has moved
    EXPECT_NE(first.bgr.data, second.bgr.data);
    EXPECT_EQ(cv::norm(first.bgr, copy, cv::NORM_INF), 0.0) << "a held frame was overwritten";
    EXPECT_GT(cv::norm(first.bgr, second.bgr, cv::NORM_INF), 0.0);
}

TEST(CameraPipeline, TheDevicePipelineDecodesBothCameraFormats) {
    // cameraGstPipeline() for a real device, with v4l2src replaced by a test source that emits
    // exactly what a UVC camera would: JPEG frames for MJPG, raw YUY2 for YUYV. Everything after
    // the source is the string the camera will run.
    for (const std::string fmt : {"MJPG", "YUYV"}) {
        CameraConfig dev = config("/dev/video9", 640, 360);
        dev.format = fmt;
        std::string p = cameraGstPipeline(dev);
        const std::string src = "v4l2src device=/dev/video9";
        const std::string fake = fmt == "MJPG"
                                     ? "videotestsrc is-live=true ! "
                                       "video/x-raw,width=640,height=360,framerate=30/1 ! jpegenc"
                                     : "videotestsrc is-live=true";
        p.replace(p.find(src), src.size(), fake);
        CameraPipeline::FrameBus bus;
        CameraPipeline cam(config("gst:" + p, 640, 360), bus);
        CameraFrame f;
        ASSERT_TRUE(cam.read(f)) << fmt;
        EXPECT_EQ(f.bgr.size(), cv::Size(640, 360)) << fmt;
        EXPECT_EQ(f.bgr.type(), CV_8UC3) << fmt;
    }
}

TEST(CameraPipeline, UndistortedFramesAgreeWithThePublishedCameraModel) {
    // A lens with strong barrel distortion images a grid of 3D points; dots are drawn where the
    // LENS puts them (raw pixels). After the pipeline undistorts, each dot must sit where
    // frameModel() (zero distortion) projects the same point. That is the contract every
    // consumer relies on (CameraConfig.h, "The camera model published with the frames").
    CameraModel lens;
    lens.width = 640;
    lens.height = 480;
    lens.fx = lens.fy = 420;
    lens.cx = 322;
    lens.cy = 238;
    lens.k1 = -0.28;
    lens.k2 = 0.09;
    std::vector<Eigen::Vector3d> pts;
    for (double x = -0.6; x <= 0.61; x += 0.2)
        for (double y = -0.45; y <= 0.46; y += 0.15) pts.emplace_back(x, y, 1.0);

    cv::Mat raw(lens.height, lens.width, CV_8UC3, cv::Scalar::all(0));
    for (const auto& p : pts) {
        Eigen::Vector2d px;
        if (lens.project(p, px))
            cv::circle(raw, {int(std::lround(px.x())), int(std::lround(px.y()))}, 3,
                       cv::Scalar::all(255), cv::FILLED);
    }
    const std::string video = ::testing::TempDir() + "lens_dots.avi";
    {
        cv::VideoWriter w(video, cv::CAP_FFMPEG, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                          raw.size());
        ASSERT_TRUE(w.isOpened());
        for (int i = 0; i < 3; ++i) w.write(raw);
    }
    CameraConfig c = config(video, 640, 480);
    c.intrinsicsPath = writeIntrinsics("lens.yaml", lens);
    c.realtime = false;
    c.loop = false;
    CameraPipeline::FrameBus bus;
    CameraPipeline cam(c, bus);
    ASSERT_TRUE(cam.calibrated());
    ASSERT_TRUE(cam.undistorting());
    const CameraModel& fm = cam.frameModel();
    EXPECT_EQ(fm.k1, 0);
    EXPECT_EQ(fm.k2, 0);

    CameraFrame f;
    ASSERT_TRUE(cam.read(f));
    cv::Mat grey;
    cv::cvtColor(f.bgr, grey, cv::COLOR_BGR2GRAY);
    int checked = 0;
    double worstNew = 0, worstIfDistortedAgain = 0;
    for (const auto& p : pts) {
        Eigen::Vector2d expect;
        if (!fm.project(p, expect)) continue;
        if (expect.x() < 10 || expect.y() < 10 || expect.x() > 630 || expect.y() > 470) continue;
        // centroid of the dot near where the frame model says it is
        const cv::Rect win(int(expect.x()) - 9, int(expect.y()) - 9, 19, 19);
        const cv::Moments mo = cv::moments(grey(win), false);
        ASSERT_GT(mo.m00, 0) << "no dot near " << expect.transpose();
        const Eigen::Vector2d found(win.x + mo.m10 / mo.m00, win.y + mo.m01 / mo.m00);
        worstNew = std::max(worstNew, (found - expect).norm());
        // What a consumer would compute if it used the raw calibration on these frames:
        Eigen::Vector2d wrong;
        CameraModel twice = fm;
        twice.k1 = lens.k1;
        twice.k2 = lens.k2;
        twice.project(p, wrong);
        worstIfDistortedAgain = std::max(worstIfDistortedAgain, (found - wrong).norm());
        ++checked;
    }
    std::printf(
        "  %d dots: worst error %.2f px against frameModel(), %.1f px if the lens "
        "distortion were applied again\n",
        checked, worstNew, worstIfDistortedAgain);
    EXPECT_GE(checked, 25);
    EXPECT_LT(worstNew, 1.0) << "undistorted frame and frameModel() disagree";
    EXPECT_GT(worstIfDistortedAgain, 5.0) << "the test lens should make double distortion visible";
}

TEST(CameraPipeline, ACalibrationForAnotherResolutionIsRefused) {
    CameraModel m;
    m.width = 1280;
    m.height = 720;
    m.fx = m.fy = 900;
    m.cx = 640;
    m.cy = 360;
    CameraConfig c = config(liveSource(640, 360, 30), 640, 360);
    c.intrinsicsPath = writeIntrinsics("other_res.yaml", m);
    CameraPipeline::FrameBus bus;
    try {
        CameraPipeline cam(c, bus);
        FAIL() << "a 1280x720 calibration was applied to 640x360 frames";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("recalibrate"), std::string::npos) << e.what();
    }
}

TEST(CameraPipeline, AStalledCameraIsAFaultNotAHang) {
    // Frames every 500 ms against a 200 ms stall limit: each read must give up at the limit
    // (bounded reads, CAP_PROP_READ_TIMEOUT_MSEC), and run() must throw. With unbounded reads it
    // would sit in read() and receive the next frame instead.
    CameraConfig c = config(liveSource(320, 240, 2), 320, 240, 2);
    c.stallTimeoutS = 0.2;
    CameraPipeline::FrameBus bus;
    CameraPipeline cam(c, bus);
    std::atomic<bool> stop{false}, threw{false};
    const auto start = Clock::now();
    double tookMs = 0;
    std::thread t([&] {
        try {
            cam.run(stop);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        tookMs = msSince(start);
    });
    while (!threw && msSince(start) < 2000)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    stop = true;  // so a broken pipeline ends the test instead of hanging it
    t.join();
    EXPECT_TRUE(threw) << "a stalled camera was not reported";
    EXPECT_LT(tookMs, 1500);
}

TEST(CameraPipeline, StopIsHonouredPromptly) {
    CameraPipeline::FrameBus bus;
    CameraPipeline cam(config(liveSource(320, 240, 30), 320, 240), bus);
    std::atomic<bool> stop{false};
    std::thread t([&] { cam.run(stop); });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const auto asked = Clock::now();
    stop = true;
    t.join();
    EXPECT_LT(msSince(asked), 300);
}

TEST(CameraPipeline, AFullBusDropsAndCountsTheNewFrame) {
    // Nobody consumes: the bus holds 4, every later frame is dropped and counted (RingBuffer.h).
    CameraPipeline::FrameBus bus;
    CameraPipeline cam(config(liveSource(320, 240, 30), 320, 240), bus);
    std::atomic<bool> stop{false};
    std::thread t([&] { cam.run(stop); });
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    stop = true;
    t.join();
    const auto s = cam.stats();
    EXPECT_GE(s.captured, 15u);
    EXPECT_EQ(s.busDrops, s.captured - CameraPipeline::FrameBus::capacity());
    CameraFrame f;
    ASSERT_TRUE(bus.pop(f));
    EXPECT_EQ(f.seq, 0u) << "the oldest frames stay; the consumer's popLatest() discards them";
}

TEST(CameraPipeline, AFileReplaysAtSizeAndEndsWhenNotLooping) {
    const std::string video = ::testing::TempDir() + "five.avi";
    {
        cv::VideoWriter w(video, cv::CAP_FFMPEG, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                          {320, 180});
        ASSERT_TRUE(w.isOpened());
        for (int i = 0; i < 5; ++i) w.write(cv::Mat(180, 320, CV_8UC3, cv::Scalar::all(40 * i)));
    }
    CameraConfig c = config(video, 640, 360);  // replayed at the configured camera size
    c.loop = false;
    c.realtime = false;
    CameraPipeline::FrameBus bus;
    CameraPipeline cam(c, bus);
    CameraFrame f;
    int n = 0;
    while (cam.read(f)) {
        EXPECT_EQ(f.bgr.size(), cv::Size(640, 360));
        ++n;
    }
    EXPECT_EQ(n, 4);  // five in the file; the first was the constructor's size probe
}
