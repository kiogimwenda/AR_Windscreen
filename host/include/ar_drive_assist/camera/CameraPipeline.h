#pragma once
// CameraPipeline — capture -> undistort -> timestamp -> frameBus. See docs/BUILD_GUIDE.md Part 6,
// and CameraConfig.h for the sources, the GStreamer pipeline and the published camera model.
//
// One thread (Part 5.1's CameraThread). It does no ML and no rendering: a frame is captured,
// stamped, undistorted and pushed, and the next capture starts.
//
// ---------------------------------------------------------------------------------------------
// One frame, step by step
//
//   capture    VideoCapture::read() into a NEW cv::Mat every time (see "Why a new Mat" below)
//   stamp      steady-clock milliseconds, taken as soon as read() returns; the sequence number
//              counts every captured frame, so a gap downstream means a frame was dropped
//   undistort  cv::remap() through maps computed ONCE at startup from the calibration
//              (cv::initUndistortRectifyMap). Same result as Part 6.2's cv::undistort() per frame,
//              which recomputes those maps on every call; remap alone costs a fraction of that
//   push       onto frameBus. If the bus is full (the consumer has stalled for 4 frames) the NEW
//              frame is dropped and counted; the consumer's popLatest() drops the old ones
//              (RingBuffer.h, "Overflow policy")
//
// ---------------------------------------------------------------------------------------------
// Why a new Mat for every frame
//
// cv::Mat is reference-counted: a frame on the bus shares its pixels with the copy the inference
// thread is reading. VideoCapture::read(mat) calls mat.create(), which REUSES the existing buffer
// whenever the size and type match, even if another thread still holds a reference to it. Reading
// every frame into the same Mat would therefore overwrite, mid-inference, the pixels of a frame
// already published. A fresh Mat per frame costs one allocation (the allocator reuses the freed
// block) and makes that impossible. test_camera_pipeline checks it.
//
// ---------------------------------------------------------------------------------------------
// Failure
//
// A camera that stops delivering (unplugged, usbipd detached, driver hang) for stallTimeoutS is a
// FAULT: run() throws, SystemManager logs it and shuts the system down, and the hub's watchdog
// releases everything. A driver-assist system that has lost its camera is not degraded, it is
// blind. Reads are bounded by a timeout, so a hung device cannot hang this thread, and run()
// returns promptly once stop is set.
// ---------------------------------------------------------------------------------------------

#include <atomic>
#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "ar_drive_assist/camera/CameraConfig.h"
#include "ar_drive_assist/camera/CameraFrame.h"
#include "ar_drive_assist/common/RingBuffer.h"

namespace ar_drive_assist {

class EventLog;

class CameraPipeline {
public:
    using FrameBus = RingBuffer<CameraFrame, 4>;

    // Opens the source and reads the first frame (so the size is known and checked). Throws
    // std::runtime_error, naming the source, if it cannot be opened, delivers the wrong size, or
    // does not match the calibration.
    CameraPipeline(const CameraConfig& cfg, FrameBus& frames, EventLog* log = nullptr);

    // SystemManager contract: capture and publish until stop.
    void run(const std::atomic<bool>& stop);

    // One frame, synchronously: what run() publishes. False at the end of a non-looping file, or
    // if the source failed. For tools and tests; never call it while run() is running.
    bool read(CameraFrame& out);

    // The camera model of the PUBLISHED frames (see CameraConfig.h): the undistorted pinhole model
    // when undistorting, otherwise the lens as calibrated. Meaningful only if calibrated().
    const CameraModel& frameModel() const { return frameModel_; }
    bool calibrated() const { return calibrated_; }
    bool undistorting() const { return !map1_.empty(); }
    cv::Size frameSize() const { return size_; }
    double sourceFps() const { return sourceFps_; }

    struct Stats {
        std::uint64_t captured = 0;  // frames read from the source
        std::uint64_t busDrops = 0;  // dropped because frameBus was full
        double meanUndistortMs = 0;  // per frame, when undistorting
    };
    Stats stats() const { return stats_; }  // read after run() returns, or from the run thread

private:
    using Clock = std::chrono::steady_clock;
    bool grab(cv::Mat& raw);
    void open();

    CameraConfig cfg_;
    FrameBus& frames_;
    EventLog* log_;
    CameraSourceKind kind_;
    cv::VideoCapture cap_;
    cv::Size size_;
    double sourceFps_ = 0;
    bool calibrated_ = false;
    CameraModel frameModel_;
    cv::Mat map1_, map2_;  // undistortion maps (empty = publish raw)
    cv::Mat pending_;      // the constructor's size probe (never published)
    std::uint64_t seq_ = 0;
    Clock::time_point fileStart_;
    std::uint64_t fileFrames_ = 0;
    Stats stats_;
    double undistortMsTotal_ = 0;
};

}  // namespace ar_drive_assist
