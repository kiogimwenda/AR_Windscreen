// CameraPipeline — see include/ar_drive_assist/camera/CameraPipeline.h.
#include "ar_drive_assist/camera/CameraPipeline.h"

#include <cstdio>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <thread>

#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {

namespace {

double msSince(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

std::uint64_t nowMs() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
}

std::string fmt(double v, int decimals = 1) {
    char b[32];
    std::snprintf(b, sizeof b, "%.*f", decimals, v);
    return b;
}

}  // namespace

CameraPipeline::CameraPipeline(const CameraConfig& cfg, FrameBus& frames, EventLog* log)
    : cfg_(cfg), frames_(frames), log_(log), kind_(cameraSourceKind(cfg.source)) {
    open();
    if (!grab(pending_))
        throw std::runtime_error("camera: " + cfg_.source + " opened but delivered no frame");
    size_ = pending_.size();
    // The first frame only tells us the size. It is not published: run() may start long after
    // this constructor, and a frame stamped "now" but captured then would be stale.
    pending_ = cv::Mat();
    if (kind_ == CameraSourceKind::Device && size_ != cv::Size(cfg_.width, cfg_.height))
        throw std::runtime_error("camera: asked " + cfg_.source + " for " +
                                 std::to_string(cfg_.width) + "x" + std::to_string(cfg_.height) +
                                 ", got " + std::to_string(size_.width) + "x" +
                                 std::to_string(size_.height));

    // Calibration (Part 12.1). Uncalibrated is allowed (the bench, before Part 12) but logged:
    // nothing that measures in the image can be trusted until it is done.
    const CameraIntrinsics in = loadCameraIntrinsics(cfg_.intrinsicsPath);
    calibrated_ = in.calibrated;
    frameModel_ = in.model;
    frameModel_.width = size_.width;
    frameModel_.height = size_.height;
    if (!calibrated_) {
        if (log_)
            log_->logGeneral("WARNING camera NOT calibrated (" + cfg_.intrinsicsPath +
                             " is the template): frames are published raw, and nothing measured "
                             "in the image can be trusted until Part 12.1 is done");
    } else if (in.model.width != size_.width || in.model.height != size_.height) {
        // A calibration belongs to one resolution (and sensor mode): never rescale it.
        throw std::runtime_error("camera: calibration is for " + std::to_string(in.model.width) +
                                 "x" + std::to_string(in.model.height) + " but frames are " +
                                 std::to_string(size_.width) + "x" + std::to_string(size_.height) +
                                 ": recalibrate at this resolution (Part 12.1)");
    } else if (cfg_.undistort) {
        const cv::Matx33d K(in.model.fx, 0, in.model.cx, 0, in.model.fy, in.model.cy, 0, 0, 1);
        const cv::Vec<double, 5> D(in.model.k1, in.model.k2, in.model.p1, in.model.p2, in.model.k3);
        // alpha = 0: zoom in just enough that every output pixel has a real source pixel, so no
        // black border reaches the detector or the lane model.
        const cv::Mat newK = cv::getOptimalNewCameraMatrix(K, D, size_, 0.0, size_);
        cv::initUndistortRectifyMap(K, D, cv::Mat(), newK, size_, CV_16SC2, map1_, map2_);
        frameModel_ = CameraModel{};
        frameModel_.fx = newK.at<double>(0, 0);
        frameModel_.fy = newK.at<double>(1, 1);
        frameModel_.cx = newK.at<double>(0, 2);
        frameModel_.cy = newK.at<double>(1, 2);
        frameModel_.width = size_.width;
        frameModel_.height = size_.height;  // k1..k3 stay 0: the frames are undistorted
    }
    if (log_)
        log_->logGeneral("camera: " + cfg_.source + " " + std::to_string(size_.width) + "x" +
                         std::to_string(size_.height) + " @ " + std::to_string(cfg_.fps) +
                         " fps requested, " + (undistorting() ? "undistorting" : "raw frames"));
}

void CameraPipeline::open() {
    if (kind_ == CameraSourceKind::File) {
        cap_.open(cfg_.source, cv::CAP_FFMPEG);
        if (!cap_.isOpened())
            throw std::runtime_error("camera: cannot open video file " + cfg_.source);
        sourceFps_ = cap_.get(cv::CAP_PROP_FPS);
        if (!(sourceFps_ > 0)) sourceFps_ = cfg_.fps;
        fileStart_ = Clock::now();
        fileFrames_ = 0;
        return;
    }
    const std::string pipeline = cameraGstPipeline(cfg_);
    // Bounded open and read: a hung device must not hang this thread (header, "Failure").
    const int timeoutMs = static_cast<int>(cfg_.stallTimeoutS * 1000);
    cap_.open(pipeline, cv::CAP_GSTREAMER,
              {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, std::max(timeoutMs, 5000),
               cv::CAP_PROP_READ_TIMEOUT_MSEC, timeoutMs});
    if (!cap_.isOpened())
        throw std::runtime_error(
            "camera: cannot open '" + pipeline +
            "'. Is the camera attached to WSL (host/scripts/attach_usb_devices.ps1), and does it "
            "offer this format, size and rate? List them with: build/host/camera_check --list");
    sourceFps_ = cfg_.fps;
}

bool CameraPipeline::grab(cv::Mat& raw) {
    if (kind_ == CameraSourceKind::File && cfg_.realtime && fileFrames_ > 0) {
        // Pace a replay at the file's own rate, as a camera would deliver it.
        const auto due = fileStart_ + std::chrono::duration<double>(fileFrames_ / sourceFps_);
        std::this_thread::sleep_until(due);
    }
    raw = cv::Mat();  // a NEW buffer, never one a published frame may still share (header)
    if (!cap_.read(raw) || raw.empty()) {
        if (kind_ != CameraSourceKind::File || !cfg_.loop) return false;
        open();  // end of the file: start again
        raw = cv::Mat();
        if (!cap_.read(raw) || raw.empty()) return false;
    }
    if (kind_ == CameraSourceKind::File) {
        ++fileFrames_;
        if (raw.cols != cfg_.width || raw.rows != cfg_.height) {
            cv::Mat resized;
            cv::resize(raw, resized, {cfg_.width, cfg_.height}, 0, 0, cv::INTER_LINEAR);
            raw = resized;
        }
    }
    return true;
}

bool CameraPipeline::read(CameraFrame& out) {
    cv::Mat raw;
    if (!grab(raw)) return false;
    out.timestampMs = nowMs();
    out.seq = seq_++;
    ++stats_.captured;
    if (!map1_.empty()) {
        const auto t0 = Clock::now();
        cv::Mat undistorted;  // new for the same reason as the capture buffer
        cv::remap(raw, undistorted, map1_, map2_, cv::INTER_LINEAR);
        out.bgr = undistorted;
        undistortMsTotal_ += msSince(t0);
        stats_.meanUndistortMs = undistortMsTotal_ / static_cast<double>(stats_.captured);
    } else {
        out.bgr = raw;
    }
    return true;
}

void CameraPipeline::run(const std::atomic<bool>& stop) {
    FrameRateMonitor monitor(kind_ == CameraSourceKind::File ? sourceFps_ : cfg_.fps);
    auto lastFrame = Clock::now();
    auto lastReport = Clock::now();
    std::uint64_t dropsAtReport = 0;
    bool ended = false;
    while (!stop.load()) {
        CameraFrame f;
        if (ended || !read(f)) {
            if (kind_ == CameraSourceKind::File && !cfg_.loop) {
                if (!ended && log_) log_->logGeneral("camera: end of " + cfg_.source);
                ended = true;  // a finished replay idles until shutdown
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            if (msSince(lastFrame) > cfg_.stallTimeoutS * 1000) {
                const std::string why = "no frame from " + cfg_.source + " for " +
                                        fmt(msSince(lastFrame) / 1000, 2) + " s";
                if (log_) log_->logFault("CameraPipeline", why);
                throw std::runtime_error("camera: " + why);  // SystemManager shuts down
            }
            continue;  // a read timed out (bounded by stallTimeoutS); try again
        }
        lastFrame = Clock::now();
        monitor.add(f.timestampMs);
        if (!frames_.push(std::move(f))) ++stats_.busDrops;

        if (msSince(lastReport) >= cfg_.reportEveryS * 1000) {
            // Part 6.2: the sustained rate, measured, logged.
            if (log_)
                log_->logGeneral(
                    "camera: " + fmt(monitor.fps()) + " fps (target " +
                    fmt(kind_ == CameraSourceKind::File ? sourceFps_ : cfg_.fps) +
                    "), longest gap " + fmt(monitor.maxGapMs(), 0) + " ms, late " +
                    std::to_string(monitor.lateFrames()) + ", bus drops " +
                    std::to_string(stats_.busDrops - dropsAtReport) +
                    (undistorting() ? ", undistort " + fmt(stats_.meanUndistortMs, 2) + " ms/frame"
                                    : ""));
            monitor.resetCounters();
            dropsAtReport = stats_.busDrops;
            lastReport = Clock::now();
        }
    }
}

}  // namespace ar_drive_assist
