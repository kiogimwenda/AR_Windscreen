#pragma once
// ExtrinsicMonitorThread — runs the ExtrinsicMonitor (Part 12.2.1) beside the pipeline, never in
// it. The camera and the LiDAR hand it frames through their taps (as they do the Recorder); about
// every `periodMs` it takes the next scan with the newest frame, brings the scan's points to the
// frame's time with the ego estimate, and gives the pair to the monitor. FusionThread reads the
// monitor's current() each frame (a mutex-guarded copy).
//
// The taps return at once: a frame is kept by reference (CameraFrame shares its pixels), and a
// scan is copied only when the monitor is ready for one.

#include <atomic>
#include <cstdint>
#include <mutex>

#include "ar_drive_assist/camera/CameraFrame.h"
#include "ar_drive_assist/fusion/EgoEstimator.h"
#include "ar_drive_assist/lidar/LidarFrame.h"
#include "ar_drive_assist/scene/ExtrinsicMonitor.h"

namespace ar_drive_assist {

class ExtrinsicMonitorThread {
public:
    ExtrinsicMonitorThread(const Eigen::Isometry3d& baselineCameraFromLidar,
                           const Eigen::Isometry3d& vehicleFromLidar, const CameraModel& camera,
                           EgoEstimator& ego, EventLog* log = nullptr,
                           ExtrinsicMonitorConfig cfg = {}, int periodMs = 500);

    void onFrame(const CameraFrame& f);
    void onScan(const LidarFrame& s);
    void run(const std::atomic<bool>& stop);

    const ExtrinsicMonitor& monitor() const { return monitor_; }

private:
    ExtrinsicMonitor monitor_;
    EgoEstimator& ego_;
    int periodMs_;

    std::mutex m_;
    CameraFrame latestFrame_;
    LidarFrame scan_;
    CameraFrame pairedFrame_;
    bool want_ = false, ready_ = false;
};

}  // namespace ar_drive_assist
