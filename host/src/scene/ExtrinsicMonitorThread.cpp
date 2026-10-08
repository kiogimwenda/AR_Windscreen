// ExtrinsicMonitorThread — see include/ar_drive_assist/scene/ExtrinsicMonitorThread.h.
#include "ar_drive_assist/scene/ExtrinsicMonitorThread.h"

#include <chrono>
#include <thread>

namespace ar_drive_assist {

ExtrinsicMonitorThread::ExtrinsicMonitorThread(const Eigen::Isometry3d& baselineCameraFromLidar,
                                               const Eigen::Isometry3d& vehicleFromLidar,
                                               const CameraModel& camera, EgoEstimator& ego,
                                               EventLog* log, ExtrinsicMonitorConfig cfg,
                                               int periodMs)
    : monitor_(baselineCameraFromLidar, vehicleFromLidar, camera, cfg, log),
      ego_(ego),
      periodMs_(periodMs) {}

void ExtrinsicMonitorThread::onFrame(const CameraFrame& f) {
    std::lock_guard<std::mutex> lock(m_);
    latestFrame_ = f;  // shares the pixels
}

void ExtrinsicMonitorThread::onScan(const LidarFrame& s) {
    std::lock_guard<std::mutex> lock(m_);
    if (!want_ || latestFrame_.bgr.empty()) return;
    scan_ = s;  // the one copy, only when wanted
    pairedFrame_ = latestFrame_;
    want_ = false;
    ready_ = true;
}

void ExtrinsicMonitorThread::run(const std::atomic<bool>& stop) {
    auto next = std::chrono::steady_clock::now();
    while (!stop.load()) {
        if (std::chrono::steady_clock::now() >= next) {
            std::lock_guard<std::mutex> lock(m_);
            if (!want_ && !ready_) want_ = true;
        }
        CameraFrame frame;
        LidarFrame scan;
        {
            std::lock_guard<std::mutex> lock(m_);
            if (ready_) {
                frame = std::move(pairedFrame_);
                scan = std::move(scan_);
                ready_ = false;
            }
        }
        if (frame.bgr.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        next = std::chrono::steady_clock::now() + std::chrono::milliseconds(periodMs_);
        const EgoEstimator::Snapshot e = ego_.snapshot(frame.timestampMs);
        const EgoMotion motion{e.ego.speedMps, e.ego.yawRateRadPerS};
        monitor_.addFrame(frame.bgr, static_cast<std::int64_t>(frame.timestampMs) * 1000,
                          scan.points, motion);
    }
}

}  // namespace ar_drive_assist
