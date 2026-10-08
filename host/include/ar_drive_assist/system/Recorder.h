#pragma once
// Recorder — writes what the live system sees as a recording (system/Recording.h), so the
// project's own drives can be replayed through the whole host exactly as KITTI is (Part 13.2), and
// so Part 12.2's LiDAR-camera calibration has synchronised frames and scans to work from
// (host/scripts/run_lidar_camera_calibration.py). `ar_drive_assist --record DIR`.
//
//   camera   every published frame (CameraPipeline::setFrameTap): AFTER undistortion, so the
//            recording's camera_intrinsics.yaml is the frame model (no distortion), the same
//            pairing a replay hands back to fusion
//   LiDAR    every scan (setScanTap), points in the LiDAR frame
//   hub      every SensorReport (VehicleInterface's report callback), by its host arrival time
//
// Never slows the system it records: the taps copy into bounded queues and return; this thread
// writes. If the writer falls behind (a slow disk), the newest data is dropped and counted, and the
// counts go to the EventLog every reportEveryS and at the end; a recording with drops says so in
// its recording.yaml. Times are the host's steady clock, in microseconds.
//
// Not kept, by the format: each LiDAR point's own time (a replay gives all of a scan's points the
// scan's time, as KITTI's do). The Livox's 100 ms scans at walking pace (calibration) are
// unaffected; on a drive the motion within a scan is not compensated on replay.

#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include "ar_drive_assist/camera/CameraFrame.h"
#include "ar_drive_assist/common/Camera.h"
#include "ar_drive_assist/lidar/LidarFrame.h"
#include "ar_drive_assist/system/Recording.h"
#include "ar_drive_assist/vehicle/HubProtocol.h"

namespace cv {
class VideoWriter;
}

namespace ar_drive_assist {

class EventLog;

struct RecorderConfig {
    std::string dir;          // created; must not already hold a recording
    std::string name, notes;  // recording.yaml
    CameraModel frameModel;   // CameraPipeline::frameModel(): what the frames are
    double fps = 30;          // the video's nominal rate (replays use camera.csv's times)
    SensorExtrinsics extrinsics;
    std::size_t maxQueuedFrames = 30, maxQueuedScans = 10;
    double reportEveryS = 10.0;
};

class Recorder {
public:
    // Creates the directory and its files. Throws std::runtime_error if it cannot, or if `dir`
    // already holds a recording (never overwritten).
    Recorder(RecorderConfig cfg, EventLog* log = nullptr);
    ~Recorder();
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    // The taps: any thread, return at once.
    void onFrame(const CameraFrame& f);
    void onScan(const LidarFrame& s);
    void onReport(const hub_protocol::SensorReport& r, std::uint64_t hostMs);

    // Writes until stop, then drains what is queued and finishes the files.
    void run(const std::atomic<bool>& stop);

    struct Stats {
        std::uint64_t frames = 0, scans = 0, reports = 0;
        std::uint64_t droppedFrames = 0, droppedScans = 0;
    };
    Stats stats() const;

private:
    bool writeOne();  // false when nothing was queued
    void finish();

    RecorderConfig cfg_;
    EventLog* log_;
    std::unique_ptr<cv::VideoWriter> video_;
    std::ofstream cameraCsv_, lidarCsv_, hubCsv_;
    bool finished_ = false;

    mutable std::mutex m_;
    std::deque<CameraFrame> frames_;
    std::deque<LidarFrame> scans_;
    Stats stats_;
};

}  // namespace ar_drive_assist
