#pragma once
// LidarReplay — Part 5.1's LidarThread when the LiDAR is a recording (Recording.h): each scan is
// read from disk and published on lidarBus at its recorded time, stamped with the host clock, as
// the Livox driver will publish live scans (Phase 6).
//
// A replay can add a test obstacle seen by both sensors (lidar/InjectedObstacle.h): a recorded
// drive rarely contains the one thing the brake path exists for.

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "ar_drive_assist/lidar/InjectedObstacle.h"
#include "ar_drive_assist/lidar/LidarProcessor.h"
#include "ar_drive_assist/system/PipelineMessages.h"
#include "ar_drive_assist/system/Recording.h"

namespace ar_drive_assist {

class EventLog;

class LidarReplay {
public:
    // Throws std::runtime_error if the recording's LiDAR files are missing or malformed. `tap`, if
    // set, is called with every published scan on this thread and must return at once
    // (system/Recorder.h).
    LidarReplay(const Recording& rec, LidarBus& out, EventLog* log = nullptr,
                std::optional<InjectedObstacle> obstacle = std::nullopt, bool loop = false,
                std::function<void(const LidarFrame&)> tap = {});

    void run(const std::atomic<bool>& stop);

    // Scan i as it will be published, with the obstacle at replay time tS (tests).
    // Fits the ground (LidarProcessor) too, so call in scan order.
    LidarFrame scan(std::size_t i, double tS, std::uint64_t hostMs);
    std::size_t scans() const { return times_.size(); }

private:
    Recording rec_;
    LidarBus& out_;
    EventLog* log_;
    std::optional<ObstacleModel> obstacle_;
    bool loop_;
    std::vector<ScanTime> times_;
    LidarProcessor processor_;
    std::function<void(const LidarFrame&)> tap_;
};

}  // namespace ar_drive_assist
