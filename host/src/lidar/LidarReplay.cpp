// LidarReplay — see include/ar_drive_assist/lidar/LidarReplay.h.
#include "ar_drive_assist/lidar/LidarReplay.h"

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>

#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {

namespace {
std::uint64_t hostMs() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
}
}  // namespace

LidarReplay::LidarReplay(const Recording& rec, LidarBus& out, EventLog* log,
                         std::optional<InjectedObstacle> obstacle, bool loop,
                         std::function<void(const LidarFrame&)> tap)
    : rec_(rec), out_(out), log_(log), loop_(loop), tap_(std::move(tap)) {
    times_ = loadScanTimes(rec_.dir);
    readScan(rec_.scanFile(times_.front().index));  // fail now, not mid-run, if unreadable
    if (obstacle) obstacle_.emplace(*obstacle, rec_.hubReports());
}

LidarFrame LidarReplay::scan(std::size_t i, double tS, std::uint64_t hostNow) {
    LidarFrame f;
    f.timestampMs = hostNow;
    const auto raw = readScan(rec_.scanFile(times_[i].index));
    f.points.reserve(raw.size());
    const std::int64_t tUs = static_cast<std::int64_t>(hostNow) * 1000;
    for (const auto& r : raw) {
        LidarPoint p;
        p.p = r.head<3>();
        // KITTI stores reflectance as 0-1; the Livox convention (and Part 8.3's thresholds) is
        // 0-255.
        p.intensity = r[3] <= 1.0f ? r[3] * 255.0f : r[3];
        p.tUs = tUs;
        f.points.push_back(p);
    }
    if (obstacle_)
        for (LidarPoint p : obstacle_->points(tS, rec_.vehicleFromLidar.inverse())) {
            p.tUs = tUs;
            f.points.push_back(p);
        }
    const auto g = processor_.fitGround(f.points, rec_.vehicleFromLidar, hostNow);
    f.ground = g.ground;
    f.groundValid = g.valid;
    return f;
}

void LidarReplay::run(const std::atomic<bool>& stop) {
    bool announced = false;
    do {
        const auto start = std::chrono::steady_clock::now();
        const std::int64_t t0 = times_.front().tUs;
        for (std::size_t i = 0; i < times_.size() && !stop.load(); ++i) {
            std::this_thread::sleep_until(start + std::chrono::microseconds(times_[i].tUs - t0));
            const double tS = (times_[i].tUs - t0) * 1e-6;
            if (obstacle_ && !announced && tS >= obstacle_->spec().appearS && log_) {
                char b[160];
                std::snprintf(b, sizeof b,
                              "replay: INJECTED test obstacle appears %.1f m ahead, speed %.1f m/s "
                              "(LiDAR and camera, testing)",
                              obstacle_->spec().startGapM, obstacle_->spec().speedMps);
                log_->logGeneral(b);
                announced = true;
            }
            LidarFrame f = scan(i, tS, hostMs());
            if (tap_) tap_(f);
            out_.push(std::move(f));  // a full bus: the fusion thread takes the newest
        }
        if (log_ && !stop.load()) log_->logGeneral("replay: end of LiDAR recording " + rec_.name);
    } while (loop_ && !stop.load());
    while (!stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

}  // namespace ar_drive_assist
