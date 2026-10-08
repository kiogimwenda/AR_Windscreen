#pragma once
// EgoEstimator — the car's own motion, shared between threads. See docs/BUILD_GUIDE.md Part 8.2.
//
// SensorFusion (the EKF) is fed by HubReportFeeder from every hub SensorReport, on
// VehicleInterface's thread. The perception, decision and navigation threads read it. One mutex
// guards both, and readers take a SNAPSHOT (a copy) so they never hold the lock while working.

#include <cstdint>
#include <mutex>

#include "ar_drive_assist/fusion/HubReportFeeder.h"
#include "ar_drive_assist/fusion/SensorFusion.h"

namespace ar_drive_assist {

class EgoEstimator {
public:
    explicit EgoEstimator(FeederConfig cfg = {}) : feeder_(fusion_, cfg) {}

    // VehicleInterface's report callback.
    void feed(const hub_protocol::SensorReport& r) {
        std::lock_guard<std::mutex> lock(m_);
        feeder_.feed(r);
    }

    struct Snapshot {
        bool valid = false;  // false until the EKF has its first GNSS fix
        EgoState ego;        // pose (world x, y, psi), speed, yaw rate, acceleration
        SensorFusion::State x = SensorFusion::State::Zero();  // for MotionPredictor (same layout)
        SensorFusion::Cov P = SensorFusion::Cov::Identity();
        VehiclePose pose;
        LocalFrame frame;  // the world frame's origin (lat/lon), for navigation
    };
    Snapshot snapshot(std::uint64_t nowMs) const {
        std::lock_guard<std::mutex> lock(m_);
        Snapshot s;
        s.valid = fusion_.initialised();
        s.ego = feeder_.egoState(nowMs);
        s.x = fusion_.state();
        s.P = fusion_.covariance();
        s.pose = fusion_.currentPose(nowMs);
        s.frame = fusion_.frame();
        return s;
    }

private:
    mutable std::mutex m_;
    SensorFusion fusion_;
    HubReportFeeder feeder_;
};

}  // namespace ar_drive_assist
