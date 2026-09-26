#include "ar_drive_assist/safety/MultiObjectTracker.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "ar_drive_assist/safety/Hungarian.h"

namespace ar_drive_assist {
namespace {

constexpr double kReject = 1e9;  // cost of a forbidden pairing (outside the gate / wrong class)

bool classesCompatible(int32_t track, int32_t meas) {
    return track < 0 || meas < 0 || track == meas;
}

}  // namespace

MotionNoise TrackerConfig::noiseFor(int32_t cls) const {
    const auto it = noiseByClass.find(cls);
    if (it != noiseByClass.end()) return it->second;
    const auto unk = noiseByClass.find(-1);
    return unk != noiseByClass.end() ? unk->second : MotionNoise{};
}

TrackerConfig loadTrackerConfig(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("config: cannot load " + path + ": " + e.what());
    }
    auto req = [&](const YAML::Node& n, const std::string& key) -> YAML::Node {
        if (!n[key]) throw std::runtime_error("config: " + path + ": '" + key + "' missing");
        return n[key];
    };
    TrackerConfig c;
    const YAML::Node t = req(root, "tracker");
    c.gateChi2 = req(t, "gate_chi2").as<double>();
    c.confirmHits = req(t, "confirm_hits").as<int>();
    c.tentativeMaxMisses = req(t, "tentative_max_misses").as<int>();
    c.maxCoastS = req(t, "max_coast_s").as<double>();
    c.replayWindowMs = req(t, "replay_window_ms").as<std::uint64_t>();
    // Class names as in ObjectClass (Postprocess.h), plus "unclassified" for LiDAR-only clusters.
    static const std::map<std::string, int32_t> kIds = {{"vehicle", 0},  {"pedestrian", 1},
                                                        {"cyclist", 2},  {"sign", 3},
                                                        {"obstacle", 4}, {"unclassified", -1}};
    const YAML::Node classes = req(root, "classes");
    for (const auto& [name, id] : kIds) {
        const YAML::Node n = req(classes, name);
        MotionNoise m;
        m.accelStd = req(n, "accel_std").as<double>();
        m.yawAccelStd = req(n, "yaw_accel_std").as<double>();
        m.headingDriftStd = req(n, "heading_drift_std").as<double>();
        m.stopTau = req(n, "stop_tau_s").as<double>();
        m.switchRate = req(n, "model_switch_rate").as<double>();
        m.initialSpeedStd = req(n, "initial_speed_std").as<double>();
        if (m.accelStd <= 0 || m.stopTau <= 0 || m.initialSpeedStd <= 0) {
            throw std::runtime_error("config: " + path + ": class '" + name +
                                     "' has a non-positive noise value");
        }
        c.noiseByClass[id] = m;
    }
    return c;
}

MultiObjectTracker::MultiObjectTracker(TrackerConfig cfg) : cfg_(std::move(cfg)) {}

Eigen::Matrix2d MultiObjectTracker::rotation(double psi) {
    Eigen::Matrix2d R;
    R << std::cos(psi), -std::sin(psi), std::sin(psi), std::cos(psi);
    return R;
}

Eigen::Vector2d MultiObjectTracker::toWorld(const EgoPose& ego, const Eigen::Vector2d& p) {
    return Eigen::Vector2d(ego.x, ego.y) + rotation(ego.psi) * p;
}

void MultiObjectTracker::update(const MeasurementBatch& batch) {
    const std::uint64_t newest = replay_.empty() ? 0 : replay_.back().timestampMs;
    if (replay_.empty() || batch.timestampMs >= newest) {
        replay_.push_back({batch.timestampMs, tracks_, nextId_, batch});
        process(batch);
    } else {
        // Late: find the first processed batch newer than this one and rewind to before it.
        auto it = std::find_if(replay_.begin(), replay_.end(), [&](const Snapshot& s) {
            return s.timestampMs > batch.timestampMs;
        });
        if (it == replay_.begin()) {
            // Older than every stored snapshot: none is guaranteed to predate it, so replaying
            // would apply it out of order. Drop it, and count it.
            ++droppedLate_;
            return;
        }
        tracks_ = it->tracks;
        nextId_ = it->nextId;
        std::vector<MeasurementBatch> redo;
        redo.push_back(batch);
        for (auto j = it; j != replay_.end(); ++j) redo.push_back(j->batch);
        replay_.erase(it, replay_.end());
        for (const MeasurementBatch& b : redo) {
            replay_.push_back({b.timestampMs, tracks_, nextId_, b});
            process(b);
        }
    }
    while (!replay_.empty() &&
           replay_.back().timestampMs - replay_.front().timestampMs > cfg_.replayWindowMs) {
        replay_.pop_front();
    }
}

void MultiObjectTracker::process(const MeasurementBatch& batch) {
    const std::uint64_t t = batch.timestampMs;
    const Eigen::Matrix2d R = rotation(batch.ego.psi);

    // 1. Predict every track to this batch's time.
    for (Track& tr : tracks_) {
        if (t > tr.lastTimeMs) tr.filter.predict((t - tr.lastTimeMs) / 1000.0);
        tr.lastTimeMs = std::max(tr.lastTimeMs, t);
    }

    // 2. Measurements into the world frame.
    std::vector<Eigen::Vector2d> zw;
    std::vector<Eigen::Matrix2d> Rw;
    for (const ObjectMeasurement& m : batch.measurements) {
        zw.push_back(toWorld(batch.ego, m.posVehicle));
        Rw.push_back(R * m.cov * R.transpose());
    }

    // 3. Cost matrix, Hungarian assignment, gate.
    std::vector<int> trackOf(batch.measurements.size(), -1);
    if (!tracks_.empty() && !batch.measurements.empty()) {
        std::vector<std::vector<double>> cost(tracks_.size(),
                                              std::vector<double>(batch.measurements.size()));
        for (size_t i = 0; i < tracks_.size(); ++i) {
            for (size_t j = 0; j < batch.measurements.size(); ++j) {
                const bool ok =
                    classesCompatible(tracks_[i].objectClass, batch.measurements[j].objectClass);
                cost[i][j] = ok ? tracks_[i].filter.mahalanobis2(zw[j], Rw[j]) : kReject;
            }
        }
        const std::vector<int> a = hungarian(cost);
        for (size_t i = 0; i < a.size(); ++i) {
            if (a[i] >= 0 && cost[i][a[i]] < cfg_.gateChi2) trackOf[a[i]] = static_cast<int>(i);
        }
    }

    // 4. Update matched tracks.
    std::vector<bool> matched(tracks_.size(), false);
    for (size_t j = 0; j < batch.measurements.size(); ++j) {
        if (trackOf[j] < 0) continue;
        Track& tr = tracks_[trackOf[j]];
        const ObjectMeasurement& m = batch.measurements[j];
        if (tr.hits == 1 && t > tr.lastUpdateMs) {
            // Second sighting: two-point initialisation of speed and heading (see
            // ImmFilter::initialiseVelocity). The velocity covariance follows from the two
            // positions' covariances.
            const double dt = (t - tr.lastUpdateMs) / 1000.0;
            const Eigen::Vector2d first = tr.history.back().second.head<2>();
            const Eigen::Matrix2d firstCov = tr.birthCov;
            tr.filter.initialiseVelocity((zw[j] - first) / dt, (firstCov + Rw[j]) / (dt * dt));
        }
        tr.filter.updatePosition(zw[j], Rw[j]);
        if (m.headingVehicle) tr.filter.updateHeadingModPi(batch.ego.psi + *m.headingVehicle, 0.1);
        if (tr.objectClass < 0 && m.objectClass >= 0) tr.objectClass = m.objectClass;
        matched[trackOf[j]] = true;
        ++tr.hits;
        tr.misses = 0;
        tr.lastUpdateMs = t;
        if (tr.state == TrackState::PREDICTED_ONLY ||
            (tr.state == TrackState::TENTATIVE && tr.hits >= cfg_.confirmHits)) {
            tr.state = TrackState::CONFIRMED;
        }
    }

    // 5. Unmatched tracks: tentative ones die quickly, confirmed ones coast, then retire.
    for (size_t i = 0; i < tracks_.size(); ++i) {
        if (matched[i]) continue;
        Track& tr = tracks_[i];
        ++tr.misses;
        if (tr.state == TrackState::TENTATIVE) {
            tr.hits = 0;  // "consecutive" hits: a gap restarts confirmation
        } else {
            tr.state = TrackState::PREDICTED_ONLY;
        }
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& tr) {
                                     if (tr.state == TrackState::TENTATIVE)
                                         return tr.misses > cfg_.tentativeMaxMisses;
                                     return tr.state == TrackState::PREDICTED_ONLY &&
                                            (t - tr.lastUpdateMs) / 1000.0 > cfg_.maxCoastS;
                                 }),
                  tracks_.end());

    // 6. New tentative tracks for unmatched measurements.
    for (size_t j = 0; j < batch.measurements.size(); ++j) {
        if (trackOf[j] >= 0) continue;
        const ObjectMeasurement& m = batch.measurements[j];
        std::optional<double> heading;
        if (m.headingVehicle) heading = batch.ego.psi + *m.headingVehicle;
        Track tr{nextId_++,
                 m.objectClass,
                 TrackState::TENTATIVE,
                 ImmFilter(zw[j], Rw[j], cfg_.noiseFor(m.objectClass), heading),
                 0,
                 0,
                 0,
                 0,
                 {},
                 {}};
        tr.hits = 1;
        tr.birthCov = Rw[j];
        tr.lastUpdateMs = tr.lastTimeMs = t;
        tracks_.push_back(std::move(tr));
    }

    // 7. History for the reckless-driving classifier.
    for (Track& tr : tracks_) {
        tr.history.emplace_back(t, tr.filter.state());
        while (tr.history.size() > cfg_.historyLength) tr.history.pop_front();
    }
}

}  // namespace ar_drive_assist
