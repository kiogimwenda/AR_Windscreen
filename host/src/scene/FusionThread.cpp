// FusionThread — see include/ar_drive_assist/scene/FusionThread.h.
#include "ar_drive_assist/scene/FusionThread.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "ar_drive_assist/lidar/LidarProcessor.h"
#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {

FusionThread::FusionThread(FusionThreadConfig cfg, MlInferenceEngine::DetectionBus& detections,
                           MlInferenceEngine::MaskBus* masks, LidarBus* lidar, EgoEstimator& ego,
                           SceneBus& toDecision, SceneBus* toRender, EventLog* log,
                           SceneBus* toNavigation)
    : cfg_(std::move(cfg)),
      detections_(detections),
      masks_(masks),
      lidar_(lidar),
      ego_(ego),
      toDecision_(toDecision),
      toRender_(toRender),
      toNav_(toNavigation),
      log_(log),
      scene_(cfg_.fusion),
      tracker_(cfg_.tracker) {}

std::vector<ObjectMeasurement> FusionThread::measurements(const SceneModel& scene,
                                                          const FusionFrame& frame) const {
    std::vector<std::pair<int, ObjectMeasurement>> all;  // (points, measurement)
    for (const FusedObject& o : scene.objects) {
        if (o.source == RangeSource::NONE) continue;  // no range: nothing to track
        if (o.detection >= 0 && o.detection < static_cast<int>(frame.detections.size()) &&
            frame.detections[o.detection].isSign)
            continue;  // signs are drawn, not tracked as hazards
        ObjectMeasurement m;
        m.posVehicle = o.nearFace.head<2>();  // the surface the car would hit (FusedObject)
        if (o.source != RangeSource::LIDAR) m.posVehicle = o.position.head<2>();  // estimate
        const double s = o.source == RangeSource::LIDAR ? cfg_.lidarSigmaM : cfg_.estimatedSigmaM;
        m.cov = Eigen::Matrix2d::Identity() * s * s;
        m.objectClass = o.classId;
        m.rangeMeasured = o.source == RangeSource::LIDAR;
        all.emplace_back(o.points, m);
    }
    for (const UnknownObstacle& u : scene.unknown) {
        ObjectMeasurement m;
        m.posVehicle = u.nearFace.head<2>();
        m.cov = Eigen::Matrix2d::Identity() * cfg_.lidarSigmaM * cfg_.lidarSigmaM;
        m.objectClass = -1;  // LiDAR only: unclassified
        m.rangeMeasured = true;
        all.emplace_back(u.points, m);
    }
    // Duplicates of one surface: keep the best-supported measurement (most points); it takes a
    // class from a duplicate if it has none. Measured ranges first, so an estimate never
    // displaces one.
    std::stable_sort(all.begin(), all.end(), [](const auto& a, const auto& b) {
        if (a.second.rangeMeasured != b.second.rangeMeasured) return a.second.rangeMeasured;
        return a.first > b.first;
    });
    std::vector<ObjectMeasurement> out;
    for (auto& [points, m] : all) {
        bool dup = false;
        for (ObjectMeasurement& k : out)
            if ((k.posVehicle - m.posVehicle).norm() < cfg_.duplicateRadiusM) {
                if (k.objectClass < 0) k.objectClass = m.objectClass;
                dup = true;
                break;
            }
        if (!dup) out.push_back(m);
    }
    return out;
}

SceneSnapshot FusionThread::process(const DetectionFrame& det, const MaskFrame* masks,
                                    const LidarFrame* scan, const EgoEstimator::Snapshot& ego) {
    ++stats_.frames;
    FusionFrame frame;
    frame.cameraTUs = static_cast<std::int64_t>(det.timestamp_ms) * 1000;
    if (masks) {
        frame.maskMapping = masks->mapping;
        frame.maskStride = masks->stride;
    }
    for (std::size_t i = 0; i < det.boxes.size(); ++i) {
        const auto& b = *det.boxes[i];
        FusionDetection d;
        d.box = {b.x, b.y, b.w, b.h, b.class_id, b.confidence};
        if (masks && i < masks->masks.size() && masks->masks[i].w > 0) d.mask = &masks->masks[i];
        frame.detections.push_back(d);
    }
    for (const auto& sp : det.signs) {
        FusionDetection d;
        d.box = {sp->x, sp->y, sp->w, sp->h, sp->class_id, sp->confidence};
        d.isSign = true;
        frame.detections.push_back(d);
    }

    static const std::vector<LidarPoint> kNoPoints;
    const std::vector<LidarPoint>* cloud = &kNoPoints;
    SceneSnapshot s;
    if (scan) {
        const std::uint64_t age = det.timestamp_ms > scan->timestampMs
                                      ? det.timestamp_ms - scan->timestampMs
                                      : scan->timestampMs - det.timestamp_ms;
        if (age <= cfg_.maxLidarAgeMs) {
            cloud = &scan->points;
            s.lidarAgeMs = age;
            s.lidarUsed = true;
            ++stats_.withLidar;
        } else {
            ++stats_.staleScans;
        }
    }
    // The road: the newest fitted surface (Part 8.1); flat only until the first good fit.
    if (scan && scan->groundValid) {
        ground_ = scan->ground;
        haveGround_ = true;
    }
    const GroundPlane road = haveGround_ ? LidarProcessor::toGroundPlane(ground_) : cfg_.ground;
    s.ground = ground_;
    s.groundValid = haveGround_;
    const EgoMotion motion{ego.ego.speedMps, ego.ego.yawRateRadPerS};
    FusionExtrinsics ext = cfg_.extrinsics;
    bool rangeWithMasks = true;
    if (cfg_.monitor) {
        const ExtrinsicState st = cfg_.monitor->current();
        ext.cameraFromLidar = st.cameraFromLidar;
        rangeWithMasks = st.status != ExtrinsicStatus::DEGRADED;
        s.extrinsicsDegraded = !rangeWithMasks;
    }
    if (rangeWithMasks) {
        s.scene = scene_.merge(frame, *cloud, cfg_.camera, ext, motion, road);
    } else {
        FusionFrame bare;  // no detections: every point is a LiDAR-only obstacle candidate
        bare.cameraTUs = frame.cameraTUs;
        s.scene = scene_.merge(bare, *cloud, cfg_.camera, ext, motion, road);
    }

    if (ego.valid) {
        MeasurementBatch batch;
        batch.timestampMs = det.timestamp_ms;
        batch.ego = ego.ego.pose;
        batch.measurements = measurements(s.scene, frame);
        batch.observed = cloud != &kNoPoints;  // no usable scan: nothing was looked at
        tracker_.update(batch);
    } else {
        ++stats_.untrackedNoEgo;
    }
    s.timestampMs = det.timestamp_ms;
    s.frameSeq = det.mask_ref;
    s.egoValid = ego.valid;
    s.ego = ego.ego;
    s.egoX = ego.x;
    s.egoP = ego.P;
    s.tracks = tracker_.tracks();
    s.detections = det;
    return s;
}

void FusionThread::run(const std::atomic<bool>& stop) {
    auto lastReport = std::chrono::steady_clock::now();
    while (!stop.load()) {
        if (lidar_) {
            LidarFrame scan;
            if (lidar_->popLatest(scan)) {
                latestScan_ = std::move(scan);
                haveScan_ = true;
            }
        }
        if (masks_) {
            MaskFrame m;
            while (masks_->pop(m)) maskBySeq_[m.seq] = std::move(m);
        }
        DetectionFrame det;
        if (!detections_.popLatest(det)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const MaskFrame* masks = nullptr;
        const auto it = maskBySeq_.find(det.mask_ref);
        if (it != maskBySeq_.end()) masks = &it->second;

        SceneSnapshot s = process(det, masks, haveScan_ ? &latestScan_ : nullptr,
                                  ego_.snapshot(det.timestamp_ms));
        // Masks of this frame and older are done with.
        maskBySeq_.erase(maskBySeq_.begin(), maskBySeq_.upper_bound(det.mask_ref));
        if (toRender_) toRender_->push(s);  // a full render bus just means the display lags
        if (toNav_) toNav_->push(s);
        toDecision_.push(std::move(s));

        const double sinceS =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - lastReport).count();
        if (log_ && sinceS >= cfg_.reportEveryS) {
            char b[200];
            std::snprintf(b, sizeof b,
                          "fusion: %llu frames, %llu with a LiDAR scan, %llu scans too old, "
                          "%zu tracks, %llu frames untracked (no GNSS fix yet)",
                          static_cast<unsigned long long>(stats_.frames),
                          static_cast<unsigned long long>(stats_.withLidar),
                          static_cast<unsigned long long>(stats_.staleScans),
                          tracker_.tracks().size(),
                          static_cast<unsigned long long>(stats_.untrackedNoEgo));
            log_->logGeneral(b);
            lastReport = std::chrono::steady_clock::now();
        }
    }
}

}  // namespace ar_drive_assist
