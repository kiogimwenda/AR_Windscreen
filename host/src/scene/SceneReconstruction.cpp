#include "ar_drive_assist/scene/SceneReconstruction.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace ar_drive_assist {
namespace {

struct Candidate {
    double depth;       // camera z, metres
    Eigen::Vector3d q;  // vehicle frame
    int index;          // into the compensated cloud
};

// Rule 3: split by depth gaps, return the nearest cluster holding at least `minFraction` of the
// points (and at least `minPoints`). Empty if none qualifies.
std::vector<Candidate> nearestDominant(std::vector<Candidate> pts, double gap, double minFraction,
                                       int minPoints) {
    std::sort(pts.begin(), pts.end(),
              [](const Candidate& a, const Candidate& b) { return a.depth < b.depth; });
    const double need = std::max<double>(minPoints, minFraction * pts.size());
    size_t start = 0;
    for (size_t i = 1; i <= pts.size(); ++i) {
        if (i == pts.size() || pts[i].depth - pts[i - 1].depth > gap) {
            if (i - start >= need) return {pts.begin() + start, pts.begin() + i};
            start = i;
        }
    }
    return {};
}

Eigen::Vector3d median(const std::vector<Candidate>& c) {
    Eigen::Vector3d m;
    for (int k = 0; k < 3; ++k) {
        std::vector<double> v;
        for (const auto& p : c) v.push_back(p.q(k));
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        m(k) = v[v.size() / 2];
    }
    return m;
}

bool inBox(const Box& b, const Eigen::Vector2d& px) {
    return px.x() >= b.x && px.x() < b.x + b.w && px.y() >= b.y && px.y() < b.y + b.h;
}

}  // namespace

bool CameraModel::project(const Eigen::Vector3d& pc, Eigen::Vector2d& px) const {
    if (pc.z() <= 0.1) return false;
    const double x = pc.x() / pc.z(), y = pc.y() / pc.z();
    const double r2 = x * x + y * y;
    const double radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const double xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    const double yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    px = {fx * xd + cx, fy * yd + cy};
    return px.x() >= 0 && px.y() >= 0 && px.x() < width && px.y() < height;
}

ObjectMask erodeMask(const ObjectMask& m, int cells) {
    ObjectMask out = m;
    for (int it = 0; it < cells; ++it) {
        const ObjectMask in = out;
        for (int gy = in.y; gy < in.y + in.h; ++gy) {
            for (int gx = in.x; gx < in.x + in.w; ++gx) {
                const bool keep = in.atGrid(gx, gy) && in.atGrid(gx - 1, gy) &&
                                  in.atGrid(gx + 1, gy) && in.atGrid(gx, gy - 1) &&
                                  in.atGrid(gx, gy + 1);
                out.bits[static_cast<size_t>(gy - in.y) * in.w + (gx - in.x)] = keep ? 1 : 0;
            }
        }
    }
    return out;
}

SceneReconstruction::SceneReconstruction(FusionConfig cfg) : cfg_(cfg) {}

Eigen::Vector3d SceneReconstruction::compensate(const Eigen::Vector3d& q, std::int64_t fromUs,
                                                std::int64_t toUs, const EgoMotion& ego) {
    // The vehicle's motion from `from` to `to`, expressed in the `from` frame (constant speed and
    // yaw rate: an arc). A world-fixed point is then seen at R(-dpsi) (q - d).
    const double dt = (toUs - fromUs) * 1e-6;
    const double dpsi = ego.yawRate * dt;
    Eigen::Vector2d d;
    if (std::abs(ego.yawRate) > 1e-6) {
        d = {ego.speed / ego.yawRate * std::sin(dpsi),
             ego.speed / ego.yawRate * (1 - std::cos(dpsi))};
    } else {
        d = {ego.speed * dt, 0};
    }
    const Eigen::Vector2d rel = q.head<2>() - d;
    const double c = std::cos(-dpsi), s = std::sin(-dpsi);
    return {c * rel.x() - s * rel.y(), s * rel.x() + c * rel.y(), q.z()};
}

SceneModel SceneReconstruction::merge(const FusionFrame& frame,
                                      const std::vector<LidarPoint>& cloud,
                                      const CameraModel& camera, const FusionExtrinsics& ext,
                                      const EgoMotion& ego, const GroundPlane& ground) const {
    const Eigen::Isometry3d cameraFromVehicle =
        ext.cameraFromLidar * ext.vehicleFromLidar.inverse();

    // 1-3. Compensate, drop the road surface, project.
    struct Proj {
        Eigen::Vector3d q;
        Eigen::Vector2d px;
        double depth;
        float intensity;
        bool inImage;
    };
    std::vector<Proj> pts;
    pts.reserve(cloud.size());
    for (const LidarPoint& lp : cloud) {
        const Eigen::Vector3d qv = ext.vehicleFromLidar * lp.p.cast<double>();
        const Eigen::Vector3d q = compensate(qv, lp.tUs, frame.cameraTUs, ego);
        if (std::abs(ground.distance(q)) < cfg_.groundTolM) continue;
        Proj p{q, {}, 0, lp.intensity, false};
        const Eigen::Vector3d pc = cameraFromVehicle * q;
        p.depth = pc.z();
        p.inImage = camera.project(pc, p.px);
        pts.push_back(p);
    }

    // 4. Depth test: only the nearest point per image cell is visible to the camera.
    std::unordered_map<long long, int> nearest;
    const long long cols = camera.width / cfg_.zBufferCellPx + 1;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        if (!pts[i].inImage) continue;
        if (!cfg_.depthTest) {  // for tests that show what the rule prevents
            nearest.emplace(-1 - i, i);
            continue;
        }
        const long long key = static_cast<long long>(pts[i].px.y() / cfg_.zBufferCellPx) * cols +
                              static_cast<long long>(pts[i].px.x() / cfg_.zBufferCellPx);
        auto [it, inserted] = nearest.try_emplace(key, i);
        if (!inserted && pts[i].depth < pts[it->second].depth) it->second = i;
    }
    std::vector<int> visible;
    for (const auto& [key, i] : nearest) visible.push_back(i);
    std::sort(visible.begin(), visible.end());  // deterministic order

    // 5-6 (and 9 for signs). Assign visible points; nearest dominant cluster per object.
    SceneModel scene;
    std::vector<bool> claimed(pts.size(), false);
    for (int di = 0; di < static_cast<int>(frame.detections.size()); ++di) {
        const FusionDetection& det = frame.detections[di];
        FusedObject obj;
        obj.detection = di;
        obj.classId = det.box.classId;
        ObjectMask eroded;
        if (!det.isSign && det.mask) eroded = erodeMask(*det.mask, cfg_.erodeCells);
        std::vector<Candidate> mine;
        for (int i : visible) {
            const Proj& p = pts[i];
            bool on;
            if (det.isSign) {
                on = inBox(det.box, p.px) && p.intensity >= cfg_.signMinIntensity;
            } else if (det.mask) {
                on = maskContains(eroded, frame.maskMapping, frame.maskStride,
                                  static_cast<float>(p.px.x()), static_cast<float>(p.px.y()));
            } else {
                on = false;  // no mask: a box would mix in the background (rule 2's reason)
            }
            if (on) mine.push_back({p.depth, p.q, i});
        }
        const auto cluster =
            nearestDominant(mine, cfg_.depthGapM, cfg_.minClusterFraction, cfg_.minObjectPoints);
        obj.maskPoints = static_cast<int>(mine.size());
        if (!cluster.empty()) {
            obj.source = RangeSource::LIDAR;
            obj.position = median(cluster);
            obj.points = static_cast<int>(cluster.size());
            // Claim the object's points for rule 8, INCLUDING those the depth test hid (several
            // points of one pedestrian share an image cell). Otherwise they would come back as an
            // UNKNOWN obstacle duplicating the pedestrian. Claimed: inside the un-eroded mask (or
            // the sign's box) and within the chosen cluster's depth band.
            const double lo = cluster.front().depth - cfg_.depthGapM;
            const double hi = cluster.back().depth + cfg_.depthGapM;
            for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
                const Proj& p = pts[i];
                if (!p.inImage || p.depth < lo || p.depth > hi) continue;
                claimed[i] =
                    claimed[i] ||
                    (det.isSign ? inBox(det.box, p.px)
                                : maskContains(*det.mask, frame.maskMapping, frame.maskStride,
                                               static_cast<float>(p.px.x()),
                                               static_cast<float>(p.px.y())));
            }
        }
        scene.objects.push_back(obj);
    }

    // 7. Estimates for unranged objects from MiDaS relative inverse depth. MiDaS output is
    // inverse depth up to scale AND shift, so rel = a / z + b is fitted by least squares to this
    // frame's LiDAR-ranged objects (at least two, a > 0). The ray through the box's bottom centre
    // uses the undistorted pinhole model: an estimate, flagged as one.
    std::vector<std::pair<double, double>> fit;  // (1 / z, rel)
    for (const FusedObject& o : scene.objects) {
        const auto& rel = frame.detections[o.detection].relInvDepth;
        if (o.source == RangeSource::LIDAR && rel) {
            fit.emplace_back(1.0 / (cameraFromVehicle * o.position).z(), *rel);
        }
    }
    if (fit.size() >= 2) {
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (const auto& [x, y] : fit) {
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        const double n = static_cast<double>(fit.size()), den = n * sxx - sx * sx;
        const double a = std::abs(den) > 1e-12 ? (n * sxy - sx * sy) / den : 0.0;
        const double b = (sy - a * sx) / n;
        const Eigen::Isometry3d vehicleFromCamera = cameraFromVehicle.inverse();
        for (FusedObject& o : scene.objects) {
            const FusionDetection& det = frame.detections[o.detection];
            if (o.source != RangeSource::NONE || !det.relInvDepth || a <= 0) continue;
            const double inv = (*det.relInvDepth - b) / a;
            if (inv <= 0) continue;
            const double z = 1.0 / inv;
            const double u = det.box.x + det.box.w / 2, v = det.box.y + det.box.h;
            const Eigen::Vector3d pc((u - camera.cx) / camera.fx * z,
                                     (v - camera.cy) / camera.fy * z, z);
            o.source = RangeSource::ESTIMATED;
            o.position = vehicleFromCamera * pc;
        }
    }
    for (FusedObject& o : scene.objects) {
        if (o.source != RangeSource::NONE)
            o.groundContact = o.position - ground.distance(o.position) * ground.n;
    }

    // 8. Unexplained obstacles in the ego corridor, from ALL points (hidden ones too). Points
    // higher than `maxObstacleHeightM` above the road (gantries, branches) cannot hit the car.
    std::vector<int> free;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
        const Eigen::Vector3d& q = pts[i].q;
        if (claimed[i] || q.x() <= 0 || q.x() > cfg_.corridorLengthM ||
            std::abs(q.y()) > cfg_.corridorHalfWidthM ||
            ground.distance(q) > cfg_.maxObstacleHeightM)
            continue;
        free.push_back(i);
    }
    // Clustering: connected components of occupied voxels (26-neighbourhood). Linear in the
    // number of points; a point-to-point search is quadratic inside the dense voxels a Livox
    // cloud produces (measured: 20 s for one test scene). Voxel edge = half the cluster radius, so
    // points up to about that radius apart stay connected.
    const double vox = cfg_.unknownClusterRadiusM / 2;
    struct Voxel {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int count = 0;
        bool seen = false;
    };
    auto keyOf = [](long long x, long long y, long long z) {
        return (x & 0x1FFFFF) | ((y & 0x1FFFFF) << 21) | ((z & 0x1FFFFF) << 42);
    };
    std::unordered_map<long long, Voxel> voxels;
    std::vector<Eigen::Matrix<long long, 3, 1>> order;  // first-seen order: deterministic output
    for (int i : free) {
        const Eigen::Vector3d& q = pts[i].q;
        const Eigen::Matrix<long long, 3, 1> c(static_cast<long long>(std::floor(q.x() / vox)),
                                               static_cast<long long>(std::floor(q.y() / vox)),
                                               static_cast<long long>(std::floor(q.z() / vox)));
        Voxel& v = voxels[keyOf(c.x(), c.y(), c.z())];
        if (v.count == 0) order.push_back(c);
        v.sum += q;
        ++v.count;
    }
    for (const auto& start : order) {
        Voxel& v0 = voxels[keyOf(start.x(), start.y(), start.z())];
        if (v0.seen) continue;
        v0.seen = true;
        UnknownObstacle u;
        std::vector<Eigen::Matrix<long long, 3, 1>> stack{start};
        while (!stack.empty()) {
            const auto c = stack.back();
            stack.pop_back();
            const Voxel& v = voxels[keyOf(c.x(), c.y(), c.z())];
            u.position += v.sum;
            u.points += v.count;
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dz = -1; dz <= 1; ++dz) {
                        const auto it = voxels.find(keyOf(c.x() + dx, c.y() + dy, c.z() + dz));
                        if (it == voxels.end() || it->second.seen) continue;
                        it->second.seen = true;
                        stack.push_back({c.x() + dx, c.y() + dy, c.z() + dz});
                    }
        }
        if (u.points < cfg_.minUnknownPoints) continue;
        u.position /= static_cast<double>(u.points);
        u.groundContact = u.position - ground.distance(u.position) * ground.n;
        scene.unknown.push_back(u);
    }
    return scene;
}

}  // namespace ar_drive_assist
