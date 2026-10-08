// LidarProcessor — see include/ar_drive_assist/lidar/LidarProcessor.h.
#include "ar_drive_assist/lidar/LidarProcessor.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <random>

namespace ar_drive_assist {

namespace {

// Unit-normal plane n . q + d = 0 with n pointing up (n.z > 0).
struct Plane {
    Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
    double d = 0;
    double dist(const Eigen::Vector3d& q) const { return n.dot(q) + d; }
};

Plane fromModel(const GroundPlaneModel& g) {
    Plane p;
    Eigen::Vector3d n(g.planeCoefficients.x(), g.planeCoefficients.y(), g.planeCoefficients.z());
    double d = g.planeCoefficients.w();
    const double len = n.norm();
    if (len < 1e-9) return p;
    n /= len;
    d /= len;
    if (n.z() < 0) {
        n = -n;
        d = -d;
    }
    p.n = n;
    p.d = d;
    return p;
}

}  // namespace

GroundPlane LidarProcessor::toGroundPlane(const GroundPlaneModel& g) {
    const Plane p = fromModel(g);
    GroundPlane out;
    out.n = p.n;
    out.d = p.d;
    return out;
}

LidarProcessor::Result LidarProcessor::fitGround(const std::vector<LidarPoint>& scan,
                                                 const Eigen::Isometry3d& vehicleFromLidar,
                                                 std::uint64_t timestampMs) {
    const Plane prev = haveLast_ ? fromModel(last_) : Plane{};
    std::vector<Eigen::Vector3d> cand;
    cand.reserve(scan.size() / 4);
    for (const LidarPoint& lp : scan) {
        const Eigen::Vector3d q = vehicleFromLidar * lp.p.cast<double>();
        if (q.x() < cfg_.minRangeM || q.x() > cfg_.maxRangeM || std::abs(q.y()) > cfg_.halfWidthM)
            continue;
        if (std::abs(prev.dist(q)) > cfg_.searchBandM) continue;
        cand.push_back(q);
    }
    Result r;
    r.ground = last_;
    if (!haveLast_) r.ground = GroundPlaneModel{};
    r.ground.timestampMs = timestampMs;
    if (static_cast<int>(cand.size()) < cfg_.minInliers) return r;

    // RANSAC, fixed seed: the same scan always gives the same plane.
    std::mt19937 rng(12345);
    std::uniform_int_distribution<std::size_t> pick(0, cand.size() - 1);
    const double minNz = std::cos(cfg_.maxTiltDeg * 3.14159265358979323846 / 180);
    Plane best;
    int bestCount = -1;
    for (int it = 0; it < cfg_.iterations; ++it) {
        const Eigen::Vector3d &a = cand[pick(rng)], &b = cand[pick(rng)], &c = cand[pick(rng)];
        Eigen::Vector3d n = (b - a).cross(c - a);
        const double len = n.norm();
        if (len < 1e-6) continue;
        n /= len;
        if (n.z() < 0) n = -n;
        if (n.z() < minNz) continue;  // too steep to be a road
        Plane p;
        p.n = n;
        p.d = -n.dot(a);
        int count = 0;
        for (const auto& q : cand)
            if (std::abs(p.dist(q)) <= cfg_.inlierTolM) ++count;
        if (count > bestCount) {
            bestCount = count;
            best = p;
        }
    }
    if (bestCount < cfg_.minInliers) return r;

    // Least squares on the inliers: the normal is the scatter's smallest principal axis.
    std::vector<Eigen::Vector3d> in;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& q : cand)
        if (std::abs(best.dist(q)) <= cfg_.inlierTolM) {
            in.push_back(q);
            mean += q;
        }
    mean /= static_cast<double>(in.size());
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (const auto& q : in) C += (q - mean) * (q - mean).transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
    Eigen::Vector3d n = es.eigenvectors().col(0);  // smallest eigenvalue first
    if (n.z() < 0) n = -n;
    Plane fit;
    fit.n = n;
    fit.d = -n.dot(mean);
    // Height of the plane under the rear axle (x = y = 0): -d / n.z.
    if (n.z() < minNz || std::abs(-fit.d / n.z()) > cfg_.maxHeightAtOriginM) return r;

    GroundPlaneModel g;
    g.planeCoefficients = Eigen::Vector4f(static_cast<float>(n.x()), static_cast<float>(n.y()),
                                          static_cast<float>(n.z()), static_cast<float>(fit.d));
    g.timestampMs = timestampMs;
    std::vector<double> xs;
    const std::size_t stride = std::max<std::size_t>(1, in.size() / cfg_.maxPatchPoints);
    for (std::size_t i = 0; i < in.size(); ++i) {
        xs.push_back(in[i].x());
        if (i % stride == 0 && std::abs(in[i].y()) <= cfg_.patchHalfWidthM)
            g.nearFieldPatch.push_back(in[i].cast<float>());
    }
    std::nth_element(xs.begin(), xs.begin() + xs.size() * 95 / 100, xs.end());
    g.maxValidRangeM = static_cast<float>(xs[xs.size() * 95 / 100]);
    last_ = g;
    haveLast_ = true;
    r.ground = g;
    r.valid = true;
    r.inliers = static_cast<int>(in.size());
    return r;
}

}  // namespace ar_drive_assist
