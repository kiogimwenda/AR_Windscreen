// ExtrinsicMonitor — see include/ar_drive_assist/scene/ExtrinsicMonitor.h.
#include "ar_drive_assist/scene/ExtrinsicMonitor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <unordered_map>

#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {
namespace {

constexpr double kDeg = M_PI / 180.0;

// The correction: rotation (rx, ry, rz) and translation, applied in the camera frame.
Eigen::Isometry3d correction(const double d[6]) {
    Eigen::Isometry3d c = Eigen::Isometry3d::Identity();
    c.linear() = (Eigen::AngleAxisd(d[2], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(d[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(d[0], Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
    c.translation() = Eigen::Vector3d(d[3], d[4], d[5]);
    return c;
}

std::string fmt(double v, int prec = 3) {
    char b[32];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}

}  // namespace

const char* toString(ExtrinsicStatus s) {
    switch (s) {
        case ExtrinsicStatus::UNVERIFIED:
            return "UNVERIFIED";
        case ExtrinsicStatus::VERIFIED:
            return "VERIFIED";
        case ExtrinsicStatus::REFINED:
            return "REFINED";
        case ExtrinsicStatus::DEGRADED:
            return "DEGRADED";
    }
    return "?";
}

ExtrinsicMonitor::ExtrinsicMonitor(const Eigen::Isometry3d& baselineCameraFromLidar,
                                   const Eigen::Isometry3d& vehicleFromLidar,
                                   const CameraModel& camera, ExtrinsicMonitorConfig cfg,
                                   EventLog* log)
    : baseline_(baselineCameraFromLidar),
      vehicleFromLidar_(vehicleFromLidar),
      camera_(camera),
      cfg_(cfg),
      log_(log),
      currentT_(baselineCameraFromLidar) {
    state_.cameraFromLidar = baseline_;
}

ExtrinsicState ExtrinsicMonitor::current() const {
    std::lock_guard<std::mutex> lock(m_);
    return state_;
}

ExtrinsicMonitor::Evidence ExtrinsicMonitor::prepare(const cv::Mat& image, std::int64_t frameTimeUs,
                                                     const std::vector<LidarPoint>& scan,
                                                     const EgoMotion& ego) const {
    Evidence e;
    // Closeness to the nearest image edge.
    cv::Mat gray, edges, dist;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.2);
    cv::Canny(gray, edges, 40, 120);
    cv::bitwise_not(edges, edges);
    cv::distanceTransform(edges, dist, cv::DIST_L2, 3);
    // Two scales: coarse (coarseScale x sigma) to find the peak from up to ~1 deg away, fine to
    // place it. Each relative to the local edge density: textured regions (foliage, facades) are
    // close to SOME edge everywhere, and without this, moving points into them raised the score
    // whatever the alignment (KITTI: pitching up into the trees scored better and better).
    const cv::Mat d2 = dist.mul(dist);
    for (int level = 0; level < 2; ++level) {
        const double sigma = cfg_.edgeSigmaPx * (level == 0 ? cfg_.coarseScale : 1.0);
        cv::Mat close, local;
        cv::exp(d2 * (-0.5 / (sigma * sigma)), close);
        const int half = std::max(cfg_.localWindowPx, static_cast<int>(6 * sigma));
        cv::boxFilter(close, local, CV_32F, cv::Size(2 * half + 1, 2 * half + 1));
        e.closeness[level] = close - local;
    }

    // Points at the frame's time, projected with the baseline into a z-buffer.
    const Eigen::Isometry3d lidarFromVehicle = vehicleFromLidar_.inverse();
    struct P {
        Eigen::Vector3d l;
        int cx, cy;
        double depth;
    };
    std::vector<P> pts;
    pts.reserve(scan.size());
    const int cw = camera_.width / cfg_.cellPx + 1, ch = camera_.height / cfg_.cellPx + 1;
    std::vector<float> zbuf(static_cast<std::size_t>(cw) * ch, 1e9f);
    std::vector<int> who(static_cast<std::size_t>(cw) * ch, -1);  // the visible point's index
    for (const LidarPoint& lp : scan) {
        const Eigen::Vector3d qv = vehicleFromLidar_ * lp.p.cast<double>();
        const Eigen::Vector3d q = SceneReconstruction::compensate(qv, lp.tUs, frameTimeUs, ego);
        const Eigen::Vector3d l = lidarFromVehicle * q;
        const Eigen::Vector3d pc = currentT_ * l;
        if (pc.z() < 1.0 || pc.norm() > cfg_.maxRangeM) continue;
        Eigen::Vector2d px;
        if (!camera_.project(pc, px)) continue;
        const int cx = static_cast<int>(px.x()) / cfg_.cellPx;
        const int cy = static_cast<int>(px.y()) / cfg_.cellPx;
        if (cx < 0 || cy < 0 || cx >= cw || cy >= ch) continue;
        pts.push_back({l, cx, cy, pc.z()});
        const std::size_t c = static_cast<std::size_t>(cy) * cw + cx;
        if (pc.z() < zbuf[c]) {
            zbuf[c] = static_cast<float>(pc.z());
            who[c] = static_cast<int>(pts.size()) - 1;
        }
    }
    // Depth edges, as PAIRS: a visible point (nearest in its cell) and, in an adjacent cell
    // (within `edgeRadiusCells`), the visible point farthest behind it, more than depthJumpM. The
    // outline lies between the two, so a pair is scored at the midpoint of their projections.
    // One side alone is biased: the near side's points all sit just inside the outline, and above
    // an object there is often no return at all (sky), so nothing balances them (KITTI: the score
    // kept rising by pitching the points up, past the bounds). A pair straddles the outline.
    int grid[3][4] = {};
    int near = 0, far = 0;
    const int r = cfg_.edgeRadiusCells;
    for (std::size_t c = 0; c < who.size(); ++c) {
        if (who[c] < 0) continue;
        const P& p = pts[static_cast<std::size_t>(who[c])];
        int behind = -1;
        float farthest = static_cast<float>(p.depth + cfg_.depthJumpM);
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                const int x = p.cx + dx, y = p.cy + dy;
                if (x < 0 || y < 0 || x >= cw || y >= ch) continue;
                const std::size_t n = static_cast<std::size_t>(y) * cw + x;
                if (who[n] >= 0 && zbuf[n] > farthest) {
                    farthest = zbuf[n];
                    behind = who[n];
                }
            }
        if (behind < 0) continue;
        e.edges.push_back({p.l, pts[static_cast<std::size_t>(behind)].l});
        ++grid[std::min(2, p.cy * cfg_.cellPx * 3 / camera_.height)]
              [std::min(3, p.cx * cfg_.cellPx * 4 / camera_.width)];
        near += p.depth < 15;
        far += p.depth > 25;
    }
    // Spread: outlines across the width (yaw, roll) and in two of three bands of height (pitch).
    int cols = 0, rows = 0;
    for (int x = 0; x < 4; ++x) cols += grid[0][x] + grid[1][x] + grid[2][x] >= 10;
    for (int y = 0; y < 3; ++y) rows += grid[y][0] + grid[y][1] + grid[y][2] + grid[y][3] >= 10;
    const double n = std::max<std::size_t>(e.edges.size(), 1);
    e.nearFraction = near / n;
    e.farFraction = far / n;
    e.usable = static_cast<int>(e.edges.size()) >= cfg_.minEdgePoints && cols >= 3 && rows >= 2;
    return e;
}

double ExtrinsicMonitor::score(const std::vector<const Evidence*>& frames,
                               const Eigen::Isometry3d& T, int level) const {
    double sum = 0;
    std::size_t n = 0;
    for (const Evidence* e : frames) {
        const cv::Mat& c = e->closeness[level];
        for (const auto& [front, back] : e->edges) {
            Eigen::Vector2d a, b;
            ++n;  // a pair projecting off the image scores 0: it cannot be rewarded for leaving
            if (!camera_.project(T * front, a) || !camera_.project(T * back, b)) continue;
            const Eigen::Vector2d px = 0.5 * (a + b);
            const int x = static_cast<int>(std::lround(px.x()));
            const int y = static_cast<int>(std::lround(px.y()));
            if (x < 0 || y < 0 || x >= c.cols || y >= c.rows) continue;
            sum += c.at<float>(y, x);
        }
    }
    return n ? sum / static_cast<double>(n) : 0.0;
}

Eigen::Isometry3d ExtrinsicMonitor::search(const std::vector<const Evidence*>& frames,
                                           const Eigen::Isometry3d& T, double& bestScore,
                                           bool& atBound, double boundScale, bool fine,
                                           bool grid) const {
    // The correction is relative to the BASELINE, so its bounds are absolute: refinements cannot
    // walk away from the calibration one bound at a time.
    const Eigen::Isometry3d fromBase = T * baseline_.inverse();
    double d[6] = {0, 0, 0, 0, 0, 0};
    {
        const Eigen::Vector3d ypr = fromBase.linear().eulerAngles(2, 1, 0);
        // eulerAngles may return the equivalent (pi-shifted) triple; small corrections only.
        auto wrap = [](double a) { return std::remainder(a, M_PI); };
        d[0] = wrap(ypr[2]);
        d[1] = wrap(ypr[1]);
        d[2] = wrap(ypr[0]);
        const Eigen::Vector3d t = fromBase.translation();
        d[3] = t.x();
        d[4] = t.y();
        d[5] = t.z();
    }
    const double rb = cfg_.maxRotDeg * kDeg * boundScale, tb = cfg_.maxTransM * boundScale;
    const double bound[6] = {rb, rb, rb, tb, tb, tb};
    // Coordinate search with step halving from 0.4 deg (1.2 cm), on the coarse map to find the
    // peak, then again on the fine one to place it. The fine stage starts as wide as the coarse:
    // where the coarse map is flat (roll, on KITTI's wide short image) the coarse stage can drift,
    // and small fine steps could not walk back.
    for (int level = 0; level < (fine ? 2 : 1); ++level) {
        auto eval = [&](const double x[6]) {
            return score(frames, correction(x) * baseline_, level);
        };
        bestScore = eval(d);
        if (level == 0 && grid && bound[0] > 0) {
            // First a grid over the three rotations (5 x 5 x 5 within the bounds): pitch and roll
            // are coupled on a wide image, and one axis at a time follows the coupling poorly.
            const double c[3] = {d[0], d[1], d[2]};
            const int m = static_cast<int>(std::lround(2 * boundScale));  // 0.5-bound spacing
            const double g = cfg_.maxRotDeg * kDeg / 2;
            for (int i = -m; i <= m; ++i)
                for (int j = -m; j <= m; ++j)
                    for (int k = -m; k <= m; ++k) {
                        double x[6];
                        std::copy(d, d + 6, x);
                        x[0] = std::clamp(c[0] + i * g, -bound[0], bound[0]);
                        x[1] = std::clamp(c[1] + j * g, -bound[1], bound[1]);
                        x[2] = std::clamp(c[2] + k * g, -bound[2], bound[2]);
                        const double sc = eval(x);
                        if (sc > bestScore + 1e-9) {
                            bestScore = sc;
                            std::copy(x, x + 6, d);
                        }
                    }
        }
        const double r0 = 0.4 * kDeg, t0 = 0.012;
        double step[6] = {r0, r0, r0, t0, t0, t0};
        for (int halving = 0; halving < (level == 0 ? 3 : 5); ++halving) {
            bool improved = true;
            for (int iter = 0; improved && iter < 20; ++iter) {
                improved = false;
                for (int i = 0; i < 6; ++i)
                    for (int sgn : {-1, 1}) {
                        if (bound[i] <= 0) continue;  // not refined
                        double x[6];
                        std::copy(d, d + 6, x);
                        x[i] = std::clamp(d[i] + sgn * step[i], -bound[i], bound[i]);
                        if (x[i] == d[i]) continue;
                        const double sc = eval(x);
                        if (sc > bestScore + 1e-9) {
                            bestScore = sc;
                            std::copy(x, x + 6, d);
                            improved = true;
                        }
                    }
            }
            for (double& st : step) st /= 2;
        }
    }
    atBound = false;
    for (int i = 0; i < 6; ++i)
        atBound |= bound[i] > 0 && std::abs(d[i]) >= cfg_.boundFraction * bound[i];
    return correction(d) * baseline_;
}

void ExtrinsicMonitor::addFrame(const cv::Mat& image, std::int64_t frameTimeUs,
                                const std::vector<LidarPoint>& scan, const EgoMotion& ego) {
    ++offered_;
    if (current().status == ExtrinsicStatus::DEGRADED) return;  // latched for the session
    const bool turning = std::abs(ego.yawRate) > cfg_.maxYawRateDegS * kDeg;
    Evidence e;
    if (!turning) e = prepare(image, frameTimeUs, scan, ego);
    if (e.usable) {
        window_.push_back(std::move(e));
        ++evidenceCount_;
        ++sinceCheck_;
        const std::size_t keep = static_cast<std::size_t>(cfg_.recheckFrames);
        while (window_.size() > keep) window_.pop_front();
    }
    const ExtrinsicStatus s = current().status;
    const int need = s == ExtrinsicStatus::UNVERIFIED ? cfg_.verifyFrames : cfg_.recheckFrames;
    if (static_cast<int>(sinceCheck_) >= need) {
        check();
        sinceCheck_ = 0;
    } else if (s == ExtrinsicStatus::UNVERIFIED &&
               offered_ >= static_cast<std::uint64_t>(cfg_.maxUnverifiedFrames)) {
        setState(ExtrinsicStatus::DEGRADED, baseline_, 0,
                 "no usable evidence in " + std::to_string(offered_) +
                     " frames (a featureless scene, or the LiDAR and camera do not overlap)");
    }
}

namespace {

// A transform as a correction of `base`: rx, ry, rz (rad), translation.
void asCorrection(const Eigen::Isometry3d& T, const Eigen::Isometry3d& base, double d[6]) {
    const Eigen::Isometry3d c = T * base.inverse();
    const Eigen::Vector3d ypr = c.linear().eulerAngles(2, 1, 0);
    d[0] = std::remainder(ypr[2], M_PI);
    d[1] = std::remainder(ypr[1], M_PI);
    d[2] = std::remainder(ypr[0], M_PI);
    d[3] = c.translation().x();
    d[4] = c.translation().y();
    d[5] = c.translation().z();
}

}  // namespace

void ExtrinsicMonitor::check() {
    // The newest frames, split alternately into a fit half and a held-out half.
    std::vector<const Evidence*> fit, hold;
    double nearF = 0, farF = 0;
    const ExtrinsicStatus was = current().status;
    const std::size_t n = std::min<std::size_t>(window_.size(), was == ExtrinsicStatus::UNVERIFIED
                                                                    ? cfg_.verifyFrames
                                                                    : cfg_.recheckFrames);
    for (std::size_t i = window_.size() - n; i < window_.size(); ++i) {
        (i % 2 ? hold : fit).push_back(&window_[i]);
        nearF += window_[i].nearFraction;
        farF += window_[i].farFraction;
    }
    const bool depthSpread = nearF / n > 0.05 && farF / n > 0.05;
    auto gain = [](double from, double to) { return (to - from) / std::max(std::abs(from), 1e-9); };
    const double fit0 = score(fit, currentT_), hold0 = score(hold, currentT_);

    // 1. Within the bounds: fit, then keep each axis only as far as the held-out frames support.
    double fitBest = 0;
    bool unused = false;
    Eigen::Isometry3d best = search(fit, currentT_, fitBest, unused);
    double d[6];
    asCorrection(best, baseline_, d);
    double holdBest = score(hold, best);
    for (int i = 0; i < 6; ++i) {
        if (d[i] == 0) continue;
        for (double f : {0.0, 0.25, 0.5, 0.75}) {
            double x[6];
            std::copy(d, d + 6, x);
            x[i] = f * d[i];
            const Eigen::Isometry3d T = correction(x) * baseline_;
            const double h = score(hold, T);
            if (h >= holdBest - 0.01 * std::abs(holdBest)) {
                std::copy(x, x + 6, d);
                best = T;
                holdBest = std::max(h, holdBest - 0.01 * std::abs(holdBest));
                break;
            }
        }
    }
    holdBest = score(hold, best);
    fitBest = score(fit, best);
    const bool better = gain(fit0, fitBest) > cfg_.minGain && gain(hold0, holdBest) > cfg_.minGain;

    // 2. A wider look, coarse: is there a clearly better alignment beyond the bounds?
    double wideFit = 0;
    const Eigen::Isometry3d wide = search(fit, currentT_, wideFit, unused, cfg_.wideScale, false);
    double w[6];
    asCorrection(wide, baseline_, w);
    bool outside = false;
    for (int i = 0; i < 3; ++i) outside |= std::abs(w[i]) > cfg_.maxRotDeg * kDeg;
    // The coarse map can be flat or biased along an axis (KITTI: roll); a verdict of "beyond"
    // must hold on the fine map too: polish the wide result there and compare held out.
    bool beyond = false;
    if (outside && gain(score(fit, best, 0), wideFit) > cfg_.minGain) {
        double wideFine = 0;
        const Eigen::Isometry3d polished = search(fit, wide, wideFine, unused, cfg_.wideScale, true,
                                                  /*grid=*/false);
        asCorrection(polished, baseline_, w);
        outside = false;
        for (int i = 0; i < 3; ++i) outside |= std::abs(w[i]) > cfg_.maxRotDeg * kDeg;
        beyond = outside && gain(fitBest, wideFine) > cfg_.minGain &&
                 gain(holdBest, score(hold, polished)) > cfg_.minGain;
    }

    auto deg = [](double a) { return fmt(a / kDeg, 2); };
    const std::string scores =
        "score " + fmt(fit0) + " -> " + fmt(fitBest) + " (fit), " + fmt(hold0) + " -> " +
        fmt(holdBest) + " (held out); correction rx " + deg(d[0]) + " ry " + deg(d[1]) + " rz " +
        deg(d[2]) + " deg" +
        (outside ? "; wider look: rx " + deg(w[0]) + " ry " + deg(w[1]) + " rz " + deg(w[2]) : "");

    if (beyond) {
        setState(
            ExtrinsicStatus::DEGRADED, baseline_, hold0,
            "misaligned beyond the refinement bounds (" + scores + "): recalibrate (Part 12.2)");
        return;
    }
    if (better && depthSpread) {
        currentT_ = best;
        setState(ExtrinsicStatus::REFINED, best, holdBest, "refined: " + scores);
        return;
    }
    if (was == ExtrinsicStatus::UNVERIFIED) {
        setState(ExtrinsicStatus::VERIFIED, currentT_, hold0,
                 "the baseline is the best within the bounds: " + scores);
    } else {
        std::lock_guard<std::mutex> lock(m_);
        state_.score = hold0;
    }
}

void ExtrinsicMonitor::setState(ExtrinsicStatus s, const Eigen::Isometry3d& T, double score,
                                const std::string& reason) {
    {
        std::lock_guard<std::mutex> lock(m_);
        state_.status = s;
        state_.cameraFromLidar = T;
        state_.score = score;
        state_.reason = reason;
    }
    if (!log_) return;
    const Eigen::Isometry3d d = T * baseline_.inverse();
    const double angle = Eigen::AngleAxisd(d.linear()).angle() / kDeg;
    const std::string msg = std::string("extrinsics ") + toString(s) + ": " + reason +
                            "; correction from the baseline " + fmt(angle, 3) + " deg, " +
                            fmt(d.translation().norm() * 100, 2) + " cm";
    if (s == ExtrinsicStatus::DEGRADED)
        log_->logFault("ExtrinsicMonitor", msg);
    else
        log_->logGeneral(msg);
}

}  // namespace ar_drive_assist
