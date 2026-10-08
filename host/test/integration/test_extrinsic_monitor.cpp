// ExtrinsicMonitor (Part 12.2.1). Integration, not unit: it needs OpenCV's image processing, which
// the CPU-only unit suite does not link (Part 13.1's note). Synthetic scenes always; the KITTI
// drive's real frames and scans when data/recordings/kitti_0005 is present.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <random>

#include "ar_drive_assist/scene/ExtrinsicMonitor.h"
#include "ar_drive_assist/system/Recording.h"

using namespace ar_drive_assist;

namespace {

constexpr double kDeg = M_PI / 180.0;

Eigen::Isometry3d perturb(const Eigen::Isometry3d& T, double rx, double ry, double rz, double tx,
                          double ty, double tz) {
    Eigen::Isometry3d d = Eigen::Isometry3d::Identity();
    d.linear() = (Eigen::AngleAxisd(rz * kDeg, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(ry * kDeg, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(rx * kDeg, Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
    d.translation() = Eigen::Vector3d(tx, ty, tz);
    return d * T;
}

double angleDeg(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
    return Eigen::AngleAxisd((a.linear() * b.linear().transpose())).angle() / kDeg;
}

// --- Synthetic scenes ---------------------------------------------------------------------------
// The LiDAR at the camera, axes swapped (x forward, y left, z up). A scene of upright boxes in
// front of a far wall; the camera image is the boxes drawn in distinct greys over the wall's, so
// every box outline is an image edge; the LiDAR samples the boxes' front faces and the wall.

struct Box {
    double x, y, w, h, zBottom;  // centre ahead (x), left (y), width, height, bottom (LiDAR frame)
    int grey;
};

const Eigen::Isometry3d kCameraFromLidar = [] {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    T.translation() = Eigen::Vector3d(0.05, 0.3, -0.2);
    return T;
}();

CameraModel camera() {
    CameraModel c;
    c.width = 1280;
    c.height = 720;
    c.fx = c.fy = 1000;
    c.cx = 640;
    c.cy = 360;
    return c;
}

std::vector<Box> boxes(std::mt19937& rng) {  // far to near
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<Box> out;
    // A street: objects across the whole view (each quarter of its width), cars and walls low,
    // poles and buildings tall.
    for (int i = 0; i < 28; ++i) {
        const double x = 6 + u(rng) * 34;
        const double bearing = (-0.55 + 1.1 * ((i % 4) + u(rng)) / 4);  // y/x, across the view
        const bool tall = i % 3 == 0;
        out.push_back({x, bearing * x, tall ? 0.3 + u(rng) * 3 : 0.8 + u(rng) * 2.0,
                       tall ? 4 + u(rng) * 6 : 0.8 + u(rng) * 1.5, -1.6 + u(rng) * 0.3,
                       30 + static_cast<int>(u(rng) * 200)});
    }
    std::sort(out.begin(), out.end(), [](const Box& a, const Box& b) { return a.x > b.x; });
    return out;
}

// Renders with the TRUE extrinsics; returns the image and the scan (LiDAR frame).
void scene(std::mt19937& rng, bool featureless, cv::Mat& img, std::vector<LidarPoint>& scan) {
    const CameraModel cam = camera();
    img = cv::Mat(cam.height, cam.width, CV_8UC1, cv::Scalar(128));
    scan.clear();
    std::normal_distribution<double> noise(0, 0.02);
    std::uniform_real_distribution<double> u(0, 1);
    auto add = [&](const Eigen::Vector3d& p) {
        LidarPoint lp;
        const Eigen::Vector3d q = p + p.normalized() * noise(rng);
        lp.p = q.cast<float>();
        lp.tUs = 0;
        scan.push_back(lp);
    };
    const std::vector<Box> bs = featureless ? std::vector<Box>{} : boxes(rng);
    // The LiDAR sees only what no nearer box hides from it (it is beside the camera, not at it).
    const Eigen::Vector3d lidarAt = Eigen::Vector3d::Zero();
    auto visible = [&](const Eigen::Vector3d& p) {
        for (const Box& b : bs) {
            if (b.x >= p.x() - 1e-6) continue;
            const double t = (b.x - lidarAt.x()) / (p.x() - lidarAt.x());
            const Eigen::Vector3d q = lidarAt + t * (p - lidarAt);
            if (std::abs(q.y() - b.y) < b.w / 2 && q.z() > b.zBottom && q.z() < b.zBottom + b.h)
                return false;
        }
        return true;
    };
    for (int i = 0; i < 60000; ++i) {  // the wall, 60 m ahead
        const Eigen::Vector3d p(60, (u(rng) - 0.5) * 80, -1.8 + u(rng) * 12);
        if (visible(p)) add(p);
    }
    if (featureless) return;
    for (const Box& b : bs) {  // far to near: nearer boxes paint over
        std::vector<cv::Point> poly;
        for (auto [dy, dz] : {std::pair{-0.5, 0.0}, {0.5, 0.0}, {0.5, 1.0}, {-0.5, 1.0}}) {
            Eigen::Vector2d px;
            cam.project(
                kCameraFromLidar * Eigen::Vector3d(b.x, b.y + dy * b.w, b.zBottom + dz * b.h), px);
            poly.emplace_back(static_cast<int>(px.x()), static_cast<int>(px.y()));
        }
        cv::fillConvexPoly(img, poly, cv::Scalar(b.grey), cv::LINE_AA);
        const int n = static_cast<int>(b.w * b.h * 1500 / (b.x / 10));  // sparser when far
        for (int k = 0; k < n; ++k) {
            const Eigen::Vector3d p(b.x, b.y + (u(rng) - 0.5) * b.w, b.zBottom + u(rng) * b.h);
            if (visible(p)) add(p);
        }
    }
    cv::GaussianBlur(img, img, cv::Size(3, 3), 0.8);
}

ExtrinsicState runSynthetic(const Eigen::Isometry3d& baseline, bool featureless, int frames,
                            ExtrinsicMonitorConfig cfg = {}) {
    ExtrinsicMonitor mon(baseline, Eigen::Isometry3d::Identity(), camera(), cfg);
    std::mt19937 rng(7);
    for (int f = 0; f < frames; ++f) {
        cv::Mat img;
        std::vector<LidarPoint> scan;
        scene(rng, featureless, img, scan);
        mon.addFrame(img, 0, scan, {});
    }
    return mon.current();
}

}  // namespace

TEST(ExtrinsicMonitor, TheTrueCalibrationIsVerified) {
    const ExtrinsicState s = runSynthetic(kCameraFromLidar, false, 14);
    EXPECT_EQ(s.status, ExtrinsicStatus::VERIFIED) << s.reason;
    EXPECT_LT(angleDeg(s.cameraFromLidar, kCameraFromLidar), 1e-9);
}

TEST(ExtrinsicMonitor, ASmallErrorIsRefinedBackToTheTruth) {
    const Eigen::Isometry3d off = perturb(kCameraFromLidar, 0.4, -0.5, 0.3, 0.0, 0.0, 0.0);
    const ExtrinsicState s = runSynthetic(off, false, 14);
    ASSERT_EQ(s.status, ExtrinsicStatus::REFINED) << s.reason;
    EXPECT_LT(angleDeg(s.cameraFromLidar, kCameraFromLidar), 0.12) << s.reason;
    EXPECT_LT(angleDeg(s.cameraFromLidar, kCameraFromLidar), angleDeg(off, kCameraFromLidar) / 4);
}

TEST(ExtrinsicMonitor, ALargeErrorIsDegradedNotFixedAtTheBound) {
    ExtrinsicMonitorConfig cfg;
    cfg.maxUnverifiedFrames = 30;  // beyond the score's reach it never verifies, then times out
    const Eigen::Isometry3d off = perturb(kCameraFromLidar, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
    const ExtrinsicState s = runSynthetic(off, false, 40, cfg);
    EXPECT_EQ(s.status, ExtrinsicStatus::DEGRADED) << s.reason;
    EXPECT_LT(angleDeg(s.cameraFromLidar, off), 1e-9) << "the baseline is kept";
}

TEST(ExtrinsicMonitor, AFeaturelessSceneIsNeverRefined) {
    ExtrinsicMonitorConfig cfg;
    cfg.maxUnverifiedFrames = 30;
    const Eigen::Isometry3d off = perturb(kCameraFromLidar, 0.4, -0.5, 0.3, 0.0, 0.0, 0.0);
    const ExtrinsicState s = runSynthetic(off, true, 40, cfg);
    EXPECT_NE(s.status, ExtrinsicStatus::REFINED);
    EXPECT_EQ(s.status, ExtrinsicStatus::DEGRADED) << "no evidence within the configured frames";
}

// --- Real data: the KITTI drive -----------------------------------------------------------------

namespace {

const std::string kKitti = std::string(HOST_SOURCE_DIR) + "/../data/recordings/kitti_0005";

ExtrinsicState runKitti(const Eigen::Isometry3d& baseline) {
    const Recording rec = loadRecording(kKitti);
    CameraModel cam;
    cam.width = rec.width;
    cam.height = rec.height;
    cam.fx = cam.fy = 721.5377;  // the recording's rectified P_rect_02 (camera_intrinsics.yaml)
    cam.cx = 609.5593;
    cam.cy = 172.854;
    ExtrinsicMonitorConfig cfg;
    cfg.cellPx = 3;  // a 1242 x 375 image: finer cells
    ExtrinsicMonitor mon(baseline, rec.vehicleFromLidar, cam, cfg);
    cv::VideoCapture v(rec.video());
    const auto times = loadScanTimes(kKitti);
    cv::Mat img;
    for (std::size_t i = 0; i < times.size() && v.read(img); ++i) {
        std::vector<LidarPoint> scan;
        for (const auto& p : readScan(rec.scanFile(times[i].index))) {
            LidarPoint lp;
            lp.p = p.head<3>();
            scan.push_back(lp);  // KITTI triggers the camera as the sweep passes ahead: same time
        }
        mon.addFrame(img, 0, scan, {});
    }
    return mon.current();
}

}  // namespace

// KITTI's published calibration is not exact truth (errors of a few tenths of a degree are
// reported for it), so the test is consistency: from KITTI's calibration and from a mount knocked
// 0.7 deg off it, the monitor must end close to KITTI's and to each other. Measured (2026-10-08):
// 0.22 and 0.34 deg from KITTI's, 0.18 deg apart, the difference almost all roll: on KITTI's wide,
// short image (1242 x 375) half a degree of roll moves the image corners by ~5 px, and the
// held-out check keeps only as much roll as the data supports.
TEST(ExtrinsicMonitorKitti, FromKittisCalibrationOrAKnockedMountItEndsInTheSamePlace) {
    if (!std::filesystem::exists(kKitti)) GTEST_SKIP() << "no " << kKitti;
    const Recording rec = loadRecording(kKitti);
    const ExtrinsicState a = runKitti(rec.cameraFromLidar);
    const Eigen::Isometry3d off = perturb(rec.cameraFromLidar, 0.0, 0.6, -0.4, 0.0, 0.0, 0.0);
    const ExtrinsicState b = runKitti(off);
    std::printf("[ info ] from KITTI's: %s, %.3f deg from it (%s)\n", toString(a.status),
                angleDeg(a.cameraFromLidar, rec.cameraFromLidar), a.reason.c_str());
    std::printf("[ info ] knocked %.3f deg: %s, %.3f deg from KITTI's (%s)\n",
                angleDeg(off, rec.cameraFromLidar), toString(b.status),
                angleDeg(b.cameraFromLidar, rec.cameraFromLidar), b.reason.c_str());
    ASSERT_NE(a.status, ExtrinsicStatus::DEGRADED) << a.reason;
    ASSERT_EQ(b.status, ExtrinsicStatus::REFINED) << b.reason;
    EXPECT_LT(angleDeg(a.cameraFromLidar, rec.cameraFromLidar), 0.3);
    EXPECT_LT(angleDeg(b.cameraFromLidar, rec.cameraFromLidar), 0.4);
    EXPECT_LT(angleDeg(b.cameraFromLidar, rec.cameraFromLidar),
              angleDeg(off, rec.cameraFromLidar) / 2);
    EXPECT_LT(angleDeg(a.cameraFromLidar, b.cameraFromLidar), 0.25) << "two starts, one answer";
}

TEST(ExtrinsicMonitorKitti, AMountKnockedTooFarIsDegraded) {
    if (!std::filesystem::exists(kKitti)) GTEST_SKIP() << "no " << kKitti;
    const Recording rec = loadRecording(kKitti);
    const ExtrinsicState s = runKitti(perturb(rec.cameraFromLidar, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0));
    std::printf("[ info ] %s: %s\n", toString(s.status), s.reason.c_str());
    EXPECT_EQ(s.status, ExtrinsicStatus::DEGRADED) << s.reason;
}

// While the car turns, nothing counts as evidence: a timing error would read as a rotation.
TEST(ExtrinsicMonitor, NoEvidenceWhileTurning) {
    ExtrinsicMonitor mon(kCameraFromLidar, Eigen::Isometry3d::Identity(), camera());
    std::mt19937 rng(7);
    for (int f = 0; f < 6; ++f) {
        cv::Mat img;
        std::vector<LidarPoint> scan;
        scene(rng, false, img, scan);
        EgoMotion turning;
        turning.speed = 8;
        turning.yawRate = 10 * kDeg;  // 10 deg/s
        mon.addFrame(img, 0, scan, turning);
    }
    EXPECT_EQ(mon.evidenceFrames(), 0u);
    EXPECT_EQ(mon.current().status, ExtrinsicStatus::UNVERIFIED);
}
