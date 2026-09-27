// WindowedSink on the real GPU — label `gpu` (Part 13.2; not in CI). Renders into a hidden window
// and reads the composited frame back.
//
// These check what unit tests cannot: that the SHADERS agree with the C++ camera model
// (common/Camera.h, including lens distortion), and that rule 4's depth-aware occlusion works on
// the GPU. A road item that the vertex shader projected differently from the perception side would
// land a few pixels off the object it marks; this test would catch that.

#include <gtest/gtest.h>

#include <cmath>

#include "ar_drive_assist/render/WindowedSink.h"

using namespace ar_drive_assist;

namespace {

constexpr int kW = 1280, kH = 720;

CameraModel distortedCamera() {
    CameraModel c;
    c.width = kW;
    c.height = kH;
    c.fx = c.fy = 700;
    c.cx = 640;
    c.cy = 360;
    c.k1 = -0.15;  // a real lens's barrel distortion
    c.k2 = 0.04;
    c.p1 = 0.001;
    return c;
}
const Eigen::Isometry3d kPose = cameraFromVehicle({1.8, 0, 1.3}, 0, 4, 0);  // pitched 4 deg down

class SinkTest : public ::testing::Test {
protected:
    void SetUp() override {
        WindowedSinkOptions o;
        o.hidden = true;
        sink = std::make_unique<WindowedSink>(o);
        ASSERT_TRUE(sink->init({kW, kH, 16.0f / 9}, distortedCamera(), kPose)) << sink->lastError();
        std::printf("[ info ] renderer: %s\n", sink->renderer().c_str());
    }
    cv::Mat render(const OverlayScene& s) {
        CompositedFrame f{cv::Mat(kH, kW, CV_8UC3, cv::Scalar(0, 0, 0)), s, 0};
        sink->present(f);
        return sink->capture();
    }
    std::unique_ptr<WindowedSink> sink;
};

OverlayItem band(double x0, double x1, double y, const Rgba& c) {
    OverlayItem b;
    b.kind = OverlayKind::ROAD_BAND;
    b.widthM = 0.4f;
    b.style = {c, 1.0f, 0, 0, 0, 1};
    b.occludedByHazards = true;
    for (double x = x0; x <= x1; x += 0.5) b.points.emplace_back(x, y, 0);
    return b;
}

}  // namespace

// Thin bands on the road; every sampled road point must appear where CameraModel::project says,
// within 2 px, through barrel distortion. Points near the image EDGE matter most: distortion grows
// with distance from the centre (a first version sampled only near the centre, where dropping
// distortion from the shader moved points by just ~3 px and went unnoticed).
TEST_F(SinkTest, ShaderProjectionMatchesTheCameraModel) {
    // Separate short DASHES, one per sample point, not continuous bands: radial distortion moves
    // a point towards the image centre, which for a band receding into the distance is almost
    // ALONG the band, so a shifted point still lands on lit band pixels (a continuous-band version
    // of this test passed with distortion removed from the shader). A dash is surrounded by road.
    OverlayScene s;
    auto dash = [&](double x, double y) {
        const double half = x > 15 ? 0.75 : 0.25;  // longer far away, to stay a few pixels tall
        s.items.push_back(band(x - half, x + half, y, palette::kGreen));
        s.items.back().points = {{x - half, y, 0}, {x, y, 0}, {x + half, y, 0}};
    };
    for (double x = 10; x <= 28; x += 3) dash(x, 1.5);  // near the centre
    for (double x = 6; x <= 11; x += 1) dash(x, 4.0);   // towards the left edge
    const cv::Mat img = render(s);
    ASSERT_FALSE(img.empty());
    const CameraModel cam = distortedCamera();
    int checked = 0;
    auto expectLit = [&](double x, double y) {
        Eigen::Vector2d px;
        ASSERT_TRUE(cam.project(kPose * Eigen::Vector3d(x, y, 0), px)) << x << ", " << y;
        bool lit = false;
        for (int dy = -2; dy <= 2 && !lit; ++dy)
            for (int dx = -2; dx <= 2 && !lit; ++dx) {
                const cv::Vec3b c =
                    img.at<cv::Vec3b>(static_cast<int>(px.y()) + dy, static_cast<int>(px.x()) + dx);
                lit = c[1] > 80 && c[1] > c[2];  // green channel dominant
            }
        EXPECT_TRUE(lit) << "(" << x << ", " << y << ") expected near (" << px.x() << ", " << px.y()
                         << ")";
        ++checked;
    };
    for (double x = 10; x <= 28; x += 3) expectLit(x, 1.5);
    for (double x = 6; x <= 11; x += 1) expectLit(x, 4.0);
    EXPECT_EQ(checked, 13);
    // How much the lens moves the edge points: must be large, or this test proves nothing.
    Eigen::Vector2d d, u;
    CameraModel pin = cam;
    pin.k1 = pin.k2 = pin.p1 = pin.p2 = pin.k3 = 0;
    cam.project(kPose * Eigen::Vector3d(6, 4, 0), d);
    pin.project(kPose * Eigen::Vector3d(6, 4, 0), u);
    std::printf("[ info ] distortion moves the edge point by %.1f px\n", (d - u).norm());
    EXPECT_GT((d - u).norm(), 10.0);
}

// Rule 4 on the GPU. A hazard silhouette at 20 m (camera depth 18.2 m) covers the middle of the
// image. A barrier at 12 m (IN FRONT) must stay visible across it; a band at 30 m (BEHIND) must be
// cut out where the silhouette is, and still visible outside it.
TEST_F(SinkTest, RoadItemsAreHiddenOnlyBehindHazards) {
    OverlayScene s;
    PixelMask m;  // covers x 540..740, y 200..520 (cells of 4 px)
    m.cellPx = 4;
    m.x = 135;
    m.y = 50;
    m.w = 50;
    m.h = 80;
    m.bits.assign(static_cast<std::size_t>(m.w) * m.h, 1);
    s.occluders.push_back({m, 18.2});
    OverlayItem barrier;
    barrier.kind = OverlayKind::ROAD_BARRIER;
    barrier.points = {{12, 1.5, 0}, {12, -1.5, 0}};
    barrier.heightM = 1.2f;
    barrier.style = {palette::kRed, 1.0f, 0, 0, 0, 2};
    barrier.occludedByHazards = true;
    s.items.push_back(barrier);
    // A wall at 30 m (BEHIND the hazard), wide enough to reach outside the silhouette. (A band
    // lying on the road 30 m away is only ~1 px tall on screen: too thin to sample reliably.)
    OverlayItem far = barrier;
    far.points = {{30, 6, 0}, {30, -6, 0}};
    far.heightM = 1.5f;
    far.style.color = palette::kGreen;
    s.items.push_back(far);
    const cv::Mat img = render(s);
    const CameraModel cam = distortedCamera();
    Eigen::Vector2d pb, pfIn, pfOut;
    cam.project(kPose * Eigen::Vector3d(12, 0, 0.5), pb);       // barrier centre: inside the mask
    cam.project(kPose * Eigen::Vector3d(30, 0, 0.7), pfIn);     // far wall centre: inside the mask
    cam.project(kPose * Eigen::Vector3d(30, 5.5, 0.7), pfOut);  // far wall left part: outside it
    ASSERT_TRUE(pb.x() > 540 && pb.x() < 740 && pb.y() > 200 && pb.y() < 520);
    ASSERT_TRUE(pfIn.x() > 540 && pfIn.x() < 740 && pfIn.y() > 200 && pfIn.y() < 520);
    const cv::Vec3b b = img.at<cv::Vec3b>(static_cast<int>(pb.y()), static_cast<int>(pb.x()));
    const cv::Vec3b fin = img.at<cv::Vec3b>(static_cast<int>(pfIn.y()), static_cast<int>(pfIn.x()));
    const cv::Vec3b fout =
        img.at<cv::Vec3b>(static_cast<int>(pfOut.y()), static_cast<int>(pfOut.x()));
    EXPECT_GT(b[2], 60) << "barrier in front of the hazard: visible (red)";
    ASSERT_FALSE(pfOut.x() > 540 && pfOut.x() < 740 && pfOut.y() > 200 && pfOut.y() < 520);
    EXPECT_LT(fin[1], 10) << "wall behind the hazard: hidden";
    EXPECT_GT(fout[1], 40) << "the same wall outside the silhouette: visible";
}

// The frame is passed through: an empty scene reproduces the video.
TEST_F(SinkTest, VideoPassesThroughUnchanged) {
    cv::Mat v(kH, kW, CV_8UC3);
    cv::randu(v, 0, 255);
    CompositedFrame f{v, {}, 0};
    sink->present(f);
    const cv::Mat out = sink->capture();
    ASSERT_EQ(out.size(), v.size());
    EXPECT_LE(cv::norm(out, v, cv::NORM_INF), 1.0);
}
