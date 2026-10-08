// LidarProcessor's ground fit (Part 8.1): the road surface each scan is measured against.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ar_drive_assist/lidar/LidarProcessor.h"

using namespace ar_drive_assist;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A road rising ahead at `pitchDeg` (height = x * tan(pitch)), sampled like LiDAR rings, with a
// little range noise; the LiDAR frame here is the vehicle frame (identity mount).
std::vector<LidarPoint> road(double pitchDeg, double noise = 0.02, unsigned seed = 1) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> n(0, noise);
    std::vector<LidarPoint> pts;
    for (double x = 2; x <= 35; x += 0.4)
        for (double y = -8; y <= 8; y += 0.4) {
            LidarPoint p;
            p.p = Eigen::Vector3f(x, y, x * std::tan(pitchDeg * kPi / 180) + n(rng));
            pts.push_back(p);
        }
    return pts;
}

void addBox(std::vector<LidarPoint>& pts, double x0, double y0, double base) {
    for (double z = 0.2; z <= 1.6; z += 0.1)
        for (double y = -0.9; y <= 0.9; y += 0.1) {
            LidarPoint p;
            p.p = Eigen::Vector3f(x0, y0 + y, base + z);
            pts.push_back(p);
        }
}

double heightAt(const GroundPlaneModel& g, double x) {
    return g.planeHeightAt(x, 0);
}

}  // namespace

TEST(GroundFit, FlatRoad) {
    LidarProcessor lp;
    const auto r = lp.fitGround(road(0), Eigen::Isometry3d::Identity(), 100);
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(heightAt(r.ground, 0), 0, 0.02);
    EXPECT_NEAR(heightAt(r.ground, 20), 0, 0.02);
    EXPECT_GT(r.ground.maxValidRangeM, 30);
    EXPECT_FALSE(r.ground.nearFieldPatch.empty());
}

// The KITTI case: the road rising 2 deg ahead. At 10 m it is 35 cm above z = 0, more than twice
// SceneReconstruction's ground tolerance, so a flat-road assumption calls it an obstacle.
TEST(GroundFit, ARisingRoadIsFittedNotTakenForAnObstacle) {
    LidarProcessor lp;
    const auto r = lp.fitGround(road(2), Eigen::Isometry3d::Identity(), 100);
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(heightAt(r.ground, 10), 10 * std::tan(2 * kPi / 180), 0.03);
    const GroundPlane gp = LidarProcessor::toGroundPlane(r.ground);
    EXPECT_NEAR(gp.distance({10, 0, 10 * std::tan(2 * kPi / 180)}), 0, 0.03);
    EXPECT_GT(gp.n.z(), 0.99);  // the normal points up
}

TEST(GroundFit, CarsAndWallsDoNotLiftTheRoad) {
    auto pts = road(0);
    addBox(pts, 8, 0, 0);
    addBox(pts, 15, -2, 0);
    for (double x = 2; x <= 35; x += 0.2)  // a building wall at the side
        for (double z = 0; z <= 6; z += 0.2) {
            LidarPoint p;
            p.p = Eigen::Vector3f(x, 7.5, z);
            pts.push_back(p);
        }
    LidarProcessor lp;
    const auto r = lp.fitGround(pts, Eigen::Isometry3d::Identity(), 100);
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(heightAt(r.ground, 8), 0, 0.03);
    // the car's points are well clear of the fitted road
    EXPECT_GT(LidarProcessor::toGroundPlane(r.ground).distance({8, 0, 0.5}), 0.4);
}

TEST(GroundFit, TheMountingIsApplied) {
    // A LiDAR 1.73 m up, pitched 15 deg down (the project's roof mount): the road must come out at
    // z = 0 in the vehicle frame.
    Eigen::Isometry3d vehicleFromLidar = Eigen::Isometry3d::Identity();
    vehicleFromLidar.translation() = Eigen::Vector3d(1.9, 0, 1.73);
    vehicleFromLidar.linear() =
        Eigen::AngleAxisd(15 * kPi / 180, Eigen::Vector3d::UnitY()).toRotationMatrix();
    std::vector<LidarPoint> pts;
    for (const LidarPoint& p : road(0)) {
        LidarPoint q;
        q.p = (vehicleFromLidar.inverse() * p.p.cast<double>()).cast<float>();
        pts.push_back(q);
    }
    LidarProcessor lp;
    const auto r = lp.fitGround(pts, vehicleFromLidar, 100);
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(heightAt(r.ground, 0), 0, 0.02);
    EXPECT_NEAR(heightAt(r.ground, 25), 0, 0.02);
}

TEST(GroundFit, NoRoadInTheScanKeepsThePreviousPlane) {
    LidarProcessor lp;
    ASSERT_TRUE(lp.fitGround(road(1), Eigen::Isometry3d::Identity(), 100).valid);
    std::vector<LidarPoint> blocked;  // only a truck's back filling the view
    addBox(blocked, 3, 0, 0);
    const auto r = lp.fitGround(blocked, Eigen::Isometry3d::Identity(), 200);
    EXPECT_FALSE(r.valid);
    EXPECT_NEAR(heightAt(r.ground, 10), 10 * std::tan(1 * kPi / 180), 0.03);
    EXPECT_EQ(r.ground.timestampMs, 200u);
}

TEST(GroundFit, SameScanSameAnswer) {
    LidarProcessor a, b;
    const auto pts = road(1.5, 0.05, 7);
    const auto ra = a.fitGround(pts, Eigen::Isometry3d::Identity(), 1);
    const auto rb = b.fitGround(pts, Eigen::Isometry3d::Identity(), 1);
    EXPECT_EQ(ra.ground.planeCoefficients, rb.ground.planeCoefficients);
}
