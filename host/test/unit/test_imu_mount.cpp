// ImuMount tests — docs/BUILD_GUIDE.md Part 12.2.2. The IMU sits in the windscreen pod, tilted;
// these check that its readings are turned into the car's axes, and that the two-step calibration
// recovers a known mount from simulated hub data (with sensor noise).

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ar_drive_assist/fusion/ImuMount.h"

using namespace ar_drive_assist;

namespace {
constexpr double kDeg = M_PI / 180.0;

// What the hub would report for a car doing (forwardAccel m/s^2, yawRate rad/s), level road.
Eigen::Vector3d accelReading(const ImuMount& truth, double forwardMps2) {
    const Eigen::Vector3d vehicle(forwardMps2 / 9.80665, 0, 1.0);  // g, specific force
    return truth.vehicleFromImu().transpose() * vehicle;
}
}  // namespace

TEST(ImuMount, IdentityPassesThrough) {
    const ImuMount m;
    EXPECT_NEAR(m.yawRateRadPerS({0, 0, 10}), 10 * kDeg, 1e-12);
    EXPECT_NEAR(m.vehicleCompassDeg(123.0), 123.0, 1e-12);
    EXPECT_NEAR(m.longitudinalAccelMps2({0.1, 0, 1}), 0.980665, 1e-9);
}

// The problem this class solves: a pod pitched 12 deg down and rolled 3 deg. The raw z gyro is
// wrong; the mounted conversion is right, including while the body rolls in a corner.
TEST(ImuMount, TiltedPodYawRateIsCorrectedAndRollIsRejected) {
    const ImuMount m = ImuMount::fromRollPitchYawDeg(3.0, -12.0, 2.0);
    const Eigen::Vector3d omegaVehicle(0.15, 0.0, 0.30);  // rad/s: body roll rate + yaw rate
    const Eigen::Vector3d gyroImuDeg = m.vehicleFromImu().transpose() * omegaVehicle / kDeg;
    EXPECT_NEAR(m.yawRateRadPerS(gyroImuDeg), 0.30, 1e-9);
    // Uncorrected, the error is several percent of the yaw rate: worth the class.
    EXPECT_GT(std::abs(gyroImuDeg.z() * kDeg - 0.30), 0.02);
}

TEST(ImuMount, CompassAddsTheMountYawAndWraps) {
    const ImuMount m = ImuMount::fromRollPitchYawDeg(0, 0, 10);
    EXPECT_NEAR(m.vehicleCompassDeg(100), 110, 1e-9);
    EXPECT_NEAR(m.vehicleCompassDeg(355), 5, 1e-9);
    const ImuMount n = ImuMount::fromRollPitchYawDeg(0, 0, -20);
    EXPECT_NEAR(n.vehicleCompassDeg(10), 350, 1e-9);
}

TEST(ImuMount, LevelRecoversRollAndPitchFromRestingGravity) {
    const ImuMount truth = ImuMount::fromRollPitchYawDeg(3.0, -12.0, 25.0);
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.01);  // g, BNO085-class noise
    std::vector<Eigen::Vector3d> rest;
    for (int i = 0; i < 250; ++i) {  // 5 s at 50 Hz
        rest.push_back(accelReading(truth, 0) +
                       Eigen::Vector3d(noise(rng), noise(rng), noise(rng)));
    }
    const auto m = ImuMount::levelFromRest(rest);
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->rollDeg(), 3.0, 0.1);
    EXPECT_NEAR(m->pitchDeg(), -12.0, 0.1);
    EXPECT_NEAR(m->yawDeg(), 0.0, 1e-12) << "gravity cannot tell yaw";
}

TEST(ImuMount, LevelRejectsAReadingThatIsNotAtRest) {
    EXPECT_FALSE(ImuMount::levelFromRest({}).has_value());
    EXPECT_FALSE(ImuMount::levelFromRest({{0.5, 0, 0.6}}).has_value());  // 0.78 g: moving
}

// Step 2 with both accelerating and braking runs, noisy, and a stretch of cruising (no signal)
// mixed in: yaw within half a degree.
TEST(ImuMount, YawFromStraightLineAccelerationAndBraking) {
    const ImuMount truth = ImuMount::fromRollPitchYawDeg(3.0, -12.0, 25.0);
    std::mt19937 rng(11);
    std::normal_distribution<double> noise(0.0, 0.01);
    std::vector<Eigen::Vector3d> rest;
    for (int i = 0; i < 250; ++i) rest.push_back(accelReading(truth, 0));
    const auto level = ImuMount::levelFromRest(rest);
    ASSERT_TRUE(level.has_value());

    std::vector<ImuMount::AccelSample> run;
    auto add = [&](double a, int n) {
        for (int i = 0; i < n; ++i) {
            ImuMount::AccelSample s;
            s.accelImuG =
                accelReading(truth, a) + Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
            s.longitudinalAccelMps2 = a;
            run.push_back(s);
        }
    };
    add(+2.0, 100);  // pulling away, 0.2 g
    add(0.0, 200);   // cruising: no information, skipped
    add(-3.0, 60);   // braking, 0.3 g
    const auto m = level->withYawFromStraightLine(run);
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->yawDeg(), 25.0, 0.5);
    EXPECT_NEAR(m->rollDeg(), 3.0, 0.1);
    EXPECT_NEAR(m->pitchDeg(), -12.0, 0.1);
    // Calibrated, the longitudinal acceleration reads correctly again.
    EXPECT_NEAR(m->longitudinalAccelMps2(accelReading(truth, -3.0)), -3.0, 0.05);
}

// A knocked pod: the calibrated mount no longer explains resting gravity.
TEST(ImuMount, TiltErrorFlagsAKnockedPod) {
    const ImuMount calibrated = ImuMount::fromRollPitchYawDeg(3.0, -12.0, 25.0);
    EXPECT_LT(calibrated.tiltErrorDeg(accelReading(calibrated, 0)), 1e-6);
    const ImuMount knocked = ImuMount::fromRollPitchYawDeg(3.0, -14.0, 25.0);  // pitched 2 deg
    EXPECT_NEAR(calibrated.tiltErrorDeg(accelReading(knocked, 0)), 2.0, 0.05);
}

TEST(ImuMount, YawNeedsRealExcitation) {
    const ImuMount level = ImuMount::fromRollPitchYawDeg(0, -12, 0);
    std::vector<ImuMount::AccelSample> gentle;
    for (int i = 0; i < 300; ++i) {
        ImuMount::AccelSample s;
        s.accelImuG = accelReading(level, 0.3);  // 0.03 g: lost in noise, below the floor
        s.longitudinalAccelMps2 = 0.3;
        gentle.push_back(s);
    }
    EXPECT_FALSE(level.withYawFromStraightLine(gentle).has_value());
}
