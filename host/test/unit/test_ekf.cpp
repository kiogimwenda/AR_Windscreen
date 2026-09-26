// SensorFusion (EKF) tests — see docs/BUILD_GUIDE.md Part 8.2 and SensorFusion.h.
//
// A simulated car drives an exactly known trajectory, and simulated sensors report it with
// realistic noise at their real rates: gyro 100 Hz; GPS, OBD and compass 10 Hz. The filter must
// estimate the truth better than the raw sensors, handle the conventions (compass vs maths angle,
// wrap-around at +-180 deg, the straight-line limit), and be CONSISTENT: its own uncertainty must
// match its actual errors (the NIS check). Fixed seeds keep every run identical.

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "ar_drive_assist/fusion/SensorFusion.h"

using ar_drive_assist::FusionNoise;
using ar_drive_assist::SensorFusion;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kR = 6378137.0;
constexpr double kLat0 = -1.2921, kLon0 = 36.8219;  // Nairobi

// Exact CTRV truth, stepped finely.
struct Truth {
    double x = 0, y = 0, psi = 0, v = 0, w = 0;
    void step(double dt) {
        if (std::abs(w) > 1e-9) {
            x += v / w * (std::sin(psi + w * dt) - std::sin(psi));
            y += v / w * (std::cos(psi) - std::cos(psi + w * dt));
        } else {
            x += v * std::cos(psi) * dt;
            y += v * std::sin(psi) * dt;
        }
        psi += w * dt;
    }
    double lat() const { return kLat0 + y / kR / kDeg; }
    double lon() const { return kLon0 + x / (kR * std::cos(kLat0 * kDeg)) / kDeg; }
    double compassDeg() const { return 90.0 - psi / kDeg; }
};

struct RunResult {
    double posErr, speedErr, headingErrDeg, yawRateErr;
    double meanGpsNis;
    int gpsUpdates;
    double speedSigma;  // the filter's OWN 1-sigma speed uncertainty, sqrt(P_vv)
};

// Drives `truth` for `seconds`, feeding noisy sensors to the filter at their real rates.
RunResult simulate(Truth truth, double seconds, unsigned seed, double headingStart = 0.0) {
    FusionNoise n;
    SensorFusion f(n);
    std::mt19937 rng(seed);
    std::normal_distribution<double> gps(0, n.gpsPosStd), obd(0, n.obdSpeedStd), gyro(0, n.gyroStd),
        head(0, n.headingStd);
    auto gpsFix = [&] {
        const double nx = gps(rng), ny = gps(rng);
        return std::pair{truth.lat() + ny / kR / kDeg,
                         truth.lon() + nx / (kR * std::cos(kLat0 * kDeg)) / kDeg};
    };
    auto [la, lo] = gpsFix();
    f.updateGps(la, lo);  // initialises
    (void)headingStart;

    const double dt = 0.01;  // 100 Hz IMU
    double nisSum = 0;
    int nisN = 0;
    const int steps = static_cast<int>(seconds / dt);
    for (int k = 1; k <= steps; ++k) {
        truth.step(dt);
        f.predict(dt);
        f.updateGyro(truth.w + gyro(rng));
        if (k % 10 == 0) {  // 10 Hz sensors
            auto [lat, lon] = gpsFix();
            const auto nis = f.updateGps(lat, lon);
            f.updateObdSpeed(truth.v + obd(rng));
            f.updateCompassHeading(truth.compassDeg() + head(rng) / kDeg);
            // Consistency is judged after the initial transient (heading and speed unknown).
            if (nis && k * dt > 10.0) {
                nisSum += *nis;
                ++nisN;
            }
        }
    }
    // The filter's origin is its (noisy) first fix, so the truth is measured in ITS frame.
    const Eigen::Vector2d t = f.toLocal(truth.lat(), truth.lon());
    const auto& s = f.state();
    return {std::hypot(s(0) - t.x(), s(1) - t.y()),
            std::abs(s(3) - truth.v),
            std::abs(SensorFusion::wrapAngle(s(2) - truth.psi)) / kDeg,
            std::abs(s(4) - truth.w),
            nisN ? nisSum / nisN : 0.0,
            nisN,
            std::sqrt(f.covariance()(3, 3))};
}

}  // namespace

TEST(Ekf, WrapAngle) {
    EXPECT_NEAR(SensorFusion::wrapAngle(0.0), 0.0, 1e-12);
    EXPECT_NEAR(SensorFusion::wrapAngle(3 * kPi), kPi, 1e-12);
    EXPECT_NEAR(SensorFusion::wrapAngle(-kPi), kPi, 1e-12);  // (-pi, pi]: -pi maps to +pi
    EXPECT_NEAR(SensorFusion::wrapAngle(359 * kDeg), -1 * kDeg, 1e-12);
}

// 0.001 deg of latitude is ~111.3 m north everywhere; 0.001 deg of longitude is 111.3 m x cos(lat)
// east. At Nairobi (1.29 deg S) that is still ~111.3 m.
TEST(Ekf, LocalProjection) {
    SensorFusion f;
    f.updateGps(kLat0, kLon0);
    const auto n = f.toLocal(kLat0 + 0.001, kLon0);
    const auto e = f.toLocal(kLat0, kLon0 + 0.001);
    EXPECT_NEAR(n.x(), 0.0, 1e-9);
    EXPECT_NEAR(n.y(), 111.32, 0.05);
    EXPECT_NEAR(e.x(), 111.32 * std::cos(kLat0 * kDeg), 0.05);
}

TEST(Ekf, NothingHappensBeforeTheFirstGpsFix) {
    SensorFusion f;
    EXPECT_FALSE(f.initialised());
    f.predict(0.01);
    EXPECT_FALSE(f.updateObdSpeed(10.0).has_value());
    EXPECT_FALSE(f.updateGyro(0.1).has_value());
    EXPECT_FALSE(f.updateCompassHeading(90.0).has_value());
    EXPECT_EQ(f.state().norm(), 0.0);
}

// Straight drive east at 10 m/s for 60 s. The fused position must beat the raw 2.5 m GPS noise
// comfortably, and speed and heading must converge. Speed is judged against the filter's own
// stated uncertainty (within 3 sigma), plus an absolute ceiling. The filter assumes the car can
// accelerate (~2 m/s^2), so it averages the 0.3 m/s OBD noise only lightly. A fixed 0.2 m/s bound
// would demand more certainty than the filter claims.
TEST(Ekf, StraightLineConvergesBelowGpsNoise) {
    Truth t;
    t.v = 10.0;
    const auto r = simulate(t, 60.0, 1);
    EXPECT_LT(r.posErr, 1.5) << "fused position should be well inside the 2.5 m GPS sigma";
    EXPECT_LT(r.speedErr, 3.0 * r.speedSigma) << "speed error inconsistent with the filter's sigma";
    EXPECT_LT(r.speedErr, 0.5);
    EXPECT_LT(r.speedSigma, 0.3) << "should be no worse than one raw OBD reading";
    EXPECT_LT(r.headingErrDeg, 2.0);
    EXPECT_LT(r.yawRateErr, 0.01);
}

// A steady left turn (0.1 rad/s at 8 m/s, a 80 m radius circle): the CTRV model must follow the
// curve, not cut it.
TEST(Ekf, SteadyTurnIsTracked) {
    Truth t;
    t.v = 8.0;
    t.w = 0.1;
    const auto r = simulate(t, 60.0, 2);
    EXPECT_LT(r.posErr, 1.5);
    EXPECT_LT(r.headingErrDeg, 2.0);
    EXPECT_LT(r.yawRateErr, 0.01);
}

// Driving NORTH: a compass heading of 0 deg must become psi = +90 deg, and y (north) must grow.
// A swapped convention would mirror every turn.
TEST(Ekf, CompassConventionDrivingNorth) {
    Truth t;
    t.v = 10.0;
    t.psi = kPi / 2;  // north
    const auto r = simulate(t, 30.0, 3);
    EXPECT_LT(r.headingErrDeg, 2.0);
    EXPECT_LT(r.posErr, 1.5);
}

// Near +-180 deg: the estimate is at +179 deg and a measurement says -179 deg. The true
// difference is 2 deg, across the wrap, so the estimate must move TO ~180, not swing through 0.
TEST(Ekf, HeadingUpdateAcrossTheWrapTakesTheShortWay) {
    SensorFusion f;
    f.updateGps(kLat0, kLon0);
    f.updateCompassHeading(90.0 - 179.0);  // psi = 179 deg
    for (int i = 0; i < 20; ++i) f.updateCompassHeading(90.0 - 179.0);
    ASSERT_NEAR(f.state()(2) / kDeg, 179.0, 0.5);
    f.updateCompassHeading(90.0 + 179.0);  // psi = -179 deg
    const double psi = f.state()(2) / kDeg;
    EXPECT_GT(std::abs(psi), 178.0) << "moved the long way round, through 0 deg: " << psi;
}

// The CTRV straight-line branch and the curved branch must agree near omega = 0. A mismatch would
// be a jump in the prediction whenever the car's yaw rate crosses the threshold.
TEST(Ekf, CurvedAndStraightBranchesAgreeNearZeroYawRate) {
    auto run = [](double w) {
        SensorFusion f;
        f.updateGps(kLat0, kLon0);
        f.updateObdSpeed(15.0);
        f.updateGyro(w);
        f.predict(0.1);
        return f.state();
    };
    const auto a = run(0.0), b = run(2e-4);  // either side of the 1e-4 threshold
    EXPECT_NEAR(a(0), b(0), 1e-3);
    EXPECT_NEAR(a(1), b(1), 1e-3);
    EXPECT_TRUE(a.allFinite() && b.allFinite());
}

// Consistency: with correctly modelled measurement noise, the GPS innovation's NIS follows a
// chi-square distribution with 2 degrees of freedom, whose mean is 2. The truth here has no
// process noise, while the filter assumes some, so the filter is slightly conservative and the
// mean may sit a little under 2, never far above it. Far above 2 = overconfident, the dangerous
// direction.
TEST(Ekf, GpsInnovationIsConsistent) {
    Truth t;
    t.v = 12.0;
    t.w = 0.05;
    const auto r = simulate(t, 120.0, 4);
    ASSERT_GT(r.gpsUpdates, 1000);
    EXPECT_GT(r.meanGpsNis, 1.5);
    EXPECT_LT(r.meanGpsNis, 2.3) << "filter overconfident: its covariance is too small";
}

// The covariance must stay a valid covariance (symmetric, positive definite) over a long run: this
// is what the Joseph-form update protects.
TEST(Ekf, CovarianceStaysSymmetricPositiveDefinite) {
    SensorFusion f;
    f.updateGps(kLat0, kLon0);
    std::mt19937 rng(5);
    std::normal_distribution<double> n(0, 1);
    for (int k = 0; k < 60000; ++k) {  // ten minutes at 100 Hz (keeps CI fast)
        f.predict(0.01);
        f.updateGyro(0.02 * n(rng));
        if (k % 10 == 0) {
            f.updateObdSpeed(10 + 0.3 * n(rng));
            f.updateGps(kLat0 + 1e-5 * n(rng), kLon0 + 1e-5 * n(rng));
        }
    }
    const auto& P = f.covariance();
    EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-9);
    Eigen::SelfAdjointEigenSolver<SensorFusion::Cov> es(P);
    EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}
