// HubReportFeeder tests — BUILD_GUIDE Part 8.2 and 12.2.2. A simulated drive (straight, a long
// turn, hard braking) produces 50 Hz SensorReports the way the hub does: GNSS fixes at 5 Hz
// repeated between sentences, OBD speed at 10 Hz with a dropout, a 100 Hz IMU tilted in the
// windscreen pod, a biased magnetometer. The EKF fed through the feeder must track the truth, and
// each rule (new fixes only, rate limits, NaN, stale fixes, clock resets, mounting) is checked.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "ar_drive_assist/common/Geo.h"
#include "ar_drive_assist/fusion/HubReportFeeder.h"

using namespace ar_drive_assist;

namespace {
constexpr double kDeg = M_PI / 180.0;
constexpr double kG = 9.80665;
const GeoPoint kOrigin{-1.2864, 36.8172};  // Nairobi CBD

struct Truth {
    double x = 0, y = 0, psi = 0.5, v = 10, w = 0, a = 0;
};

struct Sim {
    ImuMount mount = ImuMount::fromRollPitchYawDeg(3.0, -12.0, 25.0);
    std::mt19937 rng{42};
    std::normal_distribution<double> n{0.0, 1.0};
    LocalFrame frame{kOrigin};
    double obdNanFrom = 12.0, obdNanTo = 14.0;
    double magBiasDeg = 2.0;
    double slopeBias = 0.3;  // m/s^2: a 3 % grade tilts gravity into the accelerometer's x
    int distinctFixes = 0;
    Truth t;
    hub_protocol::SensorReport last{};

    // Truth motion for the time t (s): straight 0-10 s, turning 10-25 s, braking 25-28 s.
    void step(double time, double dt) {
        t.w = (time >= 10 && time < 25) ? 0.15 : 0.0;
        t.a = (time >= 25 && time < 28) ? -3.0 : 0.0;
        t.v = std::max(0.0, t.v + t.a * dt);
        t.x += t.v * std::cos(t.psi) * dt;
        t.y += t.v * std::sin(t.psi) * dt;
        t.psi += t.w * dt;
    }

    hub_protocol::SensorReport report(int k) {
        const double time = k * 0.02;
        hub_protocol::SensorReport r = last;
        r.timestampMs = static_cast<std::uint32_t>(1000 + k * 20);
        if (k % 10 == 0) {  // a new GNSS fix every 200 ms (5 Hz), repeated until the next
            const GeoPoint g = frame.toGeodetic(t.x + 1.5 * n(rng), t.y + 1.5 * n(rng));
            r.latitude = g.lat;
            r.longitude = g.lon;
            r.speedKph = static_cast<float>((t.v + 0.1 * n(rng)) * 3.6);
            r.gpsFixValid = 1;
            ++distinctFixes;
        }
        if (k % 5 == 0)  // OBD answers at 10 Hz
            r.obdSpeedKph =
                (time >= obdNanFrom && time < obdNanTo)
                    ? NAN
                    : static_cast<float>(std::round(t.v * 3.6));  // integer km/h, like PID 0x0D
        const Eigen::Vector3d gyro =
            mount.vehicleFromImu().transpose() * Eigen::Vector3d(0, 0, t.w) / kDeg +
            Eigen::Vector3d(n(rng), n(rng), n(rng)) * 0.3;
        const Eigen::Vector3d acc =
            mount.vehicleFromImu().transpose() * Eigen::Vector3d((t.a + slopeBias) / kG, 0, 1) +
            Eigen::Vector3d(n(rng), n(rng), n(rng)) * 0.02;
        r.gyroX = static_cast<float>(gyro.x());
        r.gyroY = static_cast<float>(gyro.y());
        r.gyroZ = static_cast<float>(gyro.z());
        r.accelX = static_cast<float>(acc.x());
        r.accelY = static_cast<float>(acc.y());
        r.accelZ = static_cast<float>(acc.z());
        // The hub reports the compass heading of the IMU's x axis: vehicle heading minus the
        // mount's yaw, plus the magnetometer's bias and noise.
        double compass = 90.0 - t.psi / kDeg - mount.yawDeg() + magBiasDeg + 2.0 * n(rng);
        compass = std::fmod(std::fmod(compass, 360.0) + 360.0, 360.0);
        r.headingDeg = static_cast<float>(compass);
        last = r;
        return r;
    }
};

struct DriveResult {
    double posErr = 0, headingErrDeg = 0, speedErr = 0, brakingAccel = 0;
    double maxHeadingErrDeg = 0;  // worst over the run, after the first fixes settle (t > 5 s)
    double maxPosErr = 0;
    double maxSteadySpeedErr = 0;  // worst while not braking (5-25 s)
    int gps = 0, speed = 0, speedInDropout = 0, compass = 0, fixesInDropout = 0;
};

DriveResult drive(Sim& sim, const ImuMount& feederMount, double seconds = 28.0,
                  const FeederConfig* base = nullptr) {
    SensorFusion f;
    FeederConfig cfg = base ? *base : FeederConfig{};
    cfg.mount = feederMount;
    HubReportFeeder feeder(f, cfg);
    DriveResult run;
    const int steps = static_cast<int>(seconds / 0.02);
    for (int k = 0; k < steps; ++k) {
        const double time = k * 0.02;
        if (k > 0) sim.step(time, 0.02);
        const int before = sim.distinctFixes;
        const FeedResult res = feeder.feed(sim.report(k));
        const bool inDropout = time >= sim.obdNanFrom && time < sim.obdNanTo;
        run.gps += res.gps;
        run.speed += res.speed;
        run.compass += res.compass;
        if (inDropout) {
            run.speedInDropout += res.speed;
            run.fixesInDropout += sim.distinctFixes - before;
        }
        if (time > 26.0 && time < 27.0) run.brakingAccel = feeder.longitudinalAccelMps2();
        if (time > 5.0 && f.initialised()) {
            const auto& x = f.state();
            const GeoPoint g = sim.frame.toGeodetic(sim.t.x, sim.t.y);
            const Eigen::Vector2d p = f.toLocal(g.lat, g.lon);
            run.maxPosErr = std::max(run.maxPosErr, std::hypot(x(SensorFusion::PX) - p.x(),
                                                               x(SensorFusion::PY) - p.y()));
            run.maxHeadingErrDeg = std::max(
                run.maxHeadingErrDeg,
                std::abs(SensorFusion::wrapAngle(x(SensorFusion::PSI) - sim.t.psi)) / kDeg);
            if (time < 25.0)
                run.maxSteadySpeedErr =
                    std::max(run.maxSteadySpeedErr, std::abs(x(SensorFusion::V) - sim.t.v));
        }
    }
    const Eigen::Vector2d p = f.toLocal(sim.frame.toGeodetic(sim.t.x, sim.t.y).lat,
                                        sim.frame.toGeodetic(sim.t.x, sim.t.y).lon);
    const auto& x = f.state();
    run.posErr = std::hypot(x(SensorFusion::PX) - p.x(), x(SensorFusion::PY) - p.y());
    run.headingErrDeg = std::abs(SensorFusion::wrapAngle(x(SensorFusion::PSI) - sim.t.psi)) / kDeg;
    run.speedErr = std::abs(x(SensorFusion::V) - sim.t.v);
    return run;
}

}  // namespace

// The whole path: a calibrated feeder keeps the EKF on the truth through a turn and hard braking.
TEST(HubReportFeeder, TracksASimulatedDrive) {
    Sim sim;
    const DriveResult r = drive(sim, sim.mount);
    std::printf(
        "calibrated: max pos %.2f m, max heading %.2f deg, max steady speed %.2f m/s, "
        "speed lag at the end of braking %.2f m/s\n",
        r.maxPosErr, r.maxHeadingErrDeg, r.maxSteadySpeedErr, r.speedErr);
    EXPECT_LT(r.maxPosErr, 2.5);
    EXPECT_LT(r.maxHeadingErrDeg, 3.0);
    EXPECT_LT(r.maxSteadySpeedErr, 0.35);  // includes a 0.3 m/s^2 slope bias on the accelerometer
    // With the measured acceleration as the EKF's control input, speed keeps up through 3 s of
    // -3 m/s^2 braking (without it, it lagged ~1.1 m/s: see the next test).
    EXPECT_LT(r.speedErr, 0.3);
    EXPECT_NEAR(r.brakingAccel, -3.0 + 0.3, 0.2)
        << "averaged longitudinal acceleration (with the slope's bias)";
}

// Why the acceleration input: without it the EKF's speed lags hard braking by about a metre per
// second, because its model assumes constant speed between measurements.
TEST(HubReportFeeder, AccelerationInputRemovesTheBrakingLag) {
    Sim with, without;
    FeederConfig off;
    off.mount = without.mount;
    off.accelInput = false;
    const DriveResult a = drive(with, with.mount);
    const DriveResult b = drive(without, without.mount, 28.0, &off);
    std::printf(
        "speed error at the end of braking: with acceleration input %.2f m/s, without %.2f m/s\n",
        a.speedErr, b.speedErr);
    EXPECT_LT(a.speedErr, 0.3);
    EXPECT_GT(b.speedErr, 0.6);
}

// Each GNSS fix is fed once, however many reports repeat it.
TEST(HubReportFeeder, FeedsEachGnssFixOnce) {
    Sim sim;
    const DriveResult r = drive(sim, sim.mount);
    EXPECT_EQ(r.gps, sim.distinctFixes);
}

// OBD speed at most every 100 ms; during the NaN dropout, only the GNSS speed of each new fix.
TEST(HubReportFeeder, RateLimitsSpeedAndFallsBackToGnssDuringOdbDropout) {
    Sim sim;
    const DriveResult r = drive(sim, sim.mount);
    EXPECT_EQ(r.speedInDropout, r.fixesInDropout) << "NaN OBD must not be fed; GNSS speed instead";
    EXPECT_LE(r.speed, static_cast<int>(28.0 / 0.1) + 1);
    EXPECT_GE(r.speed, static_cast<int>(26.0 / 0.1));
}

TEST(HubReportFeeder, RateLimitsTheCompass) {
    Sim sim;
    const DriveResult r = drive(sim, sim.mount);
    EXPECT_GE(r.compass, 26);
    EXPECT_LE(r.compass, 29);
}

// The point of the calibration: fed with the mount left at identity, the tilted, yawed IMU's
// compass and gyro mislead the EKF badly.
TEST(HubReportFeeder, AnUncalibratedMountMisleadsTheFilter) {
    Sim calibrated, wrong;
    const DriveResult good = drive(calibrated, calibrated.mount);
    const DriveResult bad = drive(wrong, ImuMount{});
    std::printf("max heading error: calibrated %.2f deg, uncalibrated %.2f deg\n",
                good.maxHeadingErrDeg, bad.maxHeadingErrDeg);
    EXPECT_GT(bad.maxHeadingErrDeg, 3 * good.maxHeadingErrDeg)
        << "good " << good.maxHeadingErrDeg << " bad " << bad.maxHeadingErrDeg;
}

// A fix the hub marks invalid (stale or no fix) is never fed, even with a changed position.
TEST(HubReportFeeder, IgnoresInvalidFixes) {
    SensorFusion f;
    HubReportFeeder feeder(f);
    hub_protocol::SensorReport r{};
    r.timestampMs = 1000;
    r.gpsFixValid = 0;
    r.latitude = kOrigin.lat;
    r.longitude = kOrigin.lon;
    r.obdSpeedKph = NAN;
    EXPECT_FALSE(feeder.feed(r).gps);
    EXPECT_FALSE(f.initialised());
    r.timestampMs = 1020;
    r.gpsFixValid = 1;
    EXPECT_TRUE(feeder.feed(r).gps);
    EXPECT_TRUE(f.initialised());
}

// Hub time drives prediction: a reset (time backwards) or a long gap is not predicted across.
TEST(HubReportFeeder, DoesNotPredictAcrossResetsOrLongGaps) {
    SensorFusion f;
    HubReportFeeder feeder(f);
    hub_protocol::SensorReport r{};
    r.gpsFixValid = 1;
    r.latitude = kOrigin.lat;
    r.longitude = kOrigin.lon;
    r.obdSpeedKph = 36.0f;
    r.timestampMs = 50000;
    feeder.feed(r);  // initialises
    r.timestampMs = 50020;
    EXPECT_TRUE(feeder.feed(r).predicted);
    r.timestampMs = 120;  // the hub rebooted
    const FeedResult reset = feeder.feed(r);
    EXPECT_FALSE(reset.predicted);
    EXPECT_LT(reset.dtS, 0.0);
    r.timestampMs = 3120;  // 3 s of lost frames
    EXPECT_FALSE(feeder.feed(r).predicted);
    r.timestampMs = 3140;
    EXPECT_TRUE(feeder.feed(r).predicted);
}

TEST(HubReportFeeder, NonFiniteImuValuesAreNotFed) {
    SensorFusion f;
    HubReportFeeder feeder(f);
    hub_protocol::SensorReport r{};
    r.timestampMs = 1000;
    r.gpsFixValid = 1;
    r.latitude = kOrigin.lat;
    r.longitude = kOrigin.lon;
    r.obdSpeedKph = NAN;
    r.gyroZ = NAN;
    r.headingDeg = NAN;
    r.accelX = NAN;
    const FeedResult res = feeder.feed(r);
    EXPECT_FALSE(res.gyro);
    EXPECT_FALSE(res.compass);
    EXPECT_TRUE(std::isfinite(feeder.longitudinalAccelMps2()));
}

TEST(HubReportFeeder, FillsTheArbitersHubState) {
    hub_protocol::SensorReport r{};
    r.obdBrakePedalActive = 1;
    r.killSwitchEngaged = 0;
    HubState h;
    HubReportFeeder::updateHubState(h, r, 123456);
    EXPECT_TRUE(h.haveReport);
    EXPECT_EQ(h.reportMs, 123456u);
    EXPECT_EQ(h.brakePedalActive, 1);
    EXPECT_EQ(h.killSwitchEngaged, 0);
}
