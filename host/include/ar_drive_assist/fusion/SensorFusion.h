#pragma once
// SensorFusion — the ego-vehicle state estimator: a 5-state extended Kalman filter (EKF). See
// docs/BUILD_GUIDE.md Part 8.2.
//
// ---------------------------------------------------------------------------------------------
// What a Kalman filter does
//
// The filter keeps two things:
//   x — its best estimate of the state:  [px, py, psi, v, omega]
//       position (m, local east/north), heading (rad), speed (m/s), yaw rate (rad/s)
//   P — the 5x5 covariance of that estimate: how uncertain each quantity is, and how the
//       uncertainties are correlated.
//
// It alternates two steps:
//
//   PREDICT (every IMU tick, 100 Hz): move the state forward by dt using a motion model f(x),
//     and GROW the uncertainty: P = F P F^T + Q. F is the Jacobian of f (how a small change in
//     each state variable changes the prediction). Q is the process noise: how much the real car
//     can deviate from the model in dt (it accelerates, and the turn rate changes).
//
//   UPDATE (when a sensor reports): compare the measurement z with what the state predicts it
//     should be, h(x). The difference is the INNOVATION, y = z - h(x). Its expected spread is
//     S = H P H^T + R, where R is the sensor's own noise. The Kalman gain K = P H^T S^-1 then
//     decides how far to move the estimate: x += K y. The more certain the state is relative to
//     the sensor, the less a measurement moves it. Finally the uncertainty SHRINKS:
//     P = (I - K H) P.
//
// "Extended" means f and h may be non-linear; F and H are their Jacobians at the current
// estimate. Here the motion model is non-linear (heading turns the velocity), and the
// measurements are all linear.
//
// ---------------------------------------------------------------------------------------------
// Motion model: constant turn rate and velocity (CTRV)
//
//   px' = px + v/omega * ( sin(psi + omega dt) - sin(psi) )
//   py' = py + v/omega * ( cos(psi) - cos(psi + omega dt) )
//   psi' = psi + omega dt,   v' = v,   omega' = omega
//
// As omega -> 0 this becomes 0/0. Below a small threshold, the straight-line limit
// (px' = px + v cos(psi) dt, ...) is used instead, with its own Jacobian.
//
// ---------------------------------------------------------------------------------------------
// Conventions (each is a classic source of silent bugs)
//   - Position is metres in a local east/north plane around the FIRST GPS fix. Over a few
//     kilometres the flat-plane approximation error is centimetres.
//   - psi is measured counter-clockwise from EAST (the maths convention). A compass heading
//     (clockwise from NORTH, what the BNO085 and GPS report) converts as psi = 90deg - compass.
//     Get this wrong and every turn is mirrored.
//   - Angles wrap. An innovation of 359 deg is really -1 deg. Every angle innovation, and psi
//     itself, is wrapped to (-pi, pi], or the filter swings the wrong way round the circle.
//
// Sensors (Part 4.4, via SensorReport at 50 Hz): IMU yaw rate and absolute heading, GPS
// position, OBD speed. LiDAR odometry is deliberately NOT used in v1 (Part 8.2).
// ---------------------------------------------------------------------------------------------

#include <Eigen/Dense>
#include <cstdint>
#include <optional>

#include "ar_drive_assist/common/Types.h"

namespace ar_drive_assist {

struct FusionNoise {
    // Process noise: how far the real car can depart from constant turn-rate-and-velocity.
    double accelStd = 2.0;     // m/s^2, longitudinal acceleration (a hard stop is ~7)
    double yawAccelStd = 0.5;  // rad/s^2, change of yaw rate
    // Measurement noise (1 sigma). Tuned in Part 12 / Phase 15 against real sensors.
    double gpsPosStd = 2.5;    // m, NEO-M8N open sky
    double obdSpeedStd = 0.3;  // m/s
    double gyroStd = 0.02;     // rad/s, BNO085 fused yaw rate
    double headingStd = 0.05;  // rad (~3 deg); magnetometers are disturbed inside cars
};

class SensorFusion {
public:
    using State = Eigen::Matrix<double, 5, 1>;
    using Cov = Eigen::Matrix<double, 5, 5>;
    enum Index { PX = 0, PY = 1, PSI = 2, V = 3, OMEGA = 4 };

    explicit SensorFusion(const FusionNoise& noise = {});

    // Prediction step (Part 8.2: IMU-driven, 100 Hz). Does nothing until the first GPS fix has
    // set the origin and position.
    void predict(double dt);

    // Updates. Each returns the normalised innovation squared (NIS, y^T S^-1 y) so tests and
    // monitoring can check the filter's consistency, or nullopt if the filter is not initialised.
    std::optional<double> updateGps(double latDeg, double lonDeg);
    std::optional<double> updateObdSpeed(double speedMps);
    std::optional<double> updateGyro(double yawRateRadPerS);  // counter-clockwise positive
    std::optional<double> updateCompassHeading(double compassDeg);

    bool initialised() const { return initialised_; }
    const State& state() const { return x_; }
    const Cov& covariance() const { return P_; }
    VehiclePose currentPose(std::uint64_t timestampMs) const;

    // Local east/north metres <-> latitude/longitude, around the first fix.
    Eigen::Vector2d toLocal(double latDeg, double lonDeg) const;

    static double wrapAngle(double a);  // to (-pi, pi]

private:
    template <int M>
    double update(const Eigen::Matrix<double, M, 1>& z, const Eigen::Matrix<double, M, 5>& H,
                  const Eigen::Matrix<double, M, M>& R, bool angleInnovation);

    FusionNoise noise_;
    State x_ = State::Zero();
    Cov P_ = Cov::Identity();
    bool initialised_ = false;
    double lat0_ = 0.0, lon0_ = 0.0, cosLat0_ = 1.0;
};

}  // namespace ar_drive_assist
