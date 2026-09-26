#include "ar_drive_assist/fusion/SensorFusion.h"

#include <cmath>

namespace ar_drive_assist {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kEarthRadius = 6378137.0;  // WGS-84 equatorial, m
// Below this yaw rate, CTRV's v/omega terms are replaced by the straight-line limit. 1e-4 rad/s
// is ~0.006 deg/s: far below anything a car does, far above floating-point trouble.
constexpr double kOmegaEps = 1e-4;

}  // namespace

double SensorFusion::wrapAngle(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0.0) a += 2.0 * kPi;
    return a - kPi;
}

SensorFusion::SensorFusion(const FusionNoise& noise) : noise_(noise) {}

Eigen::Vector2d SensorFusion::toLocal(double latDeg, double lonDeg) const {
    // Equirectangular projection around the origin: east = R cos(lat0) dlon, north = R dlat.
    return {kEarthRadius * cosLat0_ * (lonDeg - lon0_) * kDeg,
            kEarthRadius * (latDeg - lat0_) * kDeg};
}

void SensorFusion::predict(double dt) {
    if (!initialised_ || dt <= 0.0) return;
    const double psi = x_(PSI), v = x_(V), w = x_(OMEGA);
    const double s0 = std::sin(psi), c0 = std::cos(psi);

    Cov F = Cov::Identity();
    if (std::abs(w) > kOmegaEps) {
        const double s1 = std::sin(psi + w * dt), c1 = std::cos(psi + w * dt);
        x_(PX) += v / w * (s1 - s0);
        x_(PY) += v / w * (c0 - c1);
        F(PX, PSI) = v / w * (c1 - c0);
        F(PX, V) = (s1 - s0) / w;
        F(PX, OMEGA) = v / (w * w) * (s0 - s1) + v * dt / w * c1;
        F(PY, PSI) = v / w * (s1 - s0);
        F(PY, V) = (c0 - c1) / w;
        F(PY, OMEGA) = v / (w * w) * (c1 - c0) + v * dt / w * s1;
    } else {
        // Straight-line limit of the same model, with its Jacobian. The OMEGA column is the
        // first-order limit of the curved one, so the two branches join smoothly.
        x_(PX) += v * c0 * dt;
        x_(PY) += v * s0 * dt;
        F(PX, PSI) = -v * s0 * dt;
        F(PX, V) = c0 * dt;
        F(PX, OMEGA) = -0.5 * v * dt * dt * s0;
        F(PY, PSI) = v * c0 * dt;
        F(PY, V) = s0 * dt;
        F(PY, OMEGA) = 0.5 * v * dt * dt * c0;
    }
    x_(PSI) = wrapAngle(psi + w * dt);
    F(PSI, OMEGA) = dt;

    // Process noise from two white-noise inputs, longitudinal acceleration and yaw acceleration,
    // mapped into the state by G (how each input moves each state variable over dt).
    Eigen::Matrix<double, 5, 2> G = Eigen::Matrix<double, 5, 2>::Zero();
    G(PX, 0) = 0.5 * dt * dt * c0;
    G(PY, 0) = 0.5 * dt * dt * s0;
    G(V, 0) = dt;
    G(PSI, 1) = 0.5 * dt * dt;
    G(OMEGA, 1) = dt;
    const Eigen::Vector2d q(noise_.accelStd * noise_.accelStd,
                            noise_.yawAccelStd * noise_.yawAccelStd);
    P_ = F * P_ * F.transpose() + G * q.asDiagonal() * G.transpose();
}

template <int M>
double SensorFusion::update(const Eigen::Matrix<double, M, 1>& z,
                            const Eigen::Matrix<double, M, 5>& H,
                            const Eigen::Matrix<double, M, M>& R, bool angleInnovation) {
    Eigen::Matrix<double, M, 1> y = z - H * x_;
    if (angleInnovation) y(0) = wrapAngle(y(0));
    const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, 5, M> K = P_ * H.transpose() * S.inverse();
    x_ += K * y;
    x_(PSI) = wrapAngle(x_(PSI));
    // Joseph form: algebraically equal to (I - K H) P, but it stays symmetric and positive
    // definite despite rounding over hours of updates. The short form can drift until P is no
    // longer a valid covariance, and then the filter diverges.
    const Cov IKH = Cov::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    return (y.transpose() * S.inverse() * y)(0);
}

std::optional<double> SensorFusion::updateGps(double latDeg, double lonDeg) {
    if (!initialised_) {
        // The first fix defines the local origin and the position. Heading and speed are unknown
        // until other sensors report, so their variances start large.
        lat0_ = latDeg;
        lon0_ = lonDeg;
        cosLat0_ = std::cos(latDeg * kDeg);
        x_.setZero();
        P_ = Cov::Zero();
        P_(PX, PX) = P_(PY, PY) = noise_.gpsPosStd * noise_.gpsPosStd;
        P_(PSI, PSI) = kPi * kPi;
        P_(V, V) = 10.0 * 10.0;
        P_(OMEGA, OMEGA) = 0.5 * 0.5;
        initialised_ = true;
        return std::nullopt;
    }
    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, PX) = 1.0;
    H(1, PY) = 1.0;
    const double r = noise_.gpsPosStd * noise_.gpsPosStd;
    return update<2>(toLocal(latDeg, lonDeg), H, Eigen::Matrix2d::Identity() * r, false);
}

std::optional<double> SensorFusion::updateObdSpeed(double speedMps) {
    if (!initialised_) return std::nullopt;
    Eigen::Matrix<double, 1, 5> H = Eigen::Matrix<double, 1, 5>::Zero();
    H(0, V) = 1.0;
    Eigen::Matrix<double, 1, 1> z, R;
    z << speedMps;
    R << noise_.obdSpeedStd * noise_.obdSpeedStd;
    return update<1>(z, H, R, false);
}

std::optional<double> SensorFusion::updateGyro(double yawRateRadPerS) {
    if (!initialised_) return std::nullopt;
    Eigen::Matrix<double, 1, 5> H = Eigen::Matrix<double, 1, 5>::Zero();
    H(0, OMEGA) = 1.0;
    Eigen::Matrix<double, 1, 1> z, R;
    z << yawRateRadPerS;
    R << noise_.gyroStd * noise_.gyroStd;
    return update<1>(z, H, R, false);
}

std::optional<double> SensorFusion::updateCompassHeading(double compassDeg) {
    if (!initialised_) return std::nullopt;
    Eigen::Matrix<double, 1, 5> H = Eigen::Matrix<double, 1, 5>::Zero();
    H(0, PSI) = 1.0;
    Eigen::Matrix<double, 1, 1> z, R;
    z << wrapAngle((90.0 - compassDeg) * kDeg);  // compass (CW from north) -> psi (CCW from east)
    R << noise_.headingStd * noise_.headingStd;
    return update<1>(z, H, R, true);
}

VehiclePose SensorFusion::currentPose(std::uint64_t timestampMs) const {
    VehiclePose p;
    p.timestamp_ms = timestampMs;
    p.x = x_(PX);
    p.y = x_(PY);
    p.heading_deg = static_cast<float>(x_(PSI) / kDeg);  // CCW from east (see vehicle_pose.fbs)
    p.speed_kph = static_cast<float>(x_(V) * 3.6);
    p.yaw_rate = static_cast<float>(x_(OMEGA));  // rad/s, CCW positive
    return p;
}

template double SensorFusion::update<1>(const Eigen::Matrix<double, 1, 1>&,
                                        const Eigen::Matrix<double, 1, 5>&,
                                        const Eigen::Matrix<double, 1, 1>&, bool);
template double SensorFusion::update<2>(const Eigen::Matrix<double, 2, 1>&,
                                        const Eigen::Matrix<double, 2, 5>&,
                                        const Eigen::Matrix<double, 2, 2>&, bool);

}  // namespace ar_drive_assist
