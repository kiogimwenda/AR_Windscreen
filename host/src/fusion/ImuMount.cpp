#include "ar_drive_assist/fusion/ImuMount.h"

#include <algorithm>
#include <cmath>

namespace ar_drive_assist {

namespace {
constexpr double kDeg = M_PI / 180.0;
constexpr double kG = 9.80665;

double wrap360(double d) {
    d = std::fmod(d, 360.0);
    return d < 0 ? d + 360.0 : d;
}
}  // namespace

ImuMount ImuMount::fromRollPitchYawDeg(double rollDeg, double pitchDeg, double yawDeg) {
    ImuMount m;
    m.rollDeg_ = rollDeg;
    m.pitchDeg_ = pitchDeg;
    m.yawDeg_ = yawDeg;
    m.r_ = (Eigen::AngleAxisd(yawDeg * kDeg, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitchDeg * kDeg, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rollDeg * kDeg, Eigen::Vector3d::UnitX()))
               .toRotationMatrix();
    return m;
}

double ImuMount::yawRateRadPerS(const Eigen::Vector3d& gyroImuDegPerS) const {
    return (r_ * gyroImuDegPerS).z() * kDeg;
}

double ImuMount::longitudinalAccelMps2(const Eigen::Vector3d& accelImuG) const {
    return (r_ * accelImuG).x() * kG;  // gravity is along vehicle z: no x component on the level
}

double ImuMount::vehicleCompassDeg(double imuCompassDeg) const {
    return wrap360(imuCompassDeg + yawDeg_);
}

double ImuMount::tiltErrorDeg(const Eigen::Vector3d& meanAccelAtRestG) const {
    const Eigen::Vector3d up = (r_ * meanAccelAtRestG).normalized();
    return std::acos(std::clamp(up.z(), -1.0, 1.0)) / kDeg;
}

std::optional<ImuMount> ImuMount::levelFromRest(const std::vector<Eigen::Vector3d>& accelImuG) {
    if (accelImuG.empty()) return std::nullopt;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& a : accelImuG) mean += a;
    mean /= static_cast<double>(accelImuG.size());
    const double n = mean.norm();
    // At rest the accelerometer reads 1 g. Far from it: moving, or not an accelerometer reading.
    if (!(n > 0.9 && n < 1.1)) return std::nullopt;
    const double roll = std::atan2(mean.y(), mean.z());
    const double pitch = std::atan2(-mean.x(), std::hypot(mean.y(), mean.z()));
    return fromRollPitchYawDeg(roll / kDeg, pitch / kDeg, 0.0);
}

std::optional<ImuMount> ImuMount::withYawFromStraightLine(const std::vector<AccelSample>& samples,
                                                          double minExcitationG,
                                                          int minSamples) const {
    // Level with roll and pitch only: L = Ry(pitch) Rx(roll) a = Rz(-yaw) a_vehicle. A forward
    // acceleration a_f appears as (a_f cos yaw, -a_f sin yaw) in L's horizontal plane.
    const Eigen::Matrix3d level = (Eigen::AngleAxisd(pitchDeg_ * kDeg, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(rollDeg_ * kDeg, Eigen::Vector3d::UnitX()))
                                      .toRotationMatrix();
    double sx = 0, sy = 0;
    int used = 0;
    for (const auto& s : samples) {
        if (s.longitudinalAccelMps2 == 0) continue;
        const Eigen::Vector3d l = level * s.accelImuG;
        const double h = std::hypot(l.x(), l.y());
        if (h < minExcitationG) continue;
        const double sign = s.longitudinalAccelMps2 > 0 ? 1.0 : -1.0;
        // Weighted by magnitude (summing the vectors does exactly that): strong samples dominate.
        sx += sign * l.x();
        sy += sign * l.y();
        ++used;
    }
    if (used < minSamples) return std::nullopt;
    return fromRollPitchYawDeg(rollDeg_, pitchDeg_, std::atan2(-sy, sx) / kDeg);
}

}  // namespace ar_drive_assist
