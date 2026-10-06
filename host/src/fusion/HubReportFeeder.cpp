#include "ar_drive_assist/fusion/HubReportFeeder.h"

#include <cmath>

namespace ar_drive_assist {

namespace {
constexpr double kKphToMps = 1.0 / 3.6;
bool finite3(double a, double b, double c) {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}
}  // namespace

HubReportFeeder::HubReportFeeder(SensorFusion& fusion, FeederConfig cfg)
    : f_(fusion), cfg_(std::move(cfg)) {}

FeedResult HubReportFeeder::feed(const hub_protocol::SensorReport& r) {
    // SensorReport is packed: every field is copied to a local before use (an unaligned
    // reference into a packed struct is undefined behaviour; HubProtocol.h explains).
    const std::uint32_t ms = r.timestampMs;
    const std::uint8_t fixValid = r.gpsFixValid;
    const double lat = r.latitude, lon = r.longitude, gpsKph = r.speedKph;
    const double obdKph = r.obdSpeedKph;
    const Eigen::Vector3d gyro(r.gyroX, r.gyroY, r.gyroZ);
    const Eigen::Vector3d accel(r.accelX, r.accelY, r.accelZ);
    const double heading = r.headingDeg;

    FeedResult out;
    // --- Longitudinal acceleration, averaged (first: the prediction uses it) ----------------
    if (finite3(accel.x(), accel.y(), accel.z())) {
        accel_.push_back(cfg_.mount.longitudinalAccelMps2(accel));
        while (static_cast<int>(accel_.size()) > std::max(1, cfg_.accelAverage)) accel_.pop_front();
        double sum = 0;
        for (double a : accel_) sum += a;
        accelMean_ = sum / static_cast<double>(accel_.size());
    }

    // --- Time and prediction --------------------------------------------------------------
    if (haveTime_) {
        // Signed difference of the hub's 32-bit millisecond clock: a hub reset reads negative.
        const auto diff = static_cast<std::int32_t>(ms - lastMs_);
        out.dtS = diff / 1000.0;
        if (out.dtS > 0.0 && out.dtS <= cfg_.maxPredictS) {
            if (cfg_.accelInput && !accel_.empty())
                f_.predict(out.dtS, accelMean_);
            else
                f_.predict(out.dtS);
            out.predicted = f_.initialised();
            hubTimeS_ += out.dtS;
        } else if (out.dtS > cfg_.maxPredictS) {
            hubTimeS_ += out.dtS;  // a long gap: re-based, not predicted across
        }
    }
    haveTime_ = true;
    lastMs_ = ms;

    // --- GNSS: new fixes only -------------------------------------------------------------
    bool newFix = false;
    if (fixValid && std::isfinite(lat) && std::isfinite(lon) &&
        (!haveFix_ || lat != lastLat_ || lon != lastLon_)) {
        out.gpsNis = f_.updateGps(lat, lon);
        out.gps = newFix = true;
        haveFix_ = true;
        lastLat_ = lat;
        lastLon_ = lon;
    }

    // --- Speed ----------------------------------------------------------------------------
    if (std::isfinite(obdKph) && obdKph >= 0.0) {
        if (hubTimeS_ - lastSpeedS_ >= cfg_.obdMinPeriodS - 1e-9) {
            out.speed = f_.updateObdSpeed(obdKph * kKphToMps).has_value();
            lastSpeedS_ = hubTimeS_;
        }
    } else if (cfg_.gpsSpeedFallback && newFix && std::isfinite(gpsKph) && gpsKph >= 0.0) {
        out.speed = f_.updateObdSpeed(gpsKph * kKphToMps).has_value();
    }

    // --- Yaw rate and heading, in the vehicle frame -----------------------------------------
    if (finite3(gyro.x(), gyro.y(), gyro.z()))
        out.gyro = f_.updateGyro(cfg_.mount.yawRateRadPerS(gyro)).has_value();
    if (cfg_.useCompass && std::isfinite(heading) &&
        hubTimeS_ - lastCompassS_ >= cfg_.compassPeriodS - 1e-9) {
        out.compass = f_.updateCompassHeading(cfg_.mount.vehicleCompassDeg(heading)).has_value();
        if (out.compass) lastCompassS_ = hubTimeS_;
    }

    return out;
}

EgoState HubReportFeeder::egoState(std::uint64_t timestampMs) const {
    EgoState e;
    const auto& x = f_.state();
    e.pose.x = x(SensorFusion::PX);
    e.pose.y = x(SensorFusion::PY);
    e.pose.psi = x(SensorFusion::PSI);
    e.speedMps = x(SensorFusion::V);
    e.yawRateRadPerS = x(SensorFusion::OMEGA);
    e.longitudinalAccelMps2 = accelMean_;
    e.timestampMs = timestampMs;
    return e;
}

void HubReportFeeder::updateHubState(HubState& h, const hub_protocol::SensorReport& r,
                                     std::uint64_t hostMs) {
    h.haveReport = true;
    h.reportMs = hostMs;
    h.brakePedalActive = r.obdBrakePedalActive;
    h.killSwitchEngaged = r.killSwitchEngaged;
}

}  // namespace ar_drive_assist
