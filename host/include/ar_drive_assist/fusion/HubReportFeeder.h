#pragma once
// HubReportFeeder — turns the hub's 50 Hz SensorReports into SensorFusion's predict and update
// steps. See docs/BUILD_GUIDE.md Part 8.2 (EKF) and 12.2.2 (IMU mounting).
//
// VehicleInterface (Part 11.1) owns the serial port and decodes frames; for every SensorReport it
// calls feed(). This class is pure: no clock, no I/O, so every rule below is unit-tested with
// simulated reports.
//
// ---------------------------------------------------------------------------------------------
// The SensorReport is a SNAPSHOT, sent every 20 ms, of whatever each sensor last said. Most
// sensors are slower than that, so most fields are repeats. A Kalman filter assumes every update
// is a NEW, independent measurement; feeding a 5 Hz GNSS fix ten times would tell the filter it
// had ten independent fixes and make it ~3x over-confident (sqrt 10) in a position it saw once.
// So each field is fed at its own sensor's rate:
//
//   time      the hub's timestampMs, not the host's arrival time: the hub stamps every report on a
//             fixed 20 ms schedule (vTaskDelayUntil), while USB delivery jitters. dt = difference.
//             A gap longer than maxPredictS (lost frames, hub reset: time going backwards) is not
//             predicted across; the clock is re-based and measurements carry the filter on.
//   predict   once per report, by dt, BEFORE that report's measurements, with the averaged
//             longitudinal acceleration as a control input (SensorFusion::predict(dt, a)).
//   GNSS      only a NEW fix: valid (the hub reports valid only while under 1.5 s old) and a
//             position different from the last one fed. Fixes repeat in the report until the
//             next NMEA sentence; a new one always differs, since the receiver's noise is far
//             above the 1 cm resolution. The first fix sets the local origin (SensorFusion).
//   speed     OBD speed, at most every obdMinPeriodS (the hub polls OBD at ~10 Hz). NaN means
//             no valid reading: not fed. Without OBD, the GNSS ground speed of each new fix.
//   yaw rate  the gyro, every report (the IMU runs at 100 Hz: each report is a fresh sample),
//             rotated into the vehicle frame by the calibrated ImuMount.
//   heading   the magnetometer-based compass, at most every compassPeriodS: inside a car the
//             magnetometer is biased by the body's steel, and frequent updates of a biased
//             measurement would pull the heading towards the bias. GNSS motion corrects heading
//             as soon as the car moves.
//   accel     the longitudinal acceleration (vehicle frame, gravity removed), averaged over the
//             last accelAverage reports (100 ms): the decision arbiter's hard-braking rule
//             (Part 9.3 rule 2) must not fire on a single pothole spike.
// Non-finite values are never fed.
// ---------------------------------------------------------------------------------------------

#include <cstdint>
#include <deque>
#include <optional>

#include "ar_drive_assist/decision/DecisionArbiter.h"
#include "ar_drive_assist/fusion/ImuMount.h"
#include "ar_drive_assist/fusion/SensorFusion.h"
#include "ar_drive_assist/safety/RecklessDrivingDetector.h"
#include "ar_drive_assist/vehicle/HubProtocol.h"

namespace ar_drive_assist {

struct FeederConfig {
    ImuMount mount;               // from vehicle_params.yaml imu_mount_rpy_deg
    double maxPredictS = 1.0;     // longer gaps are not predicted across
    double obdMinPeriodS = 0.1;   // OBD speed at most this often (the hub's poll rate)
    double compassPeriodS = 1.0;  // magnetometer heading at most this often
    bool useCompass = true;
    bool gpsSpeedFallback = true;  // no OBD: use the GNSS ground speed of each new fix
    int accelAverage = 5;          // reports averaged for the longitudinal acceleration
    bool accelInput = true;        // predict with the measured acceleration (SensorFusion)
};

struct FeedResult {
    bool predicted = false;
    double dtS = 0;
    bool gps = false, speed = false, gyro = false, compass = false;
    std::optional<double> gpsNis;  // normalised innovation of the fix, for health monitoring
};

class HubReportFeeder {
public:
    HubReportFeeder(SensorFusion& fusion, FeederConfig cfg = {});

    FeedResult feed(const hub_protocol::SensorReport& r);

    // The car's state for the tracker, detector and arbiter: the EKF pose, speed and yaw rate,
    // plus the averaged longitudinal acceleration from the IMU. timestampMs: the host time to
    // stamp it with.
    EgoState egoState(std::uint64_t timestampMs) const;
    // The hub-side facts the arbiter needs from this report (Part 9.3 'armed'). hostMs: the host
    // time the report arrived (the arbiter compares it with its own clock).
    static void updateHubState(HubState& h, const hub_protocol::SensorReport& r,
                               std::uint64_t hostMs);

    double longitudinalAccelMps2() const { return accelMean_; }
    const FeederConfig& config() const { return cfg_; }

private:
    SensorFusion& f_;
    FeederConfig cfg_;
    bool haveTime_ = false;
    std::uint32_t lastMs_ = 0;
    double hubTimeS_ = 0;  // accumulated hub time, for the rate limits
    bool haveFix_ = false;
    double lastLat_ = 0, lastLon_ = 0;
    double lastSpeedS_ = -1e9, lastCompassS_ = -1e9;
    std::deque<double> accel_;
    double accelMean_ = 0;
};

}  // namespace ar_drive_assist
