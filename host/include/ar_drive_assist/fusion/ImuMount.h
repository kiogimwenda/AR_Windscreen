#pragma once
// ImuMount — the IMU's mounting orientation in the vehicle, and its calibration. See
// docs/BUILD_GUIDE.md Part 12.2.2 (added 2026-09-30 with the two-box hub, Part 4.8).
//
// ---------------------------------------------------------------------------------------------
// Why this exists. The BNO085 now sits on the pod board in the windscreen pod, rigidly beside the
// camera, not flat under the dash. The pod follows the windscreen and the camera is pitched down
// to look along the road, so the IMU's axes are NOT the vehicle's axes. Read raw, its z gyro is
// not the car's yaw rate: tilted by 12 deg, the "yaw rate" would be 2 % low and would pick up
// roll rate (sin 12 deg = 21 % of it) every time the body leans in a corner.
//
// Frames. Vehicle frame: x forward, y left, z up (Part 12.2). The mount is
//     R = vehicleFromImu = Rz(yaw) * Ry(pitch) * Rx(roll)
// so a vector measured in IMU axes becomes R * v in vehicle axes. `yaw` is the IMU x axis's
// direction relative to the car's forward axis, counter-clockwise.
//
// Calibration (Part 12.2.2), in two steps, both from data the hub already reports:
//   1. LEVEL (roll, pitch). Parked on level ground, engine running or not, the accelerometer
//      measures only gravity's reaction: +1 g straight up in vehicle axes. In IMU axes that
//      is a = R^T (0, 0, 1), so
//          roll = atan2(a_y, a_z),   pitch = atan2(-a_x, sqrt(a_y^2 + a_z^2)).
//      Averaged over a few seconds; a slope under the car shows up directly as error, hence
//      "level ground" (or average two runs facing opposite ways).
//   2. YAW. Gravity says nothing about yaw. Accelerating or braking in a straight line does: once
//      the IMU reading is levelled, what remains horizontal points along the car's x axis
//      (forwards when speeding up, backwards when braking). The sign of the longitudinal
//      acceleration comes from OBD speed (or GNSS), not from the IMU. Samples are weighted by
//      their horizontal magnitude, and at least `minExcitationG` of it is required.
// Mounting the pod to within a few degrees by eye and then calibrating is the intended workflow;
// the result goes into config/vehicle_params.yaml (imu_mount_rpy_deg). Because the IMU and the
// camera share one rigid carrier, a knocked pod shows up in BOTH: tiltErrorDeg() here, and the
// camera-LiDAR extrinsic score (Part 12.2.1).
//
// Heading. The hub's `headingDeg` is the compass heading of the IMU's x axis. The vehicle's is
// that plus the mount yaw (exact for a yaw-only mount; with the pod's pitch and a few degrees of
// roll the error is second-order, well under the ~3 deg a magnetometer manages inside a car).
// ---------------------------------------------------------------------------------------------

#include <Eigen/Geometry>
#include <optional>
#include <vector>

namespace ar_drive_assist {

class ImuMount {
public:
    ImuMount() = default;  // identity: IMU axes = vehicle axes
    static ImuMount fromRollPitchYawDeg(double rollDeg, double pitchDeg, double yawDeg);

    const Eigen::Matrix3d& vehicleFromImu() const { return r_; }
    double rollDeg() const { return rollDeg_; }
    double pitchDeg() const { return pitchDeg_; }
    double yawDeg() const { return yawDeg_; }

    Eigen::Vector3d toVehicle(const Eigen::Vector3d& vImu) const { return r_ * vImu; }
    // The car's yaw rate (rad/s, counter-clockwise positive) from the hub's gyro (deg/s, IMU axes).
    double yawRateRadPerS(const Eigen::Vector3d& gyroImuDegPerS) const;
    // The car's longitudinal acceleration (m/s^2, forward positive) from the accelerometer (g),
    // with gravity removed.
    double longitudinalAccelMps2(const Eigen::Vector3d& accelImuG) const;
    // The car's compass heading (deg, clockwise from north, [0, 360)) from the IMU's.
    double vehicleCompassDeg(double imuCompassDeg) const;

    // --- Calibration --------------------------------------------------------------------------
    struct AccelSample {
        Eigen::Vector3d accelImuG;         // as the hub reports it
        double longitudinalAccelMps2 = 0;  // from OBD speed or GNSS: only its sign is used
    };
    // Step 1: roll and pitch from the mean accelerometer reading at rest on level ground.
    static std::optional<ImuMount> levelFromRest(const std::vector<Eigen::Vector3d>& accelImuG);
    // Has the pod been knocked? At rest on level ground, the angle (deg) between gravity as this
    // mount predicts it and as measured. A few tenths is noise; above ~1 deg, recalibrate the
    // IMU AND check the camera (they share the pod's carrier plate, so they move together).
    double tiltErrorDeg(const Eigen::Vector3d& meanAccelAtRestG) const;
    // Step 2: yaw, keeping this mount's roll and pitch. nullopt without enough excitation.
    std::optional<ImuMount> withYawFromStraightLine(const std::vector<AccelSample>& samples,
                                                    double minExcitationG = 0.08,
                                                    int minSamples = 25) const;

private:
    Eigen::Matrix3d r_ = Eigen::Matrix3d::Identity();
    double rollDeg_ = 0, pitchDeg_ = 0, yawDeg_ = 0;
};

}  // namespace ar_drive_assist
