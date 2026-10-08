#pragma once
// Recording — a directory of synchronised sensor data the whole host can be run on, offline
// (Part 13.2's replay). The project's own drives will be recorded in this format; public datasets
// are converted into it (tools/bench_rig/kitti_to_recording.py).
//
//   recording.yaml           name, source, licence, notes
//   camera.avi               the camera's frames (any video OpenCV reads)
//   camera.csv               frame,t_us            capture time of each frame
//   camera_intrinsics.yaml   the format of host/config/camera_intrinsics.yaml
//   extrinsics.yaml          camera_from_lidar, vehicle_from_lidar, camera_from_vehicle:
//                            4x4 row-major (16 numbers each), metres
//   lidar/NNNNNN.bin         one scan: float32 x, y, z, intensity per point, LiDAR frame
//   lidar.csv                scan,t_us             time of each scan
//   hub.csv                  t_us,lat,lon,speed_kph,gps_fix_valid,accel_x_g,accel_y_g,accel_z_g,
//                            gyro_x_dps,gyro_y_dps,gyro_z_dps,heading_deg,obd_speed_kph
//                            the hub's SensorReport fields (hub_sim --replay plays them;
//                            obd_speed_kph empty = not valid)
//
// Times are microseconds on one clock (any epoch); a replay keeps their spacing.
// Vehicle frame (Part 12.2): x forward, y left, z up, origin on the ground under the rear axle.

#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

namespace ar_drive_assist {

struct Recording {
    std::string dir;
    std::string name, source, licence;
    int width = 0, height = 0;  // camera frame size
    Eigen::Isometry3d cameraFromLidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d vehicleFromLidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d cameraFromVehicle = Eigen::Isometry3d::Identity();

    std::string video() const { return dir + "/camera.avi"; }
    std::string frameTimes() const { return dir + "/camera.csv"; }
    std::string intrinsics() const { return dir + "/camera_intrinsics.yaml"; }
    std::string hubReports() const { return dir + "/hub.csv"; }
    std::string scanFile(std::size_t i) const;
};

// Throws std::runtime_error naming the file and what is wrong.
Recording loadRecording(const std::string& dir);

struct ScanTime {
    std::size_t index = 0;
    std::int64_t tUs = 0;
};
std::vector<ScanTime> loadScanTimes(const std::string& dir);

// float32 x, y, z, intensity per point.
std::vector<Eigen::Vector4f> readScan(const std::string& path);

// The live system's extrinsics, from host/config/camera_extrinsics.yaml (Part 12.2: position and
// roll/pitch/yaw in the vehicle frame) and lidar_camera_extrinsics.yaml (rotation, translation).
// The files are templates until Part 12.2 is done: then the camera is assumed `cameraHeightM`
// above the ground under the rear axle looking straight ahead, the LiDAR at the camera, and the
// `calibrated` flags are false (log it: geometry is not to be trusted). Throws on malformed values.
struct SensorExtrinsics {
    Eigen::Isometry3d cameraFromVehicle = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d cameraFromLidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d vehicleFromLidar = Eigen::Isometry3d::Identity();
    bool cameraCalibrated = false, lidarCalibrated = false;
};
SensorExtrinsics loadConfigExtrinsics(const std::string& configDir, double cameraHeightM);

}  // namespace ar_drive_assist
