// Recording — see include/ar_drive_assist/system/Recording.h.
#include "ar_drive_assist/system/Recording.h"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "ar_drive_assist/common/Camera.h"

namespace ar_drive_assist {

namespace {

Eigen::Isometry3d matrix(const YAML::Node& n, const char* key, const std::string& path) {
    std::vector<double> v;
    try {
        v = n[key].as<std::vector<double>>();
    } catch (const YAML::Exception&) {
        throw std::runtime_error(path + ": '" + key + "' missing or not a list of numbers");
    }
    if (v.size() != 16) throw std::runtime_error(path + ": '" + key + "' needs 16 numbers");
    Eigen::Matrix4d m;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) m(r, c) = v[r * 4 + c];
    const Eigen::Matrix3d R = m.topLeftCorner<3, 3>();
    if ((R * R.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-3 || R.determinant() < 0.99)
        throw std::runtime_error(path + ": '" + key + "' is not a rigid transform");
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = R;
    t.translation() = m.topRightCorner<3, 1>();
    return t;
}

std::vector<std::vector<std::string>> csv(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error(path + ": cannot read");
    std::vector<std::vector<std::string>> rows;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) f.push_back(cell);
        rows.push_back(f);
    }
    return rows;
}

}  // namespace

std::string Recording::scanFile(std::size_t i) const {
    char b[32];
    std::snprintf(b, sizeof b, "/lidar/%06zu.bin", i);
    return dir + b;
}

Recording loadRecording(const std::string& dir) {
    Recording r;
    r.dir = dir;
    const std::string meta = dir + "/recording.yaml", ext = dir + "/extrinsics.yaml";
    YAML::Node m, e;
    try {
        m = YAML::LoadFile(meta);
        e = YAML::LoadFile(ext);
    } catch (const YAML::Exception& x) {
        throw std::runtime_error(dir + ": not a recording (" + x.what() + ")");
    }
    r.name = m["name"].as<std::string>("");
    r.source = m["source"].as<std::string>("");
    r.licence = m["licence"].as<std::string>("");
    r.width = m["width"].as<int>(0);
    r.height = m["height"].as<int>(0);
    if (r.width <= 0 || r.height <= 0) throw std::runtime_error(meta + ": width and height needed");
    r.cameraFromLidar = matrix(e, "camera_from_lidar", ext);
    r.vehicleFromLidar = matrix(e, "vehicle_from_lidar", ext);
    r.cameraFromVehicle = matrix(e, "camera_from_vehicle", ext);
    // The three must agree: camera_from_lidar = camera_from_vehicle * vehicle_from_lidar.
    const Eigen::Isometry3d chain = r.cameraFromVehicle * r.vehicleFromLidar;
    if ((chain.matrix() - r.cameraFromLidar.matrix()).norm() > 1e-3)
        throw std::runtime_error(ext + ": the three transforms disagree");
    return r;
}

std::vector<ScanTime> loadScanTimes(const std::string& dir) {
    std::vector<ScanTime> out;
    for (const auto& f : csv(dir + "/lidar.csv")) {
        if (f.size() < 2) continue;
        out.push_back({static_cast<std::size_t>(std::stoul(f[0])), std::stoll(f[1])});
    }
    if (out.empty()) throw std::runtime_error(dir + "/lidar.csv: no scans");
    return out;
}

std::vector<Eigen::Vector4f> readScan(const std::string& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) throw std::runtime_error(path + ": cannot read");
    const std::streamsize bytes = in.tellg();
    if (bytes % 16 != 0) throw std::runtime_error(path + ": not whole float32 x,y,z,i points");
    std::vector<Eigen::Vector4f> pts(static_cast<std::size_t>(bytes / 16));
    in.seekg(0);
    in.read(reinterpret_cast<char*>(pts.data()), bytes);
    return pts;
}

SensorExtrinsics loadConfigExtrinsics(const std::string& configDir, double cameraHeightM) {
    SensorExtrinsics x;
    auto list = [](const YAML::Node& n, const char* key, const std::string& path) {
        std::vector<double> v;
        if (n[key] && !n[key].IsNull()) {
            try {
                v = n[key].as<std::vector<double>>();
            } catch (const YAML::Exception&) {
                throw std::runtime_error(path + ": '" + key + "' must be a list of numbers");
            }
        }
        return v;
    };
    const std::string camPath = configDir + "/camera_extrinsics.yaml";
    const YAML::Node cam = YAML::LoadFile(camPath);
    const std::vector<double> t = list(cam, "translation_m", camPath);
    const std::vector<double> rpy = list(cam, "rotation_deg", camPath);
    if (t.size() == 3 && rpy.size() == 3) {
        x.cameraFromVehicle = cameraFromVehicle({t[0], t[1], t[2]}, rpy[0], rpy[1], rpy[2]);
        x.cameraCalibrated = true;
    } else if (!t.empty() || !rpy.empty()) {
        throw std::runtime_error(camPath + ": translation_m and rotation_deg need 3 numbers each");
    } else {
        x.cameraFromVehicle = cameraFromVehicle({0, 0, cameraHeightM}, 0, 0, 0);
    }
    const std::string lidPath = configDir + "/lidar_camera_extrinsics.yaml";
    const YAML::Node lid = YAML::LoadFile(lidPath);
    const std::vector<double> R = list(lid, "rotation_matrix", lidPath);
    const std::vector<double> T = list(lid, "translation_m", lidPath);
    if (R.size() == 9 && T.size() == 3) {
        Eigen::Matrix3d m;
        for (int i = 0; i < 9; ++i) m(i / 3, i % 3) = R[i];
        if ((m * m.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-3)
            throw std::runtime_error(lidPath + ": rotation_matrix is not a rotation");
        x.cameraFromLidar.linear() = m;
        x.cameraFromLidar.translation() = Eigen::Vector3d(T[0], T[1], T[2]);
        x.lidarCalibrated = true;
    } else if (!R.empty() || !T.empty()) {
        throw std::runtime_error(lidPath + ": rotation_matrix needs 9 and translation_m 3 numbers");
    } else {
        // No calibration: the LiDAR at the camera, with the vehicle's axes (x forward, y left,
        // z up), which in optical axes is the fixed axis swap.
        x.cameraFromLidar = x.cameraFromVehicle * Eigen::Translation3d(0, 0, cameraHeightM);
    }
    x.vehicleFromLidar = x.cameraFromVehicle.inverse() * x.cameraFromLidar;
    return x;
}

}  // namespace ar_drive_assist
