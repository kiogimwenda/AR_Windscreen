#pragma once
// Camera — the calibrated camera model shared by everything that maps between the road and the
// image: SceneReconstruction (Part 8.3) and RoadSurfaceProjector (Part 11.4). One model, so the
// LiDAR fusion and the navigation line can never disagree about where a road point appears.
// Header-only.
//
// Frames:
//   camera (optical): x right, y down, z forward (OpenCV).
//   vehicle:          x forward, y left, z up; origin on the ground under the rear axle
//                     (Part 12.2).
// Intrinsics: config/camera_intrinsics.yaml (Part 12.1), pinhole plus OpenCV's k1, k2, p1, p2, k3.
// The model must describe the FRAMES it is used with. CameraPipeline undistorts the frames it
// publishes, so their model is CameraPipeline::frameModel() (zero distortion, a new camera
// matrix), never the raw calibration file: see camera/CameraConfig.h.
// Extrinsics: config/camera_extrinsics.yaml (Part 12.2), the camera's position [x, y, z] and
// [roll, pitch, yaw] in the vehicle frame. Pitch positive = looking DOWN (a rotation about the
// vehicle's left axis tips the forward axis towards the ground).

#include <Eigen/Geometry>
#include <cmath>

namespace ar_drive_assist {

struct CameraModel {
    double fx = 1000, fy = 1000, cx = 960, cy = 540;
    int width = 1920, height = 1080;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;  // OpenCV distortion model

    // Camera-frame point -> raw (distorted) image pixel. False if behind the camera or off-image
    // (px is still filled for a point in front of the camera).
    bool project(const Eigen::Vector3d& pc, Eigen::Vector2d& px) const {
        if (pc.z() <= 0.1) return false;
        const Eigen::Vector2d d = distort({pc.x() / pc.z(), pc.y() / pc.z()});
        px = {fx * d.x() + cx, fy * d.y() + cy};
        return px.x() >= 0 && px.y() >= 0 && px.x() < width && px.y() < height;
    }

    // Raw image pixel -> ray direction in the camera frame, as (x, y, 1). The distortion model
    // has no closed-form inverse; fixed-point iteration converges to well below 0.01 px for the
    // mild distortion of a road camera (tested).
    Eigen::Vector3d unproject(const Eigen::Vector2d& px) const {
        const Eigen::Vector2d target((px.x() - cx) / fx, (px.y() - cy) / fy);
        Eigen::Vector2d u = target;
        for (int i = 0; i < 20; ++i) u += target - distort(u);
        return {u.x(), u.y(), 1.0};
    }

    Eigen::Vector2d distort(const Eigen::Vector2d& n) const {
        const double x = n.x(), y = n.y(), r2 = x * x + y * y;
        const double radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        return {x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y};
    }
};

// cameraFromVehicle from the extrinsics file's values: `t` = camera position in the vehicle frame,
// roll/pitch/yaw in degrees (Z-Y-X order).
inline Eigen::Isometry3d cameraFromVehicle(const Eigen::Vector3d& t, double rollDeg,
                                           double pitchDeg, double yawDeg) {
    const double d = 3.14159265358979323846 / 180;
    const Eigen::Matrix3d vehicleFromBody =
        (Eigen::AngleAxisd(yawDeg * d, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitchDeg * d, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rollDeg * d, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    Eigen::Matrix3d opticalFromBody;  // body x fwd/y left/z up -> optical x right/y down/z fwd
    opticalFromBody << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = opticalFromBody * vehicleFromBody.transpose();
    T.translation() = -T.linear() * t;
    return T;
}

}  // namespace ar_drive_assist
