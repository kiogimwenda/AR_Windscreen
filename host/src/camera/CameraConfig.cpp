// CameraConfig — see include/ar_drive_assist/camera/CameraConfig.h.
#include "ar_drive_assist/camera/CameraConfig.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <vector>

namespace ar_drive_assist {

CameraSourceKind cameraSourceKind(const std::string& source) {
    if (source.rfind("gst:", 0) == 0) return CameraSourceKind::Pipeline;
    if (source.rfind("/dev/", 0) == 0) return CameraSourceKind::Device;
    return CameraSourceKind::File;
}

std::string cameraGstPipeline(const CameraConfig& cfg) {
    switch (cameraSourceKind(cfg.source)) {
        case CameraSourceKind::Pipeline:
            return cfg.source.substr(4);
        case CameraSourceKind::File:
            throw std::invalid_argument(
                "camera: a file source is opened directly, not by pipeline");
        case CameraSourceKind::Device:
            break;
    }
    const std::string size = "width=" + std::to_string(cfg.width) +
                             ",height=" + std::to_string(cfg.height) +
                             ",framerate=" + std::to_string(cfg.fps) + "/1";
    std::string caps;
    if (cfg.format == "MJPG")
        caps = "image/jpeg," + size + " ! jpegdec";
    else if (cfg.format == "YUYV")
        caps = "video/x-raw,format=YUY2," + size;
    else
        throw std::invalid_argument("camera: format must be MJPG or YUYV, not '" + cfg.format +
                                    "'");
    // appsink drop=true max-buffers=1: if this thread falls behind, GStreamer keeps only the
    // newest frame instead of queueing stale ones; sync=false: hand frames over as they arrive.
    return "v4l2src device=" + cfg.source + " ! " + caps +
           " ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false";
}

namespace {

template <typename T>
void read(const YAML::Node& n, const char* key, T& out, const std::string& path) {
    if (!n[key]) return;
    try {
        out = n[key].as<T>();
    } catch (const YAML::Exception&) {
        throw std::runtime_error(path + ": '" + key + "' is malformed");
    }
}

YAML::Node loadFile(const std::string& path) {
    try {
        return YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(path + ": cannot read (" + e.what() + ")");
    }
}

}  // namespace

CameraConfig loadCameraConfig(const std::string& path) {
    const YAML::Node n = loadFile(path);
    CameraConfig c;
    read(n, "source", c.source, path);
    read(n, "width", c.width, path);
    read(n, "height", c.height, path);
    read(n, "fps", c.fps, path);
    read(n, "format", c.format, path);
    read(n, "intrinsics", c.intrinsicsPath, path);
    read(n, "undistort", c.undistort, path);
    read(n, "loop", c.loop, path);
    read(n, "realtime", c.realtime, path);
    read(n, "stall_timeout_s", c.stallTimeoutS, path);
    read(n, "report_every_s", c.reportEveryS, path);
    if (c.width <= 0 || c.height <= 0 || c.fps <= 0)
        throw std::runtime_error(path + ": width, height and fps must be positive");
    if (c.format != "MJPG" && c.format != "YUYV")
        throw std::runtime_error(path + ": 'format' must be MJPG or YUYV");
    if (c.stallTimeoutS <= 0 || c.reportEveryS <= 0)
        throw std::runtime_error(path + ": stall_timeout_s and report_every_s must be positive");
    return c;
}

CameraIntrinsics loadCameraIntrinsics(const std::string& path) {
    const YAML::Node n = loadFile(path);
    CameraIntrinsics in;
    auto list = [&](const char* key) {
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
    const std::vector<double> K = list("camera_matrix");
    const std::vector<double> D = list("distortion_coefficients");
    if (K.empty()) return in;  // the template: not calibrated yet
    if (K.size() != 9) throw std::runtime_error(path + ": camera_matrix needs 9 numbers");
    if (D.size() != 5)
        throw std::runtime_error(path +
                                 ": distortion_coefficients needs 5 numbers (k1 k2 p1 p2 k3)");
    CameraModel& m = in.model;
    read(n, "image_width", m.width, path);
    read(n, "image_height", m.height, path);
    m.fx = K[0];
    m.cx = K[2];
    m.fy = K[4];
    m.cy = K[5];
    m.k1 = D[0];
    m.k2 = D[1];
    m.p1 = D[2];
    m.p2 = D[3];
    m.k3 = D[4];
    if (m.fx <= 0 || m.fy <= 0) throw std::runtime_error(path + ": focal lengths must be positive");
    if (m.cx <= 0 || m.cx >= m.width || m.cy <= 0 || m.cy >= m.height)
        throw std::runtime_error(path + ": principal point lies outside the image");
    read(n, "reprojection_error_px", in.reprojectionErrorPx, path);
    in.calibrated = true;
    return in;
}

}  // namespace ar_drive_assist
