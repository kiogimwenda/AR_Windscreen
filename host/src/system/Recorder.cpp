// Recorder — see include/ar_drive_assist/system/Recorder.h.
#include "ar_drive_assist/system/Recorder.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <thread>

#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {
namespace {

void writeMatrix(std::ostream& o, const char* key, const Eigen::Isometry3d& t) {
    o << key << ": [";
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) {
            char b[32];
            std::snprintf(b, sizeof b, "%.9g", t.matrix()(r, c));
            o << b << (r == 3 && c == 3 ? "" : ", ");
        }
    o << "]\n";
}

std::string quoted(const std::string& s) {
    std::string q = "\"";
    for (char c : s) q += c == '"' ? std::string("\\\"") : std::string(1, c);
    return q + "\"";
}

}  // namespace

Recorder::Recorder(RecorderConfig cfg, EventLog* log) : cfg_(std::move(cfg)), log_(log) {
    namespace fs = std::filesystem;
    if (fs::exists(cfg_.dir + "/recording.yaml"))
        throw std::runtime_error("record: " + cfg_.dir + " already holds a recording");
    fs::create_directories(cfg_.dir + "/lidar");
    const CameraModel& m = cfg_.frameModel;
    video_ = std::make_unique<cv::VideoWriter>(cfg_.dir + "/camera.avi", cv::CAP_OPENCV_MJPEG,
                                               cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                               cfg_.fps, cv::Size(m.width, m.height));
    if (!video_->isOpened())
        throw std::runtime_error("record: cannot write " + cfg_.dir + "/camera.avi");
    video_->set(cv::VIDEOWRITER_PROP_QUALITY, 95);
    cameraCsv_.open(cfg_.dir + "/camera.csv");
    lidarCsv_.open(cfg_.dir + "/lidar.csv");
    hubCsv_.open(cfg_.dir + "/hub.csv");
    if (!cameraCsv_ || !lidarCsv_ || !hubCsv_)
        throw std::runtime_error("record: cannot write in " + cfg_.dir);
    cameraCsv_ << "frame,t_us\n";
    lidarCsv_ << "scan,t_us\n";
    hubCsv_ << "t_us,lat,lon,speed_kph,gps_fix_valid,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,"
               "gyro_y_dps,gyro_z_dps,heading_deg,obd_speed_kph\n";

    std::ofstream in(cfg_.dir + "/camera_intrinsics.yaml");
    char b[512];
    std::snprintf(b, sizeof b,
                  "# The recorded frames' camera model (CameraPipeline::frameModel()).\n"
                  "image_width: %d\nimage_height: %d\n"
                  "camera_matrix: [%.6f, 0, %.6f, 0, %.6f, %.6f, 0, 0, 1]\n"
                  "distortion_coefficients: [%.8f, %.8f, %.8f, %.8f, %.8f]\n"
                  "reprojection_error_px: 0.0\n",
                  m.width, m.height, m.fx, m.cx, m.fy, m.cy, m.k1, m.k2, m.p1, m.p2, m.k3);
    in << b;
    std::ofstream ex(cfg_.dir + "/extrinsics.yaml");
    ex << "# The extrinsics the system ran with (Part 12.2"
       << (cfg_.extrinsics.cameraCalibrated && cfg_.extrinsics.lidarCalibrated
               ? ")"
               : "; NOT calibrated: placeholders)")
       << ". 4x4 row-major, metres.\n";
    writeMatrix(ex, "camera_from_lidar", cfg_.extrinsics.cameraFromLidar);
    writeMatrix(ex, "vehicle_from_lidar", cfg_.extrinsics.vehicleFromLidar);
    writeMatrix(ex, "camera_from_vehicle", cfg_.extrinsics.cameraFromVehicle);
    if (log_) log_->logGeneral("record: writing to " + cfg_.dir);
}

Recorder::~Recorder() {
    finish();
}

void Recorder::onFrame(const CameraFrame& f) {
    std::lock_guard<std::mutex> lock(m_);
    if (finished_) return;
    if (frames_.size() >= cfg_.maxQueuedFrames) {
        ++stats_.droppedFrames;
        return;
    }
    frames_.push_back(f);  // shares the pixels; the camera allocates a new Mat per frame
}

void Recorder::onScan(const LidarFrame& s) {
    std::lock_guard<std::mutex> lock(m_);
    if (finished_) return;
    if (scans_.size() >= cfg_.maxQueuedScans) {
        ++stats_.droppedScans;
        return;
    }
    scans_.push_back(s);
}

void Recorder::onReport(const hub_protocol::SensorReport& r, std::uint64_t hostMs) {
    // Small: written straight from the caller's thread (the hub link's), under the lock.
    char b[320];
    char obd[32] = "";
    if (std::isfinite(r.obdSpeedKph)) std::snprintf(obd, sizeof obd, "%.3f", r.obdSpeedKph);
    std::snprintf(b, sizeof b, "%llu,%.9f,%.9f,%.3f,%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%s\n",
                  static_cast<unsigned long long>(hostMs) * 1000ULL, r.latitude, r.longitude,
                  r.speedKph, r.gpsFixValid, r.accelX, r.accelY, r.accelZ, r.gyroX, r.gyroY,
                  r.gyroZ, r.headingDeg, obd);
    std::lock_guard<std::mutex> lock(m_);
    if (finished_) return;
    hubCsv_ << b;
    ++stats_.reports;
}

bool Recorder::writeOne() {
    CameraFrame f;
    LidarFrame s;
    bool haveFrame = false, haveScan = false;
    {
        std::lock_guard<std::mutex> lock(m_);
        if (!frames_.empty()) {
            f = std::move(frames_.front());
            frames_.pop_front();
            haveFrame = true;
        }
        if (!scans_.empty()) {
            s = std::move(scans_.front());
            scans_.pop_front();
            haveScan = true;
        }
    }
    if (haveFrame) {
        video_->write(f.bgr);
        std::lock_guard<std::mutex> lock(m_);
        cameraCsv_ << stats_.frames << ',' << f.timestampMs * 1000ULL << '\n';
        ++stats_.frames;
    }
    if (haveScan) {
        std::uint64_t index;
        {
            std::lock_guard<std::mutex> lock(m_);
            index = stats_.scans;
        }
        char name[32];
        std::snprintf(name, sizeof name, "/lidar/%06llu.bin",
                      static_cast<unsigned long long>(index));
        std::ofstream out(cfg_.dir + name, std::ios::binary);
        std::vector<float> buf;
        buf.reserve(s.points.size() * 4);
        for (const LidarPoint& p : s.points) {
            buf.push_back(static_cast<float>(p.p.x()));
            buf.push_back(static_cast<float>(p.p.y()));
            buf.push_back(static_cast<float>(p.p.z()));
            buf.push_back(p.intensity);
        }
        out.write(reinterpret_cast<const char*>(buf.data()),
                  static_cast<std::streamsize>(buf.size() * sizeof(float)));
        std::lock_guard<std::mutex> lock(m_);
        lidarCsv_ << index << ',' << s.timestampMs * 1000ULL << '\n';
        ++stats_.scans;
    }
    return haveFrame || haveScan;
}

void Recorder::run(const std::atomic<bool>& stop) {
    auto lastReport = std::chrono::steady_clock::now();
    while (!stop.load()) {
        if (!writeOne()) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (log_ && std::chrono::steady_clock::now() - lastReport >
                        std::chrono::duration<double>(cfg_.reportEveryS)) {
            const Stats s = stats();
            log_->logGeneral("record: " + std::to_string(s.frames) + " frames, " +
                             std::to_string(s.scans) + " scans, " + std::to_string(s.reports) +
                             " hub reports; dropped " + std::to_string(s.droppedFrames) +
                             " frames, " + std::to_string(s.droppedScans) + " scans");
            lastReport = std::chrono::steady_clock::now();
        }
    }
    while (writeOne()) {
    }
    finish();
}

Recorder::Stats Recorder::stats() const {
    std::lock_guard<std::mutex> lock(m_);
    return stats_;
}

void Recorder::finish() {
    Stats s;
    {
        std::lock_guard<std::mutex> lock(m_);
        if (finished_) return;
        finished_ = true;
        s = stats_;
        cameraCsv_.close();
        lidarCsv_.close();
        hubCsv_.close();
    }
    video_->release();
    // recording.yaml last: its presence marks a complete recording.
    std::ofstream y(cfg_.dir + "/recording.yaml");
    y << "name: "
      << quoted(cfg_.name.empty() ? std::filesystem::path(cfg_.dir).filename().string() : cfg_.name)
      << "\nsource: \"AR_Windscreen host, ar_drive_assist --record\"\n"
      << "licence: \"Project data\"\n"
      << "width: " << cfg_.frameModel.width << "\nheight: " << cfg_.frameModel.height << '\n'
      << "notes: "
      << quoted(cfg_.notes + (cfg_.notes.empty() ? "" : " ") + std::to_string(s.frames) +
                " frames, " + std::to_string(s.scans) + " scans, " + std::to_string(s.reports) +
                " hub reports; dropped " + std::to_string(s.droppedFrames) + " frames, " +
                std::to_string(s.droppedScans) + " scans.")
      << '\n';
    if (log_)
        log_->logGeneral("record: finished " + cfg_.dir + ": " + std::to_string(s.frames) +
                         " frames, " + std::to_string(s.scans) + " scans, " +
                         std::to_string(s.reports) + " hub reports; dropped " +
                         std::to_string(s.droppedFrames) + " frames, " +
                         std::to_string(s.droppedScans) + " scans");
}

}  // namespace ar_drive_assist
