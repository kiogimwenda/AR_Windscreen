// InjectedObstacle — see include/ar_drive_assist/lidar/InjectedObstacle.h.
#include "ar_drive_assist/lidar/InjectedObstacle.h"

#include <cmath>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>

namespace ar_drive_assist {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthR = 6378137.0;
double wrap(double a) {
    while (a > kPi) a -= 2 * kPi;
    while (a <= -kPi) a += 2 * kPi;
    return a;
}
}  // namespace

ObstacleModel::ObstacleModel(InjectedObstacle o, const std::string& hubCsv) : o_(o) {
    std::ifstream in(hubCsv);
    if (!in) throw std::runtime_error(hubCsv + ": cannot read");
    std::string line;
    std::getline(in, line);  // header: t_us,lat,lon,speed_kph,gps_fix_valid,...,heading_deg (11)
    double lat0 = 0, lon0 = 0, t0 = 0;
    while (std::getline(in, line)) {
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string c;
        while (std::getline(ss, c, ',')) f.push_back(c);
        if (f.size() < 12) continue;
        const double t = std::stod(f[0]) * 1e-6, lat = std::stod(f[1]), lon = std::stod(f[2]);
        if (track_.empty()) {
            lat0 = lat;
            lon0 = lon;
            t0 = t;
        }
        Pose p;
        p.tS = t - t0;
        p.e = (lon - lon0) * kPi / 180 * kEarthR * std::cos(lat0 * kPi / 180);
        p.n = (lat - lat0) * kPi / 180 * kEarthR;
        p.psi = (90.0 - std::stod(f[11])) * kPi / 180;  // compass -> CCW from east
        if (!track_.empty()) {
            const Pose& b = track_.back();
            p.psi = b.psi + wrap(p.psi - b.psi);  // unwrapped, for interpolation
            p.s = b.s + std::hypot(p.e - b.e, p.n - b.n);
        } else {
            p.s = 0;
        }
        track_.push_back(p);
    }
    if (track_.size() < 2) throw std::runtime_error(hubCsv + ": not enough positions");
}

ObstacleModel::Pose ObstacleModel::egoAt(double tS) const {
    if (tS <= track_.front().tS) return track_.front();
    if (tS >= track_.back().tS) return track_.back();
    std::size_t i = 1;
    while (track_[i].tS < tS) ++i;
    const Pose &a = track_[i - 1], &b = track_[i];
    const double f = (tS - a.tS) / (b.tS - a.tS);
    return {tS, a.e + f * (b.e - a.e), a.n + f * (b.n - a.n), a.psi + f * (b.psi - a.psi),
            a.s + f * (b.s - a.s)};
}

ObstacleModel::Pose ObstacleModel::alongTrack(double s) const {
    if (s <= track_.front().s) return track_.front();
    if (s >= track_.back().s) {  // beyond the recording: straight on from its end
        Pose p = track_.back();
        const double d = s - p.s;
        p.e += d * std::cos(p.psi);
        p.n += d * std::sin(p.psi);
        p.s = s;
        return p;
    }
    std::size_t i = 1;
    while (track_[i].s < s) ++i;
    const Pose &a = track_[i - 1], &b = track_[i];
    const double f = (s - a.s) / std::max(b.s - a.s, 1e-9);
    return {a.tS + f * (b.tS - a.tS), a.e + f * (b.e - a.e), a.n + f * (b.n - a.n),
            a.psi + f * (b.psi - a.psi), s};
}

std::optional<Eigen::Isometry3d> ObstacleModel::poseAt(double tS) const {
    if (tS < o_.appearS) return std::nullopt;
    const Pose start = egoAt(o_.appearS);
    const Pose obs = alongTrack(start.s + o_.startGapM + o_.speedMps * (tS - o_.appearS));
    const Pose ego = egoAt(tS);
    // world -> vehicle: rotate by -psi_ego about the ego's position
    const double c = std::cos(ego.psi), s = std::sin(ego.psi);
    const double de = obs.e - ego.e, dn = obs.n - ego.n;
    const double x = c * de + s * dn, y = -s * de + c * dn;
    if (x < 0.5) return std::nullopt;  // reached
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() =
        Eigen::AngleAxisd(wrap(obs.psi - ego.psi), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T.translation() = Eigen::Vector3d(x, y, 0) + T.linear() * Eigen::Vector3d(0, o_.lateralM, 0);
    return T;
}

std::vector<LidarPoint> ObstacleModel::points(double tS,
                                              const Eigen::Isometry3d& lidarFromVehicle) const {
    std::vector<LidarPoint> pts;
    const auto T = poseAt(tS);
    if (!T) return pts;
    const Eigen::Isometry3d lidarFromObstacle = lidarFromVehicle * *T;
    auto add = [&](double x, double y, double z) {
        LidarPoint p;
        p.p = (lidarFromObstacle * Eigen::Vector3d(x, y, z)).cast<float>();
        p.intensity = 40;
        pts.push_back(p);
    };
    const double step = 0.1, w = o_.widthM / 2;
    for (double z = 0.3; z <= o_.heightM + 1e-9; z += step) {
        for (double y = -w; y <= w + 1e-9; y += step) add(0, y, z);  // rear face
        for (double x = 0; x <= o_.lengthM + 1e-9; x += step) {
            add(x, w, z);  // the sides
            add(x, -w, z);
        }
    }
    return pts;
}

void ObstacleModel::paint(cv::Mat& bgr, double tS, const CameraModel& camera,
                          const Eigen::Isometry3d& cameraFromVehicle) const {
    const auto T = poseAt(tS);
    if (!T) return;
    // A convex box: its silhouette is the convex hull of its projected corners (corners behind the
    // camera clipped to just in front of it).
    std::vector<cv::Point> px;
    const double w = o_.widthM / 2;
    for (double x : {0.0, o_.lengthM})
        for (double y : {-w, w})
            for (double z : {0.0, o_.heightM}) {
                Eigen::Vector3d c = cameraFromVehicle * (*T * Eigen::Vector3d(x, y, z));
                if (c.z() < 0.2) c.z() = 0.2;
                px.emplace_back(static_cast<int>(camera.fx * c.x() / c.z() + camera.cx),
                                static_cast<int>(camera.fy * c.y() / c.z() + camera.cy));
            }
    std::vector<cv::Point> hull;
    cv::convexHull(px, hull);
    cv::fillConvexPoly(bgr, hull, cv::Scalar(70, 70, 70), cv::LINE_AA);
}

}  // namespace ar_drive_assist
