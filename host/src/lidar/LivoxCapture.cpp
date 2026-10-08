// LivoxCapture — see include/ar_drive_assist/lidar/LivoxCapture.h. Compiled only with Livox SDK2.
#include "ar_drive_assist/lidar/LivoxCapture.h"

#include <livox_lidar_api.h>
#include <livox_lidar_def.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <thread>

#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {

// LivoxPackets.cpp decodes the packet from its bytes at fixed offsets; these hold it to the SDK's
// own definition, so an SDK that changes the layout fails to compile rather than mis-decoding.
static_assert(offsetof(LivoxLidarEthernetPacket, time_interval) == 3);
static_assert(offsetof(LivoxLidarEthernetPacket, dot_num) == 5);
static_assert(offsetof(LivoxLidarEthernetPacket, data_type) == 10);
static_assert(offsetof(LivoxLidarEthernetPacket, time_type) == 11);
static_assert(offsetof(LivoxLidarEthernetPacket, timestamp) == 28);
static_assert(offsetof(LivoxLidarEthernetPacket, data) == 36);
static_assert(sizeof(LivoxLidarCartesianHighRawPoint) == 14);
static_assert(sizeof(LivoxLidarCartesianLowRawPoint) == 8);
static_assert(kLivoxLidarCartesianCoordinateHighData == 1 &&
              kLivoxLidarCartesianCoordinateLowData == 2);

namespace {

std::int64_t hostUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void pointCallback(const uint32_t, const uint8_t, LivoxLidarEthernetPacket* data, void* self) {
    if (!data || !self) return;
    static_cast<LivoxCapture*>(self)->onPacket(reinterpret_cast<const std::uint8_t*>(data),
                                               data->length);
}

void infoCallback(const uint32_t handle, const LivoxLidarInfo* info, void* self) {
    if (self && info)
        static_cast<LivoxCapture*>(self)->onLidarFound(handle, info->sn, info->lidar_ip);
}

}  // namespace

LivoxCapture::LivoxCapture(LivoxConfig cfg, LidarBus& out,
                           const Eigen::Isometry3d& vehicleFromLidar, EventLog* log,
                           std::function<void(const LidarFrame&)> tap)
    : cfg_(std::move(cfg)),
      out_(out),
      vehicleFromLidar_(vehicleFromLidar),
      log_(log),
      tap_(std::move(tap)) {
    // The SDK's configuration file (the format of its mid360_config.json sample).
    configPath_ = "/tmp/ar_drive_assist_mid360_" + std::to_string(::getpid()) + ".json";
    std::ofstream(configPath_)
        << "{\"MID360\": {\"lidar_net_info\": {\"cmd_data_port\": 56100, \"push_msg_port\": 56200, "
           "\"point_data_port\": 56300, \"imu_data_port\": 56400, \"log_data_port\": 56500}, "
           "\"host_net_info\": [{\"host_ip\": \""
        << cfg_.hostIp
        << "\", \"multicast_ip\": \"224.1.1.5\", \"cmd_data_port\": 56101, \"push_msg_port\": "
           "56201, \"point_data_port\": 56301, \"imu_data_port\": 56401, \"log_data_port\": "
           "56501}]}}\n";
    if (!LivoxLidarSdkInit(configPath_.c_str()))
        throw std::runtime_error("LiDAR: Livox SDK2 init failed (host IP " + cfg_.hostIp +
                                 " on this machine? BUILD_GUIDE Appendix C)");
    SetLivoxLidarPointCloudCallBack(pointCallback, this);
    SetLivoxLidarInfoChangeCallback(infoCallback, this);
    if (!LivoxLidarSdkStart()) {
        LivoxLidarSdkUninit();
        throw std::runtime_error("LiDAR: Livox SDK2 failed to start");
    }
    const auto t0 = std::chrono::steady_clock::now();
    while (lastPacketUs_.load() == 0) {
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10)) {
            LivoxLidarSdkUninit();
            throw std::runtime_error(
                "LiDAR: no point data from a Mid-360 within 10 s (powered? "
                "cabled? ping 192.168.1.1xx; BUILD_GUIDE Appendix C)");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (log_) log_->logGeneral("LiDAR: Mid-360 point data arriving (host " + cfg_.hostIp + ")");
}

LivoxCapture::~LivoxCapture() {
    LivoxLidarSdkUninit();  // stops the SDK's threads: no callback after this
    std::remove(configPath_.c_str());
}

void LivoxCapture::onLidarFound(std::uint32_t handle, const char* serial, const char* ip) {
    // A Mid-360 announced itself: make sure it is measuring (it may boot into standby).
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
    if (log_)
        log_->logGeneral(std::string("LiDAR: Mid-360 ") + std::string(serial, strnlen(serial, 16)) +
                         " at " + std::string(ip, strnlen(ip, 16)) + ", set to normal mode");
}

void LivoxCapture::onPacket(const std::uint8_t* bytes, std::size_t len) {
    const std::int64_t arrival = hostUs();
    LivoxPacketInfo info;
    std::vector<LivoxPoint> pts;
    std::lock_guard<std::mutex> lock(m_);
    ++packets_;
    if (!decodeLivoxPacket(bytes, len, info, pts)) {
        ++badPackets_;
        return;
    }
    lastPacketUs_ = arrival;
    if (auto scan = assembler_.add(pts, info.firstNs, arrival)) {
        if (ready_.size() >= 3) {  // the processing thread has stalled: keep the newest
            ready_.pop_front();
            ++droppedScans_;
        }
        ready_.push_back(std::move(*scan));
    }
}

void LivoxCapture::run(const std::atomic<bool>& stop) {
    auto lastReport = std::chrono::steady_clock::now();
    std::uint64_t scans = 0;
    while (!stop.load()) {
        LidarFrame f;
        bool have = false;
        {
            std::lock_guard<std::mutex> lock(m_);
            if (!ready_.empty()) {
                f = std::move(ready_.front());
                ready_.pop_front();
                have = true;
            }
        }
        if (!have) {
            const double quietS = (hostUs() - lastPacketUs_.load()) * 1e-6;
            if (quietS > cfg_.stallTimeoutS) {
                char b[96];
                std::snprintf(b, sizeof b, "no point data for %.2f s", quietS);
                if (log_) log_->logFault("LivoxCapture", b);
                throw std::runtime_error(std::string("LiDAR: ") + b);  // SystemManager shuts down
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        const auto g = processor_.fitGround(f.points, vehicleFromLidar_, f.timestampMs);
        f.ground = g.ground;
        f.groundValid = g.valid;
        if (tap_) tap_(f);
        out_.push(std::move(f));  // a full bus: the fusion thread takes the newest
        ++scans;
        if (log_ && std::chrono::steady_clock::now() - lastReport >
                        std::chrono::duration<double>(cfg_.reportEveryS)) {
            std::lock_guard<std::mutex> lock(m_);
            char b[200];
            std::snprintf(b, sizeof b,
                          "LiDAR: %llu scans, %llu packets (%llu bad), %llu scans dropped, clock "
                          "offset %lld us, %llu clock resets",
                          static_cast<unsigned long long>(scans),
                          static_cast<unsigned long long>(packets_),
                          static_cast<unsigned long long>(badPackets_),
                          static_cast<unsigned long long>(droppedScans_),
                          static_cast<long long>(assembler_.offsetUs()),
                          static_cast<unsigned long long>(assembler_.clockResets()));
            log_->logGeneral(b);
            lastReport = std::chrono::steady_clock::now();
        }
    }
}

}  // namespace ar_drive_assist
