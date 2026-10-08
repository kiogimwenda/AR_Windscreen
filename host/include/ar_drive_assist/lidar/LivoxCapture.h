#pragma once
// LivoxCapture — Part 5.1's LidarThread for the live Livox Mid-360 (Part 8.1), through Livox
// SDK2. Built only when the SDK is found (CMake: AR_HAVE_LIVOX); a replay uses LidarReplay.
//
//   SDK thread   every point packet's bytes -> decodeLivoxPacket -> ScanAssembler (LivoxPackets.h)
//                under a mutex; finished scans wait in a short queue
//   this thread  each finished scan -> the ground fit (LidarProcessor) -> lidarBus; and the
//                watchdog: no point packet for stallTimeoutS is a FAULT (run() throws and the
//                system shuts down, as for the camera: without its LiDAR the system cannot
//                measure a range, so it cannot brake)
//
// Network (BUILD_GUIDE Appendix C): the Mid-360 sits on 192.168.1.1xx (xx = the last two digits
// of its serial number); the host's Ethernet adapter needs a static address on the same subnet
// (hostIp). The SDK finds the LiDAR by itself; it is given a configuration file generated here
// (the SDK's mid360_config.json format: the LiDAR's ports and the host's).
//
// Mount: the point cloud stays in the LiDAR frame; vehicleFromLidar (the extrinsics) is used only
// by the ground fit. Points with a tag marking noise (Livox "tag" bits) are kept: the Mid-360's
// tags flag only likely rain/dust and confidence, and the fusion's clustering already discards
// isolated points.

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>

#include "ar_drive_assist/lidar/LidarProcessor.h"
#include "ar_drive_assist/lidar/LivoxPackets.h"
#include "ar_drive_assist/system/PipelineMessages.h"

namespace ar_drive_assist {

class EventLog;

struct LivoxConfig {
    std::string hostIp = "192.168.1.50";  // this computer's address on the LiDAR's subnet
    double stallTimeoutS = 1.0;
    double reportEveryS = 10.0;
};

class LivoxCapture {
public:
    // Starts the SDK. Throws std::runtime_error if it cannot (no network, wrong host IP), and if
    // no point packet arrives within 10 s (the LiDAR is not there or not reachable). `tap`, if
    // set, is called with every published scan on the run() thread and must return at once
    // (system/Recorder.h).
    LivoxCapture(LivoxConfig cfg, LidarBus& out, const Eigen::Isometry3d& vehicleFromLidar,
                 EventLog* log = nullptr, std::function<void(const LidarFrame&)> tap = {});
    ~LivoxCapture();
    LivoxCapture(const LivoxCapture&) = delete;
    LivoxCapture& operator=(const LivoxCapture&) = delete;

    void run(const std::atomic<bool>& stop);

    // The SDK's callback (public only so the C callback can reach it).
    void onPacket(const std::uint8_t* bytes, std::size_t len);
    void onLidarFound(std::uint32_t handle, const char* serial, const char* ip);

private:
    LivoxConfig cfg_;
    LidarBus& out_;
    Eigen::Isometry3d vehicleFromLidar_;
    EventLog* log_;
    std::string configPath_;
    LidarProcessor processor_;

    std::mutex m_;
    ScanAssembler assembler_;
    std::deque<LidarFrame> ready_;
    std::uint64_t packets_ = 0, badPackets_ = 0, droppedScans_ = 0;
    std::atomic<std::int64_t> lastPacketUs_{0};
    std::function<void(const LidarFrame&)> tap_;
};

}  // namespace ar_drive_assist
