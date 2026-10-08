#pragma once
// HubSim — a software stand-in for the sensor and actuator hub, on a pseudo-terminal. See
// docs/BUILD_GUIDE.md Part 11.1 (amended 2026-10-06).
//
// It is not a mock: it compiles in the FIRMWARE's own hub/SafetyCore.h and hub/FrameCodec.h, the
// same code the STM32 runs, and drives them the way the firmware's tasks do:
//   CommsTask      every received byte through hub::FrameParser; valid frames to
//                  SafetyCore::onValidFrame; commands to SafetyCore::onCommand, answered with an
//                  AckStatus;
//   ActuationTask  SafetyCore::tick every 10 ms with the simulated kill switch, power box and
//                  current, producing the "hardware" outputs (brake duty, magnet, relays);
//   WatchdogTask   SafetyCore::mustRelease every 20 ms;
//   SensorTask     a SensorReport every 20 ms (a car driving east at 36 km/h, or a recording's
//                  reports: setReplay) and onReportSent.
// So VehicleInterface is tested against the real hub logic end to end, and the real hub only has
// to replace the pseudo-terminal.
//
// This header deliberately exposes no protocol types: the firmware's Protocol.h and the host's
// HubProtocol.h are identical copies and cannot both be included in one translation unit, so
// HubSim.cpp is the only file that sees the firmware's.
//
// Usable from tests (in-process) and as a tool (tools/hub_sim/main.cpp: prints the device path to
// point the host at).

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ar_drive_assist::sim {

// One recorded SensorReport (a recording's hub.csv row; system/Recording.h). Plain fields: this
// header exposes no protocol types (see above).
struct ReplayReport {
    std::int64_t tUs = 0;
    double lat = 0, lon = 0;
    float speedKph = 0;
    std::uint8_t fix = 1;
    float ax = 0, ay = 0, az = 1, gx = 0, gy = 0, gz = 0;  // g, deg/s
    float headingDeg = 0;
    float obdKph = 0;  // NaN = not valid
};
// Reads a recording's hub.csv. Throws std::runtime_error.
std::vector<ReplayReport> loadReplayReports(const std::string& csvPath);
// The recording at time tUs (the recording's own clock): linear between rows, heading the short
// way round; before the first row the first, after the last the last.
ReplayReport replayAt(const std::vector<ReplayReport>& rows, std::int64_t tUs);

class HubSim {
public:
    HubSim();  // opens a pseudo-terminal; throws on failure
    ~HubSim();
    HubSim(const HubSim&) = delete;
    HubSim& operator=(const HubSim&) = delete;

    std::string devicePath() const;  // the slave side, e.g. /dev/pts/7: what the host opens
    void start();
    void stop();

    // --- Simulated hardware inputs ---
    void setKillSwitch(bool engaged);
    void setPowerBoxPresent(bool present);
    void setActuatorCurrent(float amps);
    void setReportsPaused(bool paused);
    void injectBytes(const std::vector<std::uint8_t>& bytes);  // raw bytes into the host stream
    // Replay a recording's reports instead of the synthetic drive. The replay clock starts at the
    // first valid frame from the host, so host and hub replay the same moment of the drive;
    // reports are interpolated to the 50 Hz report rate; after the last row it holds.
    void setReplay(std::vector<ReplayReport> rows);

    // --- Observed hub state ---
    struct Outputs {
        std::uint8_t brakeIntensity = 0;
        bool cableMagnet = false;
        std::uint8_t indicators = 0, lights = 0;
        bool armed = false;
        std::uint8_t faultCode = 0;
        std::uint8_t peakBrake = 0;  // highest brake intensity applied since start
    };
    Outputs outputs() const;
    std::uint64_t framesReceived() const;
    std::uint64_t frameErrors() const;
    std::uint64_t commandsReceived() const;
    std::uint64_t reportsSent() const;

    // --- The firmware's codec, for cross-checking the host's (types as raw bytes) ---
    static std::vector<std::uint8_t> firmwareEncode(std::uint8_t type,
                                                    const std::vector<std::uint8_t>& payload);
    // Every frame the firmware's parser delivers from `bytes`, and how many errors it reports.
    static std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> firmwareDecode(
        const std::vector<std::uint8_t>& bytes, int* errors = nullptr);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ar_drive_assist::sim
