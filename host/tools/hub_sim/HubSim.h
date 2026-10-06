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
//   SensorTask     a SensorReport every 20 ms (a car driving east at 36 km/h) and onReportSent.
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

    // --- Observed hub state ---
    struct Outputs {
        std::uint8_t brakeIntensity = 0;
        bool cableMagnet = false;
        std::uint8_t indicators = 0, lights = 0;
        bool armed = false;
        std::uint8_t faultCode = 0;
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
