#pragma once
// VehicleInterface — the only class that opens the serial port to the hub. See
// docs/BUILD_GUIDE.md Part 11.1, Part 3 (protocol and watchdog contract) and Part 5.4 (shutdown).
//
// ---------------------------------------------------------------------------------------------
// What it does, and what it deliberately does not
//   - Frames and unframes Part 3 messages (FrameDecoder). It applies no thresholds and does not
//     interpret reason codes: every decision is the DecisionArbiter's or the hub's.
//   - Sends a HEARTBEAT every heartbeatPeriodMs (50 ms = 20 Hz, twice the 10 Hz minimum), always,
//     whatever else is happening: the hub's 200 ms watchdog lives on it.
//   - Turns the arbiter's latest ActuationRequest into an ActuationCommand and sends it. If no new
//     request arrives for commandStaleMs, it sends a ZEROED command instead (Part 3.3: "no request"
//     must be said, not implied). A stalled arbiter therefore cannot leave a request standing.
//   - Brake commands carry a short hold (brakeHoldMs, 300 ms): the arbiter renews a brake every
//     cycle, so if it stops renewing, the hub releases after 300 ms even with the link healthy.
//     (The hub caps any hold at 1.5 s regardless.)
//   - Every SensorReport goes to the report callback (HubReportFeeder, via the fusion thread) and
//     updates the HubState the arbiter reads. Every AckStatus updates HubState. To the EventLog go
//     every ack that shows the brake applied (actuation evidence), and every ack whose state
//     (fault, accepted, applied) differs from the last one logged: fault onsets and ends, without
//     an fsync'd line per zeroed command while a fault persists.
//   - Logs a FAULT when SensorReports stop for longer than reportGapMs, and when they resume.
//   - Registers a pre-encoded zeroed command with SystemManager's crash handler, and provides the
//     final zeroed command for the ordered shutdown (Part 5.4).
//
// Time: hostNowMs() (steady clock, ms). The arbiter must use the same clock for its nowMs, since
// it compares HubState.reportMs against it (hub_state_max_age_ms).
//
// Threads: run() is the only reader and writer of the port while it runs. sendActuationRequest(),
// hubState(), latestSensorReport() and stats() are thread-safe. sendFinalZeroedCommand() and
// close() are for the final-command hook, after run() has returned (SystemManager joins first).
// ---------------------------------------------------------------------------------------------

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/decision/DecisionArbiter.h"
#include "ar_drive_assist/system/EventLog.h"
#include "ar_drive_assist/vehicle/FrameDecoder.h"
#include "ar_drive_assist/vehicle/HubProtocol.h"

namespace ar_drive_assist {

struct VehicleInterfaceOptions {
    std::string device = "/dev/ttyACM0";
    int baud = 115200;  // nominal for USB-CDC; real for a UART adapter
    int heartbeatPeriodMs = 50;
    int commandStaleMs = 150;
    std::uint16_t brakeHoldMs = 300;
    int reportGapMs = 200;
    // Open the port in the constructor, so a missing hub fails startup in main (with the device's
    // name) instead of on the subsystem thread after "running" was printed.
    bool openOnConstruction = false;
};

struct HubLinkStats {
    std::uint64_t reports = 0, acks = 0, otherFrames = 0, frameErrors = 0, heartbeatsSent = 0,
                  commandsSent = 0, zeroedSent = 0;
};

class VehicleInterface {
public:
    using ReportCallback =
        std::function<void(const hub_protocol::SensorReport& report, std::uint64_t hostMs)>;

    VehicleInterface(VehicleInterfaceOptions opts, EventLog& log, ReportCallback onReport = {});
    ~VehicleInterface();
    VehicleInterface(const VehicleInterface&) = delete;
    VehicleInterface& operator=(const VehicleInterface&) = delete;

    // Opens and configures the port (raw 8N1, no flow control). Throws std::runtime_error naming
    // the device. Also registers the crash frame with SystemManager.
    void open();
    // SystemManager subsystem loop: until `stop`.
    void run(const std::atomic<bool>& stop);
    // The latest request wins; it is sent by run() at its next iteration.
    void sendActuationRequest(const ActuationRequest& req);
    // For the final-command hook: writes one zeroed command directly. Only after run() returned.
    void sendFinalZeroedCommand();
    void close();

    HubState hubState() const;
    std::optional<hub_protocol::SensorReport> latestSensorReport() const;
    HubLinkStats stats() const;
    bool isOpen() const { return fd_ >= 0; }

    static std::uint64_t hostNowMs();
    // The mapping from the arbiter's request to the wire command (pure, tested directly).
    static hub_protocol::ActuationCommand toCommand(const ActuationRequest& req,
                                                    std::uint32_t hostTimestampMs,
                                                    std::uint16_t brakeHoldMs);
    // The command timestamp after `last`: now, or last + 1 if now has not moved past it. The hub
    // acts only on a STRICTLY newer timestamp (replay protection), so two commands in the same
    // millisecond must still count up.
    static std::uint32_t nextTimestamp(std::uint32_t last, std::uint64_t nowMs);
    // The zeroed command frame: the hub accepts it in any state (it can only release).
    static std::vector<std::uint8_t> zeroedCommandFrame(std::uint32_t hostTimestampMs = 0);

private:
    void writeAll(const std::vector<std::uint8_t>& bytes);
    void handle(const HubFrame& f, std::uint64_t nowMs);
    void sendCommand(const hub_protocol::ActuationCommand& c);
    std::uint32_t nextHostTs(std::uint64_t nowMs);

    VehicleInterfaceOptions opts_;
    EventLog& log_;
    ReportCallback onReport_;
    int fd_ = -1;
    FrameDecoder decoder_;
    std::vector<std::uint8_t> crashFrame_;

    mutable std::mutex mutex_;  // guards everything below
    std::optional<ActuationRequest> pending_;
    HubState hub_;
    std::optional<hub_protocol::SensorReport> lastReport_;
    HubLinkStats stats_;
    int lastAckFault_ = -1, lastAckAccepted_ = -1, lastAckApplied_ = -1;
    std::uint32_t lastHostTs_ = 0;
};

}  // namespace ar_drive_assist
