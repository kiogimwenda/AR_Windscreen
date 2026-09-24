#pragma once
// EventLog — the host's actuation evidence trail. See docs/BUILD_GUIDE.md Part 11.6, and Part 0's
// safety property 5: every actuation event is logged with a timestamp, on host and hub.
//
// ---------------------------------------------------------------------------------------------
// "Flushed immediately" has two different strengths, and this class uses both
//
// A write goes through up to three buffers on its way to the disk:
//
//   1. a userspace buffer (std::FILE, std::ofstream): lost if THIS PROCESS crashes
//   2. the kernel page cache:                          lost if the MACHINE loses power
//   3. the disk itself
//
// EventLog never uses (1). Every line is a single POSIX write() on a raw file descriptor, so as
// soon as write() returns the line has reached the kernel and survives any crash of this program.
// That meets Part 11.6's "survive a crash" requirement for every line.
//
// Safety-relevant lines (actuation requests, hub acks, faults) are also fsync()'d, which forces
// them through (2) to the disk. The case that matters is the car: ignition off, or a laptop
// power cut, would otherwise lose the last few seconds of the page cache, and those are exactly
// the seconds after an actuation. fsync costs milliseconds, which is fine for rare safety events
// and too much for routine lines, so logGeneral() skips it.
//
// Timestamps are MONOTONIC (std::chrono::steady_clock): milliseconds since this log was opened.
// Wall-clock time can jump when NTP or GPS corrects the clock, and a jump makes the gap between a
// brake request and its ack look wrong or even negative. The first line of every session records
// the wall-clock start time, so monotonic times can still be mapped to real time for the report.
//
// Every line also carries a sequence number. A gap in the sequence proves a line went missing,
// which timestamps alone cannot show.
//
// Line format (space-separated key=value, one event per line, easy to grep and parse):
//   <mono_ms> #<seq> <KIND> key=value ...
//   e.g.  1532.418 #17 ACTUATION_REQUEST type=BRAKE intensity=60 reason=1 req_ts=1532 ctx="x"
// ---------------------------------------------------------------------------------------------

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/vehicle/HubProtocol.h"

namespace ar_drive_assist {

class EventLog {
public:
    // Opens (creating if needed) and APPENDS to `path`. Throws std::runtime_error if the file
    // cannot be opened. Without an evidence trail, the system must not start.
    explicit EventLog(const std::string& path);
    ~EventLog();

    EventLog(const EventLog&) = delete;
    EventLog& operator=(const EventLog&) = delete;

    // All four are thread-safe, since every subsystem thread may log.
    void logActuationRequest(const ActuationRequest& req, const std::string& evaluationContext);
    void logAckStatus(const hub_protocol::AckStatus& ack);
    void logFault(const std::string& component, const std::string& description);
    void logGeneral(const std::string& message);

    // False once any write or fsync has failed (disk full, file deleted, I/O error). The log
    // cannot record its own failure, so the failure goes to stderr and this flag latches.
    // Phase 10's DecisionArbiter reads it: a system that can no longer record actuation evidence
    // should not be requesting actuation.
    bool healthy() const { return healthy_.load(std::memory_order_acquire); }

private:
    enum class Durability { Crash, PowerLoss };
    void writeLine(const std::string& kind, const std::string& body, Durability durability);

    int fd_ = -1;
    std::mutex mutex_;  // keeps seq_ order == file order, and one fsync per line
    std::uint64_t seq_ = 0;
    std::atomic<bool> healthy_{true};
    const std::chrono::steady_clock::time_point start_;
};

}  // namespace ar_drive_assist
