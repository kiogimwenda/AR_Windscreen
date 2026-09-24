#include "ar_drive_assist/system/EventLog.h"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <stdexcept>

namespace ar_drive_assist {
namespace {

// Values that may contain spaces or newlines are quoted and escaped, so one event is always
// exactly one line. A multi-line fault description would otherwise look like several events to
// anything that parses the log later.
std::string quoted(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        switch (c) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            default:
                out += c;
        }
    }
    return out + "\"";
}

// Appendix B. Kept in step with the table there; an unrecognised code is logged as UNKNOWN
// alongside its number, never dropped.
const char* faultName(std::uint8_t code) {
    switch (code) {
        case 0:
            return "NONE";
        case 1:
            return "OVERCURRENT";
        case 2:
            return "KILL_SWITCH";
        case 3:
            return "LINK_TIMEOUT";
        case 4:
            return "FRAME_ERRORS";
        default:
            return "UNKNOWN";
    }
}

std::string wallClockUtc() {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&now, &tm);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

}  // namespace

EventLog::EventLog(const std::string& path) : start_(std::chrono::steady_clock::now()) {
    // O_APPEND: every write lands at the current end of file, even if something else has the file
    //           open. Earlier sessions are never overwritten.
    // O_CLOEXEC: the descriptor is not inherited by any child process this program might spawn.
    fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (fd_ < 0) {
        throw std::runtime_error("EventLog: cannot open " + path + ": " + std::strerror(errno));
    }
    writeLine("SESSION_START", "wall_utc=" + wallClockUtc() + " pid=" + std::to_string(::getpid()),
              Durability::PowerLoss);
}

EventLog::~EventLog() {
    if (fd_ < 0) return;
    writeLine("SESSION_END", "", Durability::PowerLoss);
    ::close(fd_);
}

void EventLog::logActuationRequest(const ActuationRequest& req,
                                   const std::string& evaluationContext) {
    writeLine("ACTUATION_REQUEST",
              std::string("type=") + schema::EnumNameRequestType(req.type) + " intensity=" +
                  std::to_string(req.intensity) + " reason=" + std::to_string(req.reason_code) +
                  " req_ts=" + std::to_string(req.timestamp_ms) +
                  " ctx=" + quoted(evaluationContext),
              Durability::PowerLoss);
}

void EventLog::logAckStatus(const hub_protocol::AckStatus& ack) {
    // AckStatus is a packed struct, so its fields are copied into locals before use. Forming a
    // reference or pointer to a packed member is the unaligned access HubProtocol.h warns about.
    const std::uint32_t hubTs = ack.timestampMs;
    const std::uint8_t accepted = ack.lastCommandAccepted;
    const std::uint8_t fault = ack.actuatorFaultCode;
    const std::uint16_t applied = ack.appliedBrakeIntensity;
    writeLine("ACK_STATUS",
              "hub_ts=" + std::to_string(hubTs) + " accepted=" + std::to_string(accepted) +
                  " fault=" + std::to_string(fault) + " fault_name=" + faultName(fault) +
                  " applied=" + std::to_string(applied),
              Durability::PowerLoss);
}

void EventLog::logFault(const std::string& component, const std::string& description) {
    writeLine("FAULT", "component=" + quoted(component) + " desc=" + quoted(description),
              Durability::PowerLoss);
}

void EventLog::logGeneral(const std::string& message) {
    writeLine("INFO", "msg=" + quoted(message), Durability::Crash);
}

void EventLog::writeLine(const std::string& kind, const std::string& body, Durability durability) {
    // The timestamp is taken INSIDE the lock, so that line order, sequence order and time order
    // always agree. Taken outside, two threads could interleave and produce a line whose time is
    // earlier than the line above it.
    std::lock_guard<std::mutex> lock(mutex_);

    const auto elapsed = std::chrono::steady_clock::now() - start_;
    const double ms = std::chrono::duration<double, std::milli>(elapsed).count();
    char stamp[48];
    std::snprintf(stamp, sizeof stamp, "%.3f #%llu ", ms, static_cast<unsigned long long>(seq_++));

    std::string line = stamp + kind;
    if (!body.empty()) line += " " + body;
    line += "\n";

    // write() may legally write fewer bytes than asked (a "short write"), or be interrupted by a
    // signal before writing anything (EINTR). Both are retried. Any other error means the log is
    // no longer trustworthy.
    const char* p = line.data();
    std::size_t left = line.size();
    while (left > 0) {
        const ssize_t n = ::write(fd_, p, left);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::fprintf(stderr, "EventLog: write failed: %s\n", std::strerror(errno));
            healthy_.store(false, std::memory_order_release);
            return;
        }
        p += n;
        left -= static_cast<std::size_t>(n);
    }

    if (durability == Durability::PowerLoss && ::fsync(fd_) != 0) {
        std::fprintf(stderr, "EventLog: fsync failed: %s\n", std::strerror(errno));
        healthy_.store(false, std::memory_order_release);
    }
}

}  // namespace ar_drive_assist
