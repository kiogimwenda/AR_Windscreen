#include "ar_drive_assist/vehicle/VehicleInterface.h"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>

#include "ar_drive_assist/system/SystemManager.h"

namespace ar_drive_assist {

namespace {
speed_t baudConstant(int baud) {
    switch (baud) {
        case 9600:
            return B9600;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            throw std::runtime_error("VehicleInterface: unsupported baud " + std::to_string(baud));
    }
}
}  // namespace

std::uint64_t VehicleInterface::hostNowMs() {
    using namespace std::chrono;
    return static_cast<std::uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

hub_protocol::ActuationCommand VehicleInterface::toCommand(const ActuationRequest& req,
                                                           std::uint32_t hostTimestampMs,
                                                           std::uint16_t brakeHoldMs) {
    hub_protocol::ActuationCommand c{};
    c.hostTimestampMs = hostTimestampMs;
    switch (req.type) {
        case schema::RequestType_INDICATE_LEFT:
            c.indicators = 0x01;
            break;
        case schema::RequestType_INDICATE_RIGHT:
            c.indicators = 0x02;
            break;
        case schema::RequestType_HAZARDS:
            c.indicators = 0x04;
            break;
        case schema::RequestType_HORN:
            c.lights = 0x02;
            break;
        case schema::RequestType_BRAKE:
            // The intensity is the arbiter's (already clamped there); the hub clamps again.
            c.brakeRequest = req.intensity;
            c.brakeDurationMs = req.intensity ? brakeHoldMs : 0;
            break;
        default:
            break;  // NONE: the zeroed command
    }
    return c;
}

std::vector<std::uint8_t> VehicleInterface::zeroedCommandFrame(std::uint32_t hostTimestampMs) {
    hub_protocol::ActuationCommand c{};
    c.hostTimestampMs = hostTimestampMs;
    return encodeFrame(hub_protocol::MessageType::ACTUATION_COMMAND, &c, sizeof c);
}

VehicleInterface::VehicleInterface(VehicleInterfaceOptions opts, EventLog& log,
                                   ReportCallback onReport)
    : opts_(std::move(opts)), log_(log), onReport_(std::move(onReport)) {
    if (opts_.openOnConstruction) open();
}

VehicleInterface::~VehicleInterface() {
    close();
}

void VehicleInterface::open() {
    if (fd_ >= 0) return;
    const int fd = ::open(opts_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        throw std::runtime_error("VehicleInterface: cannot open " + opts_.device + ": " +
                                 std::strerror(errno));
    termios t{};
    if (tcgetattr(fd, &t) != 0) {
        ::close(fd);
        throw std::runtime_error("VehicleInterface: " + opts_.device + " is not a serial port");
    }
    cfmakeraw(&t);  // no echo, no line editing, no translation of bytes: a binary protocol
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~CRTSCTS;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    const speed_t b = baudConstant(opts_.baud);
    cfsetispeed(&t, b);
    cfsetospeed(&t, b);
    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        ::close(fd);
        throw std::runtime_error("VehicleInterface: cannot configure " + opts_.device);
    }
    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    crashFrame_ = zeroedCommandFrame();
    SystemManager::registerCrashFrame(fd_, crashFrame_.data(), crashFrame_.size());
    log_.logGeneral("VehicleInterface: opened " + opts_.device);
}

void VehicleInterface::close() {
    if (fd_ < 0) return;
    SystemManager::registerCrashFrame(-1, nullptr, 0);
    ::close(fd_);
    fd_ = -1;
}

void VehicleInterface::writeAll(const std::vector<std::uint8_t>& bytes) {
    std::size_t off = 0;
    while (off < bytes.size() && fd_ >= 0) {
        const ssize_t n = ::write(fd_, bytes.data() + off, bytes.size() - off);
        if (n > 0) {
            off += static_cast<std::size_t>(n);
        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            pollfd p{fd_, POLLOUT, 0};
            ::poll(&p, 1, 5);
        } else {
            throw std::runtime_error("VehicleInterface: write to " + opts_.device +
                                     " failed: " + std::strerror(errno));
        }
    }
}

std::uint32_t VehicleInterface::nextTimestamp(std::uint32_t last, std::uint64_t nowMs) {
    const auto ts = static_cast<std::uint32_t>(nowMs);
    return ts > last ? ts : last + 1;
}

std::uint32_t VehicleInterface::nextHostTs(std::uint64_t nowMs) {
    lastHostTs_ = nextTimestamp(lastHostTs_, nowMs);
    return lastHostTs_;
}

void VehicleInterface::sendCommand(const hub_protocol::ActuationCommand& c) {
    writeAll(encodeFrame(hub_protocol::MessageType::ACTUATION_COMMAND, &c, sizeof c));
}

void VehicleInterface::sendActuationRequest(const ActuationRequest& req) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = req;
}

void VehicleInterface::sendFinalZeroedCommand() {
    if (fd_ < 0) return;
    std::uint32_t ts;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ts = nextHostTs(hostNowMs());
    }
    writeAll(zeroedCommandFrame(ts));
    log_.logGeneral("VehicleInterface: final zeroed ActuationCommand sent");
}

void VehicleInterface::handle(const HubFrame& f, std::uint64_t nowMs) {
    using hub_protocol::MessageType;
    if (f.type == MessageType::SENSOR_REPORT) {
        hub_protocol::SensorReport r{};
        if (!f.as(r)) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.reports;
            lastReport_ = r;
            hub_.haveReport = true;
            hub_.reportMs = nowMs;
            hub_.brakePedalActive = r.obdBrakePedalActive;
            hub_.killSwitchEngaged = r.killSwitchEngaged;
        }
        if (onReport_) onReport_(r, nowMs);
    } else if (f.type == MessageType::ACK_STATUS) {
        hub_protocol::AckStatus a{};
        if (!f.as(a)) return;
        const std::uint8_t fault = a.actuatorFaultCode, accepted = a.lastCommandAccepted;
        const std::uint16_t applied = a.appliedBrakeIntensity;
        bool log = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.acks;
            hub_.haveAck = true;
            hub_.actuatorFaultCode = fault;
            const bool changed = fault != lastAckFault_ || accepted != lastAckAccepted_ ||
                                 applied != lastAckApplied_;
            log = applied > 0 || changed;
            lastAckFault_ = fault;
            lastAckAccepted_ = accepted;
            lastAckApplied_ = applied;
        }
        if (log) log_.logAckStatus(a);
    } else {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.otherFrames;  // BENCH_TELEMETRY (bench build) or a type this host does not use
    }
}

void VehicleInterface::run(const std::atomic<bool>& stop) {
    if (fd_ < 0) open();
    std::uint64_t lastHeartbeat = 0, lastCommandMs = hostNowMs(), lastReportMs = hostNowMs();
    bool reportGap = false;
    std::uint64_t errorsSeen = 0;
    std::uint8_t buf[512];
    while (!stop.load()) {
        pollfd p{fd_, POLLIN, 0};
        ::poll(&p, 1, 5);
        const std::uint64_t now = hostNowMs();
        if (p.revents & (POLLERR | POLLNVAL))
            throw std::runtime_error("VehicleInterface: " + opts_.device + " failed");
        if (p.revents & POLLIN) {
            const ssize_t n = ::read(fd_, buf, sizeof buf);
            if (n > 0) decoder_.push(buf, static_cast<std::size_t>(n));
        }
        while (auto f = decoder_.next()) {
            if (f->type == hub_protocol::MessageType::SENSOR_REPORT) lastReportMs = now;
            handle(*f, now);
        }
        if (decoder_.errors() != errorsSeen) {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.frameErrors = errorsSeen = decoder_.errors();
        }

        // Report gaps: logged once when they start, once when they end.
        if (!reportGap && now - lastReportMs > static_cast<std::uint64_t>(opts_.reportGapMs)) {
            reportGap = true;
            log_.logFault("VehicleInterface",
                          "no SensorReport for over " + std::to_string(opts_.reportGapMs) + " ms");
        } else if (reportGap &&
                   now - lastReportMs <= static_cast<std::uint64_t>(opts_.reportGapMs)) {
            reportGap = false;
            log_.logGeneral("VehicleInterface: SensorReports resumed");
        }

        // Heartbeat, always.
        if (now - lastHeartbeat >= static_cast<std::uint64_t>(opts_.heartbeatPeriodMs)) {
            writeAll(encodeFrame(hub_protocol::MessageType::HEARTBEAT, nullptr, 0));
            lastHeartbeat = now;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.heartbeatsSent;
        }

        // The arbiter's latest request; a zeroed command when it has gone quiet.
        std::optional<hub_protocol::ActuationCommand> cmd;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_) {
                cmd = toCommand(*pending_, nextHostTs(now), opts_.brakeHoldMs);
                pending_.reset();
                ++stats_.commandsSent;
            } else if (now - lastCommandMs >= static_cast<std::uint64_t>(opts_.commandStaleMs)) {
                cmd = hub_protocol::ActuationCommand{};
                cmd->hostTimestampMs = nextHostTs(now);
                ++stats_.zeroedSent;
            }
        }
        if (cmd) {
            sendCommand(*cmd);
            lastCommandMs = now;
        }
    }
}

HubState VehicleInterface::hubState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return hub_;
}

std::optional<hub_protocol::SensorReport> VehicleInterface::latestSensorReport() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lastReport_;
}

HubLinkStats VehicleInterface::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

}  // namespace ar_drive_assist
