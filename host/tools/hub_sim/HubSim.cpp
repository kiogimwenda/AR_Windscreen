// HubSim — see HubSim.h. Compiled with the FIRMWARE's include directory: hub/SafetyCore.h,
// hub/FrameCodec.h, hub/Protocol.h, hub/Config.h are the STM32's own files.
#include "HubSim.h"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "hub/FrameCodec.h"
#include "hub/SafetyCore.h"

namespace ar_drive_assist::sim {

struct HubSim::Impl {
    int master = -1;
    std::string slave;
    std::thread thread;
    std::atomic<bool> running{false};
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    std::atomic<bool> kill{false}, box{true}, paused{false};
    std::atomic<float> current{0.0f};
    std::mutex injectMutex;
    std::vector<std::uint8_t> inject;

    mutable std::mutex outMutex;
    Outputs out;
    std::atomic<std::uint64_t> frames{0}, errors{0}, commands{0}, reports{0};
    std::vector<ReplayReport> replay;  // set before start()
    bool replayStarted = false;
    std::uint32_t replayStartMs = 0;

    hub::SafetyCore core;
    hub::FrameParser parser;

    std::uint32_t millis() const {
        return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                              std::chrono::steady_clock::now() - t0)
                                              .count());
    }

    ReplayReport replayAt(std::uint32_t ms) const {
        return sim::replayAt(replay, replay.front().tUs + static_cast<std::int64_t>(ms) * 1000);
    }

    void send(hub_protocol::MessageType type, const void* payload, std::size_t len) {
        std::uint8_t buf[hub_protocol::MAX_FRAME_SIZE];
        const std::size_t n = hub::encodeFrame(type, payload, len, buf);
        std::size_t off = 0;
        while (off < n) {
            const ssize_t w = ::write(master, buf + off, n - off);
            if (w > 0)
                off += static_cast<std::size_t>(w);
            else if (errno == EAGAIN || errno == EINTR)
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            else
                return;  // the host closed its side: drop, like a hub with no USB host
        }
    }

    void loop() {
        std::uint32_t lastTick = millis(), lastReport = millis();
        std::uint8_t buf[512];
        double eastM = 0;
        while (running.load()) {
            pollfd p{master, POLLIN, 0};
            ::poll(&p, 1, 1);
            if (p.revents & POLLIN) {
                const ssize_t n = ::read(master, buf, sizeof buf);
                for (ssize_t i = 0; i < n; ++i) parser.push(buf[i]);
            }
            // CommsTask
            for (hub::FrameParser::Result r;
                 (r = parser.next()) != hub::FrameParser::Result::NONE;) {
                const std::uint32_t now = millis();
                if (r == hub::FrameParser::Result::ERROR) {
                    core.onFrameError();
                    ++errors;
                    continue;
                }
                ++frames;
                hub_protocol::ActuationCommand cmd{};
                const bool isCommand =
                    parser.type() == hub_protocol::MessageType::ACTUATION_COMMAND && parser.as(cmd);
                core.onValidFrame(now, parser.type());
                if (isCommand) {
                    ++commands;
                    const hub_protocol::AckStatus ack = core.onCommand(now, cmd);
                    send(hub_protocol::MessageType::ACK_STATUS, &ack, sizeof ack);
                }
            }
            {
                std::lock_guard<std::mutex> lock(injectMutex);
                if (!inject.empty()) {
                    (void)!::write(master, inject.data(), inject.size());
                    inject.clear();
                }
            }
            const std::uint32_t now = millis();
            // ActuationTask (10 ms) and WatchdogTask (its check folded in)
            if (now - lastTick >= hub_config::kActuationPeriodMs) {
                lastTick = now;
                hub::HardwareInputs in;
                in.nowMs = now;
                in.killSwitchEngaged = kill.load();
                in.powerBoxPresent = box.load();
                in.currentAmps = current.load();
                hub::HardwareOutputs o = core.tick(in);
                if (core.mustRelease(now, in.killSwitchEngaged, in.powerBoxPresent)) {
                    o.brakeIntensity = 0;
                    o.cableMagnet = false;
                    o.indicators = o.lights = 0;
                }
                std::lock_guard<std::mutex> lock(outMutex);
                out.brakeIntensity = o.brakeIntensity;
                if (o.brakeIntensity > out.peakBrake) out.peakBrake = o.brakeIntensity;
                out.cableMagnet = o.cableMagnet;
                out.indicators = o.indicators;
                out.lights = o.lights;
                out.armed = core.armed(now);
                out.faultCode = core.faultCode(now);
            }
            // SensorTask (20 ms): a car driving east at 36 km/h near Nairobi CBD
            if (now - lastReport >= hub_config::kSensorReportPeriodMs) {
                lastReport = now;
                if (paused.load()) continue;
                hub_protocol::SensorReport r{};
                r.timestampMs = now;
                if (!replay.empty()) {
                    if (!replayStarted && frames.load() > 0) {
                        replayStarted = true;  // the host is talking: the drive starts now
                        replayStartMs = now;
                    }
                    const ReplayReport row = replayAt(replayStarted ? now - replayStartMs : 0);
                    r.latitude = row.lat;
                    r.longitude = row.lon;
                    r.speedKph = row.speedKph;
                    r.gpsFixValid = row.fix;
                    r.accelX = row.ax;
                    r.accelY = row.ay;
                    r.accelZ = row.az;
                    r.gyroX = row.gx;
                    r.gyroY = row.gy;
                    r.gyroZ = row.gz;
                    r.headingDeg = row.headingDeg;
                    r.obdSpeedKph = row.obdKph;
                } else {
                    eastM += 10.0 * 0.02;
                    r.latitude = -1.2864;
                    r.longitude = 36.8172 + eastM / 111320.0;
                    r.speedKph = 36.0f;
                    r.gpsFixValid = 1;
                    r.accelZ = 1.0f;
                    r.headingDeg = 90.0f;
                    r.obdSpeedKph = 36.0f;
                }
                r.obdRpm = 1800;
                r.obdBrakePedalActive = 0xFF;
                r.ignitionOn = box.load() ? 1 : 0;
                r.killSwitchEngaged = kill.load() ? 1 : 0;
                send(hub_protocol::MessageType::SENSOR_REPORT, &r, sizeof r);
                core.onReportSent(now);
                ++reports;
            }
        }
    }
};

HubSim::HubSim() : impl_(std::make_unique<Impl>()) {
    impl_->master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (impl_->master < 0 || ::grantpt(impl_->master) != 0 || ::unlockpt(impl_->master) != 0)
        throw std::runtime_error("HubSim: cannot open a pseudo-terminal");
    impl_->slave = ::ptsname(impl_->master);
    // A new pseudo-terminal starts in "cooked" mode with ECHO on: until the host opens it and
    // switches it to raw, every report this hub writes would be echoed straight back into its own
    // input as garbage (found running the real host against it: 83 frame errors, fault 4, 1.5 s
    // disarmed). A real USB-CDC hub has no echo; set the slave raw here, once.
    const int s = ::open(impl_->slave.c_str(), O_RDWR | O_NOCTTY);
    if (s >= 0) {
        termios t{};
        if (::tcgetattr(s, &t) == 0) {
            ::cfmakeraw(&t);
            ::tcsetattr(s, TCSANOW, &t);
        }
        ::close(s);
    }
}

HubSim::~HubSim() {
    stop();
    if (impl_->master >= 0) ::close(impl_->master);
}

std::string HubSim::devicePath() const {
    return impl_->slave;
}

void HubSim::start() {
    if (impl_->running.exchange(true)) return;
    impl_->thread = std::thread([this] { impl_->loop(); });
}

void HubSim::stop() {
    if (!impl_->running.exchange(false)) return;
    if (impl_->thread.joinable()) impl_->thread.join();
}

void HubSim::setKillSwitch(bool engaged) {
    impl_->kill = engaged;
}
void HubSim::setPowerBoxPresent(bool present) {
    impl_->box = present;
}
void HubSim::setActuatorCurrent(float amps) {
    impl_->current = amps;
}
void HubSim::setReplay(std::vector<ReplayReport> rows) {
    if (impl_->running.load()) throw std::logic_error("HubSim: setReplay before start()");
    impl_->replay = std::move(rows);
}

ReplayReport replayAt(const std::vector<ReplayReport>& replay, std::int64_t t) {
    if (t <= replay.front().tUs) return replay.front();
    if (t >= replay.back().tUs) return replay.back();
    std::size_t i = 1;
    while (replay[i].tUs < t) ++i;
    const ReplayReport &a = replay[i - 1], &b = replay[i];
    const double f = double(t - a.tUs) / double(b.tUs - a.tUs);
    auto mix = [f](double x, double y) { return x + (y - x) * f; };
    ReplayReport r = a;
    r.tUs = t;
    r.lat = mix(a.lat, b.lat);
    r.lon = mix(a.lon, b.lon);
    r.speedKph = float(mix(a.speedKph, b.speedKph));
    r.ax = float(mix(a.ax, b.ax));
    r.ay = float(mix(a.ay, b.ay));
    r.az = float(mix(a.az, b.az));
    r.gx = float(mix(a.gx, b.gx));
    r.gy = float(mix(a.gy, b.gy));
    r.gz = float(mix(a.gz, b.gz));
    r.obdKph = float(mix(a.obdKph, b.obdKph));
    double dh = b.headingDeg - a.headingDeg;
    if (dh > 180) dh -= 360;
    if (dh < -180) dh += 360;
    r.headingDeg = float(std::fmod(a.headingDeg + dh * f + 360.0, 360.0));
    return r;
}

std::vector<ReplayReport> loadReplayReports(const std::string& csvPath) {
    std::ifstream in(csvPath);
    if (!in) throw std::runtime_error(csvPath + ": cannot read");
    std::vector<ReplayReport> rows;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string c;
        while (std::getline(ss, c, ',')) f.push_back(c);
        if (f.size() < 12) throw std::runtime_error(csvPath + ": short row '" + line + "'");
        ReplayReport r;
        r.tUs = std::stoll(f[0]);
        r.lat = std::stod(f[1]);
        r.lon = std::stod(f[2]);
        r.speedKph = std::stof(f[3]);
        r.fix = static_cast<std::uint8_t>(std::stoi(f[4]));
        r.ax = std::stof(f[5]);
        r.ay = std::stof(f[6]);
        r.az = std::stof(f[7]);
        r.gx = std::stof(f[8]);
        r.gy = std::stof(f[9]);
        r.gz = std::stof(f[10]);
        r.headingDeg = std::stof(f[11]);
        r.obdKph = f.size() > 12 && !f[12].empty() ? std::stof(f[12]) : NAN;
        rows.push_back(r);
    }
    if (rows.empty()) throw std::runtime_error(csvPath + ": no reports");
    return rows;
}

void HubSim::setReportsPaused(bool paused) {
    impl_->paused = paused;
}
void HubSim::injectBytes(const std::vector<std::uint8_t>& bytes) {
    std::lock_guard<std::mutex> lock(impl_->injectMutex);
    impl_->inject.insert(impl_->inject.end(), bytes.begin(), bytes.end());
}

HubSim::Outputs HubSim::outputs() const {
    std::lock_guard<std::mutex> lock(impl_->outMutex);
    return impl_->out;
}
std::uint64_t HubSim::framesReceived() const {
    return impl_->frames;
}
std::uint64_t HubSim::frameErrors() const {
    return impl_->errors;
}
std::uint64_t HubSim::commandsReceived() const {
    return impl_->commands;
}
std::uint64_t HubSim::reportsSent() const {
    return impl_->reports;
}

std::vector<std::uint8_t> HubSim::firmwareEncode(std::uint8_t type,
                                                 const std::vector<std::uint8_t>& payload) {
    std::uint8_t buf[hub_protocol::MAX_FRAME_SIZE];
    const std::size_t n = hub::encodeFrame(static_cast<hub_protocol::MessageType>(type),
                                           payload.data(), payload.size(), buf);
    return {buf, buf + n};
}

std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> HubSim::firmwareDecode(
    const std::vector<std::uint8_t>& bytes, int* errors) {
    hub::FrameParser p;
    std::vector<std::pair<std::uint8_t, std::vector<std::uint8_t>>> out;
    int err = 0;
    for (std::uint8_t b : bytes) {
        p.push(b);
        for (hub::FrameParser::Result r; (r = p.next()) != hub::FrameParser::Result::NONE;) {
            if (r == hub::FrameParser::Result::ERROR) {
                ++err;
                continue;
            }
            out.emplace_back(static_cast<std::uint8_t>(p.type()),
                             std::vector<std::uint8_t>(p.payload(), p.payload() + p.length()));
        }
    }
    if (errors) *errors = err;
    return out;
}

}  // namespace ar_drive_assist::sim
