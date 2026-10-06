// VehicleInterface tests — BUILD_GUIDE Part 11.1, Part 3 (protocol, watchdog contract), Part 5.4.
//
// Two kinds:
//   - the codec against the FIRMWARE's codec (HubSim exposes hub/FrameCodec.h): each side must
//     read the other's bytes, and no corrupted frame may ever be delivered;
//   - end to end over a pseudo-terminal against HubSim, which runs the firmware's own SafetyCore:
//     heartbeats arm it, requests reach it as commands, its acks and faults come back, and every
//     way the host can go quiet releases the brake.
// Timings use generous margins: they test the mechanism, not the scheduler of this machine.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <thread>

#include "HubSim.h"
#include "ar_drive_assist/fusion/HubReportFeeder.h"
#include "ar_drive_assist/vehicle/VehicleInterface.h"

using namespace ar_drive_assist;
using namespace std::chrono_literals;
using sim::HubSim;

namespace {

std::string tempLog(const std::string& name) {
    const std::string p = "/tmp/vi_test_" + name + "_" + std::to_string(::getpid()) + ".log";
    std::remove(p.c_str());
    return p;
}

std::string readAll(const std::string& path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

ActuationRequest request(schema::RequestType type, std::uint8_t intensity = 0) {
    ActuationRequest r;
    r.type = type;
    r.intensity = intensity;
    r.timestamp_ms = VehicleInterface::hostNowMs();
    return r;
}

// Waits up to `limit` for `cond`; returns how long it took (or limit + 1 ms if it never held).
template <typename F>
std::chrono::milliseconds waitFor(F cond, std::chrono::milliseconds limit) {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < limit) {
        if (cond())
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0);
        std::this_thread::sleep_for(2ms);
    }
    return limit + 1ms;
}

// A running pair: the simulated hub, and VehicleInterface on its own thread.
struct Link {
    std::string logPath;
    std::unique_ptr<EventLog> log;
    HubSim hub;
    std::unique_ptr<VehicleInterface> vi;
    std::atomic<bool> stop{false};
    std::thread thread;
    std::atomic<int> reports{0};

    explicit Link(const std::string& name, VehicleInterface::ReportCallback cb = {}) {
        logPath = tempLog(name);
        log = std::make_unique<EventLog>(logPath);
        hub.start();
        VehicleInterfaceOptions o;
        o.device = hub.devicePath();
        if (!cb) cb = [this](const hub_protocol::SensorReport&, std::uint64_t) { ++reports; };
        vi = std::make_unique<VehicleInterface>(o, *log, cb);
        vi->open();
        thread = std::thread([this] { vi->run(stop); });
    }
    void stopHost() {
        stop = true;
        if (thread.joinable()) thread.join();
    }
    ~Link() {
        stopHost();
        vi->close();
        hub.stop();
        std::remove(logPath.c_str());
    }
    // The arbiter's cycle: keep requesting for `d`, every 30 ms.
    void hold(const ActuationRequest& r, std::chrono::milliseconds d) {
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - t0 < d) {
            vi->sendActuationRequest(r);
            std::this_thread::sleep_for(30ms);
        }
    }
};

}  // namespace

// --- Codec, against the firmware's ---------------------------------------------------------------

TEST(HubCodec, HostAndFirmwareReadEachOthersFrames) {
    std::mt19937 rng(7);
    for (int i = 0; i < 300; ++i) {
        const std::uint8_t type = static_cast<std::uint8_t>(1 + rng() % 5);
        std::vector<std::uint8_t> payload(rng() % (hub_protocol::MAX_PAYLOAD + 1));
        for (auto& b : payload) b = static_cast<std::uint8_t>(rng());
        // host -> firmware
        const auto hostBytes = encodeFrame(static_cast<hub_protocol::MessageType>(type),
                                           payload.data(), payload.size());
        const auto fw = HubSim::firmwareDecode(hostBytes);
        ASSERT_EQ(fw.size(), 1u);
        EXPECT_EQ(fw[0].first, type);
        EXPECT_EQ(fw[0].second, payload);
        // firmware -> host
        EXPECT_EQ(HubSim::firmwareEncode(type, payload), hostBytes) << "identical bytes both ways";
        FrameDecoder d;
        d.push(hostBytes.data(), hostBytes.size());
        auto f = d.next();
        ASSERT_TRUE(f.has_value());
        EXPECT_EQ(f->payload, payload);
    }
}

// Every single-bit corruption of a SensorReport frame is rejected by the host, and the next good
// frames are still delivered.
TEST(HubCodec, HostNeverDeliversACorruptedFrameAndRecovers) {
    hub_protocol::SensorReport r{};
    r.timestampMs = 12345;
    r.latitude = -1.2864;
    const auto good = encodeFrame(hub_protocol::MessageType::SENSOR_REPORT, &r, sizeof r);
    const auto hb = encodeFrame(hub_protocol::MessageType::HEARTBEAT, nullptr, 0);
    for (std::size_t bit = 8; bit < good.size() * 8; ++bit) {
        auto bad = good;
        bad[bit / 8] ^= static_cast<std::uint8_t>(1u << (bit % 8));
        FrameDecoder d;
        d.push(bad.data(), bad.size());
        for (int k = 0; k < 3; ++k) d.push(good.data(), good.size());
        d.push(hb.data(), hb.size());
        int reports = 0, heartbeats = 0;
        while (auto f = d.next()) {
            if (f->type == hub_protocol::MessageType::SENSOR_REPORT) {
                hub_protocol::SensorReport got{};
                ASSERT_TRUE(f->as(got));
                const std::uint32_t ts = got.timestampMs;
                ASSERT_EQ(ts, 12345u) << "a corrupted report was delivered (bit " << bit << ")";
                ++reports;
            }
            heartbeats += f->type == hub_protocol::MessageType::HEARTBEAT;
        }
        EXPECT_EQ(reports, 3) << "bit " << bit;
        EXPECT_EQ(heartbeats, 1) << "bit " << bit;
    }
}

TEST(HubCodec, TheCrashFrameIsAZeroedCommandTheFirmwareAccepts) {
    const auto fw = HubSim::firmwareDecode(VehicleInterface::zeroedCommandFrame());
    ASSERT_EQ(fw.size(), 1u);
    EXPECT_EQ(fw[0].first, static_cast<std::uint8_t>(hub_protocol::MessageType::ACTUATION_COMMAND));
    ASSERT_EQ(fw[0].second.size(), sizeof(hub_protocol::ActuationCommand));
    for (std::uint8_t b : fw[0].second) EXPECT_EQ(b, 0);
}

TEST(VehicleInterfaceMapping, RequestsBecomeCommands) {
    // Packed struct: copy fields to locals before comparing (EXPECT_EQ binds references, and a
    // reference to a packed member is misaligned: UBSan caught it).
    const auto c = VehicleInterface::toCommand(request(schema::RequestType_BRAKE, 60), 7, 300);
    const std::uint8_t brake = c.brakeRequest;
    const std::uint16_t dur = c.brakeDurationMs;
    const std::uint32_t ts = c.hostTimestampMs;
    EXPECT_EQ(brake, 60);
    EXPECT_EQ(dur, 300);
    EXPECT_EQ(ts, 7u);
    EXPECT_EQ(VehicleInterface::toCommand(request(schema::RequestType_HAZARDS), 1, 300).indicators,
              0x04);
    EXPECT_EQ(
        VehicleInterface::toCommand(request(schema::RequestType_INDICATE_LEFT), 1, 300).indicators,
        0x01);
    EXPECT_EQ(
        VehicleInterface::toCommand(request(schema::RequestType_INDICATE_RIGHT), 1, 300).indicators,
        0x02);
    EXPECT_EQ(VehicleInterface::toCommand(request(schema::RequestType_HORN), 1, 300).lights, 0x02);
    const auto none = VehicleInterface::toCommand(request(schema::RequestType_NONE), 1, 300);
    EXPECT_EQ(none.brakeRequest | none.indicators | none.lights | none.brakeDurationMs, 0);
}

TEST(VehicleInterfaceMapping, CommandTimestampsAlwaysCountUp) {
    EXPECT_EQ(VehicleInterface::nextTimestamp(0, 1000), 1000u);
    EXPECT_EQ(VehicleInterface::nextTimestamp(1000, 1000), 1001u) << "same millisecond";
    EXPECT_EQ(VehicleInterface::nextTimestamp(1001, 1000), 1002u) << "clock behind the counter";
    EXPECT_EQ(VehicleInterface::nextTimestamp(1002, 5000), 5000u);
}

TEST(VehicleInterfaceOpen, NamesTheDeviceWhenItCannotOpen) {
    const auto path = tempLog("open");
    EventLog log(path);
    VehicleInterfaceOptions o;
    o.device = "/dev/does-not-exist-hub";
    VehicleInterface vi(o, log);
    try {
        vi.open();
        FAIL() << "opened a missing device";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("/dev/does-not-exist-hub"), std::string::npos);
    }
    std::remove(path.c_str());
}

// --- End to end against the firmware's SafetyCore ----------------------------------------------

TEST(VehicleInterfaceE2E, ReportsFlowAndHeartbeatsArmTheHub) {
    Link link("arm");
    EXPECT_LT(waitFor([&] { return link.hub.outputs().armed; }, 1000ms), 1000ms);
    std::this_thread::sleep_for(500ms);
    const int n = link.reports.load();
    EXPECT_GT(n, 15) << "SensorReports at ~50 Hz";
    EXPECT_TRUE(link.vi->hubState().haveReport);
    EXPECT_EQ(link.hub.frameErrors(), 0u);
    EXPECT_EQ(link.vi->stats().frameErrors, 0u);
}

TEST(VehicleInterfaceE2E, ABrakeRequestIsAppliedClampedAndAcked) {
    Link link("brake");
    waitFor([&] { return link.hub.outputs().armed; }, 1000ms);
    std::thread arbiter([&] { link.hold(request(schema::RequestType_BRAKE, 255), 400ms); });
    EXPECT_LT(waitFor([&] { return link.hub.outputs().brakeIntensity > 0; }, 400ms), 400ms);
    EXPECT_EQ(link.hub.outputs().brakeIntensity, 90) << "the hub's own ceiling";
    EXPECT_TRUE(link.hub.outputs().cableMagnet);
    arbiter.join();
    std::this_thread::sleep_for(100ms);
    const std::string log = readAll(link.logPath);
    EXPECT_NE(log.find("ACK_STATUS"), std::string::npos);
    EXPECT_NE(log.find("applied=90"), std::string::npos) << log;
}

// The arbiter stops renewing (it hung, or it decided nothing and went quiet): the zeroed command
// and the short hold release the brake well within half a second, with the link still healthy.
TEST(VehicleInterfaceE2E, AQuietArbiterReleasesTheBrake) {
    Link link("quiet");
    waitFor([&] { return link.hub.outputs().armed; }, 1000ms);
    link.hold(request(schema::RequestType_BRAKE, 60), 300ms);
    ASSERT_EQ(link.hub.outputs().brakeIntensity, 60);
    const auto t = waitFor([&] { return link.hub.outputs().brakeIntensity == 0; }, 800ms);
    std::printf("quiet arbiter: brake released %lld ms after the last request\n",
                static_cast<long long>(t.count()));
    EXPECT_LT(t, 400ms) << "released after " << t.count() << " ms";
    EXPECT_TRUE(link.hub.outputs().armed) << "still armed: this was a release, not a link loss";
    EXPECT_GT(link.vi->stats().zeroedSent, 0u);
}

// The host stops altogether (no heartbeats): the firmware's watchdog releases within 200 ms
// (plus this machine's scheduling margin) and disarms.
TEST(VehicleInterfaceE2E, HostSilenceTripsTheHubWatchdog) {
    Link link("silence");
    waitFor([&] { return link.hub.outputs().armed; }, 1000ms);
    std::thread arbiter([&] { link.hold(request(schema::RequestType_BRAKE, 60), 200ms); });
    arbiter.join();
    link.vi->sendActuationRequest(request(schema::RequestType_BRAKE, 60));
    std::this_thread::sleep_for(20ms);
    link.stopHost();
    const auto t = waitFor([&] { return !link.hub.outputs().armed; }, 1000ms);
    std::printf("host silent: hub disarmed and released %lld ms after the host stopped\n",
                static_cast<long long>(t.count()));
    EXPECT_LT(t, 300ms) << "disarmed after " << t.count() << " ms";
    EXPECT_EQ(link.hub.outputs().brakeIntensity, 0);
    EXPECT_EQ(link.hub.outputs().faultCode, 3) << "link timeout";
}

// Part 5.4: after every thread has stopped, the final zeroed command releases at once.
TEST(VehicleInterfaceE2E, TheFinalZeroedCommandReleasesImmediately) {
    Link link("final");
    waitFor([&] { return link.hub.outputs().armed; }, 1000ms);
    link.hold(request(schema::RequestType_BRAKE, 60), 200ms);
    link.vi->sendActuationRequest(request(schema::RequestType_BRAKE, 60));
    std::this_thread::sleep_for(15ms);
    link.stopHost();
    ASSERT_EQ(link.hub.outputs().brakeIntensity, 60);
    link.vi->sendFinalZeroedCommand();
    const auto t = waitFor([&] { return link.hub.outputs().brakeIntensity == 0; }, 500ms);
    std::printf("final zeroed command: released %lld ms after it was sent\n",
                static_cast<long long>(t.count()));
    EXPECT_LT(t, 100ms) << "released after " << t.count() << " ms";
    EXPECT_NE(readAll(link.logPath).find("final zeroed ActuationCommand"), std::string::npos);
}

// The two-box hub: the power box unplugged. The hub releases, reports fault 5, and the host sees
// it in HubState (the arbiter then cannot arm) and logs it by name.
TEST(VehicleInterfaceE2E, PowerBoxUnpluggedReachesTheHostAsFault5) {
    Link link("box");
    waitFor([&] { return link.hub.outputs().armed; }, 1000ms);
    link.hub.setPowerBoxPresent(false);
    EXPECT_LT(waitFor([&] { return link.vi->hubState().actuatorFaultCode == 5; }, 600ms), 600ms);
    EXPECT_FALSE(link.hub.outputs().armed);
    std::this_thread::sleep_for(50ms);
    EXPECT_NE(readAll(link.logPath).find("fault_name=POWER_BOX"), std::string::npos);
    link.hub.setPowerBoxPresent(true);
    EXPECT_LT(waitFor([&] { return link.hub.outputs().armed; }, 600ms), 600ms)
        << "re-arms on heartbeats";
    EXPECT_LT(waitFor([&] { return link.vi->hubState().actuatorFaultCode == 0; }, 600ms), 600ms);
}

TEST(VehicleInterfaceE2E, GarbageOnTheLineIsSurvived) {
    Link link("garbage");
    waitFor([&] { return link.hub.outputs().armed; }, 1000ms);
    std::mt19937 rng(3);
    for (int i = 0; i < 20; ++i) {
        std::vector<std::uint8_t> junk(37);
        for (auto& b : junk) b = static_cast<std::uint8_t>(rng());
        junk[5] = 0xAA;  // false starts
        link.hub.injectBytes(junk);
        std::this_thread::sleep_for(10ms);
    }
    const int before = link.reports.load();
    std::this_thread::sleep_for(300ms);
    EXPECT_GT(link.reports.load() - before, 8) << "reports keep flowing";
    EXPECT_TRUE(link.hub.outputs().armed);
}

TEST(VehicleInterfaceE2E, AReportGapIsLoggedAndItsEnd) {
    Link link("gap");
    waitFor([&] { return link.reports.load() > 5; }, 1000ms);
    link.hub.setReportsPaused(true);
    std::this_thread::sleep_for(400ms);
    link.hub.setReportsPaused(false);
    std::this_thread::sleep_for(150ms);
    const std::string log = readAll(link.logPath);
    EXPECT_NE(log.find("no SensorReport for over 200 ms"), std::string::npos) << log;
    EXPECT_NE(log.find("SensorReports resumed"), std::string::npos) << log;
}

// The whole chain: reports through VehicleInterface into HubReportFeeder and the EKF.
TEST(VehicleInterfaceE2E, ReportsDriveTheEkf) {
    SensorFusion fusion;
    HubReportFeeder feeder(fusion);
    std::mutex m;
    Link link("ekf", [&](const hub_protocol::SensorReport& r, std::uint64_t) {
        std::lock_guard<std::mutex> lock(m);
        feeder.feed(r);
    });
    std::this_thread::sleep_for(1500ms);
    std::lock_guard<std::mutex> lock(m);
    ASSERT_TRUE(fusion.initialised());
    EXPECT_NEAR(fusion.state()(SensorFusion::V), 10.0, 0.5) << "36 km/h from OBD";
    EXPECT_NEAR(fusion.state()(SensorFusion::PSI), 0.0, 0.2) << "driving east";
}
