// SystemManager unit tests — see docs/BUILD_GUIDE.md Part 5.4, Part 11.5.
//
// Three groups:
//   - loadConfig: the real host/config files load, and every class of bad value is rejected
//     with the file and key named.
//   - lifecycle: subsystems see `stop`, a failing subsystem takes the system down, and the
//     final-command hook runs exactly once, after every subsystem has returned.
//   - signals: in a forked child, so installing process-wide handlers cannot leak into other
//     tests. SIGINT must trigger an ordered shutdown, and a crash signal must write the
//     registered frame and still kill the process with that signal.

#include <gtest/gtest.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ar_drive_assist/system/SystemManager.h"

using ar_drive_assist::Config;
using ar_drive_assist::EventLog;
using ar_drive_assist::SystemManager;

namespace {

const std::string kRealConfigDir = std::string(HOST_SOURCE_DIR) + "/config";

std::string tempDir(const std::string& name) {
    std::string d = testing::TempDir() + "sysmgr_" + name + "_" + std::to_string(::getpid());
    std::string cmd = "rm -rf '" + d + "' && mkdir -p '" + d + "'";
    if (std::system(cmd.c_str()) != 0) throw std::runtime_error("cannot create " + d);
    return d;
}

void writeFile(const std::string& path, const std::string& text) {
    std::ofstream(path) << text;
}

const char* kVehicle =
    "wheelbase_m: 2.6\ncamera_height_m: 1.3\nobd_pid_brake_active: \"\"\n"
    "serial_device: \"/dev/ttyACM0\"\nserial_baud: 115200\n";
const char* kDecision =
    "ttc_brake_threshold_s: 1.8\nhard_brake_decel_g: 0.4\ntailgating_min_gap_s: 1.0\n"
    "brake_actuator_max_intensity: 90\n";

// Writes a valid config pair, with `key` in `file` replaced by `replacement` (or removed if
// replacement is empty).
std::string configWith(const std::string& name, const std::string& file, const std::string& key,
                       const std::string& replacement) {
    const auto dir = tempDir(name);
    for (auto [fname, body] : {std::pair<std::string, std::string>{"vehicle_params.yaml", kVehicle},
                               {"decision_thresholds.yaml", kDecision}}) {
        if (fname == file) {
            std::string out;
            std::istringstream in(body);
            for (std::string line; std::getline(in, line);) {
                if (line.rfind(key + ":", 0) == 0) {
                    if (!replacement.empty()) out += key + ": " + replacement + "\n";
                } else {
                    out += line + "\n";
                }
            }
            body = out;
        }
        writeFile(dir + "/" + fname, body);
    }
    return dir;
}

std::string loadError(const std::string& dir) {
    try {
        SystemManager::loadConfig(dir);
    } catch (const std::runtime_error& e) {
        return e.what();
    }
    return "";
}

// A minimal subsystem obeying the contract: loop until `stop`.
struct CountingSubsystem {
    std::atomic<int>& running;
    std::atomic<int>& finished;
    CountingSubsystem(std::atomic<int>& r, std::atomic<int>& f) : running(r), finished(f) {}
    void run(const std::atomic<bool>& stop) {
        ++running;
        while (!stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ++finished;
    }
};

struct ThrowingSubsystem {
    void run(const std::atomic<bool>&) { throw std::runtime_error("sensor exploded"); }
};

struct EarlyReturnSubsystem {
    void run(const std::atomic<bool>&) {}
};

std::string readAll(const std::string& path) {
    std::ifstream in(path);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

// --- loadConfig -------------------------------------------------------------------------------

TEST(SystemManagerConfig, LoadsTheRealConfigFiles) {
    const Config cfg = SystemManager::loadConfig(kRealConfigDir);
    EXPECT_DOUBLE_EQ(cfg.vehicle.wheelbaseM, 2.6);
    EXPECT_DOUBLE_EQ(cfg.vehicle.cameraHeightM, 1.3);
    EXPECT_EQ(cfg.vehicle.obdPidBrakeActive, "");
    EXPECT_EQ(cfg.vehicle.serialDevice, "/dev/ttyACM0");
    EXPECT_EQ(cfg.vehicle.serialBaud, 115200);
    EXPECT_DOUBLE_EQ(cfg.decision.ttcBrakeThresholdS, 1.8);
    EXPECT_DOUBLE_EQ(cfg.decision.hardBrakeDecelG, 0.4);
    EXPECT_DOUBLE_EQ(cfg.decision.tailgatingMinGapS, 1.0);
    EXPECT_EQ(cfg.decision.brakeActuatorMaxIntensity, 90);
}

TEST(SystemManagerConfig, MissingFileNamesTheFile) {
    const auto dir = tempDir("nofile");
    EXPECT_NE(loadError(dir).find("vehicle_params.yaml"), std::string::npos);
}

TEST(SystemManagerConfig, MissingKeyNamesFileAndKey) {
    const auto err =
        loadError(configWith("nokey", "decision_thresholds.yaml", "ttc_brake_threshold_s", ""));
    EXPECT_NE(err.find("decision_thresholds.yaml"), std::string::npos) << err;
    EXPECT_NE(err.find("'ttc_brake_threshold_s' missing"), std::string::npos) << err;
}

TEST(SystemManagerConfig, RejectsBrakeCeilingOutOfRange) {
    for (const char* bad : {"300", "256", "-1"}) {
        const auto err = loadError(
            configWith("ceiling", "decision_thresholds.yaml", "brake_actuator_max_intensity", bad));
        EXPECT_NE(err.find("brake_actuator_max_intensity"), std::string::npos)
            << bad << ": " << err;
    }
}

TEST(SystemManagerConfig, RejectsNonIntegerBrakeCeiling) {
    for (const char* bad : {"90.5", "\"high\"", "[90]"}) {
        const auto err = loadError(configWith("ceilingtype", "decision_thresholds.yaml",
                                              "brake_actuator_max_intensity", bad));
        EXPECT_NE(err.find("wrong type"), std::string::npos) << bad << ": " << err;
    }
}

TEST(SystemManagerConfig, AcceptsBrakeCeilingBounds) {
    for (const char* ok : {"0", "255"}) {
        const auto dir =
            configWith("bounds", "decision_thresholds.yaml", "brake_actuator_max_intensity", ok);
        EXPECT_EQ(loadError(dir), "") << ok;
    }
}

TEST(SystemManagerConfig, RejectsNonPositiveThresholds) {
    for (const char* bad : {"0", "-1.8"}) {
        const auto err =
            loadError(configWith("ttc", "decision_thresholds.yaml", "ttc_brake_threshold_s", bad));
        EXPECT_NE(err.find("must be > 0"), std::string::npos) << bad << ": " << err;
    }
}

TEST(SystemManagerConfig, RejectsEmptySerialDevice) {
    const auto err =
        loadError(configWith("serial", "vehicle_params.yaml", "serial_device", "\"\""));
    EXPECT_NE(err.find("'serial_device' must not be empty"), std::string::npos) << err;
}

// --- lifecycle --------------------------------------------------------------------------------

class SystemManagerLifecycle : public testing::Test {
protected:
    std::string logPath = tempDir("life") + "/session.log";
    EventLog log{logPath};
    Config cfg = SystemManager::loadConfig(kRealConfigDir);
};

TEST_F(SystemManagerLifecycle, RequestShutdownStopsEverySubsystem) {
    std::atomic<int> running{0}, finished{0};
    SystemManager mgr(cfg, log);
    mgr.start<CountingSubsystem>(running, finished);
    mgr.start<CountingSubsystem>(running, finished);

    std::thread stopper([&] {
        while (running.load() < 2) std::this_thread::yield();
        mgr.requestShutdown();
    });
    mgr.runUntilShutdown();
    stopper.join();

    EXPECT_EQ(finished.load(), 2);
}

// The Part 5.4 ordering property: when the hook runs, every subsystem has already returned, so
// nothing can send an actuation request after the final zeroed command.
TEST_F(SystemManagerLifecycle, FinalCommandHookRunsOnceAfterAllSubsystemsReturned) {
    std::atomic<int> running{0}, finished{0};
    int hookCalls = 0;
    int finishedWhenHookRan = -1;
    {
        SystemManager mgr(cfg, log);
        mgr.start<CountingSubsystem>(running, finished);
        mgr.start<CountingSubsystem>(running, finished);
        mgr.start<CountingSubsystem>(running, finished);
        mgr.setFinalCommandHook([&] {
            ++hookCalls;
            finishedWhenHookRan = finished.load();
        });
        mgr.requestShutdown();
        mgr.requestShutdown();  // idempotent
        mgr.runUntilShutdown();
    }  // the destructor must not run the hook a second time
    EXPECT_EQ(hookCalls, 1);
    EXPECT_EQ(finishedWhenHookRan, 3);
}

// If main never reaches runUntilShutdown() (an exception during startup, say), the destructor
// must still stop the threads and send the final command. Otherwise ~std::thread on a joinable
// thread calls std::terminate.
TEST_F(SystemManagerLifecycle, DestructorShutsDownIfRunUntilShutdownNeverRan) {
    std::atomic<int> running{0}, finished{0};
    int hookCalls = 0;
    {
        SystemManager mgr(cfg, log);
        mgr.start<CountingSubsystem>(running, finished);
        mgr.setFinalCommandHook([&] { ++hookCalls; });
        while (running.load() < 1) std::this_thread::yield();
    }
    EXPECT_EQ(finished.load(), 1);
    EXPECT_EQ(hookCalls, 1);
}

TEST_F(SystemManagerLifecycle, ThrowingSubsystemIsLoggedAndShutsTheSystemDown) {
    std::atomic<int> running{0}, finished{0};
    int hookCalls = 0;
    SystemManager mgr(cfg, log);
    mgr.start<CountingSubsystem>(running, finished);
    mgr.start<ThrowingSubsystem>();
    mgr.setFinalCommandHook([&] { ++hookCalls; });
    mgr.runUntilShutdown();  // returns with nobody calling requestShutdown()

    EXPECT_EQ(finished.load(), 1);
    EXPECT_EQ(hookCalls, 1);
    const auto text = readAll(logPath);
    // The component is the demangled, fully qualified type name, e.g.
    // "(anonymous namespace)::ThrowingSubsystem" here, "ar_drive_assist::CameraPipeline" for real.
    EXPECT_NE(text.find("ThrowingSubsystem\" desc=\"run() threw: sensor exploded - shutting "
                        "down\""),
              std::string::npos)
        << text;
}

TEST_F(SystemManagerLifecycle, SubsystemReturningEarlyIsAFault) {
    SystemManager mgr(cfg, log);
    mgr.start<EarlyReturnSubsystem>();
    mgr.runUntilShutdown();
    EXPECT_NE(readAll(logPath).find("returned before shutdown was requested"), std::string::npos);
}

TEST_F(SystemManagerLifecycle, StartAfterShutdownThrows) {
    std::atomic<int> running{0}, finished{0};
    SystemManager mgr(cfg, log);
    mgr.requestShutdown();
    EXPECT_THROW(mgr.start<CountingSubsystem>(running, finished), std::logic_error);
}

TEST(SystemManagerCrashFrame, RejectsOversizedFrame) {
    std::vector<std::uint8_t> big(hub_protocol::MAX_FRAME_SIZE + 1, 0);
    EXPECT_THROW(SystemManager::registerCrashFrame(1, big.data(), big.size()),
                 std::invalid_argument);
}

// --- signals (forked children) ----------------------------------------------------------------

TEST(SystemManagerSignals, SigintTriggersOrderedShutdownWithFinalCommand) {
    const auto logPath = tempDir("sigint") + "/session.log";
    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        SystemManager::installSignalHandlers();
        EventLog log(logPath);
        std::atomic<int> running{0}, finished{0};
        bool hookRan = false;
        {
            SystemManager mgr(SystemManager::loadConfig(kRealConfigDir), log);
            mgr.start<CountingSubsystem>(running, finished);
            mgr.setFinalCommandHook([&] { hookRan = true; });
            while (running.load() < 1) std::this_thread::yield();
            ::raise(SIGINT);
            mgr.runUntilShutdown();
        }
        ::_exit(finished.load() == 1 && hookRan ? 0 : 1);
    }
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status)) << "child was killed instead of shutting down cleanly";
    EXPECT_EQ(WEXITSTATUS(status), 0);
    EXPECT_NE(readAll(logPath).find("shutdown requested by signal"), std::string::npos);
}

// A crash cannot run an ordered shutdown, but the handler must still get the pre-encoded
// zeroed command out, and must not swallow the crash. A pipe stands in for the serial port.
TEST(SystemManagerSignals, CrashSignalWritesRegisteredFrameAndStillDies) {
    int fds[2];
    ASSERT_EQ(::pipe(fds), 0);
    const std::uint8_t frame[] = {0xAA, 0x02, 0x09, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xBE, 0xEF};

    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        ::close(fds[0]);
        SystemManager::installSignalHandlers();
        SystemManager::registerCrashFrame(fds[1], frame, sizeof frame);
        ::raise(SIGSEGV);
        ::_exit(0);  // reaching here means the crash was swallowed
    }
    ::close(fds[1]);
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFSIGNALED(status)) << "the crash handler swallowed the crash";
    EXPECT_EQ(WTERMSIG(status), SIGSEGV);

    std::uint8_t got[64];
    const ssize_t n = ::read(fds[0], got, sizeof got);
    ::close(fds[0]);
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof frame));
    EXPECT_EQ(std::memcmp(got, frame, sizeof frame), 0);
}

// Unregistering (fd = -1, as VehicleInterface will do before closing the port) means a later
// crash writes nothing to a descriptor number that may by then belong to something else.
TEST(SystemManagerSignals, UnregisteredCrashFrameWritesNothing) {
    int fds[2];
    ASSERT_EQ(::pipe(fds), 0);
    const std::uint8_t frame[] = {0xAA, 0x04, 0x00, 0x00, 0x12, 0x34};

    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        ::close(fds[0]);
        SystemManager::installSignalHandlers();
        SystemManager::registerCrashFrame(fds[1], frame, sizeof frame);
        SystemManager::registerCrashFrame(-1, nullptr, 0);
        ::raise(SIGABRT);
        ::_exit(0);
    }
    ::close(fds[1]);
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFSIGNALED(status));
    EXPECT_EQ(WTERMSIG(status), SIGABRT);
    std::uint8_t got[16];
    EXPECT_EQ(::read(fds[0], got, sizeof got), 0);  // EOF, nothing written
    ::close(fds[0]);
}

// A subsystem that never honours `stop` hangs the ordered shutdown in join(). A second
// Ctrl+C must then leave at once, and must still get the zeroed frame out on its way.
struct StuckSubsystem {
    std::atomic<bool>& entered;
    explicit StuckSubsystem(std::atomic<bool>& e) : entered(e) {}
    void run(const std::atomic<bool>&) {
        entered = true;
        for (;;) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};

TEST(SystemManagerSignals, SecondSigintDuringStuckShutdownWritesFrameAndExits) {
    int fds[2];
    ASSERT_EQ(::pipe(fds), 0);
    const std::uint8_t frame[] = {0xAA, 0x02, 0x09, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x12, 0x34};
    const auto logPath = tempDir("stuck") + "/session.log";

    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        ::close(fds[0]);
        SystemManager::installSignalHandlers();
        SystemManager::registerCrashFrame(fds[1], frame, sizeof frame);
        EventLog log(logPath);
        std::atomic<bool> entered{false};
        SystemManager mgr(SystemManager::loadConfig(kRealConfigDir), log);
        mgr.start<StuckSubsystem>(entered);
        while (!entered.load()) std::this_thread::yield();
        ::raise(SIGINT);         // first: begins ordered shutdown, which will hang in join()
        mgr.runUntilShutdown();  // never returns
        ::_exit(0);
    }
    ::close(fds[1]);

    // Wait until the child's log shows shutdown has begun, i.e. it is now stuck in join().
    for (int i = 0; i < 200 && readAll(logPath).find("stopping 1 subsystem") == std::string::npos;
         ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_NE(readAll(logPath).find("stopping 1 subsystem"), std::string::npos);
    ::kill(child, SIGINT);  // second

    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 128 + SIGINT);

    std::uint8_t got[64];
    const ssize_t n = ::read(fds[0], got, sizeof got);
    ::close(fds[0]);
    ASSERT_EQ(n, static_cast<ssize_t>(sizeof frame));
    EXPECT_EQ(std::memcmp(got, frame, sizeof frame), 0);
}
