#include "ar_drive_assist/system/SystemManager.h"

#include <cxxabi.h>
#include <signal.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "ar_drive_assist/vehicle/HubProtocol.h"

namespace ar_drive_assist {

// =============================================================================================
// Signal handling. Everything a handler touches lives at file scope as a lock-free atomic or a
// plain byte array, and the handlers call only async-signal-safe functions (see the header).
// =============================================================================================
namespace {

static_assert(std::atomic<bool>::is_always_lock_free, "signal flag must be lock-free");
static_assert(std::atomic<int>::is_always_lock_free, "crash fd must be lock-free");
static_assert(std::atomic<std::size_t>::is_always_lock_free, "crash len must be lock-free");

std::atomic<bool> gSignalShutdown{false};

// The crash frame. registerCrashFrame() writes the bytes, THEN publishes the length with a
// release store. The handler acquire-loads the length first, so if it sees a non-zero length the
// bytes are complete. This is the same publication pattern as RingBuffer's head_.
std::uint8_t gCrashFrame[hub_protocol::MAX_FRAME_SIZE];
std::atomic<std::size_t> gCrashFrameLen{0};
std::atomic<int> gCrashFd{-1};

void writeCrashFrame() {
    const std::size_t len = gCrashFrameLen.load(std::memory_order_acquire);
    const int fd = gCrashFd.load(std::memory_order_acquire);
    if (fd < 0 || len == 0) return;
    // One attempt only: no retry loop inside a handler for a process that is already failing.
    // The return value is deliberately ignored, since there is nothing safe to do on failure.
    [[maybe_unused]] const ssize_t n = ::write(fd, gCrashFrame, len);
}

void onShutdownSignal(int sig) {
    if (gSignalShutdown.exchange(true)) {
        // Second signal: the ordered shutdown is stuck. Do the crash action and leave now.
        writeCrashFrame();
        ::_exit(128 + sig);
    }
}

void onCrashSignal(int sig) {
    writeCrashFrame();
    // SA_RESETHAND has already restored the default action. The signal is blocked while this
    // handler runs, so raise() leaves it pending, and it is delivered with the default action (die,
    // core dump) the moment this handler returns.
    ::raise(sig);
}

void install(int sig, void (*handler)(int), int flags) {
    struct sigaction sa{};
    sa.sa_handler = handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = flags;
    ::sigaction(sig, &sa, nullptr);
}

}  // namespace

void SystemManager::installSignalHandlers() {
    // SA_RESTART: a blocking read() interrupted by Ctrl+C resumes instead of failing with EINTR,
    // so subsystem I/O code does not need an EINTR check on every call just for this.
    install(SIGINT, onShutdownSignal, SA_RESTART);
    install(SIGTERM, onShutdownSignal, SA_RESTART);
    for (int sig : {SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT}) {
        install(sig, onCrashSignal, SA_RESETHAND);
    }
}

void SystemManager::registerCrashFrame(int fd, const std::uint8_t* frame, std::size_t len) {
    if (fd >= 0 && (frame == nullptr || len == 0 || len > sizeof gCrashFrame)) {
        throw std::invalid_argument("registerCrashFrame: frame must be 1.." +
                                    std::to_string(sizeof gCrashFrame) + " bytes");
    }
    // Unpublish first, so a crash arriving mid-update never writes half-old, half-new bytes.
    gCrashFrameLen.store(0, std::memory_order_release);
    gCrashFd.store(-1, std::memory_order_release);
    if (fd < 0) return;
    std::memcpy(gCrashFrame, frame, len);
    gCrashFd.store(fd, std::memory_order_release);
    gCrashFrameLen.store(len, std::memory_order_release);
}

// =============================================================================================
// Config
// =============================================================================================
namespace {

// Every failure names the file and the key, so a bad config is fixed in one look, not by
// bisecting YAML.
class ConfigFile {
public:
    ConfigFile(const std::string& dir, const std::string& name) : path_(dir + "/" + name) {
        try {
            root_ = YAML::LoadFile(path_);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("config: cannot load " + path_ + ": " + e.what());
        }
    }

    template <typename V>
    V get(const std::string& key) const {
        const YAML::Node node = root_[key];
        if (!node || node.IsNull()) fail(key, "missing");
        try {
            return node.as<V>();
        } catch (const YAML::Exception&) {
            fail(key, "has the wrong type (value: " + YAML::Dump(node) + ")");
        }
        return V{};  // unreachable: fail() throws
    }

    double positive(const std::string& key) const {
        const auto v = get<double>(key);
        if (!(v > 0.0)) fail(key, "must be > 0 (value: " + std::to_string(v) + ")");
        return v;
    }

    [[noreturn]] void fail(const std::string& key, const std::string& why) const {
        throw std::runtime_error("config: " + path_ + ": '" + key + "' " + why);
    }

private:
    std::string path_;
    YAML::Node root_;
};

}  // namespace

Config SystemManager::loadConfig(const std::string& configDir) {
    Config cfg;

    const ConfigFile vehicle(configDir, "vehicle_params.yaml");
    cfg.vehicle.wheelbaseM = vehicle.positive("wheelbase_m");
    cfg.vehicle.cameraHeightM = vehicle.positive("camera_height_m");
    // Blank is legitimate (the vehicle does not expose the PID), but the key itself must exist,
    // so that "not exposed" is a recorded decision and not a missing line.
    cfg.vehicle.obdPidBrakeActive = vehicle.get<std::string>("obd_pid_brake_active");
    cfg.vehicle.serialDevice = vehicle.get<std::string>("serial_device");
    if (cfg.vehicle.serialDevice.empty()) vehicle.fail("serial_device", "must not be empty");
    cfg.vehicle.serialBaud = vehicle.get<int>("serial_baud");
    if (cfg.vehicle.serialBaud <= 0) vehicle.fail("serial_baud", "must be > 0");

    const ConfigFile decision(configDir, "decision_thresholds.yaml");
    cfg.decision.ttcBrakeThresholdS = decision.positive("ttc_brake_threshold_s");
    cfg.decision.hardBrakeDecelG = decision.positive("hard_brake_decel_g");
    cfg.decision.tailgatingMinGapS = decision.positive("tailgating_min_gap_s");
    // Read as a wide integer and range-checked explicitly, rather than converted straight to
    // uint8_t, so a value like 300 is rejected with a message instead of depending on how the
    // conversion treats overflow. A brake ceiling is the last value to let through silently wrong.
    const auto maxIntensity = decision.get<long long>("brake_actuator_max_intensity");
    if (maxIntensity < 0 || maxIntensity > std::numeric_limits<std::uint8_t>::max()) {
        decision.fail("brake_actuator_max_intensity",
                      "must be 0..255 (value: " + std::to_string(maxIntensity) + ")");
    }
    cfg.decision.brakeActuatorMaxIntensity = static_cast<std::uint8_t>(maxIntensity);

    return cfg;
}

// =============================================================================================
// Lifecycle
// =============================================================================================

SystemManager::SystemManager(const Config& config, EventLog& log) : config_(config), log_(log) {}

SystemManager::~SystemManager() {
    requestShutdown();
    shutdown();
}

void SystemManager::setFinalCommandHook(std::function<void()> hook) {
    std::lock_guard<std::mutex> lock(mutex_);
    finalCommandHook_ = std::move(hook);
}

void SystemManager::requestShutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdownRequested_ = true;
    }
    cv_.notify_all();
}

void SystemManager::runUntilShutdown() {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        // wait_for rather than wait: a signal handler cannot notify a condition variable, so the
        // signal flag is polled. 50 ms is imperceptible on Ctrl+C.
        while (!shutdownRequested_ && !gSignalShutdown.load()) {
            cv_.wait_for(lock, std::chrono::milliseconds(50));
        }
        if (gSignalShutdown.load()) log_.logGeneral("shutdown requested by signal");
        shutdownRequested_ = true;
    }
    shutdown();
}

void SystemManager::shutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdownDone_) return;
        shutdownDone_ = true;
    }
    log_.logGeneral("shutdown: stopping " + std::to_string(threads_.size()) + " subsystem(s)");
    stop_.store(true);
    for (std::size_t i = 0; i < threads_.size(); ++i) {
        if (threads_[i].joinable()) threads_[i].join();
        log_.logGeneral("stopped " + names_[i]);
    }

    std::function<void()> hook;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        hook = std::move(finalCommandHook_);
    }
    if (hook) {
        try {
            hook();
            log_.logGeneral("final zeroed actuation command sent");
        } catch (const std::exception& e) {
            log_.logFault("SystemManager", std::string("final command hook failed: ") + e.what());
        }
    }
    log_.logGeneral("shutdown complete");
}

void SystemManager::runGuarded(const std::string& name, const std::function<void()>& body) {
    try {
        body();
    } catch (const std::exception& e) {
        log_.logFault(name, std::string("run() threw: ") + e.what() + " - shutting down");
        requestShutdown();
        return;
    } catch (...) {
        log_.logFault(name, "run() threw a non-std exception - shutting down");
        requestShutdown();
        return;
    }
    // run() returning without being asked to is also a failure: that stage of the pipeline is
    // gone.
    if (!stop_.load()) {
        log_.logFault(name, "run() returned before shutdown was requested - shutting down");
        requestShutdown();
    }
}

std::string SystemManager::demangle(const char* mangled) {
    int status = 0;
    char* d = abi::__cxa_demangle(mangled, nullptr, nullptr, &status);
    std::string out = (status == 0 && d) ? d : mangled;
    std::free(d);
    return out;
}

}  // namespace ar_drive_assist
