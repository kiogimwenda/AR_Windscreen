#pragma once
// SystemManager — owns config load, the EventLog reference, and the lifecycle of every subsystem
// thread. See docs/BUILD_GUIDE.md Part 5.1, Part 5.4, Part 11.5.
//
// ---------------------------------------------------------------------------------------------
// The subsystem contract
//
// start<T>(args...) constructs a T from args and runs T::run(stop) on a new thread. `stop` is a
// const std::atomic<bool>& that becomes true when shutdown begins. run() must return promptly
// once it does, so every subsystem loop checks it at least once per iteration. SystemManager
// owns the T object until SystemManager itself is destroyed, which outlives the thread.
//
// If run() throws, the exception is caught on that thread, logged as an EventLog FAULT, and the
// WHOLE system shuts down. A pipeline missing one stage (say, fusion) is not a degraded system
// that can keep requesting actuation. It is a broken one, and the conservative response is to
// stop, which leaves the hub's watchdog to release everything.
//
// ---------------------------------------------------------------------------------------------
// Shutdown ordering, and the final zeroed ActuationCommand (Part 5.4)
//
// On every shutdown path, SystemManager:
//   1. sets `stop`, which every subsystem's run() sees;
//   2. joins every subsystem thread, in the order they were started;
//   3. only then calls the final-command hook (VehicleInterface registers one: send a zeroed
//      ActuationCommand, then close the port).
//
// Step 3 comes after ALL joins for a reason. Once every thread has returned, nothing in the
// process can produce a new actuation request, so the zeroed command is guaranteed to be the
// last thing the hub hears from the host. Called any earlier, a still-running DecisionArbiter
// could slip a brake request in behind it. The hub may lose its heartbeat during step 2 and trip
// its watchdog. That is harmless, because the watchdog releases, which is the safe direction.
//
// ---------------------------------------------------------------------------------------------
// Signals: what a handler may and may not do
//
// A signal (Ctrl+C = SIGINT, `kill` = SIGTERM, a segfault = SIGSEGV) interrupts a thread at an
// arbitrary instruction and runs the handler on top of it. The thread could be halfway through
// malloc(), holding a mutex, or mid-way through printing. If the handler then calls malloc(),
// locks that mutex, or prints, it can deadlock or corrupt state. POSIX therefore lists a small set
// of "async-signal-safe" functions that are the only things a handler may call (write(), _exit(),
// raise(), and a few others). Lock-free std::atomic loads and stores are also safe.
//
// So the handlers here do almost nothing:
//   - SIGINT/SIGTERM: set a lock-free atomic flag. runUntilShutdown() polls it every 50 ms and
//     does the real shutdown work in normal code, where anything is allowed. A second
//     SIGINT/SIGTERM while shutdown is already underway means shutdown is stuck. The handler then
//     does the crash action below and exits immediately.
//   - SIGSEGV/SIGBUS/SIGFPE/SIGILL/SIGABRT (a crash): the process is already corrupt, so
//     no ordered shutdown is possible. The one useful safe action is a single write() of a
//     PRE-ENCODED zeroed ActuationCommand frame to the serial fd. VehicleInterface registers
//     those bytes and that fd with registerCrashFrame() when it opens the port. The handler
//     then re-raises the signal so the process still dies with its real cause and core dump.
//     This is the belt-and-braces measure from Part 5.4. The hub's watchdog does not depend on
//     it.
// ---------------------------------------------------------------------------------------------

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include "ar_drive_assist/common/Types.h"
#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {

class SystemManager {
public:
    // Reads vehicle_params.yaml and decision_thresholds.yaml from configDir. Throws
    // std::runtime_error naming the file and key for anything missing, malformed or out of range.
    // A system must never start on a silently defaulted threshold.
    static Config loadConfig(const std::string& configDir);

    // Installs the SIGINT/SIGTERM and crash handlers described above. Process-wide; call once,
    // from main, before start().
    static void installSignalHandlers();

    // Registers the bytes the crash handler writes, and the fd it writes them to. Pass fd = -1 to
    // unregister, e.g. before the port is closed. `len` must not exceed
    // hub_protocol::MAX_FRAME_SIZE.
    static void registerCrashFrame(int fd, const std::uint8_t* frame, std::size_t len);

    SystemManager(const Config& config, EventLog& log);
    ~SystemManager();  // performs the full shutdown if runUntilShutdown() never ran

    SystemManager(const SystemManager&) = delete;
    SystemManager& operator=(const SystemManager&) = delete;

    const Config& config() const { return config_; }

    template <typename T, typename... Args>
    T& start(Args&&... args);

    // Called once, after every subsystem thread has been joined. See "Shutdown ordering" above.
    void setFinalCommandHook(std::function<void()> hook);

    // Blocks until shutdown is requested (requestShutdown(), a handled signal, or a subsystem
    // failure), then performs the ordered shutdown. Returns once it is complete.
    void runUntilShutdown();

    // Thread-safe and idempotent. Callable from any subsystem thread, e.g. on window close.
    // NOT callable from a signal handler; the handlers use their own flag.
    void requestShutdown();

private:
    void runGuarded(const std::string& name, const std::function<void()>& body);
    void shutdown();
    static std::string demangle(const char* mangled);

    Config config_;
    EventLog& log_;

    std::atomic<bool> stop_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
    bool shutdownRequested_ = false;
    bool shutdownDone_ = false;

    std::function<void()> finalCommandHook_;
    std::vector<std::shared_ptr<void>> subsystems_;  // type-erased owners, kept until destruction
    std::vector<std::thread> threads_;
    std::vector<std::string> names_;
};

template <typename T, typename... Args>
T& SystemManager::start(Args&&... args) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdownRequested_) throw std::logic_error("SystemManager::start after shutdown");
    }
    // shared_ptr<void> remembers T's real destructor, so ~T() runs correctly even through the
    // type-erased pointer. unique_ptr<void> cannot do that.
    auto owner = std::make_shared<T>(std::forward<Args>(args)...);
    T& subsystem = *owner;
    const std::string name = demangle(typeid(T).name());

    subsystems_.push_back(owner);
    names_.push_back(name);
    log_.logGeneral("starting " + name);
    threads_.emplace_back(
        [this, &subsystem, name] { runGuarded(name, [&] { subsystem.run(stop_); }); });
    return subsystem;
}

}  // namespace ar_drive_assist
