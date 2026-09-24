// Host application entry point — see docs/BUILD_GUIDE.md Part 5.4.
//
// Loads config, opens the EventLog, then starts one thread per subsystem through SystemManager and
// blocks in runUntilShutdown(). Ctrl+C or window close triggers an ordered shutdown, which sends
// one final zeroed ActuationCommand before the serial port closes (Part 5.4, and the ordering
// argument in SystemManager.h).
//
// No subsystem exists yet. Each phase adds its own mgr.start<...>() line below, in Part 5.4's
// order. Run from the repository root:
//     build/host/ar_drive_assist [config_dir] [log_path]
// The defaults are host/config and logs/session.log. The log is appended to, and each run begins
// with a SESSION_START line.

#include <cstdio>
#include <exception>
#include <string>

#include "ar_drive_assist/system/EventLog.h"
#include "ar_drive_assist/system/SystemManager.h"

using ar_drive_assist::EventLog;
using ar_drive_assist::SystemManager;

int main(int argc, char** argv) {
    const std::string configDir = argc > 1 ? argv[1] : "host/config";
    const std::string logPath = argc > 2 ? argv[2] : "logs/session.log";

    // Installed first, so even a crash during startup gets the crash handler.
    SystemManager::installSignalHandlers();

    try {
        const auto config = SystemManager::loadConfig(configDir);
        EventLog log(logPath);
        SystemManager mgr(config, log);

        // Phase 4:  mgr.start<CameraPipeline>(...);
        // Phase 5:  mgr.start<MlInferenceEngine>(...);
        // Phase 6:  mgr.start<LidarProcessor>(...);
        // Phase 7:  mgr.start<SensorFusion>(...);
        // Phase 10: mgr.start<DecisionArbiter>(...);
        // Phase 8:  mgr.start<NavigationEngine>(...);
        // Phase 11: mgr.start<ArRenderer>(windowedSink, ...);
        // Phase 3:  mgr.start<VehicleInterface>(...);  (needs the Phase 2 hub; also registers
        //           the final-command hook and the crash frame)

        log.logGeneral("host running; Ctrl+C to stop");
        std::fprintf(stderr, "ar_drive_assist: running, logging to %s (Ctrl+C to stop)\n",
                     logPath.c_str());
        mgr.runUntilShutdown();
    } catch (const std::exception& e) {
        // Config or log failure: refuse to start. Nothing has been started, so there is nothing
        // to actuate and nothing to shut down.
        std::fprintf(stderr, "ar_drive_assist: startup failed: %s\n", e.what());
        return 1;
    }
    return 0;
}
