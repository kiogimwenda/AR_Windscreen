// Host application entry point — see docs/BUILD_GUIDE.md Part 5.4.
//
// Loads config, opens the EventLog, then starts one thread per subsystem through SystemManager and
// blocks in runUntilShutdown(). Ctrl+C or window close triggers an ordered shutdown, which sends
// one final zeroed ActuationCommand before the serial port closes (Part 5.4, and the ordering
// argument in SystemManager.h).
//
// Run from the repository root:
//     build/host/ar_drive_assist [config_dir] [log_path] [--serial <device>] [--no-hub]
// The defaults are host/config, logs/session.log and vehicle_params.yaml's serial_device. The log
// is appended to, and each run begins with a SESSION_START line.
//   --serial <device>  overrides serial_device, e.g. the pseudo-terminal hub_sim prints;
//   --no-hub           starts without the hub (development only: nothing can be actuated).
// Without a reachable hub (and without --no-hub) startup fails, naming the device: no hub, no
// system.

#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ar_drive_assist/fusion/HubReportFeeder.h"
#include "ar_drive_assist/fusion/SensorFusion.h"
#include "ar_drive_assist/system/EventLog.h"
#include "ar_drive_assist/system/SystemManager.h"
#include "ar_drive_assist/vehicle/VehicleInterface.h"

using namespace ar_drive_assist;

int main(int argc, char** argv) {
    std::vector<std::string> positional;
    std::string serialOverride;
    bool noHub = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--serial") && i + 1 < argc) {
            serialOverride = argv[++i];
        } else if (!std::strcmp(argv[i], "--no-hub")) {
            noHub = true;
        } else {
            positional.emplace_back(argv[i]);
        }
    }
    const std::string configDir = positional.size() > 0 ? positional[0] : "host/config";
    const std::string logPath = positional.size() > 1 ? positional[1] : "logs/session.log";

    // Installed first, so even a crash during startup gets the crash handler.
    SystemManager::installSignalHandlers();

    try {
        const auto config = SystemManager::loadConfig(configDir);
        EventLog log(logPath);

        // Ego-motion: every hub SensorReport goes through HubReportFeeder into the EKF (Part 8.2).
        // Until Part 5.1's FusionThread exists, the feeding runs on VehicleInterface's thread,
        // under this mutex; readers of the EKF take the same mutex. Declared BEFORE the
        // SystemManager, so they outlive every subsystem thread it owns.
        SensorFusion fusion;
        FeederConfig feederCfg;
        feederCfg.mount = ImuMount::fromRollPitchYawDeg(config.vehicle.imuMountRpyDeg[0],
                                                        config.vehicle.imuMountRpyDeg[1],
                                                        config.vehicle.imuMountRpyDeg[2]);
        HubReportFeeder feeder(fusion, feederCfg);
        std::mutex fusionMutex;

        SystemManager mgr(config, log);

        // Phase 4:  mgr.start<CameraPipeline>(...);
        // Phase 5:  mgr.start<MlInferenceEngine>(...);
        // Phase 6:  mgr.start<LidarProcessor>(...);
        // Phase 7:  FusionThread (feeder above), tracker, motion predictor
        // Phase 10: mgr.start<DecisionArbiter>(...);   -> vehicle.sendActuationRequest()
        // Phase 8:  mgr.start<NavigationEngine>(...);
        // Phase 11: mgr.start<ArRenderer>(windowedSink, ...);
        if (!noHub) {
            VehicleInterfaceOptions vio;
            vio.device = serialOverride.empty() ? config.vehicle.serialDevice : serialOverride;
            vio.baud = config.vehicle.serialBaud;
            vio.openOnConstruction = true;
            auto& vehicle = mgr.start<VehicleInterface>(
                vio, log, [&](const hub_protocol::SensorReport& r, std::uint64_t) {
                    std::lock_guard<std::mutex> lock(fusionMutex);
                    feeder.feed(r);
                });
            // Part 5.4: after every thread is joined, the last word to the hub is "release".
            mgr.setFinalCommandHook([&vehicle] {
                vehicle.sendFinalZeroedCommand();
                vehicle.close();
            });
        } else {
            log.logGeneral("started with --no-hub: nothing can be actuated");
        }

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
