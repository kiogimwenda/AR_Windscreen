// Host application entry point — see docs/BUILD_GUIDE.md Part 5.1, Part 5.4.
//
// Loads config, opens the EventLog, then starts one thread per subsystem through SystemManager and
// blocks in runUntilShutdown(). Ctrl+C or window close triggers an ordered shutdown, which sends
// one final zeroed ActuationCommand before the serial port closes (Part 5.4, and the ordering
// argument in SystemManager.h).
//
// Run from the repository root:
//     build/host/ar_drive_assist [config_dir] [log_path] [options]
// The defaults are host/config, logs/session.log and vehicle_params.yaml's serial_device. The log
// is appended to, and each run begins with a SESSION_START line.
//   --serial <device>     overrides serial_device, e.g. the pseudo-terminal hub_sim prints;
//   --no-hub              starts without the hub (development only: nothing can be actuated);
//   --camera <source>     overrides camera.yaml's source: another device, a video file, or
//                         "gst:<pipeline>";
//   --no-camera           starts without the camera, so without perception (development only);
//   --replay <recording>  camera, LiDAR and calibration from a recording (system/Recording.h);
//                         run hub_sim --replay <recording>/hub.csv for its GPS/IMU (Part 13.2);
//   --inject-obstacle [gap_m[,speed_mps[,appear_s]]]
//                         replay only: a test obstacle ahead in the LiDAR (lidar/LidarReplay.h);
//   --destination <lat,lon>  navigate there (Part 11; needs the OSRM map);
//   --no-display          no window (headless runs, the replay harness);
//   --no-lidar            live only: start without the LiDAR (development only: no range, so no
//                         brake decision is possible);
//   --record <dir>        also write what the system sees to <dir> as a recording
//                         (system/Recorder.h): drives to replay, and Part 12.2's calibration
//                         captures. Needs the camera. Never overwrites a recording.
// Without a reachable hub (and without --no-hub) startup fails, naming the device: no hub, no
// system.
//
// Threads (Part 5.1), started in this order: inference first (its engines take seconds to load),
// then fusion, render, navigation, the hub link, decision, and last the sensors. In a replay the
// hub link's first heartbeat starts hub_sim's replay clock and the sensors start theirs when their
// threads start, so all three begin within milliseconds of one another.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ar_drive_assist/camera/CameraPipeline.h"
#include "ar_drive_assist/decision/DecisionThread.h"
#include "ar_drive_assist/fusion/EgoEstimator.h"
#include "ar_drive_assist/lidar/LidarReplay.h"
#ifdef AR_HAVE_LIVOX
#include <yaml-cpp/yaml.h>

#include "ar_drive_assist/lidar/LivoxCapture.h"
#endif
#include "ar_drive_assist/nav/NavigationThread.h"
#include "ar_drive_assist/render/RenderThread.h"
#include "ar_drive_assist/scene/ExtrinsicMonitorThread.h"
#include "ar_drive_assist/scene/FusionThread.h"
#include "ar_drive_assist/system/EventLog.h"
#include "ar_drive_assist/system/Recorder.h"
#include "ar_drive_assist/system/Recording.h"
#include "ar_drive_assist/system/SystemManager.h"
#include "ar_drive_assist/vehicle/VehicleInterface.h"

using namespace ar_drive_assist;

namespace {

struct Options {
    std::vector<std::string> positional;
    std::string serial, camera, replay, record;
    bool noHub = false, noCamera = false, noDisplay = false, noLidar = false;
    std::optional<InjectedObstacle> obstacle;
    std::optional<std::pair<double, double>> destination;
};

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool more = i + 1 < argc;
        if (a == "--serial" && more) {
            o.serial = argv[++i];
        } else if (a == "--no-hub") {
            o.noHub = true;
        } else if (a == "--camera" && more) {
            o.camera = argv[++i];
        } else if (a == "--no-camera") {
            o.noCamera = true;
        } else if (a == "--replay" && more) {
            o.replay = argv[++i];
        } else if (a == "--no-display") {
            o.noDisplay = true;
        } else if (a == "--no-lidar") {
            o.noLidar = true;
        } else if (a == "--record" && more) {
            o.record = argv[++i];
        } else if (a == "--inject-obstacle") {
            InjectedObstacle ob;
            if (more && argv[i + 1][0] != '-')
                std::sscanf(argv[++i], "%lf,%lf,%lf", &ob.startGapM, &ob.speedMps, &ob.appearS);
            o.obstacle = ob;
        } else if (a == "--destination" && more) {
            double lat = 0, lon = 0;
            if (std::sscanf(argv[++i], "%lf,%lf", &lat, &lon) != 2)
                throw std::runtime_error("--destination needs lat,lon");
            o.destination = {lat, lon};
        } else if (a.rfind("--", 0) == 0) {
            throw std::runtime_error("unknown option " + a);
        } else {
            o.positional.push_back(a);
        }
    }
    if (o.obstacle && o.replay.empty())
        throw std::runtime_error("--inject-obstacle is for replays only (--replay)");
    if (!o.record.empty() && o.noCamera)
        throw std::runtime_error("--record needs the camera (not with --no-camera)");
    return o;
}

}  // namespace

int main(int argc, char** argv) {
    // Installed first, so even a crash during startup gets the crash handler.
    SystemManager::installSignalHandlers();

    try {
        const Options opt = parse(argc, argv);
        const std::string configDir = opt.positional.size() > 0 ? opt.positional[0] : "host/config";
        const std::string logPath =
            opt.positional.size() > 1 ? opt.positional[1] : "logs/session.log";
        const auto config = SystemManager::loadConfig(configDir);
        EventLog log(logPath);

        // Everything the threads share is declared BEFORE the SystemManager, so it outlives every
        // thread the manager owns (it joins them in its destructor at the latest).
        FeederConfig feederCfg;
        feederCfg.mount = ImuMount::fromRollPitchYawDeg(config.vehicle.imuMountRpyDeg[0],
                                                        config.vehicle.imuMountRpyDeg[1],
                                                        config.vehicle.imuMountRpyDeg[2]);
        EgoEstimator ego(feederCfg);
        FrameBus frameBus, displayBus;
        MlInferenceEngine::DetectionBus detectionBus;
        MlInferenceEngine::MaskBus maskBus;
        LidarBus lidarBus;
        SceneBus sceneBus, renderSceneBus, navSceneBus;
        DecisionBus decisionBus;
        NavBus navBus;

        // Where the sensors are, and the camera configuration.
        std::optional<Recording> rec;
        SensorExtrinsics ext;
        CameraConfig cam = loadCameraConfig(configDir + "/camera.yaml");
        if (!opt.replay.empty()) {
            rec = loadRecording(opt.replay);
            ext.cameraFromVehicle = rec->cameraFromVehicle;
            ext.cameraFromLidar = rec->cameraFromLidar;
            ext.vehicleFromLidar = rec->vehicleFromLidar;
            ext.cameraCalibrated = ext.lidarCalibrated = true;
            cam.source = rec->video();
            cam.timestampsPath = rec->frameTimes();
            cam.intrinsicsPath = rec->intrinsics();
            cam.width = rec->width;
            cam.height = rec->height;
            cam.loop = false;
            log.logGeneral("replay: " + rec->name + " (" + rec->source + "; " + rec->licence + ")");
        } else {
            ext = loadConfigExtrinsics(configDir, config.vehicle.cameraHeightM);
            if (!opt.camera.empty()) cam.source = opt.camera;
        }
        if (!ext.cameraCalibrated || !ext.lidarCalibrated)
            log.logGeneral("WARNING extrinsics not calibrated (Part 12.2): placeholder geometry");

        // The camera is constructed first (it opens the source and learns the frame model the
        // others need) but started last, below. Declared before the manager: it must outlive the
        // thread that runs it, on every path out of this block.
        std::unique_ptr<CameraPipeline> camera;
        SystemManager mgr(config, log);
        ExtrinsicMonitorThread* extMonitor = nullptr;  // Part 12.2.1, when camera and LiDAR run
        const bool perception = !opt.noCamera;
        if (perception) {
            camera = std::make_unique<CameraPipeline>(cam, frameBus, &log,
                                                      opt.noDisplay ? nullptr : &displayBus);
            if (rec && opt.obstacle) {  // the test obstacle in the frames too (InjectedObstacle.h)
                const ObstacleModel model(*opt.obstacle, rec->hubReports());
                const CameraModel fm = camera->frameModel();
                const Eigen::Isometry3d cfv = rec->cameraFromVehicle;
                camera->setReplayHook(
                    [model, fm, cfv](cv::Mat& img, double tS) { model.paint(img, tS, fm, cfv); });
            }
            mgr.start<MlInferenceEngine>(InferenceConfig{}, frameBus, detectionBus, nullptr, &log,
                                         &maskBus);

            FusionThreadConfig fc;
            fc.camera = camera->frameModel();
            fc.extrinsics = {ext.cameraFromLidar, ext.vehicleFromLidar};
            if (rec || !opt.noLidar) {
                extMonitor = &mgr.start<ExtrinsicMonitorThread>(
                    ext.cameraFromLidar, ext.vehicleFromLidar, camera->frameModel(), ego, &log);
                fc.monitor = &extMonitor->monitor();
            }
            fc.tracker = loadTrackerConfig(configDir + "/motion_prediction.yaml");
            mgr.start<FusionThread>(fc, detectionBus, &maskBus, &lidarBus, ego, sceneBus,
                                    opt.noDisplay ? nullptr : &renderSceneBus, &log,
                                    opt.destination ? &navSceneBus : nullptr);
            if (!opt.noDisplay) {
                RenderThreadConfig rc;
                rc.renderer.frontBumperM = config.vehicle.frontBumperFromRearAxleM;
                rc.renderer.tailgatingMinGapS = config.decision.tailgatingMinGapS;
                rc.camera = camera->frameModel();
                rc.cameraFromVehicle = ext.cameraFromVehicle;
                const cv::Size s = camera->frameSize();
                if (s.width <= 1600) {
                    rc.windowWidth = s.width;
                    rc.windowHeight = s.height;
                }
                mgr.start<RenderThread>(rc, displayBus, renderSceneBus, &decisionBus, &navBus, &log,
                                        [&mgr] { mgr.requestShutdown(); });
            }
        } else {
            log.logGeneral("started with --no-camera: no perception");
        }
        if (opt.destination) {
            NavigationThreadConfig nc;
            nc.destLat = opt.destination->first;
            nc.destLon = opt.destination->second;
            nc.projection = loadRoadProjectionConfig(configDir + "/road_projection.yaml");
            if (camera) nc.camera = camera->frameModel();
            nc.cameraFromVehicle = ext.cameraFromVehicle;
            mgr.start<NavigationThread>(nc, ego, perception ? &navSceneBus : nullptr, navBus, &log);
        }

        // The recorder starts before every producer it taps (the hub link and the sensors).
        Recorder* recorder = nullptr;
        std::function<void(const LidarFrame&)> scanTap;
        if (!opt.record.empty()) {
            RecorderConfig rc;
            rc.dir = opt.record;
            rc.frameModel = camera->frameModel();
            rc.fps = rec ? camera->sourceFps() : cam.fps;
            rc.extrinsics = ext;
            if (rec) rc.notes = "Re-recorded from " + rec->name + ".";
            recorder = &mgr.start<Recorder>(rc, &log);
        }
        // The sensors' taps: the extrinsic monitor and the recorder, whichever run.
        if (camera && (extMonitor || recorder)) {
            camera->setFrameTap([extMonitor, recorder](const CameraFrame& f) {
                if (extMonitor) extMonitor->onFrame(f);
                if (recorder) recorder->onFrame(f);
            });
            scanTap = [extMonitor, recorder](const LidarFrame& s) {
                if (extMonitor) extMonitor->onScan(s);
                if (recorder) recorder->onScan(s);
            };
        }

        VehicleInterface* vehicle = nullptr;
        if (!opt.noHub) {
            VehicleInterfaceOptions vio;
            vio.device = opt.serial.empty() ? config.vehicle.serialDevice : opt.serial;
            vio.baud = config.vehicle.serialBaud;
            vio.openOnConstruction = true;
            vehicle = &mgr.start<VehicleInterface>(
                vio, log,
                [&ego, recorder](const hub_protocol::SensorReport& r, std::uint64_t hostMs) {
                    ego.feed(r);
                    if (recorder) recorder->onReport(r, hostMs);
                });
            // Part 5.4: after every thread is joined, the last word to the hub is "release".
            mgr.setFinalCommandHook([vehicle] {
                vehicle->sendFinalZeroedCommand();
                vehicle->close();
            });
        } else {
            log.logGeneral("started with --no-hub: nothing can be actuated");
        }

        if (perception) {
            DecisionThreadConfig dc;
            dc.thresholds = config.decision;
            dc.vehicle = config.vehicle;
            mgr.start<DecisionThread>(dc, sceneBus, vehicle, opt.noDisplay ? nullptr : &decisionBus,
                                      &log);
            // The sensors last: in a replay their clocks start here.
            if (rec) {
                mgr.start<LidarReplay>(*rec, lidarBus, &log, opt.obstacle, false, scanTap);
            } else if (opt.noLidar) {
                log.logGeneral("started with --no-lidar: no measured range, so no brake decision");
            } else {
#ifdef AR_HAVE_LIVOX
                const YAML::Node l = YAML::LoadFile(configDir + "/lidar.yaml");
                LivoxConfig lc;
                lc.hostIp = l["host_ip"].as<std::string>(lc.hostIp);
                lc.stallTimeoutS = l["stall_timeout_s"].as<double>(lc.stallTimeoutS);
                mgr.start<LivoxCapture>(lc, lidarBus, ext.vehicleFromLidar, &log, scanTap);
#else
                throw std::runtime_error(
                    "this build has no live LiDAR (Livox SDK2 not found at build time, BUILD_GUIDE "
                    "2.8); use --replay, or --no-lidar for development");
#endif
            }
            struct CameraThread {  // runs the camera constructed above, on its own thread
                CameraPipeline& c;
                void run(const std::atomic<bool>& stop) { c.run(stop); }
            };
            mgr.start<CameraThread>(CameraThread{*camera});
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
