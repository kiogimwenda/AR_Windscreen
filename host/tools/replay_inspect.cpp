// replay_inspect — a recording through the whole perception and decision chain, frame by frame,
// synchronously and deterministically (Part 13.2). The live system runs the same classes on
// threads in real time (main.cpp --replay); this runs them in lock-step, so every result can be
// examined, and the same input always gives the same output.
//
//   build/host/replay_inspect <recording> [--out DIR] [--inject-obstacle gap,speed,appear_s]
//                             [--frames N] [--no-video] [--jitter SEED]
//
// Per camera frame: the hub reports up to the frame's time into the ego filter (interpolated to
// 50 Hz, exactly as hub_sim sends them), the newest LiDAR scan, inference, FusionThread::process,
// DecisionThread::process with the hub armed. Writes to DIR (default: <recording>/inspect):
//   tracks.csv      every track the decision saw, every frame (state, measured run, both paths)
//   decisions.csv   every decision (rule, request, target, ttc, gap, closing speed, warnings);
//                   the injected obstacle's true gap, time to collision and lateral offset; the
//                   age of the LiDAR scan fused with the frame (-1: none was usable)
//   inspect.mp4     the camera with the LiDAR points (colour = height above the road), the camera
//                   detections, each track's ground point and id, and the decision
//   ar.mp4          with --ar-video: the DRIVER'S view, drawn by the real renderer (ArRenderer +
//                   WindowedSink, offscreen, read back from the GPU) from the same scene and
//                   decision, exactly as the running system draws it
// --jitter SEED: the timing variations of a real-time run, reproducibly: each frame is paired with
// the newest LiDAR scan or, at random, the one before (as a scan still being processed would
// leave it; one more than 150 ms old is not fused, so that frame has no LiDAR at all), and the
// hub reports' clock is shifted by a random constant within +-60 ms. Harsher than the real system
// (where the newest scan is at most one period plus processing old), deliberately.
// Run from the repository root (engine paths are relative to it).

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <random>
#include <string>

#include "HubSim.h"
#include "ar_drive_assist/camera/CameraPipeline.h"
#include "ar_drive_assist/decision/DecisionThread.h"
#include "ar_drive_assist/lidar/LidarReplay.h"
#include "ar_drive_assist/render/ArRenderer.h"
#include "ar_drive_assist/render/RenderThread.h"
#include "ar_drive_assist/render/WindowedSink.h"
#include "ar_drive_assist/scene/FusionThread.h"
#include "ar_drive_assist/system/SystemManager.h"

using namespace ar_drive_assist;

namespace {

hub_protocol::SensorReport toReport(const sim::ReplayReport& r, std::uint32_t ms) {
    hub_protocol::SensorReport s{};
    s.timestampMs = ms;
    s.latitude = r.lat;
    s.longitude = r.lon;
    s.speedKph = r.speedKph;
    s.gpsFixValid = r.fix;
    s.accelX = r.ax;
    s.accelY = r.ay;
    s.accelZ = r.az;
    s.gyroX = r.gx;
    s.gyroY = r.gy;
    s.gyroZ = r.gz;
    s.headingDeg = r.headingDeg;
    s.obdSpeedKph = r.obdKph;
    s.obdBrakePedalActive = 0xFF;
    s.ignitionOn = 1;
    return s;
}

const char* className(int c) {
    static const char* n[] = {"vehicle", "pedestrian", "cyclist", "sign", "obstacle"};
    return c >= 0 && c < 5 ? n[c] : "lidar-only";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: see the top of host/tools/replay_inspect.cpp\n");
        return 2;
    }
    const std::string dir = argv[1];
    std::string outDir = dir + "/inspect";
    std::optional<InjectedObstacle> obstacle;
    int maxFrames = 1 << 30;
    bool video = true;
    int jitterSeed = -1;
    bool arVideo = false;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) {
            outDir = argv[++i];
        } else if (a == "--inject-obstacle" && i + 1 < argc) {
            InjectedObstacle o;
            std::sscanf(argv[++i], "%lf,%lf,%lf", &o.startGapM, &o.speedMps, &o.appearS);
            obstacle = o;
        } else if (a == "--frames" && i + 1 < argc) {
            maxFrames = std::atoi(argv[++i]);
        } else if (a == "--no-video") {
            video = false;
        } else if (a == "--jitter" && i + 1 < argc) {
            jitterSeed = std::atoi(argv[++i]);
        } else if (a == "--ar-video") {
            arVideo = true;
        }
    }
    std::filesystem::create_directories(outDir);

    const Config config = SystemManager::loadConfig("host/config");
    const Recording rec = loadRecording(dir);
    const auto frameTimes = loadFrameTimes(rec.frameTimes());
    const auto scanTimes = loadScanTimes(dir);
    const auto hubRows = sim::loadReplayReports(rec.hubReports());

    CameraConfig cc;
    cc.source = rec.video();
    cc.width = rec.width;
    cc.height = rec.height;
    cc.intrinsicsPath = rec.intrinsics();
    cc.realtime = false;
    cc.loop = false;
    FrameBus unusedFrames;
    CameraPipeline camera(cc, unusedFrames);
    const CameraModel cam = camera.frameModel();

    MlInferenceEngine::DetectionBus unusedDet;
    MlInferenceEngine engine(InferenceConfig{}, unusedFrames, unusedDet);

    LidarBus unusedLidar;
    LidarReplay lidar(rec, unusedLidar, nullptr, obstacle);
    std::optional<ObstacleModel> truth;  // the injected obstacle's ground truth, for decisions.csv
    if (obstacle) {                      // the camera sees it too (InjectedObstacle.h)
        const ObstacleModel model(*obstacle, rec.hubReports());
        truth = model;
        camera.setReplayHook([model, cam, &rec](cv::Mat& img, double tS) {
            model.paint(img, tS, cam, rec.cameraFromVehicle);
        });
    }

    FeederConfig fcfg;
    EgoEstimator ego(fcfg);

    FusionThreadConfig fc;
    fc.camera = cam;
    fc.extrinsics = {rec.cameraFromLidar, rec.vehicleFromLidar};
    fc.tracker = loadTrackerConfig("host/config/motion_prediction.yaml");
    MlInferenceEngine::DetectionBus det;
    SceneBus scenes;
    FusionThread fusion(fc, det, nullptr, nullptr, ego, scenes);

    DecisionThreadConfig dc;
    dc.thresholds = config.decision;
    dc.vehicle = config.vehicle;
    DecisionThread decision(dc, scenes, nullptr);

    // The driver's view: the renderer the running system uses, offscreen.
    std::unique_ptr<WindowedSink> sink;
    std::unique_ptr<ArRenderer> arRenderer;
    cv::VideoWriter arWriter;
    if (arVideo) {
        WindowedSinkOptions wo;
        wo.hidden = true;
        sink = std::make_unique<WindowedSink>(wo);
        if (!sink->init({rec.width, rec.height, float(rec.width) / rec.height}, cam,
                        rec.cameraFromVehicle)) {
            std::fprintf(stderr, "replay_inspect: renderer: %s\n", sink->lastError().c_str());
            return 1;
        }
        RendererConfig rcfg;
        rcfg.frontBumperM = config.vehicle.frontBumperFromRearAxleM;
        rcfg.tailgatingMinGapS = config.decision.tailgatingMinGapS;
        arRenderer = std::make_unique<ArRenderer>(rcfg, cam, rec.cameraFromVehicle);
        arWriter.open(outDir + "/ar.mp4", cv::CAP_FFMPEG,
                      cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 10, {rec.width, rec.height});
    }

    std::ofstream tracksCsv(outDir + "/tracks.csv"), decCsv(outDir + "/decisions.csv");
    tracksCsv << "frame,t_ms,track,class,state,confirmed,range_measured,run_ms,x_m,y_m,in_path,"
                 "in_straight_path,gap_m,"
                 "closing_mps,threat,risk,ttc_s\n";
    decCsv << "frame,t_ms,rule,request,intensity,target,ttc_s,gap_m,closing_mps,ego_mps,warnings,"
              "true_gap_m,true_ttc_s,true_y_m,scan_age_ms\n";  // scan_age_ms -1: no scan used
    cv::VideoWriter vw;
    if (video)
        vw.open(outDir + "/inspect.mp4", cv::CAP_FFMPEG,
                cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 10, {rec.width, rec.height + 60});

    const std::int64_t t0 = frameTimes.front();
    std::mt19937 jrng(static_cast<unsigned>(std::max(jitterSeed, 0)));
    std::int64_t hubShiftUs = 0;
    if (jitterSeed >= 0)
        hubShiftUs = std::uniform_int_distribution<std::int64_t>(-60000, 60000)(jrng);
    std::int64_t fedUs = hubRows.front().tUs;
    int brakes = 0;
    std::size_t frameIdx = 0;
    CameraFrame f;
    while (static_cast<int>(frameIdx) < maxFrames && camera.read(f)) {
        ++frameIdx;  // frame 0 was the camera's size probe
        if (frameIdx >= frameTimes.size()) break;
        const std::int64_t tUs = frameTimes[frameIdx] - t0;  // recording time from the start
        const std::uint64_t tMs = static_cast<std::uint64_t>(tUs / 1000) + 1000;
        f.timestampMs = tMs;
        f.seq = frameIdx;
        // The hub reports up to this frame (50 Hz), on the same clock.
        for (; fedUs - hubRows.front().tUs <= tUs + hubShiftUs; fedUs += 20000) {
            const std::uint32_t ms =
                static_cast<std::uint32_t>((fedUs - hubRows.front().tUs) / 1000) + 1000;
            ego.feed(toReport(sim::replayAt(hubRows, fedUs), ms));
        }
        // The newest scan at or before the frame.
        std::size_t si = 0;
        while (si + 1 < scanTimes.size() && scanTimes[si + 1].tUs - t0 <= tUs) ++si;
        if (jitterSeed >= 0 && si > 0 && jrng() % 2) --si;  // the previous scan, at random
        const std::int64_t scanUs = scanTimes[si].tUs - t0;
        const LidarFrame scan =
            lidar.scan(si, scanUs * 1e-6, static_cast<std::uint64_t>(scanUs / 1000) + 1000);

        MlInferenceEngine::Result r = engine.process(f);
        const SceneSnapshot s = fusion.process(r.detections, &r.masks, &scan, ego.snapshot(tMs));
        HubState hub;
        hub.haveReport = true;
        hub.reportMs = tMs;
        hub.killSwitchEngaged = 0;
        hub.haveAck = true;
        const DecisionSnapshot d = decision.process(s, hub, tMs);

        for (const TrackView& t : d.tracks) {
            tracksCsv << frameIdx << ',' << tMs << ',' << t.trackId << ','
                      << className(t.objectClass) << ',' << t.state << ',' << t.confirmed << ','
                      << t.rangeMeasured << ',' << t.measuredRunMs << ',' << t.posVehicle.x() << ','
                      << t.posVehicle.y() << ',' << t.inEgoPath << ',' << t.inStraightPath << ','
                      << t.gapM << ',' << t.closingSpeedMps << ',' << int(t.threat.level) << ','
                      << t.threat.risk << ',' << t.threat.ttcS << '\n';
        }
        const auto& dd = d.decision;
        const bool brake = dd.request.type == schema::RequestType_BRAKE;
        brakes += brake;
        decCsv << frameIdx << ',' << tMs << ',' << dd.rule << ','
               << schema::EnumNameRequestType(dd.request.type) << ',' << int(dd.request.intensity)
               << ',' << dd.targetTrackId << ',' << dd.ttcS << ',' << dd.gapM << ','
               << dd.closingSpeedMps << ',' << s.ego.speedMps << ",\"";
        for (const auto& w : dd.warnings) decCsv << w << ';';
        decCsv << '"';
        // Ground truth of the injected obstacle: its rear face's gap from the bumper, and the time
        // to reach it at the recorded ego speed (it is stopped or moves along the road).
        double trueGap = -1, trueTtc = -1, trueY = 0;
        if (truth)
            if (const auto T = truth->poseAt(tUs * 1e-6)) {
                trueGap = T->translation().x() - config.vehicle.frontBumperFromRearAxleM;
                trueY = T->translation().y();
                const double closing = s.ego.speedMps - truth->spec().speedMps;
                if (closing > 0.1 && trueGap > 0) trueTtc = trueGap / closing;
            }
        decCsv << ',' << trueGap << ',' << trueTtc << ',' << trueY << ','
               << (s.lidarUsed ? static_cast<long long>(s.lidarAgeMs) : -1) << '\n';

        if (arVideo) {
            const RenderInputs in = RenderThread::inputs(f, &s, &d, nullptr, rec.cameraFromVehicle);
            CompositedFrame out;
            out.video = f.bgr;
            out.overlays = arRenderer->build(in);
            out.timestampMs = f.timestampMs;
            sink->present(out);
            arWriter.write(sink->capture());
        }
        if (!video) continue;
        cv::Mat img;
        cv::copyMakeBorder(f.bgr, img, 0, 60, 0, 0, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        // LiDAR points: colour by height above the fitted road (grey = road, orange = obstacle
        // height, blue = above 2.5 m)
        const GroundPlane road =
            scan.groundValid ? LidarProcessor::toGroundPlane(scan.ground) : GroundPlane{};
        for (const LidarPoint& p : scan.points) {
            const Eigen::Vector3d pv = rec.vehicleFromLidar * p.p.cast<double>();
            Eigen::Vector2d px;
            if (!cam.project(rec.cameraFromLidar * p.p.cast<double>(), px)) continue;
            const double z = road.distance(pv);
            const cv::Scalar col = z < 0.15  ? cv::Scalar(120, 120, 120)
                                   : z < 2.5 ? cv::Scalar(0, 200, 255)
                                             : cv::Scalar(255, 160, 0);
            cv::circle(img, {int(px.x()), int(px.y())}, 1, col, cv::FILLED);
        }
        for (const Box& b : r.boxes)
            cv::rectangle(img, cv::Rect2f(b.x, b.y, b.w, b.h), {0, 255, 0}, 2);
        for (const TrackView& t : d.tracks) {
            Eigen::Vector2d px;
            if (!cam.project(
                    rec.cameraFromVehicle * Eigen::Vector3d(t.posVehicle.x(), t.posVehicle.y(), 0),
                    px))
                continue;
            const bool target = t.trackId == dd.targetTrackId;
            const cv::Scalar col = target        ? cv::Scalar(0, 0, 255)
                                   : t.confirmed ? cv::Scalar(255, 255, 255)
                                                 : cv::Scalar(128, 128, 128);
            cv::drawMarker(img, {int(px.x()), int(px.y())}, col, cv::MARKER_TILTED_CROSS, 14, 2);
            char lab[80];
            std::snprintf(lab, sizeof lab, "#%d %s %.1fm %+.1f", t.trackId,
                          className(t.objectClass), t.posVehicle.x(), -t.closingSpeedMps);
            cv::putText(img, lab, {int(px.x()) + 6, int(px.y()) - 6}, cv::FONT_HERSHEY_SIMPLEX,
                        0.45, col, 1);
        }
        char banner[200];
        std::snprintf(
            banner, sizeof banner,
            "frame %zu  t=%.1f s  ego %.1f m/s  rule %d  %s  target #%d ttc %.2f s gap %.1f m",
            frameIdx, tUs * 1e-6, s.ego.speedMps, dd.rule, brake ? "BRAKE" : "-", dd.targetTrackId,
            dd.ttcS, dd.gapM);
        cv::putText(img, banner, {10, rec.height + 38}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    brake ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255), 2);
        vw.write(img);
    }
    std::printf("replay_inspect: %zu frames, %d brake decisions; %s/{tracks,decisions}.csv%s\n",
                frameIdx, brakes, outDir.c_str(), video ? ", inspect.mp4" : "");
    return 0;
}
