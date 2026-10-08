// RenderThread — see include/ar_drive_assist/render/RenderThread.h.
#include "ar_drive_assist/render/RenderThread.h"

#include <chrono>
#include <cstdio>
#include <thread>

#include "ar_drive_assist/system/EventLog.h"

namespace ar_drive_assist {

RenderThread::RenderThread(RenderThreadConfig cfg, FrameBus& frames, SceneBus& scenes,
                           DecisionBus* decisions, NavBus* routes, EventLog* log,
                           std::function<void()> onWindowClosed)
    : cfg_(std::move(cfg)),
      frames_(frames),
      scenes_(scenes),
      decisions_(decisions),
      routes_(routes),
      log_(log),
      onWindowClosed_(std::move(onWindowClosed)) {}

RenderInputs RenderThread::inputs(const CameraFrame& frame, const SceneSnapshot* scene,
                                  const DecisionSnapshot* decision, const ProjectedRoute* route,
                                  const Eigen::Isometry3d& cameraFromVehicle) {
    RenderInputs in;
    in.frameMs = frame.timestampMs;
    in.route = route;
    if (!scene) return in;
    if (scene->egoValid) {
        in.egoSpeedMps = scene->ego.speedMps;
        in.yawRateRadPerS = scene->ego.yawRateRadPerS;
    }
    in.lanes = &scene->detections;
    if (scene->extrinsicsDegraded)
        in.systemNotice = "CAMERA-LIDAR ALIGNMENT LOST: recalibrate (Part 12.2)";
    const std::size_t nBoxes = scene->detections.boxes.size();
    // Signs: the sign detector's boxes, with a LiDAR position where fusion found one.
    for (std::size_t k = 0; k < scene->detections.signs.size(); ++k) {
        const auto& b = *scene->detections.signs[k];
        SignObservation s;
        s.classId = b.class_id;
        s.confidence = b.confidence;
        s.box = {b.x, b.y, b.w, b.h};
        for (const FusedObject& o : scene->scene.objects)
            if (o.detection == static_cast<int>(nBoxes + k) && o.source == RangeSource::LIDAR)
                s.position = o.position;
        in.signs.push_back(s);
    }
    if (!decision) return in;
    // Hazards: the tracks the decision assessed, with the camera box of the fused object nearest
    // each track (the tracker does not keep the detection it came from).
    for (const TrackView& t : decision->tracks) {
        if (!t.confirmed || t.threat.level == ThreatLevel::NONE) continue;
        HazardView h;
        h.trackId = t.trackId;
        h.objectClass = t.objectClass;
        h.threat = t.threat;
        h.ground = Eigen::Vector3d(t.posVehicle.x(), t.posVehicle.y(), 0.0);
        h.gapM = t.inEgoPath ? t.gapM : -1;
        h.depthM = (cameraFromVehicle * *h.ground).z();
        double best = 2.0;  // metres
        for (const FusedObject& o : scene->scene.objects) {
            if (o.detection < 0 || o.detection >= static_cast<int>(nBoxes)) continue;
            const double d = (o.position.head<2>() - t.posVehicle).norm();
            if (o.source != RangeSource::NONE && d < best) {
                best = d;
                const auto& b = *scene->detections.boxes[o.detection];
                h.box = {b.x, b.y, b.w, b.h};
            }
        }
        in.hazards.push_back(h);
    }
    return in;
}

void RenderThread::run(const std::atomic<bool>& stop) {
    WindowedSink sink(cfg_.window);
    if (!sink.init({cfg_.windowWidth, cfg_.windowHeight,
                    static_cast<float>(cfg_.windowWidth) / cfg_.windowHeight},
                   cfg_.camera, cfg_.cameraFromVehicle))
        throw std::runtime_error("render: cannot open the window: " + sink.lastError());
    if (log_)
        log_->logGeneral("render: window open on " + sink.renderer() +
                         (sink.softwareRendering() ? " (SOFTWARE rendering)" : ""));
    ArRenderer renderer(cfg_.renderer, cfg_.camera, cfg_.cameraFromVehicle);
    SceneSnapshot scene;
    DecisionSnapshot decision;
    ProjectedRoute route;
    bool haveScene = false, haveDecision = false, haveRoute = false;
    while (!stop.load()) {
        if (!sink.pollEvents()) {  // the window was closed: shut the system down
            if (log_) log_->logGeneral("render: window closed");
            if (onWindowClosed_) onWindowClosed_();
            break;
        }
        haveScene |= scenes_.popLatest(scene);
        if (decisions_) haveDecision |= decisions_->popLatest(decision);
        if (routes_) haveRoute |= routes_->popLatest(route);
        CameraFrame frame;
        if (!frames_.popLatest(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const RenderInputs in =
            inputs(frame, haveScene ? &scene : nullptr, haveDecision ? &decision : nullptr,
                   haveRoute ? &route : nullptr, cfg_.cameraFromVehicle);
        CompositedFrame out;
        out.video = frame.bgr;
        out.overlays = renderer.build(in);
        out.timestampMs = frame.timestampMs;
        sink.present(out);
    }
    while (!stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

}  // namespace ar_drive_assist
