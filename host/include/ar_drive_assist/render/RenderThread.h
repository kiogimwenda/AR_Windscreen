#pragma once
// RenderThread — Part 5.1's thread 6: the newest camera frame + the newest scene, decision and
// route -> ArRenderer -> the window (WindowedSink). See docs/BUILD_GUIDE.md Part 10.
//
// It always shows the NEWEST frame with the newest state it has: the display never waits for
// perception, and a slow perception frame never holds the video back. The window (SDL and its
// OpenGL context) is created on this thread, the only one that touches it. Closing the window
// requests an ordered shutdown, the same as Ctrl+C.

#include <atomic>
#include <functional>

#include "ar_drive_assist/render/ArRenderer.h"
#include "ar_drive_assist/render/WindowedSink.h"
#include "ar_drive_assist/system/PipelineMessages.h"

namespace ar_drive_assist {

class EventLog;

struct RenderThreadConfig {
    RendererConfig renderer;
    CameraModel camera;  // the model of the displayed frames: CameraPipeline::frameModel()
    Eigen::Isometry3d cameraFromVehicle = Eigen::Isometry3d::Identity();
    WindowedSinkOptions window;
    int windowWidth = 1280, windowHeight = 720;
};

class RenderThread {
public:
    RenderThread(RenderThreadConfig cfg, FrameBus& frames, SceneBus& scenes,
                 DecisionBus* decisions = nullptr, NavBus* routes = nullptr,
                 EventLog* log = nullptr, std::function<void()> onWindowClosed = {});

    void run(const std::atomic<bool>& stop);

    // What the renderer is given for one frame (tests call it; no window needed).
    static RenderInputs inputs(const CameraFrame& frame, const SceneSnapshot* scene,
                               const DecisionSnapshot* decision, const ProjectedRoute* route,
                               const Eigen::Isometry3d& cameraFromVehicle);

private:
    RenderThreadConfig cfg_;
    FrameBus& frames_;
    SceneBus& scenes_;
    DecisionBus* decisions_;
    NavBus* routes_;
    EventLog* log_;
    std::function<void()> onWindowClosed_;
};

}  // namespace ar_drive_assist
