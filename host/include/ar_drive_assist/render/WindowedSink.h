#pragma once
// WindowedSink — draws the video and the OverlayScene with OpenGL in an SDL2 window. See
// docs/BUILD_GUIDE.md Part 10.2 and 10.4.
//
// ---------------------------------------------------------------------------------------------
// GPU selection under WSLg (measured 2026-09-27). Debian's Mesa defaults to llvmpipe, a SOFTWARE
// renderer on the CPU, even though the GPU path exists: Mesa's Direct3D 12 driver on the Windows
// GPU. And with it forced, it picked the laptop's INTEGRATED Intel GPU. init() therefore sets
//     GALLIUM_DRIVER=d3d12  and  MESA_D3D12_DEFAULT_ADAPTER_NAME=<adapterHint, default "NVIDIA">
// before the GL context exists (unless the environment already sets them), then reads back
// GL_RENDERER. A software renderer is reported (softwareRendering()), and refused when
// `requireGpu` is set: the hazard glow at 2K is per-pixel work the CPU should not be doing.
//
// Frame path, per present():
//   1. Video: the BGR frame is copied into a pixel buffer object and uploaded to a texture from
//      there (Part 10.2: the portable host-to-GPU path under WSLg), then drawn.
//   2. Occluders: every hazard silhouette is rasterised into one small mask texture; road-space
//      items fade to nothing inside it (overlay rule 4: a barrier appears BEHIND a pedestrian).
//   3. Items, back to front by layer:
//        road   triangle strips in metres (vehicle frame), projected in the vertex shader with the
//               calibrated camera model INCLUDING lens distortion (common/Camera.h's formulas), so
//               a road item lands exactly where the perception side measured it. Geometry behind
//               the camera is trimmed on the CPU first.
//        mask   the hazard glow: a rim of light along the object's outline plus at most a 22% tint
//               inside, with a slow moving light band (shimmer), from the object's mask texture.
//        image  badges and labels, rasterised on the CPU (OpenCV) into small textures, cached.
//   Everything is drawn into an offscreen framebuffer at VIDEO resolution, which is then scaled
//   into the window (letterboxed). The same framebuffer can be read back (capture()), which is how
//   the rendering is checked by eye and in tests.
//
// Timings (10.4): the upload and the overlay pass are timed separately on the GPU with timer
// queries, plus the CPU time of present().
// ---------------------------------------------------------------------------------------------

#include <map>
#include <memory>
#include <string>

#include "ar_drive_assist/render/DisplaySink.h"

struct SDL_Window;

namespace ar_drive_assist {

struct WindowedSinkOptions {
    std::string title = "AR Windscreen";
    bool hidden = false;      // offscreen (tools, tests): nothing appears on the desktop
    bool fullscreen = false;  // borderless fullscreen at the desktop's native resolution
    bool vsync = false;
    bool requireGpu = false;  // refuse to start on a software renderer
    std::string adapterHint = "NVIDIA";
};

struct SinkTimings {
    double uploadMs = 0;      // GPU: PBO -> texture
    double overlayMs = 0;     // GPU: occluders + all overlay items
    double presentCpuMs = 0;  // CPU: the whole present() call
};

class WindowedSink : public DisplaySink {
public:
    explicit WindowedSink(WindowedSinkOptions options = {});
    ~WindowedSink() override;

    bool init(const FrameGeometry& geometry, const CameraModel& camera,
              const Eigen::Isometry3d& cameraFromVehicle) override;
    void present(const CompositedFrame& frame) override;
    FrameGeometry outputGeometry() const override { return geometry_; }

    // The last composited frame at video resolution (BGR). Blocks until the GPU is done.
    cv::Mat capture();
    const std::string& renderer() const { return renderer_; }
    bool softwareRendering() const { return software_; }
    const SinkTimings& lastTimings() const { return timings_; }
    const std::string& lastError() const { return error_; }
    // Pumps window events; false once the window was closed.
    bool pollEvents();

private:
    struct Gl;  // all GL objects (keeps GL headers out of this header)
    WindowedSinkOptions opt_;
    FrameGeometry geometry_;
    CameraModel camera_;
    Eigen::Isometry3d cameraFromVehicle_ = Eigen::Isometry3d::Identity();
    SDL_Window* window_ = nullptr;
    void* glContext_ = nullptr;
    std::unique_ptr<Gl> gl_;
    std::string renderer_, error_;
    bool software_ = false;
    SinkTimings timings_;
    double startSeconds_ = 0;
};

}  // namespace ar_drive_assist
