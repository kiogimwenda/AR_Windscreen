// render_bench — Part 10.4's benchmark checkpoint, and a visual check of WindowedSink.
//
//   build/host/render_bench [--video data/footage/krakow_0-120s.webm] [--frames 300]
//                           [--width 2560 --height 1440] [--show] [--vsync] [--capture out.png]
//
// Draws a realistic overlay scene (Part 10.4: at least two shimmering hazard glows, a route band
// and a barrier; plus a ring, a zone, a marker, labels and a badge) over real road video at the
// target resolution, through the real WindowedSink, and reports:
//   - the renderer actually in use (a software renderer is flagged);
//   - GPU time of the video upload and of the overlay pass, separately (timer queries);
//   - CPU time of present() and the presented frame rate.
// --capture saves one composited frame for inspection. Without --show the window is hidden.
//
// The camera model here is an ESTIMATE for the recorded footage (unknown camera): 70 deg
// horizontal field of view, 1.4 m high, level. It only has to put road items plausibly on the
// road for a visual check; the car's calibrated model replaces it (Part 12.1).

#include <chrono>
#include <cmath>
#include <cstdio>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

#include "ar_drive_assist/render/WindowedSink.h"

using namespace ar_drive_assist;

namespace {

PixelMask ellipse(int cx, int cy, int rx, int ry) {  // a synthetic object mask, 4 px cells
    PixelMask m;
    m.cellPx = 4;
    m.x = (cx - rx) / 4;
    m.y = (cy - ry) / 4;
    m.w = 2 * rx / 4;
    m.h = 2 * ry / 4;
    m.bits.assign(static_cast<std::size_t>(m.w) * m.h, 0);
    for (int y = 0; y < m.h; ++y)
        for (int x = 0; x < m.w; ++x) {
            const double u = (x + 0.5) / m.w * 2 - 1, v = (y + 0.5) / m.h * 2 - 1;
            if (u * u + v * v <= 1) m.bits[static_cast<std::size_t>(y) * m.w + x] = 1;
        }
    return m;
}

OverlayScene scene(const CameraModel& cam, const Eigen::Isometry3d& T) {
    OverlayScene s;
    auto project = [&](const Eigen::Vector3d& p) {
        Eigen::Vector2d px;
        cam.project(T * p, px);
        return px;
    };
    // Route band: a gentle left curve, 60 m, graded green -> amber; beyond 30 m "not measured".
    OverlayItem route;
    route.kind = OverlayKind::ROAD_BAND;
    route.widthM = 1.0f;
    route.style = {palette::kGreen, 0.9f, 12, 0, 0, 1};
    route.occludedByHazards = true;
    for (double x = 3; x <= 60; x += 1.5) {
        route.points.emplace_back(x, 0.004 * x * x, 0);
        route.pointOpacity.push_back(x <= 30 ? 1.0f : 0.45f);
        const float f = static_cast<float>(std::clamp((x - 20) / 25, 0.0, 1.0));
        route.pointColor.push_back({palette::kGreen.r + f * (palette::kAmber.r - palette::kGreen.r),
                                    palette::kGreen.g + f * (palette::kAmber.g - palette::kGreen.g),
                                    palette::kGreen.b + f * (palette::kAmber.b - palette::kGreen.b),
                                    1});
    }
    s.items.push_back(route);
    // Collision barrier at 28 m, with its countdown label.
    OverlayItem barrier;
    barrier.kind = OverlayKind::ROAD_BARRIER;
    barrier.points = {{28, 1.8, 0}, {28, -1.8, 0}};
    barrier.heightM = 1.1f;
    barrier.style = {palette::kRed, 0.85f, 0, 0, 1.5f, 2};
    barrier.occludedByHazards = true;
    s.items.push_back(barrier);
    OverlayItem countdown;
    countdown.kind = OverlayKind::IMAGE_LABEL;
    countdown.text = "STOP 28 m";
    countdown.sizePx = 40;
    countdown.imagePos = project({28, 0, 1.5});
    countdown.style = {palette::kRed, 1, 0, 0, 0, 5};
    s.items.push_back(countdown);
    // Tailgating zone and hump chevrons.
    OverlayItem zone;
    zone.kind = OverlayKind::ROAD_ZONE;
    zone.points = {{6, 1.0, 0}, {6, -1.0, 0}, {16, -1.0, 0}, {16, 1.0, 0}};
    zone.style = {palette::kAmber, 0.6f, 0, 0, 0, 0};
    zone.occludedByHazards = true;
    s.items.push_back(zone);
    OverlayItem hump;
    hump.kind = OverlayKind::ROAD_MARKER;
    hump.points = {{40, 0.8, 0}};
    hump.widthM = 3.0f;
    hump.style = {palette::kAmber, 0.9f, 0, 0, 0, 1};
    hump.occludedByHazards = true;
    s.items.push_back(hump);
    // A pedestrian at 18 m, 1.6 m left: ring at the feet, shimmering glow on a synthetic mask.
    OverlayItem ring;
    ring.kind = OverlayKind::ROAD_RING;
    ring.points = {{18, 1.6, 0}};
    ring.widthM = 0.6f;
    ring.style = {palette::forRisk(0.7), 0.95f, 0, 0, 1.0f, 3};
    ring.occludedByHazards = true;
    s.items.push_back(ring);
    const Eigen::Vector2d feet = project({18, 1.6, 0}), head = project({18, 1.6, 1.7});
    const int ph = static_cast<int>(feet.y() - head.y());
    const PixelMask ped =
        ellipse(static_cast<int>(feet.x()), static_cast<int>(feet.y() - ph / 2), ph / 5, ph / 2);
    OverlayItem pedGlow;
    pedGlow.kind = OverlayKind::MASK_GLOW;
    pedGlow.mask = ped;
    pedGlow.style = {palette::forRisk(0.7), 1, 10, 1.2f, 0, 4};
    s.items.push_back(pedGlow);
    s.occluders.push_back({ped, 18 - 1.5});
    // A car ahead at 30 m (the barrier's object): glow at high risk.
    const Eigen::Vector2d carL = project({30, 1.0, 0}), carR = project({30, -1.0, 1.5});
    const PixelMask car = ellipse(static_cast<int>((carL.x() + carR.x()) / 2),
                                  static_cast<int>((carL.y() + carR.y()) / 2),
                                  static_cast<int>(std::abs(carR.x() - carL.x()) / 2),
                                  static_cast<int>(std::abs(carL.y() - carR.y()) / 2));
    OverlayItem carGlow = pedGlow;
    carGlow.mask = car;
    carGlow.style.color = palette::forRisk(1.0);
    carGlow.style.shimmerHz = 2.0f;
    s.items.push_back(carGlow);
    s.occluders.push_back({car, 30 - 1.5});
    // HUD: speed-limit badge next to the actual speed.
    OverlayItem badge;
    badge.kind = OverlayKind::IMAGE_BADGE;
    badge.text = "50";
    badge.sizePx = 120;
    badge.imagePos = {140, 140};
    badge.style.layer = 6;
    s.items.push_back(badge);
    OverlayItem speed;
    speed.kind = OverlayKind::IMAGE_LABEL;
    speed.text = "54 km/h";
    speed.sizePx = 44;
    speed.imagePos = {330, 140};
    speed.style = {palette::kAmber, 1, 0, 0, 0, 6};
    s.items.push_back(speed);
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    std::string video = "../data/footage/krakow_0-120s.webm", capturePath;
    int frames = 300, W = 2560, H = 1440;
    bool show = false, vsync = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&] { return std::string(i + 1 < argc ? argv[++i] : ""); };
        if (a == "--video")
            video = next();
        else if (a == "--frames")
            frames = std::stoi(next());
        else if (a == "--width")
            W = std::stoi(next());
        else if (a == "--height")
            H = std::stoi(next());
        else if (a == "--show")
            show = true;
        else if (a == "--vsync")
            vsync = true;
        else if (a == "--capture")
            capturePath = next();
    }
    cv::VideoCapture cap(video);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "cannot open %s\n", video.c_str());
        return 1;
    }
    CameraModel cam;
    cam.width = W;
    cam.height = H;
    cam.cx = W / 2.0;
    cam.cy = H / 2.0;
    cam.fx = cam.fy = (W / 2.0) / std::tan(35.0 * 3.14159265358979 / 180);
    const Eigen::Isometry3d T = cameraFromVehicle({1.5, 0, 1.4}, 0, 0, 0);

    WindowedSinkOptions opt;
    opt.hidden = !show;
    opt.vsync = vsync;
    opt.title = "render_bench";
    WindowedSink sink(opt);
    if (!sink.init({show ? 1280 : W, show ? 720 : H, 16.0f / 9}, cam, T)) {
        std::fprintf(stderr, "sink: %s\n", sink.lastError().c_str());
        return 1;
    }
    std::printf("renderer: %s%s\n", sink.renderer().c_str(),
                sink.softwareRendering() ? "  [SOFTWARE]" : "");
    const OverlayScene sc = scene(cam, T);

    // Decode frames up front so decoding does not pollute the timings.
    std::vector<cv::Mat> clip;
    for (int k = 0; k < 60; ++k) {
        cv::Mat f;
        if (!cap.read(f)) break;
        cv::resize(f, f, {W, H});
        clip.push_back(f);
    }
    double up = 0, ov = 0, cpu = 0;
    int n = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < frames; ++k) {
        if (!sink.pollEvents()) break;
        CompositedFrame f{clip[k % clip.size()], sc, static_cast<std::uint64_t>(k)};
        sink.present(f);
        if (k >= 30) {  // after warm-up
            up += sink.lastTimings().uploadMs;
            ov += sink.lastTimings().overlayMs;
            cpu += sink.lastTimings().presentCpuMs;
            ++n;
        }
        if (k == 45 && !capturePath.empty()) cv::imwrite(capturePath, sink.capture());
    }
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf(
        "%dx%d, %d frames: GPU upload %.2f ms, GPU overlay pass %.2f ms, CPU present %.2f ms, "
        "%.1f fps overall\n",
        W, H, frames, up / n, ov / n, cpu / n, frames / secs);
    return 0;
}
