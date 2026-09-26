// inference_viewer — Phase 5's debug visualisation (Part 14: "a debug visualization (even a
// throwaway cv::imshow window) shows plausible boxes on real footage").
//
// Plays a video through MlInferenceEngine::process() exactly as the live pipeline will, and draws
// what came out: boxes by class, the four lanes, the lane band the lane model saw, and the depth
// map as an inset. It also reports per-frame latency statistics. The report is the Part 7.1 FPS
// check minus the camera, which does not exist yet.
//
//   build/host/inference_viewer <video> [--frames N] [--band-top F]
//                               [--out annotated.mp4] [--snapshots DIR] [--every K]
//                               [--dump FRAME]   (writes that frame as dump_<N>.png plus its boxes
//                               as
//                                                 dump_<N>.csv, for cross-checking against
//                                                 ultralytics)
//
// Run from the repository root (engine paths are relative to it). A frame that is not 2560x1440
// is resized to it first, so results match what the Phase 4 camera will deliver.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

#include "ar_drive_assist/inference/MlInferenceEngine.h"

using namespace ar_drive_assist;

namespace {

const cv::Scalar kClassColour[] = {
    {255, 160, 0},  // Vehicle: blue-ish
    {0, 0, 255},    // Pedestrian: red
    {0, 165, 255},  // Cyclist: orange
    {0, 255, 255},  // Sign: yellow
    {255, 0, 255},  // Obstacle: magenta
};
const char* kClassName[] = {"vehicle", "pedestrian", "cyclist", "sign", "obstacle"};
const cv::Scalar kLaneColour[] = {{200, 200, 200}, {0, 255, 0}, {0, 255, 0}, {200, 200, 200}};

// The source-frame rectangle covered by a mask's crop on the prototype grid.
cv::Rect2f maskRectInSource(const ObjectMask& m, const MaskFrame& f) {
    const auto sx = [&](float g) { return f.mapping.toSourceX(g * f.stride); };
    const auto sy = [&](float g) { return f.mapping.toSourceY(g * f.stride); };
    return {sx(m.x), sy(m.y), sx(m.x + m.w) - sx(m.x), sy(m.y + m.h) - sy(m.y)};
}

// Tints each object's mask in its class colour (debug view: every detection, not only hazards).
void drawMasks(cv::Mat& img, const MlInferenceEngine::Result& r) {
    for (std::size_t i = 0; i < r.masks.masks.size() && i < r.boxes.size(); ++i) {
        const ObjectMask& m = r.masks.masks[i];
        if (m.w == 0 || m.h == 0) continue;
        const cv::Rect roi =
            cv::Rect(maskRectInSource(m, r.masks)) & cv::Rect(0, 0, img.cols, img.rows);
        if (roi.empty()) continue;
        cv::Mat grid(m.h, m.w, CV_8UC1, const_cast<uint8_t*>(m.bits.data()));
        cv::Mat up;
        cv::resize(grid * 255, up, roi.size(), 0, 0, cv::INTER_LINEAR);  // smooth the 4x upscale
        cv::Mat region = img(roi),
                tint(region.size(), region.type(), kClassColour[r.boxes[i].classId]);
        cv::Mat blended;
        cv::addWeighted(region, 0.55, tint, 0.45, 0.0, blended);
        blended.copyTo(region, up > 127);
    }
}

void draw(cv::Mat& img, const MlInferenceEngine::Result& r, const LaneBand& band) {
    drawMasks(img, r);
    cv::rectangle(img, cv::Rect2f(band.x, band.y, band.w, band.h), {255, 255, 255}, 2);
    for (const Box& b : r.boxes) {
        const auto& c = kClassColour[b.classId];
        cv::rectangle(img, cv::Rect2f(b.x, b.y, b.w, b.h), c, 4);
        char label[64];
        std::snprintf(label, sizeof label, "%s %.2f", kClassName[b.classId], b.confidence);
        cv::putText(img, label, {int(b.x), std::max(30, int(b.y) - 8)}, cv::FONT_HERSHEY_SIMPLEX,
                    1.2, c, 3);
    }
    // Road-sign detector: its own 29 classes, labelled by meaning (white outline, so they are
    // distinguishable from the COCO detector's yellow "sign" boxes).
    for (const Box& b : r.signs) {
        cv::rectangle(img, cv::Rect2f(b.x, b.y, b.w, b.h), {255, 255, 255}, 4);
        char label[64];
        std::snprintf(label, sizeof label, "%s %.2f", kSignClassNames[b.classId], b.confidence);
        cv::putText(img, label, {int(b.x), std::max(30, int(b.y) - 8)}, cv::FONT_HERSHEY_SIMPLEX,
                    1.1, {255, 255, 255}, 3);
    }
    for (int l = 0; l < 4; ++l) {
        for (const LanePoint& p : r.lanes[l])
            cv::circle(img, {int(p.x), int(p.y)}, 6, kLaneColour[l], -1);
    }
    // Depth inset, top right: min-max normalised for display only (MiDaS is relative anyway).
    cv::Mat d8, colour;
    cv::normalize(r.depth.inverseDepth, d8, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::applyColorMap(d8, colour, cv::COLORMAP_INFERNO);
    cv::resize(colour, colour, {img.cols / 4, img.cols / 4 * colour.rows / colour.cols});
    colour.copyTo(img(cv::Rect(img.cols - colour.cols - 10, 10, colour.cols, colour.rows)));

    // Text panels sit on a darkened backing so they stay readable over bright sky.
    auto panel = [&](cv::Rect area) {
        cv::Mat roi = img(area & cv::Rect(0, 0, img.cols, img.rows));
        roi.convertTo(roi, -1, 0.35);
    };
    char stats[96];
    std::snprintf(stats, sizeof stats, "gpu %.1f ms   total %.1f ms   boxes %zu   signs %zu",
                  r.gpuMs, r.totalMs, r.boxes.size(), r.signs.size());
    panel({0, 0, 1560, 90});
    cv::putText(img, stats, {20, 60}, cv::FONT_HERSHEY_SIMPLEX, 1.6, {255, 255, 255}, 4);

    panel({0, 100, 520, 330});
    int y = 150;
    for (int c = 0; c < 5; ++c, y += 44) {
        cv::rectangle(img, {20, y - 28, 34, 34}, kClassColour[c], -1);
        cv::putText(img, kClassName[c], {70, y}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {255, 255, 255}, 3);
    }
    cv::circle(img, {37, y - 11}, 12, kLaneColour[1], -1);
    cv::putText(img, "ego lane", {70, y}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {255, 255, 255}, 3);
    y += 44;
    cv::circle(img, {37, y - 11}, 12, kLaneColour[0], -1);
    cv::putText(img, "outer lane / lane band", {70, y}, cv::FONT_HERSHEY_SIMPLEX, 1.2,
                {255, 255, 255}, 3);

    // CC BY 3.0 requires attribution wherever the footage is shown (data/README.md).
    const char* credit =
        "Footage: \"City Driving 4K - Krakow Poland 2024\", Relaxing Roads 4K, CC BY 3.0";
    panel({0, img.rows - 60, img.cols, 60});
    cv::putText(img, credit, {20, img.rows - 18}, cv::FONT_HERSHEY_SIMPLEX, 1.1, {230, 230, 230},
                2);
}

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, static_cast<size_t>(p * (v.size() - 1) + 0.5))];
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: %s <video> [--frames N] [--band-top F] [--out file.mp4] "
                     "[--snapshots DIR] [--every K]\n",
                     argv[0]);
        return 2;
    }
    std::string video = argv[1], out, snapshots;
    int maxFrames = 600, every = 60, dumpFrame = -1;
    InferenceConfig cfg;
    for (int i = 2; i + 1 < argc; i += 2) {
        const std::string a = argv[i], v = argv[i + 1];
        if (a == "--frames")
            maxFrames = std::atoi(v.c_str());
        else if (a == "--band-top")
            cfg.laneBandTopFraction = std::strtof(v.c_str(), nullptr);
        else if (a == "--out")
            out = v;
        else if (a == "--snapshots")
            snapshots = v;
        else if (a == "--every")
            every = std::atoi(v.c_str());
        else if (a == "--dump")
            dumpFrame = std::atoi(v.c_str());
    }

    cv::VideoCapture cap(video);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "cannot open %s\n", video.c_str());
        return 1;
    }
    MlInferenceEngine::FrameBus frames;
    MlInferenceEngine::DetectionBus detections;
    // A missing or mismatched engine throws. Report it as an error, not an uncaught-exception
    // abort.
    std::unique_ptr<MlInferenceEngine> enginePtr;
    try {
        enginePtr = std::make_unique<MlInferenceEngine>(cfg, frames, detections);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "inference_viewer: %s\n(run from the repository root)\n", e.what());
        return 1;
    }
    MlInferenceEngine& engine = *enginePtr;

    cv::VideoWriter writer;
    if (!snapshots.empty()) std::filesystem::create_directories(snapshots);

    // The first frames include one-off costs (CUDA context warm-up, lazy module loading), which are
    // not what the pipeline costs per frame in steady state. They are excluded from the statistics.
    constexpr int kWarmup = 10;
    std::vector<double> gpu, total;
    std::vector<size_t> boxCounts, signCounts;
    int laneFrames[4] = {0, 0, 0, 0};
    cv::Mat img;
    int n = 0;
    for (; n < maxFrames && cap.read(img); ++n) {
        if (img.cols != 2560 || img.rows != 1440) cv::resize(img, img, {2560, 1440});
        CameraFrame f{static_cast<std::uint64_t>(n), static_cast<std::uint64_t>(n * 1000 / 60),
                      img};
        const auto r = engine.process(f);
        if (n >= kWarmup) {
            gpu.push_back(r.gpuMs);
            total.push_back(r.totalMs);
        }
        boxCounts.push_back(r.boxes.size());
        signCounts.push_back(r.signs.size());
        if (n == dumpFrame) {
            const std::string base = "dump_" + std::to_string(n);
            cv::imwrite(base + ".png", img);  // lossless, so the reference sees identical pixels
            // Masks as a label image on the full prototype grid: pixel value = box index + 1 (in
            // the CSV's row order), 0 = no object. Later boxes overwrite earlier ones.
            if (!r.masks.masks.empty()) {
                const int gw = 1280 / r.masks.stride, gh = 736 / r.masks.stride;
                cv::Mat labels(gh, gw, CV_8UC1, cv::Scalar(0));
                for (std::size_t i = 0; i < r.masks.masks.size(); ++i) {
                    const ObjectMask& m = r.masks.masks[i];
                    for (int gy = m.y; gy < m.y + m.h; ++gy)
                        for (int gx = m.x; gx < m.x + m.w; ++gx)
                            if (m.atGrid(gx, gy)) labels.at<uint8_t>(gy, gx) = uint8_t(i + 1);
                }
                cv::imwrite(base + "_masks.png", labels);
            }
            std::FILE* csv = std::fopen((base + ".csv").c_str(), "w");
            std::fprintf(csv, "class,confidence,x,y,w,h\n");
            for (const Box& b : r.boxes) {
                std::fprintf(csv, "%d,%.4f,%.2f,%.2f,%.2f,%.2f\n", b.classId, b.confidence, b.x,
                             b.y, b.w, b.h);
            }
            std::fclose(csv);
        }
        for (int l = 0; l < 4; ++l) laneFrames[l] += !r.lanes[l].empty();

        const bool snap = !snapshots.empty() && n % every == 0;
        if (snap || !out.empty()) {
            cv::Mat vis = img.clone();
            draw(vis, r, engine.laneBand(img.cols, img.rows));
            if (snap) cv::imwrite(snapshots + "/frame_" + std::to_string(n) + ".jpg", vis);
            if (!out.empty()) {
                cv::resize(vis, vis, {1280, 720});
                if (!writer.isOpened()) {
                    // At the source's own frame rate, so the result plays at real speed.
                    const double fps = cap.get(cv::CAP_PROP_FPS);
                    writer.open(out, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                fps > 0 ? fps : 30, vis.size());
                }
                writer.write(vis);
            }
        }
    }

    size_t totalBoxes = 0, totalSigns = 0;
    for (size_t c : signCounts) totalSigns += c;
    for (size_t c : boxCounts) totalBoxes += c;
    std::printf("frames %d (first %d excluded from timing)\n", n, kWarmup);
    std::printf("gpu   ms: median %.2f  p95 %.2f  max %.2f\n", percentile(gpu, .5),
                percentile(gpu, .95), percentile(gpu, 1));
    std::printf("total ms: median %.2f  p95 %.2f  max %.2f  -> %.0f fps at the median\n",
                percentile(total, .5), percentile(total, .95), percentile(total, 1),
                1000.0 / std::max(1e-9, percentile(total, .5)));
    std::printf("boxes/frame: mean %.1f   signs/frame: mean %.2f\n",
                n ? double(totalBoxes) / n : 0.0, n ? double(totalSigns) / n : 0.0);
    std::printf("frames with lane present: outer-L %d  ego-L %d  ego-R %d  outer-R %d\n",
                laneFrames[0], laneFrames[1], laneFrames[2], laneFrames[3]);
    return 0;
}
