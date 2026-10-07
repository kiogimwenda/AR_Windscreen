// camera_check — Phase 4's exit check (Part 14): measure the sustained frame rate the camera
// actually delivers through usbipd at each candidate size, and say which size to use.
//
//   build/host/camera_check --list [--device /dev/video0]
//       the formats, sizes and rates the camera offers (v4l2-ctl)
//   build/host/camera_check [--device /dev/video0] [--format MJPG|YUYV] [--fps 30]
//                           [--sizes 2560x1440,1920x1080] [--seconds 20] [--snapshot DIR]
//       for each size, largest first: open it exactly as the system will (CameraPipeline), skip
//       2 s of warm-up, then measure for --seconds: sustained fps, gaps, late frames.
//       A size PASSES at >= 95 % of the requested rate with no late frame (a gap over 1.5 frame
//       periods: a lost frame). The first size that passes is the one to put in camera.yaml and
//       decisions.md (Part 6.2: never assume 2K, measure it).
//   --source <file | gst:pipeline> runs the same measurement on a replay or a test source.
//
// Run from the repository root with the camera attached to WSL
// (host/scripts/attach_usb_devices.ps1).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ar_drive_assist/camera/CameraPipeline.h"

using namespace ar_drive_assist;
using Clock = std::chrono::steady_clock;

namespace {

struct Result {
    int w = 0, h = 0;
    bool opened = false;
    std::string error;
    double fps = 0, maxGapMs = 0, p99GapMs = 0;
    std::uint64_t frames = 0, late = 0;
    bool pass = false;
};

Result measure(CameraConfig cfg, int w, int h, double seconds, const std::string& snapshotDir) {
    Result r;
    r.w = w;
    r.h = h;
    cfg.width = w;
    cfg.height = h;
    cfg.undistort = false;  // the transport is under test, not the calibration
    cfg.stallTimeoutS = 2.0;
    CameraPipeline::FrameBus bus;
    try {
        CameraPipeline cam(cfg, bus);
        r.opened = true;
        std::atomic<bool> stop{false};
        std::string runError;
        std::thread t([&] {
            try {
                cam.run(stop);
            } catch (const std::exception& e) {
                runError = e.what();
            }
        });
        const double nominal =
            cameraSourceKind(cfg.source) == CameraSourceKind::File ? cam.sourceFps() : cfg.fps;
        FrameRateMonitor mon(nominal, seconds + 1);
        std::vector<double> gaps;
        std::uint64_t last = 0;
        const auto start = Clock::now();
        const auto warm = start + std::chrono::seconds(2);
        const auto end = warm + std::chrono::duration<double>(seconds);
        bool saved = snapshotDir.empty();
        while (Clock::now() < end && runError.empty()) {
            CameraFrame f;
            if (!bus.pop(f)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (Clock::now() < warm) continue;
            if (last) gaps.push_back(static_cast<double>(f.timestampMs - last));
            last = f.timestampMs;
            mon.add(f.timestampMs);
            if (!saved) {
                std::filesystem::create_directories(snapshotDir);
                cv::imwrite(
                    snapshotDir + "/camera_" + std::to_string(w) + "x" + std::to_string(h) + ".png",
                    f.bgr);
                saved = true;
            }
        }
        stop = true;
        t.join();
        if (!runError.empty()) r.error = runError;
        r.fps = mon.fps();
        r.frames = mon.frames();
        r.late = mon.lateFrames();
        r.maxGapMs = mon.maxGapMs();
        if (!gaps.empty()) {
            std::sort(gaps.begin(), gaps.end());
            r.p99GapMs = gaps[std::min(gaps.size() - 1, gaps.size() * 99 / 100)];
        }
        r.pass = r.error.empty() && r.fps >= 0.95 * nominal && r.late == 0;
    } catch (const std::exception& e) {
        r.error = e.what();
    }
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    CameraConfig cfg;
    std::vector<std::pair<int, int>> sizes = {{2560, 1440}, {1920, 1080}};
    double seconds = 20;
    std::string snapshotDir;
    bool list = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool more = i + 1 < argc;
        if (a == "--list") {
            list = true;
        } else if ((a == "--device" || a == "--source") && more) {
            cfg.source = argv[++i];
        } else if (a == "--format" && more) {
            cfg.format = argv[++i];
        } else if (a == "--fps" && more) {
            cfg.fps = std::atoi(argv[++i]);
        } else if (a == "--seconds" && more) {
            seconds = std::atof(argv[++i]);
        } else if (a == "--snapshot" && more) {
            snapshotDir = argv[++i];
        } else if (a == "--sizes" && more) {
            sizes.clear();
            std::string s = argv[++i];
            for (std::size_t p = 0; p < s.size();) {
                const std::size_t c = s.find(',', p);
                const std::string one = s.substr(p, c == std::string::npos ? s.npos : c - p);
                int w = 0, h = 0;
                if (std::sscanf(one.c_str(), "%dx%d", &w, &h) == 2) sizes.emplace_back(w, h);
                p = c == std::string::npos ? s.size() : c + 1;
            }
        } else {
            std::fprintf(stderr,
                         "usage: see the comment at the top of host/tools/camera_check.cpp\n");
            return 2;
        }
    }
    if (list) {
        const std::string cmd = "v4l2-ctl --list-formats-ext -d " + cfg.source;
        if (std::system(cmd.c_str()) != 0)
            std::fprintf(stderr,
                         "v4l2-ctl failed: install v4l-utils, and check the camera is "
                         "attached to WSL (ls /dev/video*)\n");
        return 0;
    }

    std::printf("camera_check: %s, %s, %d fps requested, %.0f s per size after 2 s warm-up\n\n",
                cfg.source.c_str(), cfg.format.c_str(), cfg.fps, seconds);
    std::printf("%-10s %-6s %8s %9s %9s %6s %8s\n", "size", "result", "fps", "max gap", "p99 gap",
                "late", "frames");
    const Result* chosen = nullptr;
    std::vector<Result> results;
    results.reserve(sizes.size());
    for (const auto& [w, h] : sizes) {
        results.push_back(measure(cfg, w, h, seconds, snapshotDir));
        const Result& r = results.back();
        char size[16];
        std::snprintf(size, sizeof size, "%dx%d", w, h);
        if (!r.opened) {
            std::printf("%-10s %-6s %s\n", size, "FAIL", r.error.c_str());
            continue;
        }
        std::printf("%-10s %-6s %8.2f %7.0f ms %7.0f ms %6llu %8llu%s%s\n", size,
                    r.pass ? "PASS" : "FAIL", r.fps, r.maxGapMs, r.p99GapMs,
                    static_cast<unsigned long long>(r.late),
                    static_cast<unsigned long long>(r.frames), r.error.empty() ? "" : "  ",
                    r.error.c_str());
        if (r.pass && !chosen) chosen = &r;
    }
    std::printf("\n");
    if (chosen) {
        std::printf(
            "Use %dx%d: set width/height in host/config/camera.yaml, record this table in "
            "docs/decisions.md (Phase 4), and recalibrate at that size (Part 12.1).\n",
            chosen->w, chosen->h);
        return 0;
    }
    std::printf(
        "No size passed. Try --format MJPG, a lower --fps, or a different USB port; see "
        "BUILD_GUIDE Part 2 (usbipd) and Part 6.2.\n");
    return 1;
}
