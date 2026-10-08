#pragma once
// CameraConfig — what CameraPipeline opens and how, the calibration it applies, and the frame-rate
// measurement Part 6.2 requires. See docs/BUILD_GUIDE.md Part 6, Part 12.1.
//
// Kept free of OpenCV (like CameraFrame's consumers in the unit suite) so the GPU-free unit suite
// can test the GStreamer pipeline strings, the YAML loading and the statistics.
//
// ---------------------------------------------------------------------------------------------
// Sources
//
//   "/dev/video0"           a UVC camera, opened through a GStreamer pipeline (Part 6.2)
//   "recording.mp4"         a video file, replayed (looped, paced at its own frame rate)
//   "gst:<pipeline>"        any GStreamer pipeline ending in an appsink, used verbatim: tests use
//                           "gst:videotestsrc ... ! appsink"
//
// Why GStreamer for the camera (Part 6.2): a UVC camera offers several pixel formats at each
// size. At 2560x1440, 30 fps, uncompressed YUYV needs ~1.8 Gbit/s, beyond USB 2.0 (480 Mbit/s)
// and far beyond what usbipd-win forwards comfortably; MJPG (compressed JPEG frames) is how such
// cameras deliver 2K at all. A GStreamer pipeline states the format, size and rate exactly, and
// fails loudly if the camera cannot do them, instead of OpenCV's V4L2 backend quietly picking
// something else.
//
// ---------------------------------------------------------------------------------------------
// The camera model published with the frames
//
// CameraPipeline UNDISTORTS every frame (Part 6.2). The pixels it publishes are therefore those of
// an ideal pinhole camera with ZERO lens distortion and a new camera matrix, not those of the
// calibrated lens. Everything that projects into the image (SceneReconstruction, RoadSurface-
// Projector, the renderer) must use that model, CameraPipeline::frameModel(), and never the raw
// calibration file: applying the lens distortion again to an already-undistorted frame would put
// LiDAR points and the navigation line in the wrong place, worst at the edges. See decisions.md.
// ---------------------------------------------------------------------------------------------

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "ar_drive_assist/common/Camera.h"

namespace ar_drive_assist {

struct CameraConfig {
    std::string source = "/dev/video0";
    int width = 2560;   // capture size: 2560x1440, or the 1920x1080 fallback (Part 6.2, Phase 4)
    int height = 1440;  // files are resized to it, so a replay matches the camera's geometry
    int fps = 30;
    std::string format = "MJPG";  // "MJPG" (JPEG-compressed, needed for 2K over USB) or "YUYV"
    std::string intrinsicsPath = "host/config/camera_intrinsics.yaml";
    bool undistort = true;  // only takes effect once the camera is calibrated (Part 12.1)
    bool loop = true;       // files: start again at the end
    bool realtime = true;   // files: publish at the file's own frame rate, like a camera would
    // files: per-frame capture times ("frame,t_us" CSV, a recording's camera.csv). When set, a
    // replay is paced by the recorded times instead of a fixed rate.
    std::string timestampsPath;
    double stallTimeoutS = 1.0;  // no frame for this long = the camera has failed (a FAULT)
    double reportEveryS = 10.0;  // how often the measured frame rate goes to the EventLog
};

enum class CameraSourceKind { Device, File, Pipeline };
CameraSourceKind cameraSourceKind(const std::string& source);

// The GStreamer pipeline for a Device or Pipeline source. Throws std::invalid_argument for an
// unknown format or a File source (files are opened directly).
std::string cameraGstPipeline(const CameraConfig& cfg);

// host/config/camera.yaml. Missing keys keep their defaults; a malformed value throws
// std::runtime_error naming the file and key.
CameraConfig loadCameraConfig(const std::string& path);

// A recording's per-frame times, microseconds, in frame order. Throws std::runtime_error.
std::vector<std::int64_t> loadFrameTimes(const std::string& csvPath);

struct CameraIntrinsics {
    bool calibrated = false;  // false for the template (empty matrix): frames are not undistorted
    CameraModel model;        // the LENS: raw-image pixels, with its distortion
    double reprojectionErrorPx = 0;
};
// config/camera_intrinsics.yaml (Part 12.1). An empty camera_matrix is the uncalibrated template;
// anything malformed (wrong count, non-positive focal length, centre outside the image) throws.
CameraIntrinsics loadCameraIntrinsics(const std::string& path);

// ---------------------------------------------------------------------------------------------
// FrameRateMonitor — the "measure and log actual sustained fps" of Part 6.2.
//
// Fed one capture timestamp per frame. Over a sliding window it reports the sustained rate, the
// longest gap between frames, and "late" frames: gaps longer than 1.5 nominal frame periods,
// which at a steady camera rate means at least one frame was lost on the way (USB, usbipd, the
// decoder). A sustained rate near the target with late frames is NOT a pass: the frames that
// matter most (a pedestrian stepping out) are as likely to be lost as any other.
// ---------------------------------------------------------------------------------------------
class FrameRateMonitor {
public:
    explicit FrameRateMonitor(double nominalFps, double windowS = 5.0)
        : periodMs_(1000.0 / nominalFps), windowMs_(windowS * 1000.0) {}

    void add(std::uint64_t timestampMs) {
        if (!stamps_.empty()) {
            const double gap = static_cast<double>(timestampMs - stamps_.back());
            if (gap > 1.5 * periodMs_) ++late_;
            if (gap > maxGapMs_) maxGapMs_ = gap;
        }
        stamps_.push_back(timestampMs);
        ++frames_;
        while (stamps_.size() > 2 &&
               static_cast<double>(stamps_.back() - stamps_.front()) > windowMs_)
            stamps_.pop_front();
    }

    // Frames per second over the window (0 until two frames have arrived).
    double fps() const {
        if (stamps_.size() < 2) return 0;
        const double span = static_cast<double>(stamps_.back() - stamps_.front());
        return span > 0 ? 1000.0 * static_cast<double>(stamps_.size() - 1) / span : 0;
    }
    std::uint64_t frames() const { return frames_; }
    std::uint64_t lateFrames() const { return late_; }  // since construction or resetCounters()
    double maxGapMs() const { return maxGapMs_; }       // likewise
    void resetCounters() {
        late_ = 0;
        maxGapMs_ = 0;
    }

private:
    double periodMs_, windowMs_;
    std::deque<std::uint64_t> stamps_;
    std::uint64_t frames_ = 0, late_ = 0;
    double maxGapMs_ = 0;
};

}  // namespace ar_drive_assist
