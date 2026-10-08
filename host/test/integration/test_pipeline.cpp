// The glue between the host's threads (Part 5.1): FusionThread's measurements, DecisionThread's
// cycle, RenderThread's inputs, the replay's injected obstacle and recordings. No GPU or sensor is
// used; this lives with the integration tests only because the message headers include the
// inference engine's (CUDA) types.   ctest --test-dir build -L pipeline
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <opencv2/videoio.hpp>
#include <thread>

#include "ar_drive_assist/camera/CameraConfig.h"
#include "ar_drive_assist/decision/DecisionThread.h"
#include "ar_drive_assist/lidar/InjectedObstacle.h"
#include "ar_drive_assist/render/RenderThread.h"
#include "ar_drive_assist/scene/FusionThread.h"
#include "ar_drive_assist/system/Recorder.h"
#include "ar_drive_assist/system/Recording.h"

using namespace ar_drive_assist;

namespace {

FusedObject lidarObject(double x, double y, int cls, int points, int detection) {
    FusedObject o;
    o.detection = detection;
    o.classId = cls;
    o.source = RangeSource::LIDAR;
    o.position = {x + 1.5, y, 0.8};
    o.nearFace = {x, y, 0.8};
    o.points = points;
    return o;
}

struct Fixture {
    MlInferenceEngine::DetectionBus det;
    SceneBus scenes;
    EgoEstimator ego;
    FusionThread fusion{FusionThreadConfig{}, det, nullptr, nullptr, ego, scenes};
};

DecisionThresholds thresholds() {
    DecisionThresholds t;
    t.ttcBrakeThresholdS = 1.8;
    t.hardBrakeDecelG = 0.4;
    t.tailgatingMinGapS = 1.0;
    t.brakeActuatorMaxIntensity = 90;
    t.brakeRequestIntensity = 60;
    t.egoPathHalfWidthM = 1.2;
    t.minClosingSpeedMps = 0.5;
    t.hubStateMaxAgeMs = 100;
    t.brakeMinMeasuredTrackMs = 200;
    t.brakeConfirmMs = 100;
    return t;
}

HubState armed(std::uint64_t now) {
    HubState h;
    h.haveReport = true;
    h.reportMs = now;
    h.killSwitchEngaged = 0;
    h.haveAck = true;
    return h;
}

// A scene in which a stopped object sits `gap` ahead of the bumper of a car doing 10 m/s, tracked
// for `frames` 100 ms frames with LiDAR.
SceneSnapshot sceneWithStoppedObject(double gapM, int frames) {
    MultiObjectTracker tracker;
    const double x0 = gapM + 3.5 + 10 * 0.1 * (frames - 1);
    for (int k = 0; k < frames; ++k) {
        MeasurementBatch b;
        b.timestampMs = 1000 + 100 * k;
        b.ego.x = 10 * 0.1 * k;  // the car moves east at 10 m/s
        ObjectMeasurement m{
            {x0 - b.ego.x, 0}, Eigen::Matrix2d::Identity() * 0.04, -1, std::nullopt, true};
        m.posVehicle = {x0 - b.ego.x, 0};
        b.measurements = {m};
        tracker.update(b);
    }
    SceneSnapshot s;
    s.timestampMs = 1000 + 100 * (frames - 1);
    s.egoValid = true;
    s.ego.pose = {10 * 0.1 * (frames - 1), 0, 0};
    s.ego.speedMps = 10;
    s.ego.timestampMs = s.timestampMs;
    s.egoX << s.ego.pose.x, 0, 0, 10, 0;
    s.egoP = ImmFilter::Cov::Identity() * 0.01;
    s.tracks = tracker.tracks();
    return s;
}

}  // namespace

TEST(FusionMeasurements, OneSurfaceSeenByTwoDetectionsIsOneMeasurement) {
    Fixture f;
    SceneModel scene;
    // A box detected as a vehicle (many points) and an occluded cyclist's partial mask picking the
    // same LiDAR cluster: the same near face. Plus a separate pedestrian 0.6 m to the side.
    scene.objects = {lidarObject(10, 0, -1, 40, 0), lidarObject(10.02, 0.05, 2, 300, 1),
                     lidarObject(10, 0.6, 1, 50, 2)};
    FusionFrame frame;
    frame.detections.resize(3);
    const auto m = f.fusion.measurements(scene, frame);
    ASSERT_EQ(m.size(), 2u) << "the duplicate merges; the pedestrian 0.6 m away does not";
    EXPECT_EQ(m[0].objectClass, 2) << "the best-supported (300 points) is kept";
    EXPECT_NEAR(m[0].posVehicle.x(), 10.02, 1e-9) << "the NEAR FACE is measured, not the median";
}

TEST(FusionMeasurements, AMeasuredRangeIsNeverDisplacedByAnEstimateAndSignsAreNotTracked) {
    Fixture f;
    SceneModel scene;
    FusedObject est = lidarObject(10, 0, 0, 999, 0);
    est.source = RangeSource::ESTIMATED;
    est.position = {10.05, 0, 0.8};  // an estimate is tracked at its position (no near face)
    FusedObject sign = lidarObject(20, 3, 3, 30, 2);
    scene.objects = {est, lidarObject(10.1, 0, 0, 20, 1), sign};
    UnknownObstacle u;
    u.nearFace = {6, -0.5, 0.5};
    u.points = 10;
    scene.unknown = {u};
    FusionFrame frame;
    frame.detections.resize(3);
    frame.detections[2].isSign = true;
    const auto m = f.fusion.measurements(scene, frame);
    ASSERT_EQ(m.size(), 2u);
    EXPECT_TRUE(m[0].rangeMeasured) << "the LiDAR range wins over the estimate";
    EXPECT_EQ(m[1].objectClass, -1) << "the LiDAR-only obstacle is tracked unclassified";
}

TEST(DecisionCycle, BrakesOnAStoppedObjectOnlyWhenArmedAndAfterTheMeasuredRun) {
    DecisionThreadConfig cfg;
    cfg.thresholds = thresholds();
    cfg.thresholds.brakeConfirmMs = 0;  // single decisions here; the onset is the arbiter's test
    cfg.vehicle.frontBumperFromRearAxleM = 3.5;
    SceneBus scenes;
    DecisionThread d(cfg, scenes, nullptr);

    const SceneSnapshot s = sceneWithStoppedObject(12, 8);  // TTC ~1.2 s, tracked 700 ms
    const DecisionSnapshot yes = d.process(s, armed(s.timestampMs), s.timestampMs);
    EXPECT_EQ(yes.decision.request.type, schema::RequestType_BRAKE);
    ASSERT_FALSE(yes.tracks.empty());
    EXPECT_TRUE(yes.tracks[0].inEgoPath);
    EXPECT_NEAR(yes.tracks[0].gapM, 12, 0.3);
    EXPECT_NEAR(yes.tracks[0].closingSpeedMps, 10, 0.5);

    HubState notArmed = armed(s.timestampMs);
    notArmed.killSwitchEngaged = 1;
    EXPECT_NE(d.process(s, notArmed, s.timestampMs).decision.request.type,
              schema::RequestType_BRAKE);

    const SceneSnapshot young = sceneWithStoppedObject(12, 3);  // confirmed, but only 200 ms...
    const DecisionSnapshot early = d.process(young, armed(young.timestampMs), young.timestampMs);
    EXPECT_EQ(early.decision.request.type, schema::RequestType_BRAKE)
        << "a 200 ms measured run is exactly enough";
}

TEST(RenderInputs, HazardsAndEgoSpeedComeFromTheSceneAndTheDecision) {
    SceneSnapshot s = sceneWithStoppedObject(12, 8);
    DecisionThreadConfig cfg;
    cfg.thresholds = thresholds();
    cfg.vehicle.frontBumperFromRearAxleM = 3.5;
    SceneBus scenes;
    DecisionThread d(cfg, scenes, nullptr);
    const DecisionSnapshot dec = d.process(s, armed(s.timestampMs), s.timestampMs);
    CameraFrame frame;
    frame.timestampMs = s.timestampMs;
    const Eigen::Isometry3d cfv = cameraFromVehicle({1.5, 0, 1.3}, 0, 0, 0);
    const RenderInputs in = RenderThread::inputs(frame, &s, &dec, nullptr, cfv);
    ASSERT_TRUE(in.egoSpeedMps.has_value());
    EXPECT_NEAR(*in.egoSpeedMps, 10, 1e-9);
    ASSERT_EQ(in.hazards.size(), 1u);
    EXPECT_NEAR(in.hazards[0].gapM, 12, 0.3);
    EXPECT_TRUE(in.hazards[0].inBrakePath) << "straight ahead: the barrier is drawn";
    EXPECT_GT(in.hazards[0].depthM, 10);  // in front of the camera
    EXPECT_EQ(in.lanes, &s.detections);
    EXPECT_FALSE(in.systemNotice.has_value());
    s.extrinsicsDegraded = true;  // Part 12.2.1: the driver is told to recalibrate
    const RenderInputs bad = RenderThread::inputs(frame, &s, &dec, nullptr, cfv);
    ASSERT_TRUE(bad.systemNotice.has_value());
    EXPECT_NE(bad.systemNotice->find("recalibrate"), std::string::npos);
}

TEST(InjectedObstacle, StaysOnTheRoadTheCarDrove) {
    // A recorded drive: 5 m/s east for 4 s, then turning left (north) along a quarter circle of
    // radius 20 m. The obstacle appears at t = 0, 30 m ahead along that road: in the bend.
    const std::string dir = ::testing::TempDir() + "obs_rec";
    std::filesystem::create_directories(dir);
    std::ofstream hub(dir + "/hub.csv");
    hub << "t_us,lat,lon,speed_kph,gps_fix_valid,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,"
           "gyro_y_dps,gyro_z_dps,heading_deg,obd_speed_kph\n";
    const double lat0 = -1.28, lon0 = 36.82, R = 6378137.0, d2r = 3.14159265358979323846 / 180;
    auto emit = [&](double t, double e, double n, double psiDeg) {
        char line[256];
        std::snprintf(line, sizeof line, "%lld,%.9f,%.9f,18,1,0,0,1,0,0,0,%.3f,18\n",
                      static_cast<long long>(t * 1e6), lat0 + n / R / d2r,
                      lon0 + e / (R * std::cos(lat0 * d2r)) / d2r, std::fmod(450 - psiDeg, 360.0));
        hub << line;
    };
    double t = 0;
    for (; t < 4.0; t += 0.1) emit(t, 5 * t, 0, 0);
    for (double a = 0; a <= 90; a += 1.43, t += 0.1)
        emit(t, 20 + 20 * std::sin(a * d2r), 20 - 20 * std::cos(a * d2r), a);
    hub.close();

    InjectedObstacle o;
    o.appearS = 0;
    o.startGapM = 30;
    const ObstacleModel m(o, dir + "/hub.csv");
    const auto at0 = m.poseAt(0);
    ASSERT_TRUE(at0.has_value());
    // 30 m along: 20 m straight, then 10 m into the bend (an angle of 0.5 rad), so to the LEFT.
    EXPECT_GT(at0->translation().y(), 1.5);
    EXPECT_LT(at0->translation().x(), 30);
    // Driving the road, the car reaches it: when the car has done 25 m it is 5 m ahead along the
    // bend, which puts it R (1 - cos(5 / R)) = 0.62 m to the left of the car's heading.
    const auto at5 = m.poseAt(5.0);
    ASSERT_TRUE(at5.has_value());
    EXPECT_NEAR(at5->translation().norm(), 5.0, 0.3);
    EXPECT_NEAR(at5->translation().y(), 20 * (1 - std::cos(5.0 / 20)), 0.15);
    EXPECT_FALSE(m.poseAt(6.5).has_value()) << "reached";
}

TEST(Recording, InconsistentTransformsAreRefused) {
    const std::string dir = ::testing::TempDir() + "bad_rec";
    std::filesystem::create_directories(dir);
    std::ofstream(dir + "/recording.yaml") << "name: x\nwidth: 640\nheight: 480\n";
    std::ofstream(dir + "/extrinsics.yaml")
        << "camera_from_lidar: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n"
           "vehicle_from_lidar: [1,0,0,1, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n"
           "camera_from_vehicle: [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]\n";
    EXPECT_THROW(loadRecording(dir), std::runtime_error);
}

// What the Recorder writes, the replay's own loaders read back: the same frames, scans, times,
// camera model and extrinsics (a truncated intrinsics file was the first bug this caught).
TEST(Recorder, WritesARecordingTheReplayReads) {
    const std::string dir = ::testing::TempDir() + "recorder_rt";
    std::filesystem::remove_all(dir);
    RecorderConfig rc;
    rc.dir = dir;
    rc.frameModel.width = 320;
    rc.frameModel.height = 240;
    rc.frameModel.fx = 300.5;
    rc.frameModel.fy = 301.25;
    rc.frameModel.cx = 161.0;
    rc.frameModel.cy = 119.5;
    rc.fps = 10;
    rc.extrinsics.cameraFromVehicle = cameraFromVehicle({1.5, 0, 1.3}, 0, 2, 0);
    rc.extrinsics.vehicleFromLidar.translation() = Eigen::Vector3d(0.5, 0, 1.8);
    rc.extrinsics.cameraFromLidar =
        rc.extrinsics.cameraFromVehicle * rc.extrinsics.vehicleFromLidar;
    {
        Recorder r(rc);
        std::atomic<bool> stop{false};
        std::thread t([&] { r.run(stop); });
        for (int k = 0; k < 5; ++k) {
            CameraFrame f;
            f.timestampMs = 1000 + 100 * k;
            f.bgr = cv::Mat(240, 320, CV_8UC3, cv::Scalar(40 * k, 100, 200));
            r.onFrame(f);
            LidarFrame s;
            s.timestampMs = 1050 + 100 * k;
            for (int i = 0; i <= k; ++i) {
                LidarPoint p;
                p.p = {10.0f + i, -1.5f, 0.25f};
                p.intensity = 80;
                s.points.push_back(p);
            }
            r.onScan(s);
            hub_protocol::SensorReport rep{};
            rep.latitude = -1.28;
            rep.longitude = 36.82;
            rep.speedKph = 20;
            rep.gpsFixValid = 1;
            rep.accelZ = 1;
            rep.obdSpeedKph = k == 2 ? NAN : 20.0f;  // not valid: an empty cell
            r.onReport(rep, 1000 + 100 * k);
        }
        while (r.stats().frames < 5 || r.stats().scans < 5)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        stop = true;
        t.join();
        EXPECT_EQ(r.stats().droppedFrames + r.stats().droppedScans, 0u);
    }
    const Recording rec = loadRecording(dir);
    EXPECT_EQ(rec.width, 320);
    EXPECT_TRUE(rec.cameraFromLidar.isApprox(rc.extrinsics.cameraFromLidar, 1e-6));
    EXPECT_TRUE(rec.vehicleFromLidar.isApprox(rc.extrinsics.vehicleFromLidar, 1e-6));
    const CameraIntrinsics in = loadCameraIntrinsics(rec.intrinsics());
    EXPECT_DOUBLE_EQ(in.model.fy, 301.25);
    EXPECT_DOUBLE_EQ(in.model.cy, 119.5);
    const auto times = loadScanTimes(dir);
    ASSERT_EQ(times.size(), 5u);
    EXPECT_EQ(times[3].tUs, 1350000);
    const auto pts = readScan(rec.scanFile(times[3].index));
    ASSERT_EQ(pts.size(), 4u);
    EXPECT_FLOAT_EQ(pts[3].x(), 13.0f);
    EXPECT_FLOAT_EQ(pts[3].w(), 80.0f);
    cv::VideoCapture v(rec.video());
    int n = 0;
    cv::Mat img;
    while (v.read(img)) ++n;
    EXPECT_EQ(n, 5);
    std::ifstream hub(rec.hubReports());
    std::string line;
    int rows = 0, emptyObd = 0;
    std::getline(hub, line);
    while (std::getline(hub, line)) {
        ++rows;
        emptyObd += line.back() == ',';
    }
    EXPECT_EQ(rows, 5);
    EXPECT_EQ(emptyObd, 1);
    EXPECT_THROW(Recorder r2(rc), std::runtime_error) << "a recording is never overwritten";
}
