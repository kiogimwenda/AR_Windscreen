// DecisionArbiter tests — see docs/BUILD_GUIDE.md Part 9.3 and Part 13.1. Phase 10's hard gate:
// this file must pass in full, INCLUDING the ceiling tests, before Phase 12 (brake hardware).
//
// Every rule of Part 9.3 is exercised, both where it must fire and where it must not; every arming
// condition; the ceiling (a fuzz over random, broken and hostile inputs and thresholds, plus the
// real config file against the firmware's ceiling); the evidence trail; and an end-to-end run
// through the real tracker.

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <regex>
#include <sstream>

#include "ar_drive_assist/decision/DecisionArbiter.h"
#include "ar_drive_assist/system/SystemManager.h"

using namespace ar_drive_assist;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

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

// Tracks live here so EgoRelativeTrack::track pointers stay valid. A deque, not a vector: a
// vector's push_back may reallocate and leave earlier pointers dangling (AddressSanitizer found
// exactly that in the first version).
std::deque<Track> gTracks;
const Track* trackWithId(int id) {
    for (auto& t : gTracks)
        if (t.id == id) return &t;
    Track t{id,
            0,
            TrackState::CONFIRMED,
            ImmFilter({0, 0}, Eigen::Matrix2d::Identity(), MotionNoise{}),
            3,
            0,
            0,
            0,
            {},
            {}};
    gTracks.push_back(t);
    return &gTracks.back();
}

// A qualifying object: confirmed, LiDAR-ranged without a break for 0.5 s, in the ego path (both
// the curved and the straight one).
EgoRelativeTrack object(int id, double gapM, double closingMps) {
    EgoRelativeTrack o;
    o.track = trackWithId(id);
    o.xM = gapM + 3.5;
    o.gapM = gapM;
    o.closingSpeedMps = closingMps;
    o.inEgoPath = true;
    o.inStraightPath = true;
    o.measuredRunMs = 500;
    o.confirmed = true;
    o.rangeMeasured = true;
    return o;
}

HubState armedHub(std::uint64_t nowMs) {
    HubState h;
    h.haveReport = true;
    h.reportMs = nowMs - 20;
    h.brakePedalActive = 0;
    h.killSwitchEngaged = 0;
    h.haveAck = true;
    h.actuatorFaultCode = 0;
    return h;
}

ArbiterInput input(std::vector<EgoRelativeTrack> objs, std::uint64_t nowMs = 10000) {
    ArbiterInput in;
    in.nowMs = nowMs;
    in.ego.speedMps = 15;
    in.objects = std::move(objs);
    for (auto& o : in.objects)
        if (o.lastMeasuredMs == 0) o.lastMeasuredMs = nowMs;  // measured this cycle, by default
    in.hub = armedHub(nowMs);
    return in;
}

}  // namespace

// --- Rule 1 ------------------------------------------------------------------------------------

// 15 m gap closing at 10 m/s: TTC 1.5 s < 1.8 s. Armed, pedal up: BRAKE at the configured
// intensity, reason FCW_TTC.
TEST(Arbiter, Rule1BrakesBelowTheTtcThreshold) {
    const DecisionArbiter a(thresholds());
    const Decision d = a.evaluate(input({object(1, 15, 10)}));
    EXPECT_EQ(d.rule, 1);
    EXPECT_EQ(d.request.type, RequestType::RequestType_BRAKE);
    EXPECT_EQ(d.request.intensity, 60);
    EXPECT_EQ(d.request.reason_code, static_cast<std::uint8_t>(ReasonCode::FCW_TTC));
    EXPECT_NEAR(d.ttcS, 1.5, 1e-9);
    EXPECT_EQ(d.targetTrackId, 1);
}

TEST(Arbiter, Rule1DoesNotBrakeAtOrAboveTheThreshold) {
    const DecisionArbiter a(thresholds());
    EXPECT_NE(a.evaluate(input({object(1, 18, 10)})).request.type,
              RequestType::RequestType_BRAKE);  // 1.8 s
    EXPECT_NE(a.evaluate(input({object(1, 30, 10)})).request.type,
              RequestType::RequestType_BRAKE);  // 3.0 s
}

// The nearest-in-time object decides, not the nearest in distance.
TEST(Arbiter, Rule1UsesTheSmallestTimeToCollision) {
    const DecisionArbiter a(thresholds());
    const Decision d = a.evaluate(input({object(1, 10, 2), object(2, 20, 15)}));  // 5 s vs 1.33 s
    EXPECT_EQ(d.request.type, RequestType::RequestType_BRAKE);
    EXPECT_EQ(d.targetTrackId, 2);
}

TEST(Arbiter, DriverBrakingSuppressesRule1ButUnknownPedalDoesNot) {
    const DecisionArbiter a(thresholds());
    auto in = input({object(1, 15, 10)});
    in.hub.brakePedalActive = 1;  // the driver is already braking
    EXPECT_NE(a.evaluate(in).request.type, RequestType::RequestType_BRAKE);
    in.hub.brakePedalActive = 0xFF;  // unknown (vehicle has no OBD brake PID): Part 9.3 allows
    EXPECT_EQ(a.evaluate(in).request.type, RequestType::RequestType_BRAKE);
}

// Each object condition on its own blocks braking.
TEST(Arbiter, OnlyConfirmedMeasuredInPathClosingObjectsCanBrake) {
    const DecisionArbiter a(thresholds());
    auto check = [&](EgoRelativeTrack o, const char* why) {
        EXPECT_NE(a.evaluate(input({o})).request.type, RequestType::RequestType_BRAKE) << why;
    };
    auto o = object(1, 15, 10);
    o.confirmed = false;
    check(o, "tentative or coasting on prediction");
    o = object(1, 15, 10);
    o.rangeMeasured = false;
    check(o, "range only estimated (MiDaS) or stale");
    o = object(1, 15, 10);
    o.inEgoPath = false;
    check(o, "adjacent lane");
    o = object(1, 15, 10);
    o.inStraightPath = false;
    check(o, "in the curved path only (a corner exit sweeping across a parked car)");
    o = object(1, 15, 10);
    o.measuredRunMs = 100;
    check(o, "measured for only 100 ms without a break (a re-associated ghost)");
    o = object(1, 15, 10);
    o.measuredRunMs = 200;
    EXPECT_EQ(a.evaluate(input({o})).request.type, RequestType::RequestType_BRAKE)
        << "exactly brake_min_measured_track_ms is enough";
    check(object(1, 15, 0.3), "closing slower than min_closing_speed_mps");
    check(object(1, 15, -5), "pulling away");
    check(object(1, -1, 10), "behind the bumper");
}

TEST(Arbiter, NonFiniteNumbersNeverBrake) {
    const DecisionArbiter a(thresholds());
    for (double gap : {kNaN, kInf, -kInf})
        EXPECT_NE(a.evaluate(input({object(1, gap, 10)})).request.type,
                  RequestType::RequestType_BRAKE)
            << gap;
    for (double cl : {kNaN, kInf})
        EXPECT_NE(a.evaluate(input({object(1, 15, cl)})).request.type,
                  RequestType::RequestType_BRAKE)
            << cl;
}

// --- Arming ------------------------------------------------------------------------------------

TEST(Arbiter, NotArmedMeansNoBrake) {
    const DecisionArbiter a(thresholds());
    auto base = input({object(1, 15, 10)});
    ASSERT_EQ(a.evaluate(base).request.type, RequestType::RequestType_BRAKE);
    auto in = base;
    in.hub.killSwitchEngaged = 1;
    EXPECT_NE(a.evaluate(in).request.type, RequestType::RequestType_BRAKE) << "kill switch";
    in = base;
    in.hub.actuatorFaultCode = 3;
    EXPECT_NE(a.evaluate(in).request.type, RequestType::RequestType_BRAKE) << "hub fault";
    in = base;
    in.hub.reportMs = in.nowMs - 150;
    EXPECT_NE(a.evaluate(in).request.type, RequestType::RequestType_BRAKE)
        << "hub report 150 ms old";
    in = base;
    in.hub.haveReport = false;
    EXPECT_NE(a.evaluate(in).request.type, RequestType::RequestType_BRAKE)
        << "never heard from the hub";
    in = base;
    in.hub.haveAck = false;
    EXPECT_NE(a.evaluate(in).request.type, RequestType::RequestType_BRAKE) << "no AckStatus yet";
    in = base;
    in.hub.reportMs = in.nowMs + 50;
    EXPECT_NE(a.evaluate(in).request.type, RequestType::RequestType_BRAKE)
        << "report from the future";
    in = base;
    in.eventLogHealthy = false;
    EXPECT_NE(a.evaluate(in).request.type, RequestType::RequestType_BRAKE)
        << "cannot record evidence";
    EXPECT_FALSE(a.evaluate(HubState{}.haveReport ? base : [&] {
                          auto x = base;
                          x.hub = HubState{};
                          return x;
                      }())
                     .armed)
        << "a default HubState is not armed (kill switch assumed engaged)";
}

// --- Rules 2, 3, 4 -----------------------------------------------------------------------------

TEST(Arbiter, Rule2HazardsOnHardBraking) {
    const DecisionArbiter a(thresholds());
    auto in = input({});
    in.ego.longitudinalAccelMps2 = -0.5 * 9.80665;
    const Decision d = a.evaluate(in);
    EXPECT_EQ(d.rule, 2);
    EXPECT_EQ(d.request.type, RequestType::RequestType_HAZARDS);
    EXPECT_EQ(d.request.reason_code, static_cast<std::uint8_t>(ReasonCode::HARD_BRAKE_DETECTED));
    in.ego.longitudinalAccelMps2 = -0.3 * 9.80665;
    EXPECT_EQ(a.evaluate(in).request.type, RequestType::RequestType_NONE);
    in.ego.longitudinalAccelMps2 = +0.5 * 9.80665;  // accelerating hard is not braking
    EXPECT_EQ(a.evaluate(in).request.type, RequestType::RequestType_NONE);
}

TEST(Arbiter, Rule1TakesPriorityOverRule2) {
    const DecisionArbiter a(thresholds());
    auto in = input({object(1, 15, 10)});
    in.ego.longitudinalAccelMps2 = -0.5 * 9.80665;
    EXPECT_EQ(a.evaluate(in).request.type, RequestType::RequestType_BRAKE);
    in.hub.killSwitchEngaged = 1;  // cannot brake: rule 2 still applies
    EXPECT_EQ(a.evaluate(in).request.type, RequestType::RequestType_HAZARDS);
}

// A CRITICAL reckless-driving assessment ahead gives a WARNING and never an actuation.
TEST(Arbiter, Rule3WarnsButNeverActuates) {
    const DecisionArbiter a(thresholds());
    auto in = input({});
    ThreatAssessment t;
    t.trackId = 7;
    t.inEgoPath = true;
    t.level = ThreatLevel::CRITICAL;
    t.flags = SWERVING | ERRATIC_SPEED;
    t.risk = 1.0;
    in.threats = {t};
    const Decision d = a.evaluate(in);
    EXPECT_EQ(d.rule, 3);
    EXPECT_EQ(d.request.type, RequestType::RequestType_NONE);
    ASSERT_EQ(d.warnings.size(), 1u);
    EXPECT_EQ(d.targetTrackId, 7);
    t.inEgoPath = false;  // not ahead: no warning from the arbiter
    in.threats = {t};
    EXPECT_EQ(a.evaluate(in).rule, 4);
}

TEST(Arbiter, Rule4IsAZeroedRequest) {
    const DecisionArbiter a(thresholds());
    const Decision d = a.evaluate(input({}));
    EXPECT_EQ(d.rule, 4);
    EXPECT_EQ(d.request.type, RequestType::RequestType_NONE);
    EXPECT_EQ(d.request.intensity, 0);
    EXPECT_EQ(d.request.reason_code, 0);
    EXPECT_EQ(d.request.timestamp_ms, 10000u);
}

// --- The ceiling (Part 13.1: "no combination of inputs produces a BRAKE above
// brake_actuator_max_intensity") -----------------------------------------------------------------

// 200,000 random cycles: random objects (including NaN, infinities, negative and huge values),
// random threats, random hub states, and random THRESHOLDS, including values the config loader
// would reject (intensity requests up to 255, ceilings up to 255). Whenever a BRAKE comes out:
//   - its intensity is within the configured request, the configured ceiling AND the hub's 90;
//   - the system was armed, the pedal was not pressed;
//   - a confirmed, measured, in-path, finite, closing object was below the TTC threshold.
TEST(ArbiterCeiling, NoInputCombinationExceedsTheCeilingOrBypassesTheConditions) {
    std::mt19937 rng(20260927);
    std::uniform_real_distribution<double> u(0, 1);
    auto wild = [&](double lo, double hi) {
        const double p = u(rng);
        if (p < 0.03) return kNaN;
        if (p < 0.05) return u(rng) < 0.5 ? kInf : -kInf;
        return lo + (hi - lo) * u(rng);
    };
    int brakes = 0;
    for (int k = 0; k < 200000; ++k) {
        DecisionThresholds th = thresholds();
        th.brakeRequestIntensity = static_cast<std::uint8_t>(rng() % 256);
        th.brakeActuatorMaxIntensity = static_cast<std::uint8_t>(rng() % 256);
        th.ttcBrakeThresholdS = 0.5 + 3 * u(rng);
        const DecisionArbiter a(th);
        ArbiterInput in = input({}, 5000 + rng() % 100000);
        const int n = rng() % 4;
        for (int j = 0; j < n; ++j) {
            EgoRelativeTrack o = object(j + 1, wild(-5, 80), wild(-20, 30));
            o.confirmed = u(rng) < 0.8;
            o.rangeMeasured = u(rng) < 0.8;
            o.inEgoPath = u(rng) < 0.7;
            o.inStraightPath = u(rng) < 0.8;
            o.measuredRunMs = static_cast<std::uint64_t>(u(rng) * 400);
            in.objects.push_back(o);
        }
        if (u(rng) < 0.3) {
            ThreatAssessment t;
            t.trackId = 99;
            t.inEgoPath = true;
            t.level = ThreatLevel::CRITICAL;
            in.threats = {t};
        }
        in.hub.killSwitchEngaged = u(rng) < 0.15;
        in.hub.actuatorFaultCode = u(rng) < 0.1 ? 2 : 0;
        in.hub.brakePedalActive = std::array<std::uint8_t, 3>{0, 1, 0xFF}[rng() % 3];
        in.hub.haveReport = u(rng) < 0.95;
        in.hub.reportMs = in.nowMs - static_cast<std::uint64_t>(u(rng) * 200);
        in.eventLogHealthy = u(rng) < 0.95;
        in.ego.longitudinalAccelMps2 = wild(-8, 4);

        const Decision d = a.evaluate(in);
        if (d.request.type != RequestType::RequestType_BRAKE) continue;
        ++brakes;
        ASSERT_LE(d.request.intensity, th.brakeActuatorMaxIntensity) << "k=" << k;
        ASSERT_LE(d.request.intensity, th.brakeRequestIntensity) << "k=" << k;
        ASSERT_LE(d.request.intensity, kHubMaxSafeBrakeIntensity) << "k=" << k;
        ASSERT_TRUE(a.armed(in)) << "k=" << k;
        ASSERT_NE(in.hub.brakePedalActive, 1) << "k=" << k;
        bool justified = false;
        for (const auto& o : in.objects) {
            justified =
                justified || (o.confirmed && o.rangeMeasured && o.inEgoPath && o.inStraightPath &&
                              o.measuredRunMs >= th.brakeMinMeasuredTrackMs &&
                              std::isfinite(o.gapM) && std::isfinite(o.closingSpeedMps) &&
                              o.gapM > 0 && o.closingSpeedMps > th.minClosingSpeedMps &&
                              o.gapM / o.closingSpeedMps < th.ttcBrakeThresholdS);
        }
        ASSERT_TRUE(justified) << "k=" << k;
    }
    EXPECT_GT(brakes, 1000) << "the fuzz must actually reach the brake branch";
}

// A thresholds struct built wrongly IN CODE (not through the loader) still cannot exceed the
// hub's ceiling.
TEST(ArbiterCeiling, HubCeilingHoldsEvenWithBadThresholdsInCode) {
    DecisionThresholds th = thresholds();
    th.brakeRequestIntensity = 255;
    th.brakeActuatorMaxIntensity = 255;
    const DecisionArbiter a(th);
    const Decision d = a.evaluate(input({object(1, 10, 10)}));
    ASSERT_EQ(d.request.type, RequestType::RequestType_BRAKE);
    EXPECT_EQ(d.request.intensity, kHubMaxSafeBrakeIntensity);
}

// The REAL config file: it loads (so every key is present and in range), the request is within
// the ceiling, and the ceiling equals the firmware's BrakeActuatorDriver::kMaxSafeIntensity. Until
// Phase 12 defines that constant in the firmware, the check is against the documented 90.
TEST(ArbiterCeiling, RealConfigMatchesTheFirmwareCeiling) {
    const auto cfg = SystemManager::loadConfig(std::string(HOST_SOURCE_DIR) + "/config");
    EXPECT_LE(cfg.decision.brakeRequestIntensity, cfg.decision.brakeActuatorMaxIntensity);
    const std::string fw = std::string(HOST_SOURCE_DIR) +
                           "/../firmware/sensor_actuator_hub/src/drivers/BrakeActuatorDriver.h";
    std::ifstream f(fw);
    ASSERT_TRUE(f) << fw;
    std::stringstream ss;
    ss << f.rdbuf();
    std::smatch m;
    const std::string src = ss.str();
    if (std::regex_search(src, m, std::regex(R"(kMaxSafeIntensity\s*=\s*(\d+))"))) {
        EXPECT_EQ(cfg.decision.brakeActuatorMaxIntensity, std::stoi(m[1]))
            << "decision_thresholds.yaml must match the firmware's kMaxSafeIntensity";
        EXPECT_EQ(kHubMaxSafeBrakeIntensity, std::stoi(m[1]))
            << "Types.h kHubMaxSafeBrakeIntensity must match the firmware";
    } else {
        std::printf(
            "[ info ] firmware kMaxSafeIntensity not defined yet (Phase 12); checked "
            "against the documented %d\n",
            kHubMaxSafeBrakeIntensity);
        EXPECT_EQ(cfg.decision.brakeActuatorMaxIntensity, kHubMaxSafeBrakeIntensity);
    }
}

// --- Evidence trail ----------------------------------------------------------------------------

// step() logs every active request WITH its context, and the first NONE after it (the release);
// idle NONE cycles are not logged.
TEST(Arbiter, StepWritesTheEvidenceTrail) {
    const auto path = std::filesystem::temp_directory_path() / "ar_arbiter_evidence.log";
    std::filesystem::remove(path);
    {
        EventLog log(path.string());
        DecisionArbiter a(thresholds(), &log);
        a.step(input({}, 1000));                   // idle: not logged
        a.step(input({object(1, 16, 10)}, 1100));  // confirming: not logged
        a.step(input({object(1, 15, 10)}, 1200));  // BRAKE
        a.step(input({object(1, 12, 10)}, 1300));  // BRAKE
        a.step(input({}, 1400));                   // release: logged
        a.step(input({}, 1500));                   // idle: not logged
    }
    std::ifstream f(path);
    std::string line;
    int requests = 0, brakes = 0;
    bool contextOk = false;
    while (std::getline(f, line)) {
        if (line.find("ACTUATION_REQUEST") == std::string::npos) continue;
        ++requests;
        if (line.find("type=BRAKE") != std::string::npos) {
            ++brakes;
            contextOk = line.find("rule=1") != std::string::npos &&
                        line.find("ttc_s=") != std::string::npos &&
                        line.find("gap_m=") != std::string::npos &&
                        line.find("closing_mps=") != std::string::npos &&
                        line.find("armed=1") != std::string::npos;
        }
    }
    EXPECT_EQ(requests, 3);
    EXPECT_EQ(brakes, 2);
    EXPECT_TRUE(contextOk);
    std::filesystem::remove(path);
}

// Onset confirmation (step()): braking starts only once rule 1 has held on the SAME target on every
// cycle for brake_confirm_ms; once braking, it continues while any target qualifies.
TEST(Arbiter, StepConfirmsTheOnsetOnOneTarget) {
    auto brakes = [](DecisionArbiter& a, std::vector<EgoRelativeTrack> objs, std::uint64_t ms) {
        return a.step(input(std::move(objs), ms)).request.type == RequestType::RequestType_BRAKE;
    };
    {  // one qualifying cycle (a velocity spike): never brakes
        DecisionArbiter a(thresholds());
        EXPECT_FALSE(brakes(a, {object(1, 15, 10)}, 1000));
        EXPECT_FALSE(brakes(a, {object(1, 15, 2)}, 1100));   // TTC 7.5 s: no longer qualifies
        EXPECT_FALSE(brakes(a, {object(1, 15, 10)}, 1200));  // starts again
        EXPECT_FALSE(brakes(a, {object(1, 15, 10)}, 1250));
        EXPECT_TRUE(brakes(a, {object(1, 14, 10)}, 1300));  // held 100 ms
    }
    {  // the target changes while confirming: the confirmation restarts
        DecisionArbiter a(thresholds());
        EXPECT_FALSE(brakes(a, {object(1, 15, 10)}, 1000));
        EXPECT_FALSE(brakes(a, {object(2, 15, 10)}, 1100));
        EXPECT_TRUE(brakes(a, {object(2, 14, 10)}, 1200));
    }
    {  // once braking, a change of target (a re-associated track) does not lapse it
        DecisionArbiter a(thresholds());
        brakes(a, {object(1, 15, 10)}, 1000);
        EXPECT_TRUE(brakes(a, {object(1, 14, 10)}, 1100));
        EXPECT_TRUE(brakes(a, {object(3, 13, 10)}, 1200));
    }
    {  // a prediction does not confirm: the target must be MEASURED brake_confirm_ms later
        DecisionArbiter a(thresholds());
        EXPECT_FALSE(brakes(a, {object(1, 15, 10)}, 1000));
        auto predicted = object(1, 14, 10);
        predicted.lastMeasuredMs = 1000;  // no scan since (the tracker only predicted)
        EXPECT_FALSE(brakes(a, {predicted}, 1100));
        EXPECT_TRUE(brakes(a, {object(1, 13, 10)}, 1200));  // measured again: confirmed
    }
    {  // while confirming, rule 2 still applies, and the context says so
        DecisionArbiter a(thresholds());
        auto in = input({object(1, 15, 10)}, 1000);
        in.ego.longitudinalAccelMps2 = -0.5 * 9.80665;
        const Decision d = a.step(in);
        EXPECT_EQ(d.request.type, RequestType::RequestType_HAZARDS);
        EXPECT_NE(d.context().find("confirming_ms=0"), std::string::npos) << d.context();
    }
    // evaluate() alone is the single-cycle rule, without the confirmation.
    EXPECT_EQ(DecisionArbiter(thresholds()).evaluate(input({object(1, 15, 10)})).request.type,
              RequestType::RequestType_BRAKE);
}

// --- End to end through the real tracker -------------------------------------------------------

namespace {

struct Drive {
    MultiObjectTracker tracker{
        loadTrackerConfig(std::string(HOST_SOURCE_DIR) + "/config/motion_prediction.yaml")};
    RecklessDrivingDetector detector;
    DecisionArbiter arbiter{thresholds()};
    EgoState ego;
    double egoX = 0;
};

ObjectMeasurement lidar(double x, double y) {
    return {{x, y}, Eigen::Matrix2d::Identity() * 0.15 * 0.15, 0, std::nullopt, true};
}

}  // namespace

// Ego at 15 m/s; a car 30 m ahead (bumper gap 26.5 m) at 15 m/s brakes at 6 m/s^2 from t = 2 s.
// The arbiter works on the tracker's estimate, which lags. Requirement: the first BRAKE comes
// while the TRUE time to collision is between 1.2 s and 1.9 s (not before the real risk, and not
// so late that the brake cannot help), and never while the car ahead was cruising.
TEST(ArbiterEndToEnd, LeadCarBrakingHardTriggersBrakeInTime) {
    Drive dr;
    double leadX = 30, leadV = 15;
    double firstBrakeTrueTtc = -1;
    for (int k = 0; k < 80; ++k) {  // 8 s at 10 Hz
        const double t = 0.1 * k;
        dr.egoX += 15 * 0.1;
        if (t >= 2.0) leadV = std::max(0.0, leadV - 6 * 0.1);
        leadX += leadV * 0.1;
        const std::uint64_t ms = 100 * k;
        dr.ego.pose = {dr.egoX, 0, 0};
        dr.ego.speedMps = 15;
        dr.ego.timestampMs = ms;
        dr.tracker.update({ms, dr.ego.pose, {lidar(leadX - dr.egoX, 0)}});
        ArbiterInput in = input(toEgoFrame(dr.tracker.tracks(), dr.ego, {}), ms);
        in.ego = dr.ego;
        const Decision d = dr.arbiter.step(in);
        const double gap = leadX - dr.egoX - 3.5, closing = 15 - leadV;
        const double trueTtc = closing > 0 ? gap / closing : kInf;
        if (d.request.type == RequestType::RequestType_BRAKE) {
            ASSERT_GE(t, 2.0) << "braked while the car ahead was cruising";
            firstBrakeTrueTtc = trueTtc;
            break;
        }
    }
    std::printf("[ info ] first BRAKE at true TTC %.2f s\n", firstBrakeTrueTtc);
    EXPECT_GE(firstBrakeTrueTtc, 1.2);
    EXPECT_LE(firstBrakeTrueTtc, 1.9);
}

// Phantom-braking checks: following at a steady gap, a car in the next lane closing fast, and a
// parked car on the outside of a bend the ego is turning through. None may brake.
TEST(ArbiterEndToEnd, NoPhantomBraking) {
    auto run = [](auto measure, double yawRate) {
        Drive dr;
        for (int k = 0; k < 60; ++k) {
            const std::uint64_t ms = 100 * k;
            dr.egoX += 15 * 0.1;
            dr.ego.pose = {dr.egoX, 0, 0};
            dr.ego.speedMps = 15;
            dr.ego.yawRateRadPerS = yawRate;
            dr.ego.timestampMs = ms;
            dr.tracker.update({ms, dr.ego.pose, {measure(k, dr.egoX)}});
            ArbiterInput in = input(toEgoFrame(dr.tracker.tracks(), dr.ego, {}), ms);
            in.ego = dr.ego;
            if (dr.arbiter.step(in).request.type == RequestType::RequestType_BRAKE) return k;
        }
        return -1;
    };
    // Same speed, 20 m ahead.
    EXPECT_EQ(run([](int, double) { return lidar(20, 0); }, 0.0), -1);
    // Oncoming in the next lane (3.5 m to the right), closing at 30 m/s.
    EXPECT_EQ(run([](int k, double) { return lidar(80 - 3.0 * k, -3.5); }, 0.0), -1);
    // The ego turns left (radius 150 m); a parked car straight ahead, 25 m, sits 2.1 m OUTSIDE
    // the bend's path there (kappa x^2 / 2 = 2.1 m), so it is not in the ego path.
    EXPECT_EQ(run([](int, double) { return lidar(25 + 3.5, 0); }, 15.0 / 150.0), -1);
}

// The gap is measured from the FRONT BUMPER, not the rear axle (the vehicle frame's origin). A
// stopped car 18.5 m ahead of the rear axle, with the bumper 3.5 m ahead of the axle, is a 15 m
// gap: at 10 m/s, TTC 1.5 s < 1.8 s, so BRAKE. Measured from the axle it would read 1.85 s and
// not brake: 0.35 s later, 3.5 m closer. (A mutation dropping the offset was not caught by the
// end-to-end scenario above, whose timing window absorbed it; this test pins it directly.)
TEST(ArbiterEndToEnd, TimeToCollisionIsMeasuredFromTheFrontBumper) {
    MultiObjectTracker tracker(
        loadTrackerConfig(std::string(HOST_SOURCE_DIR) + "/config/motion_prediction.yaml"));
    for (int k = 0; k < 5; ++k)
        tracker.update({static_cast<std::uint64_t>(100 * k), {}, {lidar(18.5, 0)}});
    EgoState ego;
    ego.speedMps = 10;
    ego.timestampMs = 400;
    EgoFrameConfig ef;
    ef.frontBumperFromRearAxleM = 3.5;
    ArbiterInput in = input(toEgoFrame(tracker.tracks(), ego, ef), 400);
    in.ego = ego;
    const Decision d = DecisionArbiter(thresholds()).evaluate(in);
    EXPECT_NEAR(d.gapM, 15.0, 0.1);
    EXPECT_NEAR(d.ttcS, 1.5, 0.05);
    EXPECT_EQ(d.request.type, RequestType::RequestType_BRAKE);
}
