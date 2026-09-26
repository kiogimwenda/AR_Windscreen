#pragma once
// MotionPredictor — where tracked objects and the ego car are going, and how likely they are to
// meet. See docs/BUILD_GUIDE.md Part 9.1.1 (limits), 9.1.2 (CPA, collision probability).
//
// ---------------------------------------------------------------------------------------------
// What it produces, and what uses it
//   predict()     each object's predicted position distribution at future horizons: a mixture of
//                 the IMM's models, weighted by their probabilities, plus its moment-matched
//                 mean and covariance. It drives the predicted-path ribbons in the AR display
//                 (ribbon width = uncertainty).
//   predictEgo()  the same for the ego car, from the SensorFusion state and covariance (same
//                 [px, py, psi, v, omega] layout), under the turn-rate model. Route-following
//                 ego prediction is the next step (Part 9.1.2).
//   risk()        closest point of approach (CPA) between object and ego, and a Monte-Carlo
//                 COLLISION PROBABILITY. These drive warnings and the hazard glow's risk level
//                 (docs/architecture/ar-overlay-design.md §3.1), and NEVER the brake
//                 (Part 9.3: braking uses only the current measured range and closing speed).
//
// ---------------------------------------------------------------------------------------------
// Closest point of approach. With relative position r and relative velocity v (object minus
// ego):
//     t* = -(r . v) / |v|^2          when they are closest
//     d* = | r + v t* |              how close they get
// TTC (range / closing speed) only sees objects coming straight at the car. CPA also covers
// crossing traffic and cut-ins, the pedestrian stepping out from the side.
//
// Collision probability. CPA uses the means only. The probability propagates the uncertainty:
// N sampled futures of the object (a model drawn by its IMM probability, an initial state drawn
// from that model's Gaussian, random accelerations along the way) and of the ego car are stepped
// forward together, and it is the fraction in which they come within the safety radius (ego
// half-width + object half-width + margin) inside the horizon. The random generator is seeded
// from the track id, so the same inputs always give the same answer (testable, reproducible).
// ---------------------------------------------------------------------------------------------

#include <Eigen/Dense>
#include <map>
#include <vector>

#include "ar_drive_assist/safety/MultiObjectTracker.h"

namespace ar_drive_assist {

struct PredictedPoint {
    double t = 0;  // seconds ahead
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
};

struct CollisionRisk {
    double tcpa = 0;         // s, time of closest approach (0 if already diverging / static)
    double dcpa = 0;         // m, closest-approach distance (means only)
    double probability = 0;  // P(within safety radius inside the horizon)
    bool conflict = false;   // dcpa < safety radius and 0 < tcpa < horizon
};

struct PredictorConfig {
    double horizonS = 2.0;  // 9.1.1: physics predicts usefully to ~1-1.5 s; 2 s is shown faded
    double stepS = 0.1;
    double egoHalfWidthM = 0.9;
    double marginM = 0.5;
    std::map<int32_t, double> objectHalfWidthM = {{0, 0.9}, {1, 0.3}, {2, 0.4},
                                                  {3, 0.3}, {4, 0.6}, {-1, 0.5}};
    int samples = 300;
    MotionNoise egoNoise{2.0, 0.5, 0.05, 0.8, 0.0, 0.0, 1.0};  // ego: CTRV only, no switching
};

class MotionPredictor {
public:
    explicit MotionPredictor(PredictorConfig cfg = {});

    std::vector<PredictedPoint> predict(const Track& t, const std::vector<double>& horizons) const;
    std::vector<PredictedPoint> predictEgo(const ImmFilter::State& ego, const ImmFilter::Cov& P,
                                           const std::vector<double>& horizons) const;
    CollisionRisk risk(const Track& t, const ImmFilter::State& ego, const ImmFilter::Cov& P) const;

    // CPA of two constant-velocity points (exposed for tests and reports).
    static void cpa(const Eigen::Vector2d& r, const Eigen::Vector2d& v, double& tStar,
                    double& dStar);

    const PredictorConfig& config() const { return cfg_; }

private:
    double safetyRadius(int32_t cls) const;
    PredictorConfig cfg_;
};

}  // namespace ar_drive_assist
