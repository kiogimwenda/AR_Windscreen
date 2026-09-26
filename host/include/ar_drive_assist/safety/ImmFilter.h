#pragma once
// ImmFilter — the motion estimate of ONE tracked object: an interacting-multiple-model (IMM) filter
// over three motion models, each an unscented Kalman filter (UKF). See docs/BUILD_GUIDE.md
// Part 9.1, 9.1.1.
//
// ---------------------------------------------------------------------------------------------
// State (world-fixed frame, the same as SensorFusion's local east/north plane):
//     x = [px, py, psi, v, omega]   position (m), heading (rad, CCW from east), speed (m/s),
//                                   turn rate (rad/s)
// All three models share this state, which is what lets the IMM mix them.
//
// ---------------------------------------------------------------------------------------------
// The three motion models
//   CV    straight line at constant speed; heading can drift (process noise). Pedestrians,
//         animals, unknown obstacles, and vehicles going straight.
//   CTRV  constant turn rate and velocity (as in SensorFusion.h). Vehicles and two-wheelers
//         turning.
//   STOP  straight line, speed decaying fast (v' = v * exp(-dt / tau)). Captures a hard stop,
//         e.g. a matatu pulling in, well before the speed estimate itself has dropped.
//
// ---------------------------------------------------------------------------------------------
// The unscented Kalman filter (UKF), and why not an EKF here
//
// An EKF linearises the motion model with its Jacobian. CTRV is strongly non-linear in heading
// and turn rate, so in a sharp turn the linearisation is poor and the covariance comes out wrong.
// A UKF instead picks 2n + 1 = 11 "sigma points": the mean, plus pairs placed +-sqrt((n+lambda)P)
// along each principal direction of the uncertainty. It pushes each point through the EXACT
// motion model, then recomputes the mean and covariance from where the points land. No
// derivatives are needed, and it is accurate to second order.
//
// Measurements here (position, optionally heading) are LINEAR in the state, so the update step is
// the ordinary Kalman update. Only the prediction needs the unscented transform.
//
// ---------------------------------------------------------------------------------------------
// IMM, one step
//   1. MIX:     each model starts from a blend of all three models' estimates, weighted by how
//               likely a switch into it is (the Markov transition matrix) and how probable each
//               model currently is.
//   2. PREDICT: each model's UKF moves its mixed estimate forward by dt.
//   3. UPDATE:  each model is corrected by the measurement. The likelihood of the innovation
//               under that model says how well the model explained what was seen.
//   4. WEIGH:   model probabilities are updated by those likelihoods.
//   5. COMBINE: the output is the probability-weighted blend of the three.
// A rising STOP probability is therefore itself a signal: the stopping model explains the
// measurements better than the others (Part 9.2, "sudden braking ahead").
//
// ---------------------------------------------------------------------------------------------
// Angles: psi is an angle. Averages use the CIRCULAR mean (atan2 of weighted sin/cos), and
// differences are wrapped to (-pi, pi]. Averaging 179 deg and -179 deg must give 180, not 0.
// ---------------------------------------------------------------------------------------------

#include <Eigen/Dense>
#include <array>
#include <optional>

namespace ar_drive_assist {

enum class MotionModel : int { CV = 0, CTRV = 1, STOP = 2 };
constexpr int kNumModels = 3;

// Per-class unpredictability (config/motion_prediction.yaml). 1 sigma.
struct MotionNoise {
    double accelStd = 2.0;         // m/s^2, along the heading
    double yawAccelStd = 0.5;      // rad/s^2 (CTRV turn-rate changes)
    double headingDriftStd = 0.3;  // rad/sqrt(s) (CV: how freely heading wanders)
    double stopTau = 0.8;          // s, STOP model speed-decay time constant
    double switchRate = 0.5;       // 1/s, straight <-> turning, and leaving STOP
    // 1/s, rate of ENTERING the stopping model. Kept much lower than switchRate on purpose: with
    // symmetric rates the model probabilities drift towards 1/3 each whenever the measurements
    // cannot tell the models apart (e.g. a slow pedestrian over 0.1 s). STOP then keeps a share
    // at cruise and drags the blended speed low (measured: -9% with exact measurements). With a
    // rare entry, STOP gains probability only when the measurements actually show deceleration.
    double stopEntryRate = 0.05;
    double initialSpeedStd = 5.0;  // m/s, speed uncertainty of a brand-new track
};

class ImmFilter {
public:
    using State = Eigen::Matrix<double, 5, 1>;
    using Cov = Eigen::Matrix<double, 5, 5>;

    // Starts a track from its first measured position (world frame). The heading is unknown
    // unless given (e.g. from a vehicle's L-shape fit).
    ImmFilter(const Eigen::Vector2d& pos, const Eigen::Matrix2d& posCov, const MotionNoise& noise,
              std::optional<double> heading = std::nullopt);

    // Two-point initialisation: sets speed and heading of every model from a velocity estimate
    // (world frame) and its covariance, typically the displacement between a track's first two
    // measurements divided by dt. Why it is needed: the state stores motion as speed + heading,
    // and at speed 0 the heading has no effect on position, so no position measurement can ever
    // correct it. A track born at v = 0 would otherwise lag a walking pedestrian until the
    // association gate rejected them (found by the tracker occlusion test). The displacement's
    // direction is always used as the heading, with its (possibly large) uncertainty.
    void initialiseVelocity(const Eigen::Vector2d& vel, const Eigen::Matrix2d& velCov);

    // Moves every model forward by dt (IMM steps 1-2). Probabilities are mixed but not updated.
    void predict(double dt);

    // Position measurement (world frame) with its covariance (IMM steps 3-5). Returns the NIS of
    // the combined estimate's innovation, for consistency checks.
    double updatePosition(const Eigen::Vector2d& z, const Eigen::Matrix2d& R);

    // Heading measurement, modulo pi: an L-shape rectangle fit cannot tell front from back, so the
    // measurement or its reverse, whichever is closer to the estimate, is used.
    void updateHeadingModPi(double psiMeas, double stdRad);

    // Mahalanobis distance^2 of a position from the predicted position, and its covariance.
    // This is the association cost in the tracker.
    double mahalanobis2(const Eigen::Vector2d& z, const Eigen::Matrix2d& R) const;

    State state() const;  // combined (IMM step 5)
    Cov covariance() const;
    Eigen::Vector2d position() const { return state().head<2>(); }
    Eigen::Vector2d velocity() const;  // world-frame velocity vector, m/s
    const std::array<double, kNumModels>& modelProbabilities() const { return mu_; }
    const State& modelState(MotionModel m) const { return x_[int(m)]; }
    const Cov& modelCovariance(MotionModel m) const { return P_[int(m)]; }

    // Model dynamics, exposed for the predictor's forward simulation (MotionPredictor).
    static State propagate(MotionModel m, const State& x, double dt, double stopTau);
    // One unscented prediction step of (x, P) under model m (what each IMM model does in
    // predict()). Also used by MotionPredictor to extrapolate a track or the ego vehicle.
    static void ukfPredict(MotionModel m, State& x, Cov& P, double dt, const MotionNoise& n);
    const MotionNoise& noise() const { return noise_; }
    static Cov processNoise(MotionModel m, const State& x, double dt, const MotionNoise& n);
    static double wrap(double a);

private:
    void mix(double dt);

    MotionNoise noise_;
    std::array<State, kNumModels> x_;
    std::array<Cov, kNumModels> P_;
    std::array<double, kNumModels> mu_{{0.6, 0.3, 0.1}};  // prior: mostly CV, some turning
};

}  // namespace ar_drive_assist
