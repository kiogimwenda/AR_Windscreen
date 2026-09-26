#include "ar_drive_assist/safety/ImmFilter.h"

#include <cmath>
#include <limits>

namespace ar_drive_assist {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int N = 5;
enum { PX = 0, PY = 1, PSI = 2, V = 3, W = 4 };
constexpr double kOmegaEps = 1e-4;   // as SensorFusion: below this, CTRV's straight-line limit
constexpr double kProbFloor = 1e-6;  // a model is never ruled out entirely (see normalise())

using State = ImmFilter::State;
using Cov = ImmFilter::Cov;

// Circular-aware weighted mean of states: psi by atan2 of weighted sin/cos, others linearly.
template <typename Weights, typename States>
State weightedMean(const Weights& w, const States& xs, int n) {
    State m = State::Zero();
    double s = 0, c = 0;
    for (int i = 0; i < n; ++i) {
        m += w[i] * xs[i];
        s += w[i] * std::sin(xs[i](PSI));
        c += w[i] * std::cos(xs[i](PSI));
    }
    m(PSI) = std::atan2(s, c);
    return m;
}

State diff(const State& a, const State& b) {
    State d = a - b;
    d(PSI) = ImmFilter::wrap(d(PSI));
    return d;
}

// Keeps model probabilities normalised, with a small floor: a model whose probability underflowed
// to exactly 0 could never become likely again (e.g. STOP, just as a car brakes).
void normalise(std::array<double, kNumModels>& mu) {
    double sum = 0;
    for (double& m : mu) {
        m = std::max(m, kProbFloor);
        sum += m;
    }
    for (double& m : mu) m /= sum;
}

// Linear Kalman update of (x, P) by z = H x + noise(R), Joseph form. Returns the innovation's log
// likelihood. `angleRow` >= 0 marks a measurement row that is an angle (innovation wrapped).
template <int M>
double kalmanUpdate(State& x, Cov& P, const Eigen::Matrix<double, M, 1>& z,
                    const Eigen::Matrix<double, M, N>& H, const Eigen::Matrix<double, M, M>& R,
                    int angleRow = -1) {
    Eigen::Matrix<double, M, 1> y = z - H * x;
    if (angleRow >= 0) y(angleRow) = ImmFilter::wrap(y(angleRow));
    const Eigen::Matrix<double, M, M> S = H * P * H.transpose() + R;
    const Eigen::Matrix<double, M, M> Si = S.inverse();
    const Eigen::Matrix<double, N, M> K = P * H.transpose() * Si;
    x += K * y;
    x(PSI) = ImmFilter::wrap(x(PSI));
    const Cov IKH = Cov::Identity() - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();
    const double maha = (y.transpose() * Si * y)(0);
    return -0.5 * (maha + std::log(S.determinant()) + M * std::log(2 * kPi));
}

}  // namespace

double ImmFilter::wrap(double a) {
    a = std::fmod(a + kPi, 2.0 * kPi);
    if (a <= 0.0) a += 2.0 * kPi;
    return a - kPi;
}

ImmFilter::ImmFilter(const Eigen::Vector2d& pos, const Eigen::Matrix2d& posCov,
                     const MotionNoise& noise, std::optional<double> heading)
    : noise_(noise) {
    State x = State::Zero();
    x.head<2>() = pos;
    x(PSI) = heading ? wrap(*heading) : 0.0;
    Cov P = Cov::Zero();
    P.topLeftCorner<2, 2>() = posCov;
    P(PSI, PSI) = heading ? 0.1 * 0.1 : kPi * kPi;
    P(V, V) = noise.initialSpeedStd * noise.initialSpeedStd;
    P(W, W) = 0.3 * 0.3;
    x_.fill(x);
    P_.fill(P);
}

State ImmFilter::propagate(MotionModel m, const State& x, double dt, double stopTau) {
    State o = x;
    const double psi = x(PSI), v = x(V), w = x(W);
    switch (m) {
        case MotionModel::CTRV:
            if (std::abs(w) > kOmegaEps) {
                o(PX) += v / w * (std::sin(psi + w * dt) - std::sin(psi));
                o(PY) += v / w * (std::cos(psi) - std::cos(psi + w * dt));
            } else {
                o(PX) += v * std::cos(psi) * dt;
                o(PY) += v * std::sin(psi) * dt;
            }
            o(PSI) = wrap(psi + w * dt);
            break;
        case MotionModel::CV:
            o(PX) += v * std::cos(psi) * dt;
            o(PY) += v * std::sin(psi) * dt;
            o(W) = 0.0;
            break;
        case MotionModel::STOP: {
            // v(t) = v e^(-t/tau): distance covered in dt = v tau (1 - e^(-dt/tau)).
            const double decay = std::exp(-dt / stopTau);
            const double dist = v * stopTau * (1.0 - decay);
            o(PX) += dist * std::cos(psi);
            o(PY) += dist * std::sin(psi);
            o(V) = v * decay;
            o(W) = 0.0;
            break;
        }
    }
    return o;
}

Cov ImmFilter::processNoise(MotionModel m, const State& x, double dt, const MotionNoise& n) {
    // Unknown acceleration along the heading moves position and speed together.
    State ga = State::Zero();
    ga(PX) = 0.5 * dt * dt * std::cos(x(PSI));
    ga(PY) = 0.5 * dt * dt * std::sin(x(PSI));
    ga(V) = dt;
    Cov Q = n.accelStd * n.accelStd * ga * ga.transpose();
    if (m == MotionModel::CTRV) {
        State gw = State::Zero();  // unknown yaw acceleration moves heading and turn rate
        gw(PSI) = 0.5 * dt * dt;
        gw(W) = dt;
        Q += n.yawAccelStd * n.yawAccelStd * gw * gw.transpose();
    } else {
        // CV and STOP hold omega at 0; their heading wanders as a random walk.
        Q(PSI, PSI) += n.headingDriftStd * n.headingDriftStd * dt;
        Q(W, W) += 1e-6;  // keep P positive definite in the unused omega direction
    }
    return Q;
}

void ImmFilter::mix(double dt) {
    // Markov transitions over dt, from per-pair rates r[i][j] (1/s). Model i is left with
    // probability 1 - e^(-R_i dt), R_i = sum_j r[i][j], shared between targets in proportion to
    // their rates. Entry into STOP is deliberately rare (see MotionNoise::stopEntryRate).
    const double s = noise_.switchRate, e = noise_.stopEntryRate;
    const double r[kNumModels][kNumModels] = {
        /* from CV   */ {0.0, s, e},
        /* from CTRV */ {s, 0.0, e},
        /* from STOP */ {s, 0.1 * s, 0.0},
    };
    double pi[kNumModels][kNumModels];
    for (int i = 0; i < kNumModels; ++i) {
        const double Ri = r[i][0] + r[i][1] + r[i][2];
        const double leave = Ri > 0 ? 1.0 - std::exp(-Ri * dt) : 0.0;
        for (int j = 0; j < kNumModels; ++j) {
            pi[i][j] = (i == j) ? 1.0 - leave : (Ri > 0 ? leave * r[i][j] / Ri : 0.0);
        }
    }
    std::array<double, kNumModels> cbar{};
    double wij[kNumModels][kNumModels];
    for (int j = 0; j < kNumModels; ++j) {
        for (int i = 0; i < kNumModels; ++i) cbar[j] += pi[i][j] * mu_[i];
        for (int i = 0; i < kNumModels; ++i) wij[i][j] = pi[i][j] * mu_[i] / cbar[j];
    }
    std::array<State, kNumModels> x0;
    std::array<Cov, kNumModels> P0;
    for (int j = 0; j < kNumModels; ++j) {
        double w[kNumModels];
        for (int i = 0; i < kNumModels; ++i) w[i] = wij[i][j];
        x0[j] = weightedMean(w, x_, kNumModels);
        P0[j] = Cov::Zero();
        for (int i = 0; i < kNumModels; ++i) {
            const State d = diff(x_[i], x0[j]);
            P0[j] += w[i] * (P_[i] + d * d.transpose());
        }
    }
    x_ = x0;
    P_ = P0;
    mu_ = cbar;  // predicted model probabilities (updated by the next measurement)
}

void ImmFilter::ukfPredict(MotionModel m, State& xIo, Cov& PIo, double dt, const MotionNoise& n) {
    // Sigma points with alpha = 1, beta = 2, kappa = 0: lambda = 0, so the mean point has
    // weight 0 for the mean (2 for the covariance), and the 2n others weight 1/(2n). There are no
    // negative weights, which keeps the covariance well-behaved with the circular heading.
    Eigen::LLT<Cov> llt(N * PIo);
    Cov Pj = PIo;
    for (double jitter = 1e-9; llt.info() != Eigen::Success && jitter < 1.0; jitter *= 10) {
        Pj += jitter * Cov::Identity();
        llt.compute(N * Pj);
    }
    const Cov L = llt.matrixL();
    std::array<State, 2 * N + 1> sig;
    sig[0] = propagate(m, xIo, dt, n.stopTau);
    for (int k = 0; k < N; ++k) {
        State a = xIo + L.col(k), b = xIo - L.col(k);
        a(PSI) = wrap(a(PSI));
        b(PSI) = wrap(b(PSI));
        sig[1 + k] = propagate(m, a, dt, n.stopTau);
        sig[1 + N + k] = propagate(m, b, dt, n.stopTau);
    }
    double wm[2 * N + 1];
    wm[0] = 0.0;
    for (int k = 1; k <= 2 * N; ++k) wm[k] = 1.0 / (2 * N);
    const State mean = weightedMean(wm, sig, 2 * N + 1);
    Cov P = 2.0 * diff(sig[0], mean) * diff(sig[0], mean).transpose();  // W0c = 1 - 1 + beta
    for (int k = 1; k <= 2 * N; ++k) {
        const State d = diff(sig[k], mean);
        P += wm[k] * d * d.transpose();
    }
    xIo = mean;
    PIo = P + processNoise(m, mean, dt, n);
}

void ImmFilter::initialiseVelocity(const Eigen::Vector2d& vel, const Eigen::Matrix2d& velCov) {
    const double v = vel.norm();
    for (int j = 0; j < kNumModels; ++j) {
        // Drop old speed/heading correlations: the new values replace the old ones.
        P_[j].row(V).setZero();
        P_[j].col(V).setZero();
        P_[j].row(PSI).setZero();
        P_[j].col(PSI).setZero();
        x_[j](V) = v;
        if (v < 1e-6) {  // no displacement at all: direction genuinely unknown
            P_[j](V, V) = velCov.trace();
            P_[j](PSI, PSI) = kPi * kPi;
            continue;
        }
        // The displacement's direction is the best available heading estimate even when it is
        // uncertain (two noisy points 0.1 s apart give ~2 m/s velocity noise, more than a walking
        // pedestrian's speed). So it is always used, with its uncertainty stated honestly (capped
        // at a full circle), and later measurements refine it. Keeping an arbitrary old heading
        // instead left tracks facing the wrong way (found by NewTrackLearnsVelocityInAnyDirection).
        const double psi = std::atan2(vel.y(), vel.x());
        Eigen::Matrix2d J;  // polar transform: d(v, psi) / d(vx, vy)
        J << std::cos(psi), std::sin(psi), -std::sin(psi) / v, std::cos(psi) / v;
        const Eigen::Matrix2d Pp = J * velCov * J.transpose();
        x_[j](PSI) = psi;
        P_[j](V, V) = Pp(0, 0);
        P_[j](PSI, PSI) = std::min(Pp(1, 1), kPi * kPi);
        const double c = std::sqrt(P_[j](V, V) * P_[j](PSI, PSI) / (Pp(0, 0) * Pp(1, 1)));
        P_[j](V, PSI) = P_[j](PSI, V) =
            Pp(0, 1) * c;  // keep the correlation consistent with the cap
    }
}

void ImmFilter::predict(double dt) {
    if (dt <= 0.0) return;
    mix(dt);
    for (int j = 0; j < kNumModels; ++j)
        ukfPredict(static_cast<MotionModel>(j), x_[j], P_[j], dt, noise_);
}

double ImmFilter::updatePosition(const Eigen::Vector2d& z, const Eigen::Matrix2d& R) {
    Eigen::Matrix<double, 2, N> H = Eigen::Matrix<double, 2, N>::Zero();
    H(0, PX) = 1.0;
    H(1, PY) = 1.0;

    // NIS of the combined (pre-update) estimate, for consistency checks.
    const State xc = state();
    const Cov Pc = covariance();
    const Eigen::Vector2d y = z - xc.head<2>();
    const double nis = y.dot((Pc.topLeftCorner<2, 2>() + R).inverse() * y);

    // Each model's update, and its log likelihood. Probabilities are combined in log space: plain
    // likelihoods of a surprising measurement underflow to 0 for every model.
    double logL[kNumModels];
    double maxLog = -std::numeric_limits<double>::infinity();
    for (int j = 0; j < kNumModels; ++j) {
        logL[j] = kalmanUpdate<2>(x_[j], P_[j], z, H, R);
        maxLog = std::max(maxLog, logL[j] + std::log(mu_[j]));
    }
    for (int j = 0; j < kNumModels; ++j) mu_[j] = std::exp(logL[j] + std::log(mu_[j]) - maxLog);
    normalise(mu_);
    return nis;
}

void ImmFilter::updateHeadingModPi(double psiMeas, double stdRad) {
    Eigen::Matrix<double, 1, N> H = Eigen::Matrix<double, 1, N>::Zero();
    H(0, PSI) = 1.0;
    Eigen::Matrix<double, 1, 1> R;
    R << stdRad * stdRad;
    for (int j = 0; j < kNumModels; ++j) {
        // A rectangle fit gives the heading only up to 180 deg: use whichever of psi, psi + pi
        // is nearer the model's own heading.
        double zz = psiMeas;
        if (std::abs(wrap(zz - x_[j](PSI))) > kPi / 2) zz = wrap(zz + kPi);
        Eigen::Matrix<double, 1, 1> z;
        z << zz;
        kalmanUpdate<1>(x_[j], P_[j], z, H, R, 0);
    }
}

// The models may carry a NEGATIVE speed: (-v, psi) and (v, psi + pi) are the same motion, and
// when a noisy start makes the first heading point the wrong way, the filter can slide the speed
// through zero instead of turning the heading around (found by the MotionPredictor tests: a
// pedestrian walking north reported as v = -1.2 m/s facing south). The models are left as they
// are, because flipping them one by one would make the mixing average opposite headings. The
// REPORTED state is flipped to the positive-speed form, which the display and the reckless-driving
// classifier read as "the way it is facing".
namespace {
void toPositiveSpeed(State& x, Cov* P) {
    if (x(V) >= 0) return;
    x(V) = -x(V);
    x(PSI) = ImmFilter::wrap(x(PSI) + kPi);
    if (P) {  // Jacobian diag(1, 1, 1, -1, 1): flips the sign of v's cross-covariances
        P->row(V) *= -1;
        P->col(V) *= -1;
    }
}
}  // namespace

State ImmFilter::state() const {
    State x = weightedMean(mu_, x_, kNumModels);
    toPositiveSpeed(x, nullptr);
    return x;
}

Cov ImmFilter::covariance() const {
    State m = weightedMean(mu_, x_, kNumModels);
    Cov P = Cov::Zero();
    for (int j = 0; j < kNumModels; ++j) {
        const State d = diff(x_[j], m);
        P += mu_[j] * (P_[j] + d * d.transpose());
    }
    toPositiveSpeed(m, &P);
    return P;
}

Eigen::Vector2d ImmFilter::velocity() const {
    const State s = state();
    return {s(V) * std::cos(s(PSI)), s(V) * std::sin(s(PSI))};
}

double ImmFilter::mahalanobis2(const Eigen::Vector2d& z, const Eigen::Matrix2d& R) const {
    const Eigen::Vector2d y = z - position();
    return y.dot((covariance().topLeftCorner<2, 2>() + R).inverse() * y);
}

}  // namespace ar_drive_assist
