#include "ar_drive_assist/safety/MotionPredictor.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace ar_drive_assist {
namespace {

using State = ImmFilter::State;
using Cov = ImmFilter::Cov;

// Step (x, P) forward to each horizon (sorted ascending) under one model, recording the position
// mean and covariance at each.
std::vector<std::pair<Eigen::Vector2d, Eigen::Matrix2d>> extrapolate(
    MotionModel m, State x, Cov P, const MotionNoise& n, const std::vector<double>& horizons,
    double step) {
    std::vector<std::pair<Eigen::Vector2d, Eigen::Matrix2d>> out;
    double t = 0;
    for (double h : horizons) {
        while (t + 1e-9 < h) {
            const double dt = std::min(step, h - t);
            ImmFilter::ukfPredict(m, x, P, dt, n);
            t += dt;
        }
        out.emplace_back(x.head<2>(), P.topLeftCorner<2, 2>());
    }
    return out;
}

// A draw from N(x, P); P is regularised if rounding has made it slightly indefinite.
State sample(const State& x, const Cov& P, std::mt19937& rng) {
    std::normal_distribution<double> g(0, 1);
    Eigen::LLT<Cov> llt(P);
    Cov Pr = P;
    for (double j = 1e-9; llt.info() != Eigen::Success && j < 1.0; j *= 10) {
        Pr += j * Cov::Identity();
        llt.compute(Pr);
    }
    State z;
    for (int i = 0; i < 5; ++i) z(i) = g(rng);
    State s = x + Cov(llt.matrixL()) * z;
    s(2) = ImmFilter::wrap(s(2));
    return s;
}

// One noisy step: the model's motion plus a random acceleration and turn change, drawn from
// EXACTLY the process noise the filter assumes (ImmFilter::processNoise): discrete white-noise
// acceleration, so speed and turn rate change by accelStd * dt and yawAccelStd * dt per step, and
// the straight models' heading wanders by headingDriftStd * sqrt(dt). Sampling with more noise
// than the filter assumes made collision courses look unlikely (found by the head-on test).
void noisyStep(MotionModel m, State& s, double dt, const MotionNoise& n, std::mt19937& rng) {
    std::normal_distribution<double> g(0, 1);
    s = ImmFilter::propagate(m, s, dt, n.stopTau);
    s(3) += n.accelStd * dt * g(rng);
    if (m == MotionModel::CTRV) {
        s(4) += n.yawAccelStd * dt * g(rng);
    } else {
        s(2) = ImmFilter::wrap(s(2) + n.headingDriftStd * std::sqrt(dt) * g(rng));
    }
}

}  // namespace

MotionPredictor::MotionPredictor(PredictorConfig cfg) : cfg_(std::move(cfg)) {}

double MotionPredictor::safetyRadius(int32_t cls) const {
    const auto it = cfg_.objectHalfWidthM.find(cls);
    const double obj = it != cfg_.objectHalfWidthM.end() ? it->second : 0.5;
    return cfg_.egoHalfWidthM + obj + cfg_.marginM;
}

void MotionPredictor::cpa(const Eigen::Vector2d& r, const Eigen::Vector2d& v, double& tStar,
                          double& dStar) {
    const double vv = v.squaredNorm();
    // Closest in the past (diverging) or no relative motion: the closest FUTURE point is now.
    tStar = vv > 1e-12 ? std::max(0.0, -r.dot(v) / vv) : 0.0;
    dStar = (r + v * tStar).norm();
}

std::vector<PredictedPoint> MotionPredictor::predict(const Track& t,
                                                     const std::vector<double>& horizons) const {
    const auto& mu = t.filter.modelProbabilities();
    std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Matrix2d>>> per(kNumModels);
    for (int j = 0; j < kNumModels; ++j) {
        const auto m = static_cast<MotionModel>(j);
        per[j] = extrapolate(m, t.filter.modelState(m), t.filter.modelCovariance(m),
                             t.filter.noise(), horizons, cfg_.stepS);
    }
    // Moment-matched Gaussian of the models' mixture at each horizon (for ribbons and summaries).
    std::vector<PredictedPoint> out;
    for (size_t h = 0; h < horizons.size(); ++h) {
        PredictedPoint p;
        p.t = horizons[h];
        for (int j = 0; j < kNumModels; ++j) p.mean += mu[j] * per[j][h].first;
        for (int j = 0; j < kNumModels; ++j) {
            const Eigen::Vector2d d = per[j][h].first - p.mean;
            p.cov += mu[j] * (per[j][h].second + d * d.transpose());
        }
        out.push_back(p);
    }
    return out;
}

std::vector<PredictedPoint> MotionPredictor::predictEgo(const State& ego, const Cov& P,
                                                        const std::vector<double>& horizons) const {
    const auto e = extrapolate(MotionModel::CTRV, ego, P, cfg_.egoNoise, horizons, cfg_.stepS);
    std::vector<PredictedPoint> out;
    for (size_t h = 0; h < horizons.size(); ++h)
        out.push_back({horizons[h], e[h].first, e[h].second});
    return out;
}

CollisionRisk MotionPredictor::risk(const Track& t, const State& ego, const Cov& P) const {
    CollisionRisk r;
    const double radius = safetyRadius(t.objectClass);

    // CPA from the current means.
    const Eigen::Vector2d egoVel(ego(3) * std::cos(ego(2)), ego(3) * std::sin(ego(2)));
    cpa(t.filter.position() - ego.head<2>(), t.filter.velocity() - egoVel, r.tcpa, r.dcpa);
    r.conflict = r.dcpa < radius && r.tcpa > 0 && r.tcpa < cfg_.horizonS;

    // Collision probability: sampled futures of object and ego, stepped together.
    std::mt19937 rng(static_cast<unsigned>(t.id) * 2654435761u);  // same inputs, same answer
    const auto& mu = t.filter.modelProbabilities();
    std::discrete_distribution<int> pickModel(mu.begin(), mu.end());
    int hits = 0;
    const int steps = static_cast<int>(std::ceil(cfg_.horizonS / cfg_.stepS));
    for (int s = 0; s < cfg_.samples; ++s) {
        const auto m = static_cast<MotionModel>(pickModel(rng));
        State o = sample(t.filter.modelState(m), t.filter.modelCovariance(m), rng);
        State e = sample(ego, P, rng);
        bool hit = (o.head<2>() - e.head<2>()).norm() < radius;
        for (int k = 0; k < steps && !hit; ++k) {
            noisyStep(m, o, cfg_.stepS, t.filter.noise(), rng);
            noisyStep(MotionModel::CTRV, e, cfg_.stepS, cfg_.egoNoise, rng);
            hit = (o.head<2>() - e.head<2>()).norm() < radius;
        }
        hits += hit;
    }
    r.probability = static_cast<double>(hits) / cfg_.samples;
    return r;
}

}  // namespace ar_drive_assist
