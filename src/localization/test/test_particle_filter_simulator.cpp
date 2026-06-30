// Standalone simulation harness for the FastSLAM ParticleFilter.
//
// It drives the filter against a known ground-truth landmark map and a
// scripted vehicle trajectory, injecting Gaussian range/bearing noise, then
// reports tracking/mapping accuracy and asserts a handful of regression
// properties (no NaN/crash on degenerate input, bounded pose error, stable
// landmark counts, landmark expiry). Run it directly — it needs no ROS graph.
//
// To add a scenario:
//   1. Write  void scenarioFoo(TestEnv& e) { ... }
//   2. Call   addTest("foo description", scenarioFoo);  inside registerTests().
// Parameterised variants: capture the parameter in a lambda passed to addTest.

#include "ParticleFilter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace Eigen;
using namespace LocalizationHelpers;

namespace {

// ----- tiny test reporting ---------------------------------------------------
int g_failures = 0;

void check(const std::string& name, bool cond, const std::string& detail = "") {
    std::printf("  [%s] %s%s%s\n", cond ? "PASS" : "FAIL", name.c_str(),
                detail.empty() ? "" : " - ", detail.c_str());
    if (!cond) g_failures++;
}

// ----- world / motion helpers ------------------------------------------------

// A 5x5 grid of landmarks spanning [-4, 4] in x and y (25 landmarks).
std::vector<Vector2d> gridLandmarks() {
    std::vector<Vector2d> lms;
    for (int ix = -2; ix <= 2; ++ix)
        for (int iy = -2; iy <= 2; ++iy)
            lms.emplace_back(Vector2d(2.0 * ix, 2.0 * iy));
    return lms;
}

// Exact unicycle integration of the ground-truth pose (matches the filter's
// motion model so motion error comes only from the filter's process noise).
Vector3d stepPose(const Vector3d& p, const Vector2d& u, double dt) {
    const double v = u[0], w = u[1], th = p[2];
    Vector3d np;
    if (std::abs(w) < 1e-6) {
        np[0] = p[0] + v * dt * std::cos(th);
        np[1] = p[1] + v * dt * std::sin(th);
        np[2] = th;
    } else {
        const double thn = wrapAngle(th + w * dt);
        np[0] = p[0] + (v / w) * (std::sin(thn) - std::sin(th));
        np[1] = p[1] - (v / w) * (std::cos(thn) - std::cos(th));
        np[2] = thn;
    }
    return np;
}

// Noisy range/bearing observations of every landmark within maxRange.
// `skip` lets a scenario occlude specific landmarks (return true to drop one).
std::vector<Vector2d> simMeasurements(
    const Vector3d& pose, const std::vector<Vector2d>& lms, double maxRange,
    double sigR, double sigB, std::mt19937& gen,
    const std::function<bool(const Vector2d&)>& skip = nullptr) {
    std::normal_distribution<> nr(0.0, sigR), nb(0.0, sigB);
    std::vector<Vector2d> zs;
    for (const auto& lm : lms) {
        if (skip && skip(lm)) continue;
        const double dx = lm[0] - pose[0], dy = lm[1] - pose[1];
        const double r = std::sqrt(dx * dx + dy * dy);
        if (r > maxRange) continue;
        const double b = wrapAngle(std::atan2(dy, dx) - pose[2]);
        zs.emplace_back(Vector2d(r + nr(gen), wrapAngle(b + nb(gen))));
    }
    return zs;
}

bool isFinite(const Vector3d& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
double posError(const Vector3d& a, const Vector3d& b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}
double headingError(const Vector3d& a, const Vector3d& b) {
    return std::abs(wrapAngle(a[2] - b[2]));
}
int landmarksNear(std::vector<Vector2d> est, const Vector2d& target, double tol) {
    int n = 0;
    for (const auto& e : est) if ((e - target).norm() <= tol) ++n;
    return n;
}
bool allParticlesFinite(const ParticleFilter& pf) {
    for (const auto& p : pf.getParticles()) if (!isFinite(p.x)) return false;
    return true;
}

// ----- shared test environment -----------------------------------------------
//
// Constants and step/arc helpers shared by all scenarios. Scenarios declare
// only their unique parameters — they call into TestEnv for the actual
// simulation work.

struct ArcStats { bool finite; double avgErr, maxErr, lastErr, lastHead; };

struct TestEnv {
    static constexpr double maxRange = 5.0;
    static constexpr double dt       = 0.1;
    static constexpr double sigR     = 0.05;
    static constexpr double sigB     = 0.02;

    const std::vector<Vector2d> lms = gridLandmarks();

    // Single filter step: observe from `truth` with motion `u`, optionally
    // occluding landmarks via `skip`.
    void step(ParticleFilter& pf, std::mt19937& gen,
              Vector2d u = Vector2d::Zero(), Vector3d truth = Vector3d::Zero(),
              const std::function<bool(const Vector2d&)>& skip = nullptr) const {
        pf.particleFilterLoop(u, simMeasurements(truth, lms, maxRange, sigR, sigB, gen, skip), dt);
    }

    // Stationary-at-origin shorthand when only the skip predicate varies.
    void step(ParticleFilter& pf, std::mt19937& gen,
              const std::function<bool(const Vector2d&)>& skip) const {
        step(pf, gen, Vector2d::Zero(), Vector3d::Zero(), skip);
    }

    // Drive `steps` unicycle steps from `truth`, accumulating error statistics.
    ArcStats runArc(ParticleFilter& pf, std::mt19937& gen,
                    Vector2d u, int steps, Vector3d truth = Vector3d::Zero()) const {
        ArcStats r{true, 0.0, 0.0, 0.0, 0.0};
        for (int s = 0; s < steps; ++s) {
            truth = stepPose(truth, u, dt);
            step(pf, gen, u, truth);
            const Vector3d est = pf.getBestParticle().x;
            r.finite = r.finite && isFinite(est) && allParticlesFinite(pf);
            const double e = posError(est, truth);
            r.avgErr += e; r.maxErr = std::max(r.maxErr, e); r.lastErr = e;
            r.lastHead = headingError(est, truth);
        }
        r.avgErr /= steps;
        return r;
    }

    // Count landmarks within sensor range of `from`.
    int countVisible(Vector2d from = Vector2d::Zero()) const {
        int n = 0;
        for (const auto& lm : lms) if ((lm - from).norm() <= maxRange) ++n;
        return n;
    }
};

// ----- test registry ---------------------------------------------------------

struct TestCase { std::string name; std::function<void(TestEnv&)> fn; };
std::vector<TestCase> g_tests;

void addTest(std::string name, std::function<void(TestEnv&)> fn) {
    g_tests.push_back({std::move(name), std::move(fn)});
}

int runAll() {
    TestEnv env;
    for (auto& t : g_tests) {
        std::printf("\n== %s ==\n", t.name.c_str());
        t.fn(env);
    }
    return g_failures;
}

// ----- scenarios -------------------------------------------------------------

// Drive a gentle arc through the map; the best particle should track the true
// pose and not blow up.
void scenarioTracking(TestEnv& e) {
    ParticleFilter pf(100); std::mt19937 gen(1);
    auto r = e.runArc(pf, gen, Vector2d(0.4, 0.15), /*steps=*/120);
    std::printf("  avg=%.3f max=%.3f final=%.3f head=%.3f\n",
                r.avgErr, r.maxErr, r.lastErr, r.lastHead);
    check("poses stay finite",             r.finite);
    check("avg position error < 0.75 m",   r.avgErr  < 0.75, "avg="   + std::to_string(r.avgErr));
    check("final position error < 0.75 m", r.lastErr < 0.75, "final=" + std::to_string(r.lastErr));
    check("final heading error < 0.3 rad", r.lastHead < 0.3, "final=" + std::to_string(r.lastHead));
}

// No usable measurements for many cycles: exercises the degenerate-weight guard
// (A2) and the resample bounds (A1). Must not crash or produce NaN.
void scenarioDegenerate(TestEnv& e) {
    ParticleFilter pf(50);
    const Vector2d u(0.3, -0.2);
    bool finite = true;
    for (int s = 0; s < 200; ++s) {
        std::vector<Vector2d> z;
        if (s % 2 == 0) z.emplace_back(Vector2d(e.maxRange + 10.0, 0.3));  // gated by range
        pf.particleFilterLoop(u, z, e.dt);
        finite = finite && isFinite(pf.getBestParticle().x) && allParticlesFinite(pf);
    }
    check("no NaN/crash over 200 empty cycles", finite);
}

// Stationary observer: the map should converge to roughly the visible landmark
// count, not explode with duplicates (validates Mahalanobis-gated association, C2).
void scenarioMapStability(int numParticles, TestEnv& e) {
    ParticleFilter pf(numParticles); std::mt19937 gen(7);
    std::printf("  (numParticles=%d)\n", numParticles);
    const int visible = e.countVisible();
    for (int s = 0; s < 80; ++s) e.step(pf, gen);
    const auto est = pf.getBestParticle().getLandmarks();
    const int mapped = static_cast<int>(est.size());
    int near = 0, maxDup = 0;
    for (const auto& lm : e.lms) {
        if (lm.norm() > e.maxRange) continue;
        const int d = landmarksNear(est, lm, 0.5);
        near += d; maxDup = std::max(maxDup, d);
    }
    std::printf("  visible=%d mapped=%d near=%d maxDup=%d orphans=%d\n",
                visible, mapped, near, maxDup, mapped - near);
    check("mapped >= visible",              mapped >= visible,      "mapped=" + std::to_string(mapped));
    check("no duplicate explosion (<= 2x)", mapped <= 2 * visible, "mapped=" + std::to_string(mapped));
}

// A landmark observed briefly then occluded (while still in range) should be
// purged within a bounded number of cycles. (Validates B1.)
void scenarioLandmarkExpiry(TestEnv& e) {
    ParticleFilter pf(50); std::mt19937 gen(11);
    const Vector2d target(2.0, 0.0);
    auto occlude = [&](const Vector2d& lm) { return (lm - target).norm() < 1e-6; };

    for (int s = 0; s < 4; ++s) e.step(pf, gen);  // establish target
    const bool presentBefore = landmarksNear(pf.getBestParticle().getLandmarks(), target, 0.5) > 0;

    int removedAt = -1;
    for (int s = 0; s < 25; ++s) {
        e.step(pf, gen, occlude);
        if (removedAt < 0 && landmarksNear(pf.getBestParticle().getLandmarks(), target, 0.5) == 0)
            removedAt = s + 1;
    }
    std::printf("  present before=%d, removed at cycle %d\n", presentBefore ? 1 : 0, removedAt);
    check("target mapped before occlusion", presentBefore);
    check("target purged after occlusion",  removedAt >= 0,
          removedAt < 0 ? "never" : "cycle " + std::to_string(removedAt));
    check("target purged within 15 cycles", removedAt >= 0 && removedAt <= 15,
          "cycle " + std::to_string(removedAt));
}

// ----- registration ----------------------------------------------------------
// KD-tree unit tests live in test/test_persistent_kdtree.cpp (GTest).

void registerTests() {
    addTest("tracking + mapping",                    scenarioTracking);
    addTest("degenerate (no measurements)",          scenarioDegenerate);
    addTest("map stability, 1 particle",   [](TestEnv& e) { scenarioMapStability(1,  e); });
    addTest("map stability, 50 particles", [](TestEnv& e) { scenarioMapStability(50, e); });
    addTest("landmark expiry after occlusion",       scenarioLandmarkExpiry);
}

}  // namespace

int main() {
    std::printf("ParticleFilter simulation harness\n");
    registerTests();
    const int failures = runAll();
    std::printf("\n%s (%d failure%s)\n", failures == 0 ? "ALL PASSED" : "FAILURES",
                failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
