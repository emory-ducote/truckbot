// Standalone simulation harness for the FastSLAM ParticleFilter.
//
// It drives the filter against a known ground-truth landmark map and a
// scripted vehicle trajectory, injecting Gaussian range/bearing noise, then
// reports tracking/mapping accuracy and asserts a handful of regression
// properties (no NaN/crash on degenerate input, bounded pose error, stable
// landmark counts, landmark expiry). Run it directly — it needs no ROS graph.
//
// Add scenarios in main(); each scenario uses the small helpers below.

#include "ParticleFilter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
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

// Count landmarks in `est` that lie within `tol` of `target`.
int landmarksNear(std::vector<Vector2d> est, const Vector2d& target, double tol) {
    int n = 0;
    for (const auto& e : est)
        if ((e - target).norm() <= tol) ++n;
    return n;
}

bool allParticlesFinite(const ParticleFilter& pf) {
    for (const auto& p : pf.getParticles())
        if (!isFinite(p.x)) return false;
    return true;
}

double maxRange = 5.0;  // keep in sync with the filter's maxRange below

// ----- scenarios -------------------------------------------------------------

// Drive a gentle arc through the map with observations every step; the best
// particle should track the true pose and not blow up.
void scenarioTracking() {
    std::printf("\n== Scenario: tracking + mapping ==\n");
    ParticleFilter pf(/*numParticles=*/100);
    const auto lms = gridLandmarks();
    std::mt19937 gen(1);

    Vector3d truth(0, 0, 0);
    Vector2d u(0.4, 0.15);  // forward + slight turn
    const double dt = 0.1;

    double sumErr = 0.0, maxErr = 0.0, lastErr = 0.0, lastHead = 0.0;
    bool finite = true;
    const int steps = 120;
    for (int s = 0; s < steps; ++s) {
        truth = stepPose(truth, u, dt);
        auto z = simMeasurements(truth, lms, maxRange, 0.05, 0.02, gen);
        pf.particleFilterLoop(u, z, dt);
        const Vector3d est = pf.getBestParticle().x;
        finite = finite && isFinite(est) && allParticlesFinite(pf);
        const double e = posError(est, truth);
        sumErr += e;
        maxErr = std::max(maxErr, e);
        lastErr = e;
        lastHead = headingError(est, truth);
    }
    const double avgErr = sumErr / steps;
    std::printf("  avg pos err=%.3f m, max=%.3f m, final=%.3f m, final heading err=%.3f rad\n",
                avgErr, maxErr, lastErr, lastHead);
    check("poses stay finite", finite);
    check("avg position error < 0.75 m", avgErr < 0.75,
          "avg=" + std::to_string(avgErr));
    check("final position error < 0.75 m", lastErr < 0.75,
          "final=" + std::to_string(lastErr));
    check("final heading error < 0.3 rad", lastHead < 0.3,
          "final=" + std::to_string(lastHead));
}

// No usable measurements for many cycles: exercises the degenerate-weight guard
// (A2) and the resample bounds (A1). Must not crash or produce NaN.
void scenarioDegenerate() {
    std::printf("\n== Scenario: degenerate (no measurements) ==\n");
    ParticleFilter pf(/*numParticles=*/50);
    Vector2d u(0.3, -0.2);
    const double dt = 0.1;
    bool finite = true;
    for (int s = 0; s < 200; ++s) {
        // Empty measurements every other cycle, all-out-of-range the rest.
        std::vector<Vector2d> z;
        if (s % 2 == 0) z.emplace_back(Vector2d(maxRange + 10.0, 0.3));  // gated by range
        pf.particleFilterLoop(u, z, dt);
        finite = finite && isFinite(pf.getBestParticle().x) && allParticlesFinite(pf);
    }
    check("no NaN/crash over 200 empty cycles", finite);
}

// Stationary observer seeing the same landmarks repeatedly: the map should
// converge to roughly the visible landmark count, not explode with duplicates
// (validates the Mahalanobis-gated association, C2).
void scenarioMapStability(int numParticles) {
    std::printf("\n== Scenario: map stability, %d particle(s) ==\n", numParticles);
    ParticleFilter pf(numParticles);
    const auto lms = gridLandmarks();
    std::mt19937 gen(7);
    Vector2d u(0.0, 0.0);  // stationary
    const double dt = 0.1;
    Vector3d truth(0, 0, 0);

    int visibleTrue = 0;
    for (const auto& lm : lms)
        if (lm.norm() <= maxRange) ++visibleTrue;

    for (int s = 0; s < 80; ++s) {
        auto z = simMeasurements(truth, lms, maxRange, 0.05, 0.02, gen);
        pf.particleFilterLoop(u, z, dt);
    }
    const auto est = pf.getBestParticle().getLandmarks();
    const int mapped = static_cast<int>(est.size());
    std::printf("  visible true landmarks=%d, mapped=%d\n", visibleTrue, mapped);
    // Diagnostic: how do estimates distribute around each visible true landmark?
    int near = 0, orphan = 0, maxDup = 0;
    for (const auto& lm : lms) {
        if (lm.norm() > maxRange) continue;
        const int d = landmarksNear(est, lm, 0.5);
        near += d;
        maxDup = std::max(maxDup, d);
    }
    orphan = mapped - near;  // estimates not within 0.5m of any visible true landmark
    std::printf("  estimates within 0.5m of a true lm=%d (max dup on one=%d), orphans=%d\n",
                near, maxDup, orphan);
    check("mapped >= visible true count", mapped >= visibleTrue,
          "mapped=" + std::to_string(mapped));
    check("no duplicate explosion (mapped <= 2x visible)", mapped <= 2 * visibleTrue,
          "mapped=" + std::to_string(mapped));
}

// A landmark that is observed a few times and then occluded (while still in
// range) should be purged within a bounded number of cycles. With the old
// max()-instead-of-min() bug its count would jump to 20 and take ~20 cycles;
// with the fix it expires far sooner. (Validates B1.)
void scenarioLandmarkExpiry() {
    std::printf("\n== Scenario: landmark expiry after occlusion ==\n");
    ParticleFilter pf(/*numParticles=*/50);
    const auto lms = gridLandmarks();
    std::mt19937 gen(11);
    Vector2d u(0.0, 0.0);
    const double dt = 0.1;
    Vector3d truth(0, 0, 0);

    const Vector2d target(2.0, 0.0);  // in range (r=2), will be occluded
    auto occludeTarget = [&](const Vector2d& lm) { return (lm - target).norm() < 1e-6; };

    // Observe everything (incl. target) a few times to establish + match it.
    for (int s = 0; s < 4; ++s) {
        auto z = simMeasurements(truth, lms, maxRange, 0.05, 0.02, gen);
        pf.particleFilterLoop(u, z, dt);
    }
    const bool presentBefore =
        landmarksNear(pf.getBestParticle().getLandmarks(), target, 0.5) > 0;

    // Now occlude the target (still in range, just never reported).
    int removedAt = -1;
    for (int s = 0; s < 25; ++s) {
        auto z = simMeasurements(truth, lms, maxRange, 0.05, 0.02, gen, occludeTarget);
        pf.particleFilterLoop(u, z, dt);
        if (removedAt < 0 &&
            landmarksNear(pf.getBestParticle().getLandmarks(), target, 0.5) == 0)
            removedAt = s + 1;
    }
    std::printf("  present before occlusion=%d, removed after %d cycles\n",
                presentBefore ? 1 : 0, removedAt);
    check("target mapped before occlusion", presentBefore);
    check("target purged after occlusion", removedAt >= 0,
          removedAt < 0 ? "never removed" : "cycle " + std::to_string(removedAt));
    check("target purged within 15 cycles", removedAt >= 0 && removedAt <= 15,
          "cycle " + std::to_string(removedAt));
}

// Directly test the persistent KD-tree's nearest-neighbour search against a
// brute-force baseline, after a mix of inserts and deletes. If association
// duplicates come from a faulty tree, this fails; if it passes, the duplication
// lives in the gate/association logic instead.
void scenarioKDTree() {
    std::printf("\n== Scenario: KD-tree nearest-neighbour correctness ==\n");
    std::mt19937 gen(42);
    std::uniform_real_distribution<> coord(-5.0, 5.0);

    auto nnMismatches = [](std::shared_ptr<const Node> r, std::mt19937& g, int n) {
        std::uniform_real_distribution<> c(-5.0, 5.0);
        std::vector<Vector2d> contents;
        collectKDTreeLandmarks(r, contents);
        int bad = 0;
        for (int q = 0; q < n; ++q) {
            double t[2] = {c(g), c(g)};
            const Node* got = findNearest(r, t);
            double bestD = std::numeric_limits<double>::infinity();
            for (const auto& p : contents) {
                double d = (p - Vector2d(t[0], t[1])).squaredNorm();
                if (d < bestD) bestD = d;
            }
            double gotD = got ? std::pow(got->point[0] - t[0], 2) + std::pow(got->point[1] - t[1], 2)
                              : std::numeric_limits<double>::infinity();
            if (!got || std::abs(gotD - bestD) > 1e-9) bad++;
        }
        return bad;
    };

    std::shared_ptr<const Node> root = nullptr;
    std::vector<Vector2d> inserted;
    for (int i = 0; i < 200; ++i) {
        double pt[2] = {coord(gen), coord(gen)};
        root = insert(root, pt, Matrix2d::Identity());
        inserted.emplace_back(Vector2d(pt[0], pt[1]));
    }
    // findNearest correctness on an insert-only tree (isolates search from delete).
    const int preBad = nnMismatches(root, gen, 2000);
    check("findNearest correct on insert-only tree", preBad == 0,
          std::to_string(preBad) + " mismatches");

    // Delete ~40% of the points.
    std::shuffle(inserted.begin(), inserted.end(), gen);
    std::vector<Vector2d> live(inserted.begin() + 80, inserted.end());
    for (int i = 0; i < 80; ++i) {
        double pt[2] = {inserted[i][0], inserted[i][1]};
        root = deleteNode(root, pt);
    }

    // The set the tree actually holds now.
    std::vector<Vector2d> treeContents;
    collectKDTreeLandmarks(root, treeContents);

    int mismatches = 0, missing = 0;
    for (int q = 0; q < 2000; ++q) {
        double target[2] = {coord(gen), coord(gen)};
        const Node* got = findNearest(root, target);
        // brute force over what the tree reports it contains
        double bestD = std::numeric_limits<double>::infinity();
        Vector2d bestPt = Vector2d::Zero();
        for (const auto& p : treeContents) {
            double d = (p - Vector2d(target[0], target[1])).squaredNorm();
            if (d < bestD) { bestD = d; bestPt = p; }
        }
        if (!got) { missing++; continue; }
        double gotD = std::pow(got->point[0] - target[0], 2) + std::pow(got->point[1] - target[1], 2);
        if (std::abs(gotD - bestD) > 1e-9) mismatches++;
    }
    std::printf("  tree size=%zu (expected ~%zu), queries=2000, mismatches=%d, missing=%d\n",
                treeContents.size(), live.size(), mismatches, missing);
    check("tree reports expected size after deletes", treeContents.size() == live.size(),
          "got " + std::to_string(treeContents.size()) + " expected " + std::to_string(live.size()));
    check("findNearest matches brute force", mismatches == 0,
          std::to_string(mismatches) + " mismatches");
    check("findNearest never spuriously null", missing == 0);
}

}  // namespace

int main() {
    std::printf("ParticleFilter simulation harness\n");
    scenarioKDTree();
    scenarioTracking();
    scenarioDegenerate();
    scenarioMapStability(1);
    scenarioMapStability(50);
    scenarioLandmarkExpiry();

    std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "ALL PASSED" : "FAILURES",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
