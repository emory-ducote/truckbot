// Unit tests for the persistent (immutable, copy-on-write) KD-tree used by the
// particle filter to store and query per-particle landmark maps.
//
// These guard the operations the filter depends on:
//   - insert      (build the map)
//   - findNearest (data association)
//   - deleteNode  (landmark purging)
//
// The decisive checks compare findNearest against a brute-force linear scan over
// the tree's own reported contents, and verify the KD invariant directly, after
// random insert/delete sequences. A regression here previously corrupted the
// tree on delete (findMin was called with the wrong depth), silently breaking
// nearest-neighbour search and spawning duplicate landmarks.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "PersistentKDTree.h"

using Eigen::Matrix2d;
using Eigen::Vector2d;

namespace {

std::shared_ptr<const Node> insertPt(std::shared_ptr<const Node> root, double x, double y) {
    double p[2] = {x, y};
    return insert(root, p, Matrix2d::Identity());
}

std::shared_ptr<const Node> deletePt(std::shared_ptr<const Node> root, double x, double y) {
    double p[2] = {x, y};
    return deleteNode(root, p);
}

std::vector<Vector2d> contents(std::shared_ptr<const Node> root) {
    std::vector<Vector2d> v;
    collectKDTreeLandmarks(root, v);
    return v;
}

// Brute-force nearest over an explicit point set.
Vector2d bruteNearest(const std::vector<Vector2d>& pts, const Vector2d& q) {
    double best = std::numeric_limits<double>::infinity();
    Vector2d bestPt = Vector2d::Zero();
    for (const auto& p : pts) {
        double d = (p - q).squaredNorm();
        if (d < best) { best = d; bestPt = p; }
    }
    return bestPt;
}

// Verify the KD invariant: at depth d (split dim = d % k), every node in the
// left subtree has point[dim] < node[dim], and every node in the right subtree
// has point[dim] >= node[dim].
bool subtreeRespects(std::shared_ptr<const Node> node, int dim, double bound, bool isLess) {
    if (!node) return true;
    if (isLess) { if (!(node->point[dim] < bound)) return false; }
    else        { if (!(node->point[dim] >= bound)) return false; }
    return true;  // bound checked per-ancestor by checkInvariant below
}

bool checkInvariant(std::shared_ptr<const Node> node, unsigned depth = 0) {
    if (!node) return true;
    int dim = depth % k;
    // All nodes in left subtree must be < node on this dim; right must be >=.
    std::vector<Vector2d> leftPts = contents(node->left);
    std::vector<Vector2d> rightPts = contents(node->right);
    for (const auto& p : leftPts)
        if (!(p[dim] < node->point[dim])) return false;
    for (const auto& p : rightPts)
        if (!(p[dim] >= node->point[dim])) return false;
    return checkInvariant(node->left, depth + 1) && checkInvariant(node->right, depth + 1);
}

}  // namespace

TEST(PersistentKDTree, EmptyTreeNearestIsNull) {
    double q[2] = {0.0, 0.0};
    EXPECT_EQ(findNearest(nullptr, q), nullptr);
}

TEST(PersistentKDTree, SingleNode) {
    auto root = insertPt(nullptr, 1.0, 2.0);
    double q[2] = {5.0, 5.0};
    const Node* got = findNearest(root, q);
    ASSERT_NE(got, nullptr);
    EXPECT_DOUBLE_EQ(got->point[0], 1.0);
    EXPECT_DOUBLE_EQ(got->point[1], 2.0);
}

TEST(PersistentKDTree, InsertIsImmutable) {
    auto a = insertPt(nullptr, 0.0, 0.0);
    auto b = insertPt(a, 1.0, 1.0);
    // Inserting into `a` must not mutate `a` (copy-on-write).
    EXPECT_EQ(contents(a).size(), 1u);
    EXPECT_EQ(contents(b).size(), 2u);
}

TEST(PersistentKDTree, NearestOnInsertOnlyTreeMatchesBruteForce) {
    std::mt19937 gen(1);
    std::uniform_real_distribution<> coord(-10.0, 10.0);
    std::shared_ptr<const Node> root = nullptr;
    std::vector<Vector2d> pts;
    for (int i = 0; i < 300; ++i) {
        double x = coord(gen), y = coord(gen);
        root = insertPt(root, x, y);
        pts.emplace_back(x, y);
    }
    for (int q = 0; q < 3000; ++q) {
        Vector2d query(coord(gen), coord(gen));
        double t[2] = {query[0], query[1]};
        const Node* got = findNearest(root, t);
        ASSERT_NE(got, nullptr);
        Vector2d expect = bruteNearest(pts, query);
        Vector2d gotV(got->point[0], got->point[1]);
        EXPECT_NEAR((gotV - query).squaredNorm(), (expect - query).squaredNorm(), 1e-9);
    }
}

TEST(PersistentKDTree, DeleteRemovesExactlyOneNode) {
    auto root = insertPt(nullptr, 0, 0);
    root = insertPt(root, 1, 1);
    root = insertPt(root, -1, 2);
    ASSERT_EQ(contents(root).size(), 3u);
    root = deletePt(root, 1, 1);
    EXPECT_EQ(contents(root).size(), 2u);
    // The deleted point is gone; the others remain.
    for (const auto& p : contents(root)) EXPECT_FALSE(p[0] == 1.0 && p[1] == 1.0);
}

TEST(PersistentKDTree, DeleteNonexistentIsNoOp) {
    auto root = insertPt(nullptr, 0, 0);
    root = insertPt(root, 1, 1);
    auto same = deletePt(root, 9, 9);
    EXPECT_EQ(contents(same).size(), 2u);
}

TEST(PersistentKDTree, DeleteEverythingEmptiesTree) {
    std::mt19937 gen(2);
    std::uniform_real_distribution<> coord(-5.0, 5.0);
    std::shared_ptr<const Node> root = nullptr;
    std::vector<Vector2d> pts;
    for (int i = 0; i < 100; ++i) {
        double x = coord(gen), y = coord(gen);
        root = insertPt(root, x, y);
        pts.emplace_back(x, y);
    }
    for (const auto& p : pts) root = deletePt(root, p[0], p[1]);
    EXPECT_EQ(root, nullptr);
    EXPECT_EQ(contents(root).size(), 0u);
}

// The regression test for the depth bug: after a batch of deletes, the size must
// be exact, the KD invariant must hold, and nearest-neighbour must still match
// brute force over whatever the tree now contains.
TEST(PersistentKDTree, NearestAfterDeletesMatchesBruteForce) {
    std::mt19937 gen(42);
    std::uniform_real_distribution<> coord(-10.0, 10.0);
    std::shared_ptr<const Node> root = nullptr;
    std::vector<Vector2d> pts;
    for (int i = 0; i < 300; ++i) {
        double x = coord(gen), y = coord(gen);
        root = insertPt(root, x, y);
        pts.emplace_back(x, y);
    }
    // Delete a random ~40%.
    std::shuffle(pts.begin(), pts.end(), gen);
    const size_t toDelete = 120;
    for (size_t i = 0; i < toDelete; ++i) root = deletePt(root, pts[i][0], pts[i][1]);
    std::vector<Vector2d> live(pts.begin() + toDelete, pts.end());

    EXPECT_EQ(contents(root).size(), live.size());
    EXPECT_TRUE(checkInvariant(root)) << "KD invariant violated after deletes";

    auto treePts = contents(root);
    for (int q = 0; q < 3000; ++q) {
        Vector2d query(coord(gen), coord(gen));
        double t[2] = {query[0], query[1]};
        const Node* got = findNearest(root, t);
        ASSERT_NE(got, nullptr);
        Vector2d expect = bruteNearest(treePts, query);
        Vector2d gotV(got->point[0], got->point[1]);
        EXPECT_NEAR((gotV - query).squaredNorm(), (expect - query).squaredNorm(), 1e-9);
    }
}

// Property test: interleave random inserts and deletes, checking the invariant
// and a brute-force size mirror throughout.
TEST(PersistentKDTree, RandomInsertDeleteSequenceKeepsInvariant) {
    std::mt19937 gen(7);
    std::uniform_real_distribution<> coord(-8.0, 8.0);
    std::uniform_real_distribution<> op(0.0, 1.0);
    std::shared_ptr<const Node> root = nullptr;
    std::vector<Vector2d> model;  // brute-force mirror of tree contents

    for (int step = 0; step < 2000; ++step) {
        if (model.empty() || op(gen) < 0.6) {
            double x = coord(gen), y = coord(gen);
            root = insertPt(root, x, y);
            model.emplace_back(x, y);
        } else {
            std::uniform_int_distribution<size_t> pick(0, model.size() - 1);
            size_t idx = pick(gen);
            Vector2d victim = model[idx];
            root = deletePt(root, victim[0], victim[1]);
            model.erase(model.begin() + idx);
        }
        ASSERT_EQ(contents(root).size(), model.size()) << "size mismatch at step " << step;
        ASSERT_TRUE(checkInvariant(root)) << "invariant broken at step " << step;
    }

    // Final spot-check of nearest-neighbour correctness.
    auto treePts = contents(root);
    for (int q = 0; q < 1000; ++q) {
        Vector2d query(coord(gen), coord(gen));
        double t[2] = {query[0], query[1]};
        const Node* got = findNearest(root, t);
        ASSERT_NE(got, nullptr);
        Vector2d expect = bruteNearest(treePts, query);
        Vector2d gotV(got->point[0], got->point[1]);
        EXPECT_NEAR((gotV - query).squaredNorm(), (expect - query).squaredNorm(), 1e-9);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
