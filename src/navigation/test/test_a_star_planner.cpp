// Unit tests for AStarPlanner: grid search over an occupancy-style cost array.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <set>
#include <utility>
#include <vector>

#include "AStarPlanner.h"

namespace {

using Cell = std::pair<int, int>;
using Path = std::vector<Cell>;

constexpr int8_t kFree = 0;
constexpr int8_t kBlocked = 100;

class AStarPlannerTest : public ::testing::Test {
  protected:
    void makeGrid(int w, int h, int8_t fill = kFree) {
        width = w;
        height = h;
        costs.assign(static_cast<size_t>(w) * h, fill);
    }

    void block(int col, int row, int8_t cost = kBlocked) {
        costs[static_cast<size_t>(row) * width + col] = cost;
    }

    void load(AStarPlanner& planner) {
        planner.setCosts(costs, 1.0, width, height, 0.0, 0.0);
    }

    bool isBlocked(const Cell& c, int8_t thresh = 65) const {
        return costs[static_cast<size_t>(c.second) * width + c.first] >= thresh;
    }

    static bool isContiguous(const Path& path) {
        for (size_t i = 1; i < path.size(); ++i) {
            if (std::abs(path[i].first - path[i - 1].first) > 1 ||
                std::abs(path[i].second - path[i - 1].second) > 1) {
                return false;
            }
        }
        return true;
    }

    int width = 0;
    int height = 0;
    std::vector<int8_t> costs;
};

}  // namespace

// On an empty grid, a horizontal move should return the 5 cells from start to goal in order.
TEST_F(AStarPlannerTest, StraightLineOnEmptyGrid) {
    makeGrid(5, 5);
    AStarPlanner planner;
    load(planner);

    Path path = planner.findPath({0, 0}, {4, 0});

    ASSERT_EQ(path.size(), 5u);
    EXPECT_EQ(path.front(), Cell(0, 0));
    EXPECT_EQ(path.back(), Cell(4, 0));
    EXPECT_TRUE(isContiguous(path));
}

// Diagonal moves are allowed, so (0,0) to (3,3) should take 4 cells, not 7.
TEST_F(AStarPlannerTest, DiagonalIsShorterThanManhattan) {
    makeGrid(6, 6);
    AStarPlanner planner;
    load(planner);

    Path path = planner.findPath({0, 0}, {3, 3});

    ASSERT_EQ(path.size(), 4u);
    EXPECT_EQ(path.front(), Cell(0, 0));
    EXPECT_EQ(path.back(), Cell(3, 3));
    EXPECT_TRUE(isContiguous(path));
}

// If start and goal are the same cell, the path is just that one cell.
TEST_F(AStarPlannerTest, StartEqualsGoalReturnsSingleCell) {
    makeGrid(4, 4);
    AStarPlanner planner;
    load(planner);

    Path path = planner.findPath({2, 2}, {2, 2});

    ASSERT_EQ(path.size(), 1u);
    EXPECT_EQ(path.front(), Cell(2, 2));
}

// A wall with one gap forces the path to go around through the gap and never touch an obstacle.
TEST_F(AStarPlannerTest, RoutesThroughGapInWall) {
    makeGrid(5, 5);
    for (int row = 0; row < 4; ++row) {
        block(2, row);
    }
    AStarPlanner planner;
    load(planner);

    Path path = planner.findPath({0, 0}, {4, 0});

    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), Cell(0, 0));
    EXPECT_EQ(path.back(), Cell(4, 0));
    EXPECT_TRUE(isContiguous(path));
    for (const auto& cell : path) {
        EXPECT_FALSE(isBlocked(cell)) << "path crosses obstacle at (" << cell.first << ", "
                                      << cell.second << ")";
    }
    EXPECT_NE(std::find(path.begin(), path.end(), Cell(2, 4)), path.end());
}

// If a wall fully separates start from goal, no path exists and the result is empty.
TEST_F(AStarPlannerTest, NoPathWhenWallIsComplete) {
    makeGrid(5, 5);
    for (int row = 0; row < 5; ++row) {
        block(2, row);
    }
    AStarPlanner planner;
    load(planner);

    EXPECT_TRUE(planner.findPath({0, 0}, {4, 0}).empty());
}

// A goal cell that is blocked is unreachable, so the result is empty.
TEST_F(AStarPlannerTest, GoalInsideObstacleReturnsEmpty) {
    makeGrid(5, 5);
    block(4, 4);
    AStarPlanner planner;
    load(planner);

    EXPECT_TRUE(planner.findPath({0, 0}, {4, 4}).empty());
}

// A start or goal outside the grid returns an empty path instead of crashing.
TEST_F(AStarPlannerTest, OutOfBoundsStartOrGoalReturnsEmpty) {
    makeGrid(5, 5);
    AStarPlanner planner;
    load(planner);

    EXPECT_TRUE(planner.findPath({-1, 0}, {4, 4}).empty());
    EXPECT_TRUE(planner.findPath({0, 0}, {5, 4}).empty());
    EXPECT_TRUE(planner.findPath({0, 0}, {4, 5}).empty());
}

// A cell with cost 64 is passable and cost 65 is blocked, matching the default threshold of 65.
TEST_F(AStarPlannerTest, CostThresholdBoundary) {
    makeGrid(3, 1);

    block(1, 0, 64);
    {
        AStarPlanner planner(65);
        load(planner);
        EXPECT_EQ(planner.findPath({0, 0}, {2, 0}).size(), 3u);
    }

    block(1, 0, 65);
    {
        AStarPlanner planner(65);
        load(planner);
        EXPECT_TRUE(planner.findPath({0, 0}, {2, 0}).empty());
    }
}

// The path around obstacles should visit each cell at most once and still connect start to goal.
TEST_F(AStarPlannerTest, PathHasNoRepeatedCells) {
    makeGrid(8, 8);
    block(3, 2);
    block(3, 3);
    block(3, 4);
    block(4, 4);
    AStarPlanner planner;
    load(planner);

    Path path = planner.findPath({0, 3}, {7, 3});

    ASSERT_FALSE(path.empty());
    std::set<Cell> unique(path.begin(), path.end());
    EXPECT_EQ(unique.size(), path.size());
    EXPECT_EQ(path.front(), Cell(0, 3));
    EXPECT_EQ(path.back(), Cell(7, 3));
}
