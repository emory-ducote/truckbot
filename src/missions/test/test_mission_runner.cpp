#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "MissionRunner.h"

namespace {

std::string writeMissions(const std::string& name, const std::string& yaml)
{
    std::string path = (std::filesystem::temp_directory_path() / name).string();
    std::ofstream(path) << yaml;
    return path;
}

}

TEST(MissionRunner, RejectsEmptyState)
{
    MissionRunner runner;
    EXPECT_THROW(runner.loadMissions(writeMissions("empty_state.yaml", "M:\n  S:\n    Other: 1\n")), std::runtime_error);
}

TEST(MissionRunner, NoMissionsCompletesImmediately)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("no_missions.yaml", "{}\n"));
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_EQ(runner.getState(), MissionState::COMPLETE);
}

TEST(MissionRunner, WaitsOnEachStateUntilDone)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("two_missions.yaml", R"(
First:
  Drive:
    Goal: {x: 1.0, y: 2.0}
  Raise:
    LiftEnabled: true
Second:
  DriveAndLower:
    Goal: {x: 3.0, y: 4.0}
    LiftEnabled: false
)"));

    auto state = runner.step();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->name, "Drive");
    EXPECT_DOUBLE_EQ(state->goal->y, 2.0);
    EXPECT_FALSE(state->liftEnabled.has_value());
    EXPECT_TRUE(runner.isWaitingForGoal());
    EXPECT_FALSE(runner.step().has_value());

    runner.goalReached();
    state = runner.step();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->name, "Raise");
    EXPECT_EQ(runner.getCurrentState()->name, "Raise");
    EXPECT_TRUE(*state->liftEnabled);
    EXPECT_TRUE(runner.isWaitingForLift());

    runner.liftDone();
    state = runner.step();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(runner.getCurrentMission()->name, "Second");
    EXPECT_TRUE(state->goal.has_value());
    EXPECT_FALSE(*state->liftEnabled);

    runner.goalReached();
    EXPECT_FALSE(runner.step().has_value());
    runner.liftDone();
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_EQ(runner.getState(), MissionState::COMPLETE);
    EXPECT_EQ(runner.getCurrentState(), nullptr);
}

TEST(MissionRunner, IgnoresDoneSignalsBeforeStart)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("one_goal.yaml", "Only:\n  Drive:\n    Goal: {x: 1.0, y: 0.0}\n"));
    runner.goalReached();
    EXPECT_TRUE(runner.step().has_value());
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_TRUE(runner.isWaitingForGoal());
}
