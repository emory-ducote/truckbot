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
    runner.setSystemReady();
    runner.requestStart();
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
    runner.setSystemReady();
    runner.requestStart();

    auto state = runner.step();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->name, "Drive");
    EXPECT_DOUBLE_EQ(state->goal->y, 2.0);
    EXPECT_FALSE(state->liftEnabled.has_value());
    EXPECT_TRUE(runner.isWaitingForGoal());
    EXPECT_FALSE(runner.step().has_value());

    runner.goalReached(1.0, 2.0);
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

    runner.goalReached(3.0, 4.0);
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
    runner.setSystemReady();
    runner.requestStart();
    runner.goalReached(1.0, 0.0);
    EXPECT_TRUE(runner.step().has_value());
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_TRUE(runner.isWaitingForGoal());
}

TEST(MissionRunner, WaitsForSystemReady)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("one_goal.yaml", "Only:\n  Drive:\n    Goal: {x: 1.0, y: 0.0}\n"));
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_EQ(runner.getState(), MissionState::IDLE);
    runner.setSystemReady();
    EXPECT_TRUE(runner.requestStart());
    EXPECT_TRUE(runner.step().has_value());
}

TEST(MissionRunner, IgnoresReachedForOtherGoals)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("three_goals.yaml", R"(
M:
  A:
    Goal: {x: 1.0, y: 0.0}
  B:
    Goal: {x: 2.0, y: 0.0}
  C:
    Goal: {x: 3.0, y: 0.0}
)"));
    runner.setSystemReady();
    runner.requestStart();
    EXPECT_EQ(runner.step()->name, "A");

    runner.goalReached(2.0, 0.0);
    EXPECT_FALSE(runner.step().has_value());

    runner.goalReached(1.0, 0.0);
    EXPECT_EQ(runner.step()->name, "B");

    // A repeated report for A must not finish B and skip to C
    runner.goalReached(1.0, 0.0);
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_TRUE(runner.isWaitingForGoal());
}

TEST(MissionRunner, DropsGoalAlreadyReached)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("repeat_goal.yaml", R"(
M:
  Drive:
    Goal: {x: 1.0, y: 0.0}
  RaiseHere:
    Goal: {x: 1.0, y: 0.0}
    LiftEnabled: true
)"));
    runner.setSystemReady();
    runner.requestStart();
    runner.step();
    runner.goalReached(1.0, 0.0);

    auto state = runner.step();
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->name, "RaiseHere");
    EXPECT_FALSE(state->goal.has_value());
    EXPECT_FALSE(runner.isWaitingForGoal());
    EXPECT_TRUE(runner.isWaitingForLift());
}

TEST(MissionRunner, ErrorIgnoredBeforeSystemReady)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("one_goal.yaml", "Only:\n  Drive:\n    Goal: {x: 1.0, y: 0.0}\n"));
    EXPECT_FALSE(runner.enterError());
    EXPECT_EQ(runner.getState(), MissionState::IDLE);
}

TEST(MissionRunner, ErrorAbortsRunningState)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("goal_and_lift.yaml", R"(
M:
  DriveAndRaise:
    Goal: {x: 1.0, y: 0.0}
    LiftEnabled: true
  Next:
    Goal: {x: 2.0, y: 0.0}
)"));
    runner.setSystemReady();
    runner.requestStart();
    runner.step();

    EXPECT_TRUE(runner.enterError());
    EXPECT_FALSE(runner.enterError());
    EXPECT_EQ(runner.getState(), MissionState::ERROR);
    EXPECT_FALSE(runner.isWaitingForGoal());
    EXPECT_FALSE(runner.isWaitingForLift());
    EXPECT_EQ(runner.getCurrentState()->name, "DriveAndRaise");

    // Late done reports must not restart the mission
    runner.goalReached(1.0, 0.0);
    runner.liftDone();
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_EQ(runner.getState(), MissionState::ERROR);
}

TEST(MissionRunner, ErrorIgnoredAfterComplete)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("one_lift.yaml", "Only:\n  Raise:\n    LiftEnabled: true\n"));
    runner.setSystemReady();
    runner.requestStart();
    runner.step();
    runner.liftDone();
    runner.step();
    EXPECT_EQ(runner.getState(), MissionState::COMPLETE);
    EXPECT_FALSE(runner.enterError());
}

TEST(MissionRunner, WaitsForStartRequest)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("one_goal.yaml", "Only:\n  Drive:\n    Goal: {x: 1.0, y: 0.0}\n"));
    runner.setSystemReady();
    EXPECT_TRUE(runner.isWaitingForStart());
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_EQ(runner.getState(), MissionState::IDLE);

    EXPECT_TRUE(runner.requestStart());
    EXPECT_FALSE(runner.isWaitingForStart());
    EXPECT_FALSE(runner.requestStart());
    EXPECT_TRUE(runner.step().has_value());
    EXPECT_EQ(runner.getState(), MissionState::RUNNING);
    EXPECT_FALSE(runner.requestStart());
}

TEST(MissionRunner, StartRejectedBeforeSystemReady)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("one_goal.yaml", "Only:\n  Drive:\n    Goal: {x: 1.0, y: 0.0}\n"));
    EXPECT_FALSE(runner.requestStart());
    runner.setSystemReady();
    // The early request was dropped, not latched
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_TRUE(runner.isWaitingForStart());
}

TEST(MissionRunner, ErrorWhileWaitingForStartDropsRequest)
{
    MissionRunner runner;
    runner.loadMissions(writeMissions("one_goal.yaml", "Only:\n  Drive:\n    Goal: {x: 1.0, y: 0.0}\n"));
    runner.setSystemReady();
    EXPECT_TRUE(runner.requestStart());
    EXPECT_FALSE(runner.enterError());
    EXPECT_EQ(runner.getState(), MissionState::IDLE);
    EXPECT_FALSE(runner.step().has_value());
    EXPECT_TRUE(runner.isWaitingForStart());
    EXPECT_TRUE(runner.requestStart());
    EXPECT_TRUE(runner.step().has_value());
}
