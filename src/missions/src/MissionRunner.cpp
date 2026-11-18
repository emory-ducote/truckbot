#include "MissionRunner.h"
#include <cmath>
#include <stdexcept>

MissionRunner::MissionRunner(double goalMatchTolerance)
    : missionIndex(0), stateIndex(0), waitingForGoal(false), waitingForLift(false), systemReady(false), stateActive(false),
      goalMatchTolerance(goalMatchTolerance), state(MissionState::IDLE)
{
}

void MissionRunner::loadMissions(const std::string& path)
{
    missionQueue.clear();
    // yaml-cpp iterates maps in file order, so states run top to bottom
    for (const auto& missionEntry : YAML::LoadFile(path)) {
        missions::Mission mission;
        mission.name = missionEntry.first.as<std::string>();
        for (const auto& stateEntry : missionEntry.second) {
            mission.states.push_back(parseState(mission.name, stateEntry.first.as<std::string>(), stateEntry.second));
        }
        missionQueue.push_back(mission);
    }
    missionIndex = 0;
    stateIndex = 0;
    waitingForGoal = false;
    waitingForLift = false;
    lastReachedGoal.reset();
    stateActive = false;
    state = MissionState::IDLE;
}

void MissionRunner::setSystemReady()
{
    systemReady = true;
}

// Returns true only on the transition, so the caller sends its stop commands once
bool MissionRunner::enterError()
{
    // Failures before the mission starts are startup noise, and a finished mission has nothing to abort
    if (!systemReady || state == MissionState::COMPLETE || state == MissionState::ERROR) {
        return false;
    }
    waitingForGoal = false;
    waitingForLift = false;
    state = MissionState::ERROR;
    return true;
}

// Only a report for the current goal counts, so stale or repeated reports can't skip states
void MissionRunner::goalReached(double x, double y)
{
    if (!waitingForGoal) {
        return;
    }
    const missions::Goal& goal = *missionQueue[missionIndex].states[stateIndex].goal;
    if (!sameGoal(goal, {x, y})) {
        return;
    }
    waitingForGoal = false;
    lastReachedGoal = goal;
}

void MissionRunner::liftDone()
{
    waitingForLift = false;
}

// Returns the state to dispatch when the machine moves on to a new one
std::optional<missions::State> MissionRunner::step()
{
    switch (state) {
        case MissionState::IDLE:
            if (!systemReady) {
                return std::nullopt;
            }
            return startNextState();
        case MissionState::RUNNING:
            if (waitingForGoal || waitingForLift) {
                return std::nullopt;
            }
            stateIndex++;
            return startNextState();
        case MissionState::COMPLETE:
        case MissionState::ERROR:
            break;
    }
    return std::nullopt;
}

MissionState MissionRunner::getState() const
{
    return state;
}

bool MissionRunner::isWaitingForGoal() const
{
    return waitingForGoal;
}

bool MissionRunner::isWaitingForLift() const
{
    return waitingForLift;
}

const missions::Mission* MissionRunner::getCurrentMission() const
{
    if (missionIndex >= missionQueue.size()) {
        return nullptr;
    }
    return &missionQueue[missionIndex];
}

// Still reports the interrupted state in ERROR so it shows which state was aborted
const missions::State* MissionRunner::getCurrentState() const
{
    if (!stateActive) {
        return nullptr;
    }
    return &missionQueue[missionIndex].states[stateIndex];
}

missions::State MissionRunner::parseState(const std::string& missionName, const std::string& stateName, const YAML::Node& node)
{
    missions::State state;
    state.name = stateName;
    if (node["Goal"]) {
        state.goal = missions::Goal{node["Goal"]["x"].as<double>(), node["Goal"]["y"].as<double>()};
    }
    if (node["LiftEnabled"]) {
        state.liftEnabled = node["LiftEnabled"].as<bool>();
    }
    if (!state.goal && !state.liftEnabled) {
        throw std::runtime_error("state " + missionName + "/" + stateName + " has no Goal or LiftEnabled");
    }
    return state;
}

std::optional<missions::State> MissionRunner::startNextState()
{
    while (missionIndex < missionQueue.size() && stateIndex >= missionQueue[missionIndex].states.size()) {
        missionIndex++;
        stateIndex = 0;
    }
    if (missionIndex >= missionQueue.size()) {
        stateActive = false;
        state = MissionState::COMPLETE;
        return std::nullopt;
    }

    missions::State next = missionQueue[missionIndex].states[stateIndex];
    // Already sitting at this goal, so drop it rather than wait on a report that won't come
    if (next.goal && lastReachedGoal && sameGoal(*next.goal, *lastReachedGoal)) {
        next.goal.reset();
    }
    waitingForGoal = next.goal.has_value();
    waitingForLift = next.liftEnabled.has_value();
    stateActive = true;
    state = MissionState::RUNNING;
    return next;
}

bool MissionRunner::sameGoal(const missions::Goal& a, const missions::Goal& b) const
{
    return std::hypot(a.x - b.x, a.y - b.y) < goalMatchTolerance;
}
