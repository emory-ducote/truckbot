#include "MissionRunner.h"
#include <stdexcept>

MissionRunner::MissionRunner()
    : missionIndex(0), stateIndex(0), waitingForGoal(false), waitingForLift(false), state(MissionState::IDLE)
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
    state = MissionState::IDLE;
}

void MissionRunner::goalReached()
{
    waitingForGoal = false;
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
            return startNextState();
        case MissionState::RUNNING:
            if (waitingForGoal || waitingForLift) {
                return std::nullopt;
            }
            stateIndex++;
            return startNextState();
        case MissionState::COMPLETE:
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

const missions::State* MissionRunner::getCurrentState() const
{
    if (state != MissionState::RUNNING) {
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
        state = MissionState::COMPLETE;
        return std::nullopt;
    }

    const missions::State& next = missionQueue[missionIndex].states[stateIndex];
    waitingForGoal = next.goal.has_value();
    waitingForLift = next.liftEnabled.has_value();
    state = MissionState::RUNNING;
    return next;
}
