#ifndef MISSIONRUNNER_H_
#define MISSIONRUNNER_H_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "Mission.h"

enum class MissionState { IDLE, RUNNING, COMPLETE };

class MissionRunner {
    public:
        MissionRunner();

        void loadMissions(const std::string& path);
        void goalReached();
        void liftDone();
        std::optional<missions::State> step();
        MissionState getState() const;
        bool isWaitingForGoal() const;
        bool isWaitingForLift() const;
        const missions::Mission* getCurrentMission() const;
        const missions::State* getCurrentState() const;

    private:
        static missions::State parseState(const std::string& missionName, const std::string& stateName, const YAML::Node& node);
        std::optional<missions::State> startNextState();

        std::vector<missions::Mission> missionQueue;
        std::size_t missionIndex;
        std::size_t stateIndex;
        bool waitingForGoal;
        bool waitingForLift;
        MissionState state;
};

#endif
