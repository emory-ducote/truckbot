#ifndef MISSIONRUNNER_H_
#define MISSIONRUNNER_H_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "Mission.h"

enum class MissionState { IDLE, RUNNING, COMPLETE, ERROR };

class MissionRunner {
    public:
        explicit MissionRunner(double goalMatchTolerance = 1e-3);

        void loadMissions(const std::string& path);
        void setSystemReady();
        bool requestStart();
        bool enterError();
        void goalReached(double x, double y);
        void liftDone();
        std::optional<missions::State> step();
        MissionState getState() const;
        bool isWaitingForGoal() const;
        bool isWaitingForLift() const;
        bool isWaitingForStart() const;
        bool isSystemReady() const;
        const missions::Mission* getCurrentMission() const;
        const missions::State* getCurrentState() const;

    private:
        bool sameGoal(const missions::Goal& a, const missions::Goal& b) const;
        static missions::State parseState(const std::string& missionName, const std::string& stateName, const YAML::Node& node);
        std::optional<missions::State> startNextState();

        std::vector<missions::Mission> missionQueue;
        std::size_t missionIndex;
        std::size_t stateIndex;
        bool waitingForGoal;
        bool waitingForLift;
        bool systemReady;
        bool startRequested;
        bool stateActive;
        double goalMatchTolerance;
        std::optional<missions::Goal> lastReachedGoal;
        MissionState state;
};

#endif
