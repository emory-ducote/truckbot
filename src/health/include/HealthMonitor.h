#ifndef HEALTHMONITOR_H_
#define HEALTHMONITOR_H_

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include "truckbot_msgs/msg/heartbeat.hpp"

class HealthMonitor {
    public:
        explicit HealthMonitor(double heartbeatTimeout = 3.0);

        void updateHeartbeat(const truckbot_msgs::msg::Heartbeat& heartbeat, std::chrono::steady_clock::time_point now);
        std::vector<truckbot_msgs::msg::Heartbeat> getModules(std::chrono::steady_clock::time_point now) const;
        static uint8_t overallState(const std::vector<truckbot_msgs::msg::Heartbeat>& modules);

    private:
        struct Module {
            truckbot_msgs::msg::Heartbeat heartbeat;
            std::chrono::steady_clock::time_point received;
        };

        std::map<std::string, Module> modules;
        std::chrono::duration<double> heartbeatTimeout;
};

#endif
