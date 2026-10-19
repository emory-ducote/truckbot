#include "HealthMonitor.h"

HealthMonitor::HealthMonitor(double heartbeatTimeout)
    : heartbeatTimeout(heartbeatTimeout)
{
}

void HealthMonitor::updateHeartbeat(const truckbot_msgs::msg::Heartbeat& heartbeat, std::chrono::steady_clock::time_point now)
{
    modules[heartbeat.node_name] = {heartbeat, now};
}

std::vector<truckbot_msgs::msg::Heartbeat> HealthMonitor::getModules(std::chrono::steady_clock::time_point now) const
{
    std::vector<truckbot_msgs::msg::Heartbeat> result;
    result.reserve(modules.size());
    for (const auto& [name, module] : modules) {
        truckbot_msgs::msg::Heartbeat heartbeat = module.heartbeat;
        if (now - module.received > heartbeatTimeout) {
            heartbeat.state = truckbot_msgs::msg::Heartbeat::FAILED;
            heartbeat.detail = "timed out";
        }
        result.push_back(heartbeat);
    }
    return result;
}

uint8_t HealthMonitor::overallState(const std::vector<truckbot_msgs::msg::Heartbeat>& modules)
{
    bool anyFailed = false;
    bool anyDegraded = false;
    bool anyInit = modules.empty();
    for (const auto& heartbeat : modules) {
        anyFailed |= heartbeat.state == truckbot_msgs::msg::Heartbeat::FAILED;
        anyDegraded |= heartbeat.state == truckbot_msgs::msg::Heartbeat::DEGRADED;
        anyInit |= heartbeat.state == truckbot_msgs::msg::Heartbeat::INIT;
    }

    return anyFailed ? truckbot_msgs::msg::Heartbeat::FAILED
         : anyDegraded ? truckbot_msgs::msg::Heartbeat::DEGRADED
         : anyInit ? truckbot_msgs::msg::Heartbeat::INIT
         : truckbot_msgs::msg::Heartbeat::GOOD;
}
