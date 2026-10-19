#include <chrono>
#include "HealthMonitor.h"
#include "rclcpp/rclcpp.hpp"
#include "truckbot_msgs/msg/heartbeat.hpp"
#include "truckbot_msgs/msg/system_health.hpp"

class HealthMonitorMiddleware : public rclcpp::Node {

  public:
    HealthMonitorMiddleware()
      : Node("health_monitor")
    {
      double heartbeatTimeout = this->declare_parameter<double>("heartbeat_timeout", 3.0);

      health_monitor_ = std::make_shared<HealthMonitor>(heartbeatTimeout);

      heartbeat_sub_ = this->create_subscription<truckbot_msgs::msg::Heartbeat>(
          "/health/heartbeat", 10,
          std::bind(&HealthMonitorMiddleware::heartbeat_callback, this, std::placeholders::_1));

      system_health_pub_ = this->create_publisher<truckbot_msgs::msg::SystemHealth>("/health/system", 10);

      check_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&HealthMonitorMiddleware::check_callback, this));
    }

  private:
    void heartbeat_callback(const truckbot_msgs::msg::Heartbeat::SharedPtr msg)
    {
      health_monitor_->updateHeartbeat(*msg, std::chrono::steady_clock::now());
    }

    void check_callback()
    {
      truckbot_msgs::msg::SystemHealth msg;
      msg.stamp = this->now();
      msg.modules = health_monitor_->getModules(std::chrono::steady_clock::now());
      msg.state = HealthMonitor::overallState(msg.modules);
      system_health_pub_->publish(msg);
    }

    rclcpp::Subscription<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_;
    rclcpp::Publisher<truckbot_msgs::msg::SystemHealth>::SharedPtr system_health_pub_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    std::shared_ptr<HealthMonitor> health_monitor_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HealthMonitorMiddleware>());
    rclcpp::shutdown();
    return 0;
  }
