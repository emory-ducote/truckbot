#include <chrono>
#include <memory>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "MissionRunner.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "truckbot_msgs/msg/heartbeat.hpp"

class MissionRunnerMiddleware : public rclcpp::Node {

  public:
    MissionRunnerMiddleware()
      : Node("mission_runner")
    {
      heartbeat_pub_ = this->create_publisher<truckbot_msgs::msg::Heartbeat>("/health/heartbeat", 10);
      heartbeat_timer_ = this->create_wall_timer(
          std::chrono::seconds(1), std::bind(&MissionRunnerMiddleware::heartbeat_callback, this));
      heartbeat_callback();

      global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
      int timer_period_ms = this->declare_parameter<int>("timer_period_ms", 100);
      std::string missions_file = this->declare_parameter<std::string>("missions_file",
          ament_index_cpp::get_package_share_directory("missions") + "/config/missions.yaml");

      mission_runner_ = std::make_shared<MissionRunner>();
      try {
        mission_runner_->loadMissions(missions_file);
      } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load missions from %s: %s", missions_file.c_str(), e.what());
        health_state_ = truckbot_msgs::msg::Heartbeat::FAILED;
        health_detail_ = e.what();
        return;
      }

      goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
      actuator_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_actuator", 10);

      goal_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          "/goal_reached", 10, [this](const std_msgs::msg::Bool::SharedPtr) { mission_runner_->goalReached(); });
      lift_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          "/lift_done", 10, [this](const std_msgs::msg::Bool::SharedPtr) { mission_runner_->liftDone(); });

      mission_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(timer_period_ms),
          std::bind(&MissionRunnerMiddleware::mission_callback, this));
    }

  private:
    void heartbeat_callback()
    {
      truckbot_msgs::msg::Heartbeat heartbeat;
      heartbeat.stamp = this->now();
      heartbeat.node_name = this->get_name();
      heartbeat.state = health_state_;
      heartbeat.detail = health_detail_;
      heartbeat_pub_->publish(heartbeat);
    }

    void mission_callback()
    {
      health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
      auto state = mission_runner_->step();
      if (!state) {
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Mission %s: entering state %s",
          mission_runner_->getCurrentMission()->name.c_str(), state->name.c_str());
      if (state->goal) {
        send_goal(state->goal->x, state->goal->y);
      }
      if (state->liftEnabled) {
        send_lift(*state->liftEnabled);
      }
    }

    void send_goal(double x, double y)
    {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.stamp = this->now();
      goal.header.frame_id = global_frame_;
      goal.pose.position.x = x;
      goal.pose.position.y = y;
      goal.pose.orientation.w = 1.0;
      goal_pub_->publish(goal);
    }

    void send_lift(bool enabled)
    {
      std_msgs::msg::Bool msg;
      msg.data = enabled;
      actuator_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr actuator_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lift_done_sub_;
    rclcpp::Publisher<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    std::shared_ptr<MissionRunner> mission_runner_;
    std::string global_frame_;
    uint8_t health_state_ = truckbot_msgs::msg::Heartbeat::INIT;
    std::string health_detail_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionRunnerMiddleware>());
    rclcpp::shutdown();
    return 0;
}
