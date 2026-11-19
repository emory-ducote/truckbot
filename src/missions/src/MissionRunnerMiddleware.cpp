#include <chrono>
#include <memory>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "MissionRunner.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "truckbot_msgs/msg/heartbeat.hpp"
#include "truckbot_msgs/msg/mission_status.hpp"
#include "truckbot_msgs/msg/system_health.hpp"

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
      double goal_match_tolerance = this->declare_parameter<double>("goal_match_tolerance", 1e-3);
      std::string missions_file = this->declare_parameter<std::string>("missions_file",
          ament_index_cpp::get_package_share_directory("missions") + "/config/missions.yaml");

      mission_runner_ = std::make_shared<MissionRunner>(goal_match_tolerance);
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
      status_pub_ = this->create_publisher<truckbot_msgs::msg::MissionStatus>("/mission/status", 10);
      goal_cancel_pub_ = this->create_publisher<std_msgs::msg::Empty>("/goal_cancel", 10);
      lift_stop_pub_ = this->create_publisher<std_msgs::msg::Empty>("/lift_stop", 10);

      goal_reached_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
          "/goal_reached", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            mission_runner_->goalReached(msg->point.x, msg->point.y);
          });
      lift_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          "/lift_done", 10, [this](const std_msgs::msg::Bool::SharedPtr) { mission_runner_->liftDone(); });
      system_health_sub_ = this->create_subscription<truckbot_msgs::msg::SystemHealth>(
          "/health/system", 10, std::bind(&MissionRunnerMiddleware::system_health_callback, this, std::placeholders::_1));
      start_srv_ = this->create_service<std_srvs::srv::Trigger>(
          "/mission/start", std::bind(&MissionRunnerMiddleware::start_callback, this, std::placeholders::_1, std::placeholders::_2));

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

    void system_health_callback(const truckbot_msgs::msg::SystemHealth::SharedPtr msg)
    {
      system_good_ = msg->state == truckbot_msgs::msg::Heartbeat::GOOD;
      if (msg->state == truckbot_msgs::msg::Heartbeat::GOOD) {
        mission_runner_->setSystemReady();
      } else if (msg->state == truckbot_msgs::msg::Heartbeat::FAILED && mission_runner_->enterError()) {
        RCLCPP_ERROR(this->get_logger(), "System health FAILED, aborting mission: clearing goal and stopping lift");
        goal_cancel_pub_->publish(std_msgs::msg::Empty());
        lift_stop_pub_->publish(std_msgs::msg::Empty());
      }
    }

    void start_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!system_good_) {
        response->success = false;
        response->message = "system health is not GOOD";
      } else if (mission_runner_->requestStart()) {
        response->success = true;
        response->message = "mission started";
        RCLCPP_INFO(this->get_logger(), "Mission start requested");
        return;
      } else if (mission_runner_->getState() == MissionState::IDLE) {
        response->success = false;
        response->message = "mission already starting";
      } else {
        response->success = false;
        response->message = "mission already started";
      }
      RCLCPP_WARN(this->get_logger(), "Mission start rejected: %s", response->message.c_str());
    }

    void mission_callback()
    {
      // Stay GOOD while waiting, otherwise this node would hold the system out of GOOD forever
      health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
      health_detail_ = "";
      if (mission_runner_->getState() == MissionState::IDLE) {
        health_detail_ = mission_runner_->isSystemReady() ? "waiting for start on /mission/start" : "waiting for system health";
      }
      if (mission_runner_->getState() == MissionState::ERROR) {
        health_state_ = truckbot_msgs::msg::Heartbeat::DEGRADED;
        health_detail_ = "mission aborted on system failure";
      }
      auto state = mission_runner_->step();
      if (state) {
        RCLCPP_INFO(this->get_logger(), "Mission %s: entering state %s",
            mission_runner_->getCurrentMission()->name.c_str(), state->name.c_str());
        if (state->goal) {
          send_goal(state->goal->x, state->goal->y);
        }
        if (state->liftEnabled) {
          send_lift(*state->liftEnabled);
        }
      }
      publish_status();
    }

    void publish_status()
    {
      truckbot_msgs::msg::MissionStatus msg;
      msg.stamp = this->now();
      switch (mission_runner_->getState()) {
        case MissionState::IDLE: msg.state = truckbot_msgs::msg::MissionStatus::IDLE; break;
        case MissionState::RUNNING: msg.state = truckbot_msgs::msg::MissionStatus::RUNNING; break;
        case MissionState::COMPLETE: msg.state = truckbot_msgs::msg::MissionStatus::COMPLETE; break;
        case MissionState::ERROR: msg.state = truckbot_msgs::msg::MissionStatus::ERROR; break;
      }
      if (const auto* mission = mission_runner_->getCurrentMission()) {
        msg.mission_name = mission->name;
      }
      if (const auto* current = mission_runner_->getCurrentState()) {
        msg.state_name = current->name;
      }
      msg.waiting_for_start = mission_runner_->isWaitingForStart();
      msg.waiting_for_goal = mission_runner_->isWaitingForGoal();
      msg.waiting_for_lift = mission_runner_->isWaitingForLift();
      status_pub_->publish(msg);
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
    rclcpp::Publisher<truckbot_msgs::msg::MissionStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr goal_cancel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr lift_stop_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lift_done_sub_;
    rclcpp::Subscription<truckbot_msgs::msg::SystemHealth>::SharedPtr system_health_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Publisher<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    std::shared_ptr<MissionRunner> mission_runner_;
    std::string global_frame_;
    bool system_good_ = false;
    uint8_t health_state_ = truckbot_msgs::msg::Heartbeat::INIT;
    std::string health_detail_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionRunnerMiddleware>());
    rclcpp::shutdown();
    return 0;
}
