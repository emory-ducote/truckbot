#include <chrono>
#include <cmath>
#include "AStarPlanner.h"
#include "VehiclePose.h"
#include "rclcpp/rclcpp.hpp"
#include "truckbot_msgs/msg/heartbeat.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class AStarPlannerMiddleware : public rclcpp::Node {

  public:
    AStarPlannerMiddleware()
      : Node("a_star_planner")
    {
      heartbeat_pub_ = this->create_publisher<truckbot_msgs::msg::Heartbeat>("/health/heartbeat", 10);
      heartbeat_timer_ = this->create_wall_timer(
          std::chrono::seconds(1), std::bind(&AStarPlannerMiddleware::heartbeat_callback, this));
      cost_grid_topic_ = this->declare_parameter<std::string>("cost_grid_topic", "/cost_grid");
      goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
      pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/heaviest_particle_pose");
      path_topic_ = this->declare_parameter<std::string>("path_topic", "/global_path");
      int costThresh = this->declare_parameter<int>("cost_thresh", 65);
      int replan_period_ms = this->declare_parameter<int>("replan_period_ms", 500);

      a_star_planner_ = std::make_shared<AStarPlanner>(static_cast<int8_t>(costThresh));

      auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

      cost_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          cost_grid_topic_, map_qos,
          std::bind(&AStarPlannerMiddleware::cost_grid_callback, this, std::placeholders::_1));

      goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_topic_, 10,
          std::bind(&AStarPlannerMiddleware::goal_callback, this, std::placeholders::_1));

      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          pose_topic_, 10,
          std::bind(&AStarPlannerMiddleware::pose_callback, this, std::placeholders::_1));

      path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

      replan_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(replan_period_ms),
          std::bind(&AStarPlannerMiddleware::replan_timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "Initialized A* Planner");
    }

  private:
    void heartbeat_callback()
    {
      auto heartbeat = truckbot_msgs::msg::Heartbeat();
      heartbeat.stamp = this->now();
      heartbeat.node_name = this->get_name();
      heartbeat.state = health_state_;
      heartbeat.detail = health_detail_;
      heartbeat_pub_->publish(heartbeat);
    }

    void cost_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
      if (msg->info.width == 0 || msg->info.height == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Received empty cost grid; nothing to do.");
        return;
      }

      a_star_planner_->setCosts(msg->data,
                                msg->info.resolution,
                                static_cast<int>(msg->info.width),
                                static_cast<int>(msg->info.height),
                                msg->info.origin.position.x,
                                msg->info.origin.position.y);
      grid_resolution_ = msg->info.resolution;
      grid_origin_x_ = msg->info.origin.position.x;
      grid_origin_y_ = msg->info.origin.position.y;
      grid_frame_ = msg->header.frame_id;
      cost_grid_received_ = true;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      auto& q = msg->pose.orientation;
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      goal_pose_ = navigation::VehiclePose(msg->pose.position.x,
                                           msg->pose.position.y,
                                           yaw);
      goal_received_ = true;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      auto& q = msg->pose.orientation;
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      current_pose_ = navigation::VehiclePose(msg->pose.position.x,
                                              msg->pose.position.y,
                                              yaw);
      pose_received_ = true;
    }

    void replan_timer_callback()
    {
      if (!pose_received_ || !cost_grid_received_) {
        return;
      }
      // Ready and waiting on a goal is normal operation, not INIT
      if (!goal_received_) {
        health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
        health_detail_ = "";
        return;
      }

      std::pair<int, int> start = world_to_cell(current_pose_.x, current_pose_.y);
      std::pair<int, int> goal = world_to_cell(goal_pose_.x, goal_pose_.y);

      auto path = a_star_planner_->findPath(start, goal);
      if (path.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "A* found no path.");
        health_state_ = truckbot_msgs::msg::Heartbeat::DEGRADED;
        health_detail_ = "no path to goal";
        return;
      }

      publish_path(path);
      health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
      health_detail_ = "";
    }

    std::pair<int, int> world_to_cell(double x, double y) const
    {
      int col = static_cast<int>(std::floor((x - grid_origin_x_) / grid_resolution_));
      int row = static_cast<int>(std::floor((y - grid_origin_y_) / grid_resolution_));
      return {col, row};
    }

    void publish_path(const std::vector<std::pair<int, int>>& path)
    {
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = this->now();
      path_msg.header.frame_id = grid_frame_;
      path_msg.poses.reserve(path.size());

      for (const auto& [col, row] : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = grid_origin_x_ + (col + 0.5) * grid_resolution_;
        pose.pose.position.y = grid_origin_y_ + (row + 0.5) * grid_resolution_;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
      }

      path_pub_->publish(path_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    std::shared_ptr<AStarPlanner> a_star_planner_;
    std::string cost_grid_topic_;
    std::string goal_topic_;
    std::string pose_topic_;
    std::string path_topic_;
    std::string grid_frame_;
    double grid_resolution_ = 1.0;
    double grid_origin_x_ = 0.0;
    double grid_origin_y_ = 0.0;
    navigation::VehiclePose goal_pose_;
    navigation::VehiclePose current_pose_;
    bool goal_received_ = false;
    bool pose_received_ = false;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Publisher<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
    uint8_t health_state_ = truckbot_msgs::msg::Heartbeat::INIT;
    std::string health_detail_;
    bool cost_grid_received_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlannerMiddleware>());
    rclcpp::shutdown();
    return 0;
  }
