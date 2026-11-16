#include <chrono>
#include <cmath>
#include "AStarPlanner.h"
#include "TfPoseLookup.h"
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
      heartbeat_callback();  // publish INIT immediately so the monitor knows this node exists
      cost_grid_topic_ = this->declare_parameter<std::string>("cost_grid_topic", "/cost_grid");
      goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
      path_topic_ = this->declare_parameter<std::string>("path_topic", "/global_path");
      int costThresh = this->declare_parameter<int>("cost_thresh", 65);
      int replan_period_ms = this->declare_parameter<int>("replan_period_ms", 500);

      goal_snap_radius_ = this->declare_parameter<double>("goal_snap_radius", 1.0);
      double cost_weight = this->declare_parameter<double>("cost_weight", 3.0);

      a_star_planner_ = std::make_shared<AStarPlanner>(static_cast<int8_t>(costThresh));
      a_star_planner_->setCostWeight(cost_weight);

      auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

      cost_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          cost_grid_topic_, map_qos,
          std::bind(&AStarPlannerMiddleware::cost_grid_callback, this, std::placeholders::_1));

      goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_topic_, 10,
          std::bind(&AStarPlannerMiddleware::goal_callback, this, std::placeholders::_1));

      navigation::TfPoseLookupConfig pose_config;
      pose_config.globalFrame = this->declare_parameter<std::string>("global_frame", pose_config.globalFrame);
      pose_config.odomFrame = this->declare_parameter<std::string>("odom_frame", pose_config.odomFrame);
      pose_config.baseFrame = this->declare_parameter<std::string>("base_frame", pose_config.baseFrame);
      pose_config.maxCorrectionAge = this->declare_parameter<double>("max_correction_age", pose_config.maxCorrectionAge);
      pose_lookup_ = std::make_unique<navigation::TfPoseLookup>(*this, pose_config);

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

    void replan_timer_callback()
    {
      auto current_pose = pose_lookup_->lookup();
      if (!current_pose) {
        // Planning from a stale pose would hand the followers a path that starts in the wrong place
        if (pose_received_) {
          health_state_ = truckbot_msgs::msg::Heartbeat::DEGRADED;
          health_detail_ = pose_lookup_->chainName() + " tf unavailable";
        }
        return;
      }
      pose_received_ = true;
      if (!cost_grid_received_) {
        return;
      }
      // Ready and waiting on a goal is normal operation, not INIT
      if (!goal_received_) {
        health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
        health_detail_ = "";
        return;
      }

      std::pair<int, int> start = world_to_cell(current_pose->x, current_pose->y);
      std::pair<int, int> goal = world_to_cell(goal_pose_.x, goal_pose_.y);

      // A goal inside an inflated obstacle zone is unreachable safely; move it to the nearest free cell
      int snap_radius_cells = static_cast<int>(std::ceil(goal_snap_radius_ / grid_resolution_));
      auto free_goal = a_star_planner_->nearestFreeCell(goal, snap_radius_cells);
      if (free_goal.first < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Goal is in an obstacle zone with no free cell within %.2f m.", goal_snap_radius_);
        health_state_ = truckbot_msgs::msg::Heartbeat::DEGRADED;
        health_detail_ = "goal blocked";
        return;
      }
      if (free_goal != goal) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Goal is in an obstacle zone; snapped by %d cells.",
                             static_cast<int>(std::lround(std::hypot(free_goal.first - goal.first,
                                                                     free_goal.second - goal.second))));
        goal = free_goal;
      }

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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    std::shared_ptr<AStarPlanner> a_star_planner_;
    std::unique_ptr<navigation::TfPoseLookup> pose_lookup_;
    std::string cost_grid_topic_;
    std::string goal_topic_;
    std::string path_topic_;
    std::string grid_frame_;
    double goal_snap_radius_ = 1.0;
    double grid_resolution_ = 1.0;
    double grid_origin_x_ = 0.0;
    double grid_origin_y_ = 0.0;
    navigation::VehiclePose goal_pose_;
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
