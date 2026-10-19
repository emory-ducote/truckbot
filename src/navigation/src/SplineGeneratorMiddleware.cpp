
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "truckbot_msgs/msg/heartbeat.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int64.hpp"
#include "SplineGenerator.h"


class SplineGeneratorMiddleware : public rclcpp::Node {
  public:
    SplineGeneratorMiddleware() : Node("spline_generator")
    {
      heartbeat_pub_ = this->create_publisher<truckbot_msgs::msg::Heartbeat>("/health/heartbeat", 10);
      heartbeat_timer_ = this->create_wall_timer(
          std::chrono::seconds(1), std::bind(&SplineGeneratorMiddleware::heartbeat_callback, this));
      double sample_scale = this->declare_parameter<double>("sample_scale", 4.0);
      splineGenerator = std::make_shared<SplineGenerator>(sample_scale);
      
      path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/global_path", 10, std::bind(&SplineGeneratorMiddleware::pathCallback, this, std::placeholders::_1));
      global_path_index_sub_ = this->create_subscription<std_msgs::msg::Int64>(
                "/global_path_index", 10, std::bind(&SplineGeneratorMiddleware::pathIndexCallback, this, std::placeholders::_1));
      local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
      // No startup inputs to wait on; idle until a path arrives is normal operation
      health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
      health_detail_ = "";
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

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
      std::vector<navigation::VehiclePose> newPath;
      for (const auto& pose: msg->poses)
      {
        navigation::VehiclePose newPose(pose.pose.position.x, 
                                        pose.pose.position.y,
                                        pose.pose.orientation.z);
        newPath.push_back(newPose);
      }
      splineGenerator->setGlobalPath(newPath);
    }
    void pathIndexCallback(const std_msgs::msg::Int64::SharedPtr msg)
    {
      splineGenerator->setNearestPoseIndex(msg->data);
      auto splinePath = splineGenerator->generateSpline();
      if (splinePath.empty()) {
        RCLCPP_WARN(this->get_logger(), "Generated spline is empty. No local path published.");
        return;
      }

      nav_msgs::msg::Path localPathMsg;
      localPathMsg.header.stamp = this->now();
      localPathMsg.header.frame_id = "map";
      for (const auto& pose : splinePath)
      {
        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.header = localPathMsg.header;
        poseStamped.pose.position.x = pose.x;
        poseStamped.pose.position.y = pose.y;
        poseStamped.pose.position.z = 0.0;
        poseStamped.pose.orientation.x = 0.0;
        poseStamped.pose.orientation.y = 0.0;
        poseStamped.pose.orientation.z = 0.0;
        poseStamped.pose.orientation.w = 1.0;
        localPathMsg.poses.push_back(poseStamped);
      }
      local_path_pub_->publish(localPathMsg);
      health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
      health_detail_ = "";
      RCLCPP_INFO(this->get_logger(), "Published local path with %zu spline points.", splinePath.size());
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr global_path_index_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Publisher<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
    uint8_t health_state_ = truckbot_msgs::msg::Heartbeat::INIT;
    std::string health_detail_;
    std::shared_ptr<SplineGenerator> splineGenerator;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SplineGeneratorMiddleware>());
  rclcpp::shutdown();
  return 0;
}
