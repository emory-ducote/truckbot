
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int64.hpp"
#include "SplineGenerator.h"


class SplineGeneratorMiddleware : public rclcpp::Node {
  public:
    SplineGeneratorMiddleware(std::shared_ptr<SplineGenerator> splineGenerator) : Node("spline_generator_middleware"), splineGenerator(splineGenerator)
    {
      path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/global_path", 10, std::bind(&SplineGeneratorMiddleware::pathCallback, this, std::placeholders::_1));
      global_path_index_sub_ = this->create_subscription<std_msgs::msg::Int64>(
                "/global_path_index", 10, std::bind(&SplineGeneratorMiddleware::pathIndexCallback, this, std::placeholders::_1));
      local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    }

  private:
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
      RCLCPP_INFO(this->get_logger(), "Published local path with %zu spline points.", splinePath.size());
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr global_path_index_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    std::shared_ptr<SplineGenerator> splineGenerator;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto splineGenerator = std::make_shared<SplineGenerator>();
  rclcpp::spin(std::make_shared<SplineGeneratorMiddleware>(splineGenerator));
  rclcpp::shutdown();
  return 0;
}
