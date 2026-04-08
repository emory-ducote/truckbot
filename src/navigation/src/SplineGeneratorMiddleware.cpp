
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int64.hpp"
#include "SplineGenerator.h"


class SplineGeneratorMiddleware : public rclcpp::Node {
  public:
    SplineGeneratorMiddleware(std::shared_ptr<SplineGenerator> splineGenerator) : Node("path_tracker_middleware"), splineGenerator(splineGenerator)
    {
      path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/global_path", 10, std::bind(&SplineGeneratorMiddleware::pathCallback, this, std::placeholders::_1));
      global_path_index_sub_ = this->create_subscription<std_msgs::msg::Int64>(
                "/global_path_index", 10, std::bind(&SplineGeneratorMiddleware::pathIndexCallback, this, std::placeholders::_1));
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
      splineGenerator->generateSpline();
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr global_path_index_sub_;
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
