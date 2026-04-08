
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int64.hpp"
#include "PathTracker.h"


class PathTrackerMiddleware : public rclcpp::Node {
  public:
    PathTrackerMiddleware(std::shared_ptr<PathTracker> pathTracker) : Node("path_tracker_middleware"), pathTracker(pathTracker)
    {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/local_odom", 10, std::bind(&PathTrackerMiddleware::odomCallback, this, std::placeholders::_1));
      path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/global_path", 10, std::bind(&PathTrackerMiddleware::pathCallback, this, std::placeholders::_1));
      global_path_index_pub_ = this->create_publisher<std_msgs::msg::Int64>("/global_path_index", 10);
      timer_ = this->create_wall_timer(
              std::chrono::milliseconds(100),
              std::bind(&PathTrackerMiddleware::timerCallback, this));
    }

  private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      navigation::VehiclePose newPose(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.orientation.z);
      pathTracker->setVehiclePose(newPose);
      
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
      pathTracker->setGlobalPath(newPath);
    }
    void timerCallback()
    {
      int closestIndex = pathTracker->findNearestPose();
      RCLCPP_INFO(this->get_logger(), "Nearest Path Index: %d", closestIndex);
      auto closestIndexMessage = std_msgs::msg::Int64();
      closestIndexMessage.data = closestIndex;  
      global_path_index_pub_->publish(closestIndexMessage);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr global_path_index_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<PathTracker> pathTracker;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pathTracker = std::make_shared<PathTracker>();
  rclcpp::spin(std::make_shared<PathTrackerMiddleware>(pathTracker));
  rclcpp::shutdown();
  return 0;
}
