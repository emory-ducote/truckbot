
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "PathTracker.h"


class PathTrackerMiddleware : public rclcpp::Node {
  public:
    PathTrackerMiddleware(std::shared_ptr<PathTracker> pathTracker) : Node("path_tracker_middleware"), pathTracker(pathTracker)
    {
      odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/heaviest_particle_pose", 10, std::bind(&PathTrackerMiddleware::odomCallback, this, std::placeholders::_1));
      path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/global_path", 10, std::bind(&PathTrackerMiddleware::pathCallback, this, std::placeholders::_1));
      global_path_index_pub_ = this->create_publisher<std_msgs::msg::Int64>("/global_path_index", 10);
      nearest_path_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/nearest_path_point", 10);
      timer_ = this->create_wall_timer(
              std::chrono::milliseconds(100),
              std::bind(&PathTrackerMiddleware::timerCallback, this));
    }

  private:
    void odomCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      auto& q = msg->pose.orientation;
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      navigation::VehiclePose newPose(msg->pose.position.x,
                                      msg->pose.position.y,
                                      yaw);
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

      // Publish tracked coordinate
      const auto& globalPath = pathTracker->getGlobalPath();
      if (closestIndex >= 0 && closestIndex < static_cast<int>(globalPath.size())) {
        geometry_msgs::msg::PointStamped trackedCoordinate;
        trackedCoordinate.header.stamp = this->get_clock()->now();
        trackedCoordinate.header.frame_id = "map";
        trackedCoordinate.point.x = globalPath[closestIndex].x;
        trackedCoordinate.point.y = globalPath[closestIndex].y;
        trackedCoordinate.point.z = 0.0;
        nearest_path_point_pub_->publish(trackedCoordinate);
      }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr global_path_index_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_path_point_pub_;
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
