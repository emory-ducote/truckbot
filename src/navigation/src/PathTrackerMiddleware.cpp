
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "truckbot_msgs/msg/heartbeat.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "PathTracker.h"
#include "TfPoseLookup.h"


class PathTrackerMiddleware : public rclcpp::Node {
  public:
    PathTrackerMiddleware(std::shared_ptr<PathTracker> pathTracker) : Node("path_tracker"), pathTracker(pathTracker)
    {
      heartbeat_pub_ = this->create_publisher<truckbot_msgs::msg::Heartbeat>("/health/heartbeat", 10);
      heartbeat_timer_ = this->create_wall_timer(
          std::chrono::seconds(1), std::bind(&PathTrackerMiddleware::heartbeat_callback, this));
      heartbeat_callback();  // publish INIT immediately so the monitor knows this node exists
      // Same TF-composed pose as the pure pursuit controller, so both agree on where the vehicle is
      navigation::TfPoseLookupConfig pose_config;
      pose_config.globalFrame = this->declare_parameter<std::string>("global_frame", pose_config.globalFrame);
      pose_config.odomFrame = this->declare_parameter<std::string>("odom_frame", pose_config.odomFrame);
      pose_config.baseFrame = this->declare_parameter<std::string>("base_frame", pose_config.baseFrame);
      pose_config.maxCorrectionAge = this->declare_parameter<double>("max_correction_age", pose_config.maxCorrectionAge);
      pose_lookup_ = std::make_unique<navigation::TfPoseLookup>(*this, pose_config);
      path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/global_path", 10, std::bind(&PathTrackerMiddleware::pathCallback, this, std::placeholders::_1));
      global_path_index_pub_ = this->create_publisher<std_msgs::msg::Int64>("/global_path_index", 10);
      nearest_path_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/nearest_path_point", 10);
      timer_ = this->create_wall_timer(
              std::chrono::milliseconds(100),
              std::bind(&PathTrackerMiddleware::timerCallback, this));
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

    bool updatePoseFromTf()
    {
      auto pose = pose_lookup_->lookup();
      if (!pose) {
        return false;
      }
      pathTracker->setVehiclePose(*pose);
      pose_received_ = true;
      return true;
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
      if (!updatePoseFromTf() && pose_received_) {
        health_state_ = truckbot_msgs::msg::Heartbeat::FAILED;
        health_detail_ = pose_lookup_->chainName() + " tf unavailable";
        return;
      }
      int closestIndex = pathTracker->findNearestPose();
      if (pose_received_) {
        // A path with no nearest index means the pose or path contains NaNs
        if (!pathTracker->getGlobalPath().empty() && closestIndex < 0) {
          health_state_ = truckbot_msgs::msg::Heartbeat::FAILED;
          health_detail_ = "no nearest path index found";
        } else {
          health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
          health_detail_ = "";
        }
      }
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

    std::unique_ptr<navigation::TfPoseLookup> pose_lookup_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr global_path_index_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_path_point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Publisher<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
    uint8_t health_state_ = truckbot_msgs::msg::Heartbeat::INIT;
    std::string health_detail_;
    std::shared_ptr<PathTracker> pathTracker;
    bool pose_received_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pathTracker = std::make_shared<PathTracker>();
  rclcpp::spin(std::make_shared<PathTrackerMiddleware>(pathTracker));
  rclcpp::shutdown();
  return 0;
}
