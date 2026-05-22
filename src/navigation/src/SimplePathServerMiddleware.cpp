#include <chrono>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

class SimplePathServerMiddleware : public rclcpp::Node {
public:
  SimplePathServerMiddleware()
  : Node("simple_path_server_middleware")
  {
    path_frame_ = this->declare_parameter<std::string>("path_frame", "map");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/global_path");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 5.0);
    path_x_ = this->declare_parameter<std::vector<double>>("path_x", {});
    path_y_ = this->declare_parameter<std::vector<double>>("path_y", {});

    if (path_x_.size() != path_y_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Path parameter arrays must have equal length. "
                 "path_x (%zu), path_y (%zu)",
                 path_x_.size(), path_y_.size());
    }

    local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    preparePathMessage();

    auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate_, 1e-3));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SimplePathServerMiddleware::timerCallback, this)
    );
  }

private:
  void preparePathMessage()
  {
    local_path_msg_.header.frame_id = path_frame_;
    local_path_msg_.poses.clear();

    if (path_x_.empty() || path_y_.empty() || path_theta_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path points configured. Publish loop will still run but no path will be published.");
      return;
    }

    size_t point_count = std::min(path_x_.size(), path_y_.size());
    local_path_msg_.poses.reserve(point_count);

    for (size_t i = 0; i < point_count; ++i) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = path_frame_;
      pose_stamped.pose.position.x = path_x_[i];
      pose_stamped.pose.position.y = path_y_[i];
      pose_stamped.pose.position.z = 0.0;
      pose_stamped.pose.orientation.x = 0.0;
      pose_stamped.pose.orientation.y = 0.0;
      pose_stamped.pose.orientation.z = 0.0;
      pose_stamped.pose.orientation.w = 1.0;
      local_path_msg_.poses.push_back(pose_stamped);
    }
  }

  void timerCallback()
  {
    if (local_path_msg_.poses.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Empty high-level path; no path published.");
      return;
    }

    local_path_msg_.header.stamp = this->now();
    for (auto & pose : local_path_msg_.poses) {
      pose.header = local_path_msg_.header;
    }
    local_path_pub_->publish(local_path_msg_);
    RCLCPP_DEBUG(this->get_logger(), "Published global path with %zu points.", local_path_msg_.poses.size());
  }

std::string path_frame_;
  std::string path_topic_;
  double publish_rate_;
  std::vector<double> path_x_;
  std::vector<double> path_y_;

  nav_msgs::msg::Path local_path_msg_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePathServerMiddleware>());
  rclcpp::shutdown();
  return 0;
}
