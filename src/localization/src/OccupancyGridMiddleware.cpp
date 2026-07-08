#include "OccupancyGrid.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

class OccupancyGridMiddleware : public rclcpp::Node {

  public:
    OccupancyGridMiddleware()
      : Node("occupancy_grid_middleware")
    {
      double resolution = this->declare_parameter<double>("resolution", 0.05);
      int width = this->declare_parameter<int>("width", 200);
      int height = this->declare_parameter<int>("height", 200);
      double originX = this->declare_parameter<double>("origin_x", -5.0);
      double originY = this->declare_parameter<double>("origin_y", -5.0);
      double logOddsOccupied = this->declare_parameter<double>("log_odds_occupied", 0.85);
      double logOddsFree = this->declare_parameter<double>("log_odds_free", -0.4);
      double logOddsMin = this->declare_parameter<double>("log_odds_min", -2.0);
      double logOddsMax = this->declare_parameter<double>("log_odds_max", 3.5);
      double maxRange = this->declare_parameter<double>("max_range", 9.0);
      double publishRate = this->declare_parameter<double>("publish_rate", 1.0);
      map_frame_ = this->declare_parameter<std::string>("map_frame", "map");

      grid_ = std::make_shared<OccupancyGrid>(resolution, width, height,
                                              originX, originY,
                                              logOddsOccupied, logOddsFree,
                                              logOddsMin, logOddsMax, maxRange);

      scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 10,
          std::bind(&OccupancyGridMiddleware::scan_callback, this, std::placeholders::_1));

      map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      publish_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / publishRate),
          std::bind(&OccupancyGridMiddleware::publish_map, this));

      RCLCPP_INFO(this->get_logger(), "Initialized Occupancy Grid");
    }

  private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      // Look up the sensor pose in the map frame at the scan time, then fold
      // the scan into the grid.
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id,
                                         msg->header.stamp,
                                         rclcpp::Duration::from_seconds(0.05));
      } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "scan dropped, %s->%s unavailable: %s",
                             map_frame_.c_str(), msg->header.frame_id.c_str(), e.what());
        return;
      }

      Vector3d sensor_pose;
      sensor_pose << tf.transform.translation.x,
                     tf.transform.translation.y,
                     tf2::getYaw(tf.transform.rotation);

      grid_->integrateScan(sensor_pose, msg->ranges,
                           msg->angle_min, msg->angle_increment);
    }

    void publish_map()
    {
      nav_msgs::msg::OccupancyGrid msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = map_frame_;
      msg.info.resolution = grid_->getResolution();
      msg.info.width = grid_->getWidth();
      msg.info.height = grid_->getHeight();
      msg.info.origin.position.x = grid_->getOriginX();
      msg.info.origin.position.y = grid_->getOriginY();
      msg.info.origin.position.z = 0.0;
      msg.info.origin.orientation.w = 1.0;
      msg.data = grid_->toOccupancyData();
      map_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<OccupancyGrid> grid_;
    std::string map_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridMiddleware>());
    rclcpp::shutdown();
    return 0;
  }
