#include "CostGrid.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"

class CostGridMiddleware : public rclcpp::Node {

  public:
    CostGridMiddleware()
      : Node("cost_grid_middleware")
    {
      double collisionRadius = this->declare_parameter<double>("collision_radius", 0.23);
      double clearanceRadius = this->declare_parameter<double>("clearance_radius", 0.45);
      int occupiedThreshold = this->declare_parameter<int>("occupied_threshold", 65);
      occupancy_topic_ = this->declare_parameter<std::string>("occupancy_topic", "/map");
      cost_grid_topic_ = this->declare_parameter<std::string>("cost_grid_topic", "/cost_grid");

      cost_grid_ = std::make_shared<CostGrid>(collisionRadius, clearanceRadius,
                                              occupiedThreshold);

      auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

      occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          occupancy_topic_, map_qos,
          std::bind(&CostGridMiddleware::occupancy_callback, this, std::placeholders::_1));

      cost_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(cost_grid_topic_, map_qos);

      RCLCPP_INFO(this->get_logger(), "Initialized Cost Grid");
    }

  private:
    void occupancy_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
      if (msg->info.width == 0 || msg->info.height == 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Received empty occupancy grid; nothing to do.");
        return;
      }

      cost_grid_->setOccupancy(msg->data,
                               msg->info.resolution,
                               static_cast<int>(msg->info.width),
                               static_cast<int>(msg->info.height),
                               msg->info.origin.position.x,
                               msg->info.origin.position.y);

      cost_grid_->computeCosts();

      publish_cost_grid(msg->header);
    }

    void publish_cost_grid(const std_msgs::msg::Header& source_header)
    {
      nav_msgs::msg::OccupancyGrid msg;
      msg.header.stamp = source_header.stamp;
      msg.header.frame_id = source_header.frame_id;
      msg.info.resolution = cost_grid_->getResolution();
      msg.info.width = cost_grid_->getWidth();
      msg.info.height = cost_grid_->getHeight();
      msg.info.origin.position.x = cost_grid_->getOriginX();
      msg.info.origin.position.y = cost_grid_->getOriginY();
      msg.info.origin.position.z = 0.0;
      msg.info.origin.orientation.w = 1.0;
      msg.data = cost_grid_->toOccupancyData();
      cost_grid_pub_->publish(msg);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_grid_pub_;
    std::shared_ptr<CostGrid> cost_grid_;
    std::string occupancy_topic_;
    std::string cost_grid_topic_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostGridMiddleware>());
    rclcpp::shutdown();
    return 0;
  }
