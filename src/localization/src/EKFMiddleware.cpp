#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float32.hpp"
#include "EKF.h"

class EKFMiddleware : public rclcpp::Node {
  public:
      EKFMiddleware(std::shared_ptr<EKF> state) : Node("imu_listener"), state(state) 
      {
            odom_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
              "/odom", 10, std::bind(&EKFMiddleware::odomCallback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/local_odom", 10);
      }

    private:
        void odomCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          double v_t = msg->linear.x;
          double w_t = msg->angular.z;
          state->ekf_loop(state->x, v_t, w_t);
          auto message = nav_msgs::msg::Odometry();
          message.pose.pose.position.x = state->x[0];
          message.pose.pose.position.y = state->x[1];
          double yaw = state->x[2];
          message.pose.pose.orientation.x = 0.0;
          message.pose.pose.orientation.y = 0.0;
          message.pose.pose.orientation.z = sin(yaw * 0.5);
          message.pose.pose.orientation.w = cos(yaw * 0.5);
          message.header.frame_id = "map";
          publisher_->publish(message);
          RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, yaw: %f", state->x[0], state->x[1], yaw * 180.0 / M_PI);
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        std::shared_ptr<EKF> state;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto state = std::make_shared<EKF>();
    rclcpp::spin(std::make_shared<EKFMiddleware>(state));
    rclcpp::shutdown();
    return 0;
  }