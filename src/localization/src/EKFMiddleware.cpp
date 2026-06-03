#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
            prevTime = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
          }

    private:
        void odomCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          double v_t = msg->linear.x;
          double w_t = msg->angular.z;
          double curTime = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
          double dt = curTime - prevTime;
          prevTime = curTime;
          state->ekf_loop(state->x, v_t, w_t, dt);
          auto message = nav_msgs::msg::Odometry();
          message.header.stamp = this->now();
          message.header.frame_id = "map";
          message.pose.pose.position.x = state->x[0];
          message.pose.pose.position.y = state->x[1];
          double yaw = state->x[2];
          message.pose.pose.orientation.x = 0.0;
          message.pose.pose.orientation.y = 0.0;
          message.pose.pose.orientation.z = sin(yaw * 0.5);
          message.pose.pose.orientation.w = cos(yaw * 0.5);

          // Populate pose covariance from the EKF state covariance P (x, y, yaw).
          message.pose.covariance.fill(0.0);
          if (state->P.rows() == 3 && state->P.cols() == 3) {
            message.pose.covariance[0] = state->P(0, 0);
            message.pose.covariance[1] = state->P(0, 1);
            message.pose.covariance[5] = state->P(0, 2);
            message.pose.covariance[6] = state->P(1, 0);
            message.pose.covariance[7] = state->P(1, 1);
            message.pose.covariance[11] = state->P(1, 2);
            message.pose.covariance[30] = state->P(2, 0);
            message.pose.covariance[31] = state->P(2, 1);
            message.pose.covariance[35] = state->P(2, 2);
          }

          publisher_->publish(message);
          RCLCPP_DEBUG(this->get_logger(), "X: %f, Y: %f, yaw: %f", state->x[0], state->x[1], yaw * 180.0 / M_PI);
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        std::shared_ptr<EKF> state;
        double prevTime;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto state = std::make_shared<EKF>();
    rclcpp::spin(std::make_shared<EKFMiddleware>(state));
    rclcpp::shutdown();
    return 0;
  }
