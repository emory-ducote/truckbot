#include "EKF.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/joy.hpp>


class ImuListener : public rclcpp::Node {
  public:
      ImuListener(std::shared_ptr<EKF> state) : Node("imu_listener"), state(state) 
      {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
              "/imu", 10, std::bind(&ImuListener::imuCallback, this, std::placeholders::_1));
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
              "/joy", 10, std::bind(&ImuListener::joyCallback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            
      }

    private:
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) 
        {
            RCLCPP_INFO(this->get_logger(), "Received imu data");
            Vector3d accel_data(msg->linear_acceleration.x, msg->linear_acceleration.y,0);
            Vector3d gyro_data(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            if ((leftStick == 0.0) && (rightStick == 0.0)) 
            {
              accel_data << 0, 0, 0;
              gyro_data << 0, 0, 0;
              state->velocity << 0, 0, 0;
            }
            
            state->ekf_loop(accel_data, gyro_data);
            auto message = nav_msgs::msg::Odometry();
            message.pose.pose.position.x = state->position.x();
            message.pose.pose.position.y = state->position.y();
            message.pose.pose.position.z = state->position.z();
            message.twist.twist.linear.x = state->velocity.x();
            message.twist.twist.linear.y = state->velocity.y();
            message.twist.twist.linear.z = state->velocity.z();
            message.pose.pose.orientation.x = state->orientation.x();
            message.pose.pose.orientation.y = state->orientation.y();
            message.pose.pose.orientation.z = state->orientation.z();
            message.pose.pose.orientation.w = state->orientation.w();
            message.header.frame_id = "base_link";
            publisher_->publish(message);
            
        }
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
        {
            leftStick = msg->axes[1];  // Typically left stick horizontal
            rightStick = msg->axes[4];  // Typically left stick vertical
        }

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        std::shared_ptr<EKF> state;
        float leftStick = 0;
        float rightStick = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto state = std::make_shared<EKF>();
    rclcpp::spin(std::make_shared<ImuListener>(state));
    rclcpp::shutdown();
    return 0;
  }