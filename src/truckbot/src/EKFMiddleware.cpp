#include "truckbot/EKF.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"


class ImuListener : public rclcpp::Node {
  public:
      ImuListener(std::shared_ptr<EKF> state) : Node("imu_listener"), state(state) 
      {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
              "/imu", 10, std::bind(&ImuListener::imuCallback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

      }

    private:
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) 
        {
            RCLCPP_INFO(this->get_logger(), "Received imu data");
            Vector3d accel_data(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Vector3d gyro_data(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            Vector3d mag_data(msg->orientation.x, msg->orientation.y, msg->orientation.z);
            state->ekf_loop(accel_data, gyro_data, mag_data);
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

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        std::shared_ptr<EKF> state;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto state = std::make_shared<EKF>();
    rclcpp::spin(std::make_shared<ImuListener>(state));
    rclcpp::shutdown();
    return 0;
  }