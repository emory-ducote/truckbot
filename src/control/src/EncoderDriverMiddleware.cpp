#include "EncoderDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


class EncoderDriverMiddleware : public rclcpp::Node {
    public:
      EncoderDriverMiddleware()
        : Node("encoder_driver_node")
      {
        // Declare and get parameters from YAML
        double wheelRadius = this->declare_parameter<double>("wheel_radius", 0.03);
        int encoderCPR = this->declare_parameter<int>("encoder_cpr", 700);
        int encoderMultiplier = this->declare_parameter<int>("encoder_multiplier", 4);
        int chip = this->declare_parameter<int>("chip", 4);
        int rightFrontA = this->declare_parameter<int>("rightFrontA", 15);
        int rightFrontB = this->declare_parameter<int>("rightFrontB", 14);
        int rightRearA = this->declare_parameter<int>("rightRearA", 1);
        int rightRearB = this->declare_parameter<int>("rightRearB", 7);
        int leftFrontA = this->declare_parameter<int>("leftFrontA", 23);
        int leftFrontB = this->declare_parameter<int>("leftFrontB", 24);
        int leftRearA = this->declare_parameter<int>("leftRearA", 25);
        int leftRearB = this->declare_parameter<int>("leftRearB", 8);

        std::string encoder_topic = this->declare_parameter<std::string>("encoder_odometry_topic", "/odom");
        int update_rate = this->declare_parameter<int>("update_rate", 10);

        // Create EncoderDriver objects
        rightFront = std::make_shared<EncoderDriver>(chip, rightFrontA, rightFrontB, wheelRadius, encoderCPR, encoderMultiplier);
        rightRear = std::make_shared<EncoderDriver>(chip, rightRearA, rightRearB, wheelRadius, encoderCPR, encoderMultiplier);
        leftFront = std::make_shared<EncoderDriver>(chip, leftFrontA, leftFrontB, wheelRadius, encoderCPR, encoderMultiplier);
        leftRear = std::make_shared<EncoderDriver>(chip, leftRearA, leftRearB, wheelRadius, encoderCPR, encoderMultiplier);

        // Use update_rate for timer period
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / update_rate),
            std::bind(&EncoderDriverMiddleware::timer_callback, this));
        // Use encoder_topic for publisher topic
        odom_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(encoder_topic, 10);
        // You can use frame_id later when publishing messages if needed
      }
    private:
        void timer_callback()
        {
          auto rightFrontWheelSpeed = rightFront->getWheelSpeeds(0.10);
          auto rightRearWheelSpeed = rightRear->getWheelSpeeds(0.10);
          auto leftFrontWheelSpeed = leftFront->getWheelSpeeds(0.10);
          auto leftRearWheelSpeed = leftRear->getWheelSpeeds(0.10);

          auto rightWheelSpeed = (rightFrontWheelSpeed + rightRearWheelSpeed) / 2;
          auto leftWheelSpeed = (leftFrontWheelSpeed + leftRearWheelSpeed) / 2;

          RCLCPP_INFO(this->get_logger(), "LEFT WHEEL: %f, RIGHT WHEEL %f", leftWheelSpeed, rightWheelSpeed);
          
          auto message = geometry_msgs::msg::Twist();
          message.linear.x = (rightWheelSpeed + leftWheelSpeed) / 2;
          message.angular.z = (rightWheelSpeed - leftWheelSpeed) / 0.2;
          odom_publisher_->publish(message);
        }

        std::shared_ptr<EncoderDriver> rightFront;
        std::shared_ptr<EncoderDriver> rightRear;
        std::shared_ptr<EncoderDriver> leftFront;
        std::shared_ptr<EncoderDriver> leftRear;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr odom_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderDriverMiddleware>());
  rclcpp::shutdown();
  return 0;
}