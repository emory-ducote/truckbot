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
        int rightFrontA = this->declare_parameter<int>("right_front_a", 24);
        int rightFrontB = this->declare_parameter<int>("right_front_b", 25);
        int rightRearA = this->declare_parameter<int>("right_rear_a", 8);
        int rightRearB = this->declare_parameter<int>("right_rear_b", 7);
        int leftFrontA = this->declare_parameter<int>("left_front_a", 1);
        int leftFrontB = this->declare_parameter<int>("left_front_b", 16);
        int leftRearA = this->declare_parameter<int>("left_rear_a", 20);
        int leftRearB = this->declare_parameter<int>("left_rear_b", 21);

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

          RCLCPP_INFO(this->get_logger(), "LEFT FRONT: %f, LEFT REAR  %f", leftFrontWheelSpeed, leftRearWheelSpeed);
          RCLCPP_INFO(this->get_logger(), "RIGHT FRONT: %f, RIGHT REAR %f", rightFrontWheelSpeed, rightRearWheelSpeed);


          // auto rightWheelSpeed = (rightFrontWheelSpeed + rightRearWheelSpeed) / 2;
          // auto leftWheelSpeed = (leftFrontWheelSpeed + leftRearWheelSpeed) / 2;
          auto rightWheelSpeed = rightFrontWheelSpeed;
          auto leftWheelSpeed = -rightRearWheelSpeed;

          // RCLCPP_INFO(this->get_logger(), "LEFT WHEEL: %f, RIGHT WHEEL %f", leftWheelSpeed, rightWheelSpeed);
          
          auto message = geometry_msgs::msg::Twist();

          //TODO:  this is wrong but seems to makes things work  
          message.linear.x = (rightWheelSpeed + leftWheelSpeed) / 2; // should be / 2 - hm
          
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
