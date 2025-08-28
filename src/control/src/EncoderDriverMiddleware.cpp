#include "EncoderDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


class EncoderDriverMiddleware : public rclcpp::Node {
    public:
      EncoderDriverMiddleware(std::shared_ptr<EncoderDriver> rightFront,
                              std::shared_ptr<EncoderDriver> rightRear,
                              std::shared_ptr<EncoderDriver> leftFront,
                              std::shared_ptr<EncoderDriver> leftRear)
                               : Node("encoder_driver"), 
                              rightFront(rightFront),
                              rightRear(rightRear),
                              leftFront(leftFront),
                              leftRear(leftRear)
      {
        timer_ = this->create_wall_timer(
                100ms, std::bind(&EncoderDriverMiddleware::timer_callback, this));
        odom_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/odom", 10);

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
  const uint8_t chip = 4;
  const uint8_t rightFrontA = 15;
  const uint8_t rightFrontB = 14;
  const uint8_t rightRearA = 1;
  const uint8_t rightRearB = 7;
  const uint8_t leftFrontA = 23;
  const uint8_t leftFrontB = 24;
  const uint8_t leftRearA = 25;
  const uint8_t leftRearB = 8;
  rclcpp::init(argc, argv);
  auto rightFront = std::make_shared<EncoderDriver>(chip, rightFrontA, rightFrontB);
  auto rightRear = std::make_shared<EncoderDriver>(chip, rightRearA, rightRearB);
  auto leftFront = std::make_shared<EncoderDriver>(chip, leftFrontA, leftFrontB);
  auto leftRear = std::make_shared<EncoderDriver>(chip, leftRearA, leftRearB);



  rclcpp::spin(std::make_shared<EncoderDriverMiddleware>(rightFront, rightRear, leftFront, leftRear)); 
  rclcpp::shutdown();
  return 0;
}