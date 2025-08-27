#include "EncoderDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

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
                10ms, std::bind(&EncoderDriverMiddleware::timer_callback, this));
      }
    private:
        void timer_callback()
        {
          double rightFrontRotations = rightFront->getRotations();
          double rightRearRotations = rightRear->getRotations();
          double leftFrontRotations = leftFront->getRotations();
          double leftRearRotations = leftRear->getRotations();
          RCLCPP_INFO(this->get_logger(), "READINGS");
          RCLCPP_INFO(this->get_logger(), "right front: %f", rightFrontRotations);
          RCLCPP_INFO(this->get_logger(), "right rear: %f", rightRearRotations);
          RCLCPP_INFO(this->get_logger(), "left front: %f", leftFrontRotations);
          RCLCPP_INFO(this->get_logger(), "left rear: %f", leftRearRotations);
        }

        std::shared_ptr<EncoderDriver> rightFront;
        std::shared_ptr<EncoderDriver> rightRear;
        std::shared_ptr<EncoderDriver> leftFront;
        std::shared_ptr<EncoderDriver> leftRear;

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