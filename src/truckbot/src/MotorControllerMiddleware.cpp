#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <lgpio.h>
#include <iostream>
#include <sensor_msgs/msg/joy.hpp>
#include "truckbot/MotorController.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class JoyListener : public rclcpp::Node {
  public:
      JoyListener(std::shared_ptr<MotorController> motorController) : Node("joy_listener"), motorController(motorController) {
          joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
              "/joy", 10, std::bind(&JoyListener::joyCallback, this, std::placeholders::_1));
      }
  
  private:
      void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Received joystick data");
          float left_stick_x = msg->axes[1];  // Typically left stick horizontal
          float right_stick_x = msg->axes[4];  // Typically left stick vertical
          motorController->setMotorSpeed(left_stick_x, right_stick_x);
          std::cout << left_stick_x << "  " << right_stick_x << std::endl;
      }
  
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      std::shared_ptr<MotorController> motorController;
  };

int main(int argc, char** argv) {


  const uint8_t chip = 4;
  const uint8_t leftOne = 26;
  const uint8_t leftTwo = 19;
  const uint8_t rightOne = 6;
  const uint8_t rightTwo = 13;

  rclcpp::init(argc, argv);
  auto controller = std::make_shared<MotorController>(chip, leftOne, leftTwo, rightOne, rightTwo);
  rclcpp::spin(std::make_shared<JoyListener>(controller));
  rclcpp::shutdown();
  return 0;
}