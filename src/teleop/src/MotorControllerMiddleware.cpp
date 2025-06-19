#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <lgpio.h>
#include <iostream>
#include <sensor_msgs/msg/joy.hpp>
#include "MotorController.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class JoyListener : public rclcpp::Node {
  public:
      JoyListener(std::shared_ptr<MotorController> motorController) : Node("joy_listener"), motorController(motorController) 
      {
          joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
              "/joy", 10, std::bind(&JoyListener::joyCallback, this, std::placeholders::_1));
      }
  
  private:
      void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
      {
          RCLCPP_INFO(this->get_logger(), "Received joystick data");
          float left_stick_x = msg->axes[1];  // Typically left stick horizontal
          float right_stick_x = msg->axes[4];  // Typically left stick vertical
          int y_button = msg->buttons[0];
          int a_button = msg->buttons[1];
          motorController->setMotorSpeed(left_stick_x, right_stick_x);
          bool up = (y_button > 0) ? true : (a_button > 0) ? false : up;
          motorController->moveActuator(up);
          std::cout << left_stick_x << "  " << right_stick_x << std::endl;
          std::cout << "Y: " << y_button << " A:" << a_button << std::endl;
      }
  
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      std::shared_ptr<MotorController> motorController;
  };


int main(int argc, char** argv) {
  const uint8_t chip = 4;
  const uint8_t leftOne = 23;
  const uint8_t leftTwo = 24;
  const uint8_t rightOne = 17;
  const uint8_t rightTwo = 27;
  const uint8_t liftOne = 6;
  const uint8_t liftTwo = 13;
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<MotorController>(chip, leftOne, leftTwo, rightOne, rightTwo, liftOne, liftTwo);
  rclcpp::spin(std::make_shared<JoyListener>(controller));
  rclcpp::shutdown();
  return 0;
}