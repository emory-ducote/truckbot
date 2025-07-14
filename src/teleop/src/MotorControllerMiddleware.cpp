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
  const uint8_t rightFrontOne = 13;
  const uint8_t rightFrontTwo = 6;
  const uint8_t rightRearOne = 22;
  const uint8_t rightRearTwo = 10;
  const uint8_t leftFrontOne = 26;
  const uint8_t leftFrontTwo = 19;
  const uint8_t leftRearOne = 17;
  const uint8_t leftRearTwo = 27;
  const uint8_t liftOne = 9;
  const uint8_t liftTwo = 11;
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<MotorController>(chip, 
                                                      leftFrontOne, 
                                                      leftFrontTwo, 
                                                      leftRearOne, 
                                                      leftRearTwo,
                                                      rightFrontOne, 
                                                      rightFrontTwo,
                                                      rightRearOne, 
                                                      rightRearTwo, 
                                                      liftOne, 
                                                      liftTwo);
  rclcpp::spin(std::make_shared<JoyListener>(controller));
  rclcpp::shutdown();
  return 0;
}