#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <lgpio.h>
#include <iostream>
#include "MotorController.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelListener : public rclcpp::Node {
  public:
      CmdVelListener(std::shared_ptr<MotorController> motorController) : Node("cmd_vel_listener"), motorController(motorController) 
      {
          cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
              "/cmd_vel", 10, std::bind(&CmdVelListener::cmdVelCallback, this, std::placeholders::_1));
          cmd_actuator_sub_ = this->create_subscription<std_msgs::msg::Bool>(
              "/cmd_actuator", 10, std::bind(&CmdVelListener::cmdActuatorCallback, this, std::placeholders::_1));
      }
  
  private:
      void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) 
      {
        double linearX = msg->linear.x;
        double angularZ = msg->angular.z;

        RCLCPP_INFO(this->get_logger(), "Received Velocity Command: linear x: %f, angular z: %f", linearX, angularZ);
        motorController->applySpeedCommand(linearX, angularZ);
      }
      void cmdActuatorCallback(const std_msgs::msg::Bool::SharedPtr msg) 
      {
        bool actuatorEnabled = msg->data;
        motorController->moveActuator(actuatorEnabled);
        RCLCPP_INFO(this->get_logger(), "Received Actuator Command: enabled: %d", actuatorEnabled);
      }
  
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_actuator_sub_;
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
  rclcpp::spin(std::make_shared<CmdVelListener>(controller));
  rclcpp::shutdown();
  return 0;
}