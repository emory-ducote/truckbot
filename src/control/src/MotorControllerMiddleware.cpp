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

class MotorControllerMiddleware : public rclcpp::Node {
  public:
      MotorControllerMiddleware() : Node("motor_controller_node")
      {
          // Declare hardware/config parameters
          double vehicleWidth = this->declare_parameter<double>("vehicle_width", 0.2);
          double wheelRadius = this->declare_parameter<double>("wheel_radius", 0.03);
          int maxWheelMotorRpm = this->declare_parameter<int>("max_wheel_motor_rpm", 251);
          int chip = this->declare_parameter<int>("chip", 4);
          int rightFrontOne = this->declare_parameter<int>("right_front_one", 22);
          int rightFrontTwo = this->declare_parameter<int>("right_front_two", 0);
          int rightRearOne = this->declare_parameter<int>("right_rear_one", 5);
          int rightRearTwo = this->declare_parameter<int>("right_rear_two", 6);
          int leftFrontOne = this->declare_parameter<int>("left_front_one", 26);
          int leftFrontTwo = this->declare_parameter<int>("left_front_two", 19);
          int leftRearOne = this->declare_parameter<int>("left_rear_one", 23);
          int leftRearTwo = this->declare_parameter<int>("left_rear_two", 18);
          int liftOne = this->declare_parameter<int>("lift_one", 17);
          int liftTwo = this->declare_parameter<int>("lift_two", 27);
          std::string wheel_control_topic = this->declare_parameter<std::string>("wheel_control_topic", "/cmd_vel");
          std::string actuator_control_topic = this->declare_parameter<std::string>("actuator_control_topic", "/cmd_actuator");

          // Construct MotorController using parameters
          motorController = std::make_shared<MotorController>(chip,
              leftFrontOne, leftFrontTwo, leftRearOne, leftRearTwo,
              rightFrontOne, rightFrontTwo, rightRearOne, rightRearTwo,
              liftOne, liftTwo, vehicleWidth, wheelRadius, maxWheelMotorRpm);

          cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
              "/cmd_vel", 10, std::bind(&MotorControllerMiddleware::cmdVelCallback, this, std::placeholders::_1));
          cmd_actuator_sub_ = this->create_subscription<std_msgs::msg::Bool>(
              "/cmd_actuator", 10, std::bind(&MotorControllerMiddleware::cmdActuatorCallback, this, std::placeholders::_1));
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
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControllerMiddleware>());
  rclcpp::shutdown();
  return 0;
}