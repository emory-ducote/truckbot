#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <lgpio.h>
#include <iostream>
#include "MotorDriver.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class MotorDriverMiddleware : public rclcpp::Node {
  public:
      MotorDriverMiddleware() : Node("motor_driver_node")
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
          int Kp = this->declare_parameter<double>("Kp", 1.25);
          int Ki = this->declare_parameter<double>("Ki", 0.0);
          int Kd = this->declare_parameter<double>("Kd", 0.0);


          std::string wheel_control_topic = this->declare_parameter<std::string>("wheel_control_topic", "/cmd_vel");
          std::string actuator_control_topic = this->declare_parameter<std::string>("actuator_control_topic", "/cmd_actuator");

          rightFront = std::make_shared<MotorDriver>(chip, rightFrontOne, rightFrontTwo, Kp, Ki, Kd);
          rightRear = std::make_shared<MotorDriver>(chip, rightRearOne, rightRearTwo, Kp, Ki, Kd);
          leftFront = std::make_shared<MotorDriver>(chip, leftFrontOne, leftFrontTwo, Kp, Ki, Kd);
          leftRear = std::make_shared<MotorDriver>(chip, leftRearOne, leftRearTwo, Kp, Ki, Kd);
          lift = std::make_shared<MotorDriver>(chip, liftOne, liftTwo);

          // Subscribe to per-wheel encoder topics (published by EncoderDriverMiddleware)
          std::string leftFrontTopic = this->declare_parameter<std::string>("left_front_encoder_topic", "/encoder/left_front");
          std::string leftRearTopic = this->declare_parameter<std::string>("left_rear_encoder_topic", "/encoder/left_rear");
          std::string rightFrontTopic = this->declare_parameter<std::string>("right_front_encoder_topic", "/encoder/right_front");
          std::string rightRearTopic = this->declare_parameter<std::string>("right_rear_encoder_topic", "/encoder/right_rear");

          left_front_enc_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            leftFrontTopic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg){ leftFront->setMeasuredFL(msg->data); });
          left_rear_enc_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            leftRearTopic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg){ leftRear->setMeasuredRL(msg->data); });
          right_front_enc_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            rightFrontTopic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg){ rightFront->setMeasuredFR(msg->data); });
          right_rear_enc_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            rightRearTopic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg){ rightRear->setMeasuredRR(msg->data); });

          cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
              wheel_control_topic, 10, std::bind(&MotorDriverMiddleware::cmdVelCallback, this, std::placeholders::_1));
          cmd_actuator_sub_ = this->create_subscription<std_msgs::msg::Bool>(
              actuator_control_topic, 10, std::bind(&MotorDriverMiddleware::cmdActuatorCallback, this, std::placeholders::_1));
      }
  
  private:
      void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) 
      {
        double linearX = msg->linear.x;
        double angularZ = msg->angular.z;

        RCLCPP_INFO(this->get_logger(), "Received Velocity Command: linear x: %f, angular z: %f", linearX, angularZ);

        double angularScaleFactor = 1.75;
    
        // Wheelbase in meters (8 inches ≈ 0.2032 m)
        const double wheelbase = 0.2032;

        // Compute individual wheel velocities for skid steer with wheelbase compensation
        double rightFrontVel = linearX + (vehicleWidth/2.0) * angularZ * angularScaleFactor;
        double rightRearVel = linearX + (vehicleWidth/2.0) * angularZ * angularScaleFactor;
        double leftFrontVel = linearX - (vehicleWidth/2.0) * angularZ * angularScaleFactor;
        double leftRearVel = linearX - (vehicleWidth/2.0) * angularZ * angularScaleFactor;

        rightFront->applySpeedCommand(rightFrontVel, true);
        rightRear->applySpeedCommand(rightRearVel, true);
        leftFront->applySpeedCommand(leftFrontVel, true);
        leftRear->applySpeedCommand(leftRearVel, true);

      }
      void cmdActuatorCallback(const std_msgs::msg::Bool::SharedPtr msg) 
      {
        bool actuatorEnabled = msg->data;
        if (actuatorEnabled)
        {
          lift->setMotorSpeed(1.0, false);
        }
        else
        {
          lift->setMotorSpeed(-1.0, false)
        }
        RCLCPP_INFO(this->get_logger(), "Received Actuator Command: enabled: %d", actuatorEnabled);
      }
  
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_actuator_sub_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_front_enc_sub_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_rear_enc_sub_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_front_enc_sub_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_rear_enc_sub_;
      std::shared_ptr<MotorDriver> rightFront;
      std::shared_ptr<MotorDriver> rightRear;
      std::shared_ptr<MotorDriver> leftFront;
      std::shared_ptr<MotorDriver> leftRear;
      std::shared_ptr<MotorDriver> lift;

};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriverMiddleware>());
  rclcpp::shutdown();
  return 0;
}