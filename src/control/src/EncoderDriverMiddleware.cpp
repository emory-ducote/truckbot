#include "EncoderDriver.h"
#include "rclcpp/rclcpp.hpp"
#include "truckbot_msgs/msg/heartbeat.hpp"
#include <chrono>
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;


class EncoderDriverMiddleware : public rclcpp::Node {
    public:
      EncoderDriverMiddleware()
        : Node("encoder_driver")
      {
        heartbeat_pub_ = this->create_publisher<truckbot_msgs::msg::Heartbeat>("/health/heartbeat", 10);
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&EncoderDriverMiddleware::heartbeat_callback, this));
        heartbeat_callback();  // publish INIT immediately so the monitor knows this node exists
        // Declare and get parameters from YAML
        wheelSeparation = this->declare_parameter<double>("wheel_separation", 0.2);
        double wheelRadius = this->declare_parameter<double>("wheel_radius", 0.05);
        int encoderCPR = this->declare_parameter<int>("encoder_cpr", 700);
        int encoderTicksPerRevolution = this->declare_parameter<int>("encoder_ticks_per_revolution", 4);
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
        std::string leftFrontTopic = this->declare_parameter<std::string>("left_front_encoder_topic", "/encoder/left_front");
        std::string leftRearTopic = this->declare_parameter<std::string>("left_rear_encoder_topic", "/encoder/left_rear");
        std::string rightFrontTopic = this->declare_parameter<std::string>("right_front_encoder_topic", "/encoder/right_front");
        std::string rightRearTopic = this->declare_parameter<std::string>("right_rear_encoder_topic", "/encoder/right_rear");
        int update_rate = this->declare_parameter<int>("update_rate", 10);

        // On GPIO setup failure, stay alive only to report FAILED
        try {
          rightFront = std::make_shared<EncoderDriver>(chip, rightFrontA, rightFrontB, wheelRadius, encoderCPR, encoderTicksPerRevolution);
          rightRear = std::make_shared<EncoderDriver>(chip, rightRearA, rightRearB, wheelRadius, encoderCPR, encoderTicksPerRevolution);
          leftFront = std::make_shared<EncoderDriver>(chip, leftFrontA, leftFrontB, wheelRadius, encoderCPR, encoderTicksPerRevolution);
          leftRear = std::make_shared<EncoderDriver>(chip, leftRearA, leftRearB, wheelRadius, encoderCPR, encoderTicksPerRevolution);
        } catch (const std::exception& e) {
          RCLCPP_FATAL(this->get_logger(), "Encoder setup failed: %s", e.what());
          health_state_ = truckbot_msgs::msg::Heartbeat::FAILED;
          health_detail_ = e.what();
          return;
        }

        // Create publishers and use update_rate for timer period
        left_front_pub_ = this->create_publisher<std_msgs::msg::Float64>(leftFrontTopic, 10);
        left_rear_pub_ = this->create_publisher<std_msgs::msg::Float64>(leftRearTopic, 10);
        right_front_pub_ = this->create_publisher<std_msgs::msg::Float64>(rightFrontTopic, 10);
        right_rear_pub_ = this->create_publisher<std_msgs::msg::Float64>(rightRearTopic, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / update_rate),
            std::bind(&EncoderDriverMiddleware::timer_callback, this));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(encoder_topic, 10);
      }
    private:
        void heartbeat_callback()
        {
          auto heartbeat = truckbot_msgs::msg::Heartbeat();
          heartbeat.stamp = this->now();
          heartbeat.node_name = this->get_name();
          heartbeat.state = health_state_;
          heartbeat.detail = health_detail_;
          heartbeat_pub_->publish(heartbeat);
        }

        void timer_callback()
        {
          auto rightFrontWheelSpeed = rightFront->getWheelSpeeds();
          auto rightRearWheelSpeed = rightRear->getWheelSpeeds();
          auto leftFrontWheelSpeed = leftFront->getWheelSpeeds();
          auto leftRearWheelSpeed = leftRear->getWheelSpeeds();

          int readErrors = rightFront->getReadErrors() + rightRear->getReadErrors() +
                           leftFront->getReadErrors() + leftRear->getReadErrors();
          if (readErrors > lastReadErrors) {
            health_state_ = truckbot_msgs::msg::Heartbeat::FAILED;
            health_detail_ = "encoder gpio read failed";
          } else {
            health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
            health_detail_ = "";
          }
          lastReadErrors = readErrors;

          // RCLCPP_INFO(this->get_logger(), "LF: %f, LR: %f, RF: %f, RR: %f", leftFrontWheelSpeed, leftRearWheelSpeed, rightFrontWheelSpeed, rightRearWheelSpeed);

          auto rightWheelSpeed = (rightFrontWheelSpeed + rightRearWheelSpeed) / 2;
          auto leftWheelSpeed = (leftFrontWheelSpeed + leftRearWheelSpeed) / 2;

          auto message = nav_msgs::msg::Odometry();
          message.header.stamp = this->now();
          message.header.frame_id = "odom";
          message.child_frame_id = "base_link";
          message.twist.twist.linear.x = (rightWheelSpeed + leftWheelSpeed) / 2;
          message.twist.twist.angular.z = (rightWheelSpeed - leftWheelSpeed) / wheelSeparation;

          std_msgs::msg::Float64 m;
          m.data = leftFrontWheelSpeed; left_front_pub_->publish(m);
          m.data = leftRearWheelSpeed; left_rear_pub_->publish(m);
          m.data = rightFrontWheelSpeed; right_front_pub_->publish(m);
          m.data = rightRearWheelSpeed; right_rear_pub_->publish(m);
          odom_publisher_->publish(message);
        }

        std::shared_ptr<EncoderDriver> rightFront;
        std::shared_ptr<EncoderDriver> rightRear;
        std::shared_ptr<EncoderDriver> leftFront;
        std::shared_ptr<EncoderDriver> leftRear;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_front_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rear_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_front_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rear_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr heartbeat_timer_;
        rclcpp::Publisher<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
        uint8_t health_state_ = truckbot_msgs::msg::Heartbeat::INIT;
        std::string health_detail_;
        double wheelSeparation;
        int lastReadErrors = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderDriverMiddleware>());
  rclcpp::shutdown();
  return 0;
}
