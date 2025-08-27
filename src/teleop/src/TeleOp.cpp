#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>


class JoyListener : public rclcpp::Node {
  public:
      JoyListener() : Node("joy_listener")
      {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyListener::joyCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      }
  
  private:
      void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
      {
        RCLCPP_INFO(this->get_logger(), "Received joystick data");
        double left_stick_x = msg->axes[1];  // Typically left stick horizontal
        double right_stick_x = msg->axes[4];  // Typically left stick vertical
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = left_stick_x * 0.75;
        message.angular.z = right_stick_x;
        publisher_->publish(message);
        int y_button = msg->buttons[0];
        int a_button = msg->buttons[1];
        //   motorController->applySpeedCommand(left_stick_x, right_stick_x);
        bool up = (y_button > 0) ? true : (a_button > 0) ? false : up;
        //   motorController->moveActuator(up);
        std::cout << left_stick_x << "  " << right_stick_x << std::endl;
        std::cout << "Y: " << y_button << " A:" << a_button << std::endl;
      }
  
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  };

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyListener>());
    rclcpp::shutdown();
    return 0;
}