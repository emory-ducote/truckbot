#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"


class JoyListener : public rclcpp::Node {
  public:
      JoyListener() : Node("joy_listener")
      {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyListener::joyCallback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        cmd_actuator_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_actuator", 10);
      }
  
  private:
      void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) 
      {
        double left_stick_x = msg->axes[1];  
        double right_stick_x = msg->axes[4];  
        auto twist_message = geometry_msgs::msg::Twist();
        twist_message.linear.x = left_stick_x * 0.75;
        twist_message.angular.z = right_stick_x * 5.0;
        cmd_vel_publisher_->publish(twist_message);
        int y_button = msg->buttons[0];
        int a_button = msg->buttons[1];
        bool up = false;
        bool up = (y_button > 0) ? true : (a_button > 0) ? false : up;
        auto bool_message = std_msgs::msg::Bool();
        bool_message.data = up;
        cmd_actuator_publisher_->publish(bool_message);
        RCLCPP_INFO(this->get_logger(), "Received data: left stick: %f, right stick %f, Y button: %d, A button %d",
                    left_stick_x, right_stick_x, y_button, a_button);
      }
  
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_actuator_publisher_;
  };

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyListener>());
    rclcpp::shutdown();
    return 0;
}
