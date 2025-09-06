#include "rclcpp/rclcpp.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // auto state = std::make_shared<EKF>();
    // rclcpp::spin(std::make_shared<EKFMiddleware>(state));
    rclcpp::shutdown();
    return 0;
  }