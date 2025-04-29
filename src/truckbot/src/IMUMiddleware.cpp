#include "truckbot/rpi5ICM20948.h"
#include <iostream> 
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node {
    public:
    IMUPublisher(std::shared_ptr<rpi5ICM20948> imu) : Node("joy_listener"), imu(imu)
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
            timer_ = this->create_wall_timer(
                10ms, std::bind(&IMUPublisher::timer_callback, this));
        }
        std::shared_ptr<rpi5ICM20948> imu;
    private:
        void timer_callback()
        {
            auto message = sensor_msgs::msg::Imu();
            float ax, ay, az, gx, gy, gz, ux, uy, uz;
            imu->getAccelerometerAndGyroscopeData(ax, ay, az, gx, gy, gz);
            imu->getMagnetometerData(ux, uy, uz);
            message.angular_velocity.x = gx;
            message.angular_velocity.y = gy;
            message.angular_velocity.z = gz;
            message.linear_acceleration.x = ax;
            message.linear_acceleration.y = ay;
            message.linear_acceleration.z = az;
            message.orientation.x = ux;
            message.orientation.y = uy;
            message.orientation.z = uz;
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        
    };

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const uint8_t i2cDevice = 0x68;
    auto imu = std::make_shared<rpi5ICM20948>(i2cDevice);
    auto node = std::make_shared<IMUPublisher>(imu);
    rclcpp::on_shutdown([node]() {
        RCLCPP_INFO(node->get_logger(), "Shutdown callback triggered.");
        node->imu->resetMaster();
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }