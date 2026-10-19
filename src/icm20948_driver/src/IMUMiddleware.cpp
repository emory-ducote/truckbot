#include "rpi5ICM20948.h"
#include <iostream> 
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include "truckbot_msgs/msg/heartbeat.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node {
    public:
    IMUPublisher() : Node("imu_middleware")
        {
            int deviceId = this->declare_parameter<int>("i2c_device", 0x68);

            publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
            timer_ = this->create_wall_timer(
                10ms, std::bind(&IMUPublisher::timer_callback, this));
            heartbeat_pub_ = this->create_publisher<truckbot_msgs::msg::Heartbeat>("/health/heartbeat", 10);
            heartbeat_timer_ = this->create_wall_timer(
                1s, std::bind(&IMUPublisher::heartbeat_callback, this));
	    imu = std::make_shared<rpi5ICM20948>(deviceId);
        }

            std::shared_ptr<rpi5ICM20948> imu; 
    private:
        void timer_callback()
        {
            auto message = sensor_msgs::msg::Imu();
            float ax, ay, az, gx, gy, gz, ux, uy, uz;
            int accelGyroRead = imu->getAccelerometerAndGyroscopeData(ax, ay, az, gx, gy, gz);
            int magRead = imu->getMagnetometerData(ux, uy, uz);
            if (accelGyroRead < 0 || magRead < 0) {
                health_state_ = truckbot_msgs::msg::Heartbeat::FAILED;
                health_detail_ = "i2c read failed";
            } else {
                health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
                health_detail_ = "";
            }
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

        void heartbeat_callback()
        {
            auto heartbeat = truckbot_msgs::msg::Heartbeat();
            heartbeat.stamp = this->now();
            heartbeat.node_name = this->get_name();
            heartbeat.state = health_state_;
            heartbeat.detail = health_detail_;
            heartbeat_pub_->publish(heartbeat);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr heartbeat_timer_;
        rclcpp::Publisher<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
        uint8_t health_state_ = truckbot_msgs::msg::Heartbeat::INIT;
        std::string health_detail_;
        
    };

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<IMUPublisher> node;
    try {
        node = std::make_shared<IMUPublisher>();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("imu_middleware"), "Failed to initialize IMU: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::on_shutdown([node]() {
        RCLCPP_INFO(node->get_logger(), "Shutdown callback triggered.");
        try {
            node->imu->resetMaster();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "IMU reset on shutdown failed: %s", e.what());
        }
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
