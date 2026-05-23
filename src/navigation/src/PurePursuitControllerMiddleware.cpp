#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "PurePursuitController.h"

class PurePursuitControllerMiddleware : public rclcpp::Node {
public:
	PurePursuitControllerMiddleware() : Node("pure_pursuit_controller_middleware")
	{
		double lookahead_distance = this->declare_parameter<double>("lookahead_distance", 1.0);
      	purePursuitController = std::make_shared<PurePursuitController>(lookahead_distance);

		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/local_odom", 10, std::bind(&PurePursuitControllerMiddleware::odomCallback, this, std::placeholders::_1));
		local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
			"/local_path", 10, std::bind(&PurePursuitControllerMiddleware::localPathCallback, this, std::placeholders::_1));
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		global_lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/global_lookahead", 10);
		local_lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/local_lookahead", 10);
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100),
			std::bind(&PurePursuitControllerMiddleware::timerCallback, this));
	}

private:
	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      navigation::VehiclePose newPose(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.orientation.z);
      purePursuitController->setVehiclePose(newPose);
      
    }

	void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
	{
		std::vector<navigation::VehiclePose> path;
		for (const auto& pose : msg->poses) {
			path.emplace_back(pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z);
		}
		purePursuitController->setLocalPath(path);
	}

	void timerCallback()
	{
		// Guard: check if pose and path are initialized
		auto pose = purePursuitController->getVehiclePose();
		if ((pose.x == 0 && pose.y == 0 && pose.theta == 0) || purePursuitController->getLocalPath().empty()) {
			RCLCPP_INFO(this->get_logger(), "Waiting for pose and path initialization");
			return;
		}
		
		// Calculate lookahead point and control
		try {
			auto globalLookahead = purePursuitController->calculateLookaheadPoint();
			auto localLookahead = purePursuitController->localLookahead(globalLookahead);
			auto omega = purePursuitController->computeControl(localLookahead, 0.3);
			RCLCPP_INFO(this->get_logger(), "PurePursuit omega chosen: %f", omega);
			geometry_msgs::msg::Twist cmd_msg;
			double distance = purePursuitController->getVehiclePose().euclideanDistanceTo(globalLookahead);
			RCLCPP_INFO(this->get_logger(), "Dist from point: %f", distance);
			if ((purePursuitController->getVehiclePose().euclideanDistanceTo(globalLookahead) < 0.15) || (mission_done))
			{
				cmd_msg.linear.x = 0.0;
				cmd_msg.angular.z = 0.0;
				mission_done = true;
			}
			else
			{
				cmd_msg.linear.x = 0.15;
				cmd_msg.angular.z = omega * 10;
			}
			cmd_vel_pub_->publish(cmd_msg);
			
			// Publish global lookahead point for debugging
			geometry_msgs::msg::PointStamped global_lookahead_msg;
			global_lookahead_msg.header.stamp = this->now();
			global_lookahead_msg.header.frame_id = "map";
			global_lookahead_msg.point.x = globalLookahead.x;
			global_lookahead_msg.point.y = globalLookahead.y;
			global_lookahead_msg.point.z = 0.0;
			global_lookahead_pub_->publish(global_lookahead_msg);
			
			// Publish local lookahead point for debugging
			geometry_msgs::msg::PointStamped local_lookahead_msg;
			local_lookahead_msg.header.stamp = this->now();
			local_lookahead_msg.header.frame_id = "base_link";
			local_lookahead_msg.point.x = localLookahead.x;
			local_lookahead_msg.point.y = localLookahead.y;
			local_lookahead_msg.point.z = 0.0;
			local_lookahead_pub_->publish(local_lookahead_msg);
		} catch (const std::exception& e) {
			RCLCPP_WARN(this->get_logger(), "PurePursuit control skipped: %s", e.what());
		}
	}

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr global_lookahead_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_lookahead_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<PurePursuitController> purePursuitController;
	bool mission_done = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitControllerMiddleware>());
    rclcpp::shutdown();
    return 0;
}
