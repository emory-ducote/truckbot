#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "truckbot_msgs/msg/heartbeat.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "PurePursuitController.h"
#include "TfPoseLookup.h"

class PurePursuitControllerMiddleware : public rclcpp::Node {
public:
	PurePursuitControllerMiddleware() : Node("pure_pursuit_controller")
	{
		heartbeat_pub_ = this->create_publisher<truckbot_msgs::msg::Heartbeat>("/health/heartbeat", 10);
		heartbeat_timer_ = this->create_wall_timer(
				std::chrono::seconds(1), std::bind(&PurePursuitControllerMiddleware::heartbeat_callback, this));
		heartbeat_callback();  // publish INIT immediately so the monitor knows this node exists
		double lookahead_distance = this->declare_parameter<double>("lookahead_distance", 1.0);
		linear_velocity_ = this->declare_parameter<double>("linear_velocity", 0.15);
		goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.1);
		int timer_period_ms = this->declare_parameter<int>("timer_period_ms", 10);
      	purePursuitController = std::make_shared<PurePursuitController>(lookahead_distance);
		constexpr double deg2rad = M_PI / 180.0;
		PurePursuitTuning tuning;
		tuning.slowdownAngle = deg2rad * this->declare_parameter<double>("slowdown_angle_deg", 30.0);
		tuning.minSpeedScale = this->declare_parameter<double>("min_speed_scale", tuning.minSpeedScale);
		tuning.turnInPlaceAngle = deg2rad * this->declare_parameter<double>("turn_in_place_angle_deg", 90.0);
		tuning.turnInPlaceExitAngle = deg2rad * this->declare_parameter<double>("turn_in_place_exit_angle_deg", 45.0);
		tuning.turnInPlaceOmega = this->declare_parameter<double>("turn_in_place_omega", tuning.turnInPlaceOmega);
		purePursuitController->setTuning(tuning);

		navigation::TfPoseLookupConfig pose_config;
		pose_config.globalFrame = this->declare_parameter<std::string>("global_frame", pose_config.globalFrame);
		pose_config.odomFrame = this->declare_parameter<std::string>("odom_frame", pose_config.odomFrame);
		pose_config.baseFrame = this->declare_parameter<std::string>("base_frame", pose_config.baseFrame);
		pose_config.maxCorrectionAge = this->declare_parameter<double>("max_correction_age", pose_config.maxCorrectionAge);
		pose_lookup_ = std::make_unique<navigation::TfPoseLookup>(*this, pose_config);
		local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
			"/local_path", 10, std::bind(&PurePursuitControllerMiddleware::localPathCallback, this, std::placeholders::_1));
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		global_lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/global_lookahead", 10);
		local_lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/local_lookahead", 10);
		goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(timer_period_ms),
			std::bind(&PurePursuitControllerMiddleware::timerCallback, this));
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

	bool updatePoseFromTf()
	{
		auto pose = pose_lookup_->lookup();
		if (!pose) {
			return false;
		}
		purePursuitController->setVehiclePose(*pose);
		pose_received_ = true;
		return true;
	}

	void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
	{
		std::vector<navigation::VehiclePose> path;
		for (const auto& pose : msg->poses) {
			path.emplace_back(pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z);
		}
		if (!path.empty()) {
			// A local path ending somewhere new means a new goal, so re-arm after a finished mission
			if (!have_goal_ || path.back().euclideanDistanceTo(final_goal_) > goal_tolerance_) {
				mission_done = false;
			}
			final_goal_ = path.back();
			have_goal_ = true;
		}
		purePursuitController->setLocalPath(path);
	}

	void timerCallback()
	{
		if (!updatePoseFromTf() && pose_received_) {
			// Lost localization after having it: stop rather than drive on a frozen pose
			cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
			health_state_ = truckbot_msgs::msg::Heartbeat::FAILED;
			health_detail_ = pose_lookup_->chainName() + " tf unavailable";
			return;
		}
		if (!pose_received_ || purePursuitController->getLocalPath().empty()) {
			RCLCPP_INFO(this->get_logger(), "Waiting for pose and path initialization");
			// Having a pose but no path just means no goal yet
			if (pose_received_) {
				health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
				health_detail_ = "";
			}
			return;
		}
		
		// Calculate lookahead point and control
		try {
			auto globalLookahead = purePursuitController->calculateLookaheadPoint();
			auto localLookahead = purePursuitController->localLookahead(globalLookahead);
			auto command = purePursuitController->computeCommand(localLookahead, linear_velocity_);
			RCLCPP_INFO(this->get_logger(), "PurePursuit v: %f omega: %f", command.linear, command.angular);
			geometry_msgs::msg::Twist cmd_msg;
			// Stop on distance to the final goal, not the lookahead point (which trails the vehicle by Ld)
			double distance = purePursuitController->getVehiclePose().euclideanDistanceTo(final_goal_);
			RCLCPP_INFO(this->get_logger(), "Dist to goal: %f", distance);
			if ((distance < goal_tolerance_) || (mission_done))
			{
				cmd_msg.linear.x = 0.0;
				cmd_msg.angular.z = 0.0;
				if (!mission_done) {
					std_msgs::msg::Bool reached;
					reached.data = true;
					goal_reached_pub_->publish(reached);
				}
				mission_done = true;
			}
			else
			{
				cmd_msg.linear.x = command.linear;
				cmd_msg.angular.z = command.angular;
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
			local_lookahead_msg.header.stamp = rclcpp::Time(0);
			local_lookahead_msg.header.frame_id = "base_link";
			local_lookahead_msg.point.x = localLookahead.x;
			local_lookahead_msg.point.y = localLookahead.y;
			local_lookahead_msg.point.z = 0.0;
			local_lookahead_pub_->publish(local_lookahead_msg);
			health_state_ = truckbot_msgs::msg::Heartbeat::GOOD;
			health_detail_ = "";
		} catch (const std::exception& e) {
			RCLCPP_WARN(this->get_logger(), "PurePursuit control skipped: %s", e.what());
			health_state_ = truckbot_msgs::msg::Heartbeat::FAILED;
			health_detail_ = e.what();
		}
	}

	std::unique_ptr<navigation::TfPoseLookup> pose_lookup_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr global_lookahead_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_lookahead_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<PurePursuitController> purePursuitController;
	double linear_velocity_;
	double goal_tolerance_;
	bool pose_received_ = false;
	bool mission_done = false;
	bool have_goal_ = false;
	rclcpp::TimerBase::SharedPtr heartbeat_timer_;
	rclcpp::Publisher<truckbot_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
	uint8_t health_state_ = truckbot_msgs::msg::Heartbeat::INIT;
	std::string health_detail_;
	navigation::VehiclePose final_goal_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitControllerMiddleware>());
    rclcpp::shutdown();
    return 0;
}
