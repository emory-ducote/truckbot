#include "ParticleFilter.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>


using namespace LocalizationHelpers;

class ParticleFilterMiddleware : public rclcpp::Node {
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_particles_pose_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  public:
    ParticleFilterMiddleware() 
      : Node("particle_filter_middleware")
    {
      int numParticles = this->declare_parameter<int>("num_particles", 100);
      int newParticleIncrease = this->declare_parameter<int>("new_particle_increase", 1);
      double maxRange = this->declare_parameter<double>("max_range", 5.0);
      double maxAngle = this->declare_parameter<double>("max_angle", 180.0);
      double newParticleThreshold = this->declare_parameter<double>("new_particle_threshold", 0.8);
      double neffThreshold = this->declare_parameter<double>("neff_threshold", 0.6);
      double measurementNoiseRange = this->declare_parameter<double>("measurement_noise_range", 0.1);
      double measurementNoiseBearing = this->declare_parameter<double>("measurement_noise_bearing", 0.01);
      double linearVelocityAlpha1 = this->declare_parameter<double>("linear_velocity_alpha_1", 0.2);
      double linearVelocityAlpha2 = this->declare_parameter<double>("linear_velocity_alpha_2", 0.05);
      double angularVelocityAlpha1 = this->declare_parameter<double>("angular_velocity_alpha_1", 0.05);
      double angularVelocityAlpha2 = this->declare_parameter<double>("angular_velocity_alpha_2", 0.2);
      

      particleFilter = std::make_shared<ParticleFilter>(numParticles,
                                                             newParticleIncrease,
                                                             maxRange,
                                                             maxAngle * M_PI / 180.0,
                                                             newParticleThreshold,
                                                             neffThreshold,
                                                             measurementNoiseRange,
                                                             measurementNoiseBearing,
                                                             linearVelocityAlpha1,
                                                             linearVelocityAlpha2,
                                                             angularVelocityAlpha1,
                                                             angularVelocityAlpha2);
                                                             
      u_t << 0.0, 0.0;    

      cluster_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/cluster_markers",
                                                                                     10,
                                                                                    std::bind(&ParticleFilterMiddleware::clusterCallback, this, std::placeholders::_1));
      control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/odom",
                                                                          10,
                                                                          std::bind(&ParticleFilterMiddleware::controlCallback, this, std::placeholders::_1));

      landmark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/heaviest_particle_landmarks", 10);
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/heaviest_particle_pose", 10);
      all_particles_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/all_particles_poses", 10);
      
      static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);                                       
    
      RCLCPP_INFO(this->get_logger(), "Initialized Filter");

    }
    double prev_time = 0;
  
  private:
    void publishHeaviestParticleTransform(const Particle& heaviest)
    {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = this->now();
      tf_msg.header.frame_id = "map";
      tf_msg.child_frame_id = "laser";
      tf_msg.transform.translation.x = heaviest.x[0];
      tf_msg.transform.translation.y = heaviest.x[1];
      tf_msg.transform.translation.z = 0.0;
      double theta = heaviest.x[2] + M_PI;
      tf_msg.transform.rotation.x = 0.0;
      tf_msg.transform.rotation.y = 0.0;
      tf_msg.transform.rotation.z = sin(theta/2.0);
      tf_msg.transform.rotation.w = cos(theta/2.0);
      static_broadcaster_->sendTransform(tf_msg);
    }
    void clusterCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
      double timestamp;
      if (!msg->markers.empty()) {
          timestamp = msg->markers[0].header.stamp.sec + 1e-9 * msg->markers[0].header.stamp.nanosec;
      }
      else {
        return;
      }
      double dt = timestamp - prev_time;
      if (dt > 100) {
        dt = 0.5;
      }
      prev_time = timestamp;

      std::vector<Vector2d> z_t_s;
      for (const auto& marker : msg->markers)
      {
        double q = pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2);
        double r = std::sqrt(q);
        double theta1 = wrapAngle(atan2(marker.pose.position.y, marker.pose.position.x) + M_PI);
        Vector2d z_t(r, theta1);
        z_t_s.push_back(z_t);
      }
      std::vector<Particle> result = particleFilter->particleFilterLoop(u_t, z_t_s, dt);
      
      // Publish all particle poses
      geometry_msgs::msg::PoseArray all_poses_msg;
      all_poses_msg.header.stamp = this->now();
      all_poses_msg.header.frame_id = "map";
      all_poses_msg.poses.reserve(result.size());
      for (const auto& particle : result) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = particle.x[0];
        pose.position.y = particle.x[1];
        pose.position.z = 0.0;
        double theta = particle.x[2];
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = sin(theta/2.0);
        pose.orientation.w = cos(theta/2.0);
        all_poses_msg.poses.push_back(pose);
      }
      all_particles_pose_pub_->publish(all_poses_msg);
      
      // Find the heaviest weighted particle
      if (!result.empty()) {
        auto max_it = std::max_element(result.begin(), result.end(), [](const Particle& a, const Particle& b) {
          return a.weight < b.weight;
        });
        if (max_it != result.end()) {
          Particle& heaviest = *max_it;
          publishHeaviestParticleTransform(heaviest);
          // Publish the transform from map to laser as the heaviest particle pose
          
          // Publish pose estimate
          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header.stamp = this->now();
          pose_msg.header.frame_id = "map";
          pose_msg.pose.position.x = heaviest.x[0];
          pose_msg.pose.position.y = heaviest.x[1];
          pose_msg.pose.position.z = 0.0;
          double theta = heaviest.x[2];
          pose_msg.pose.orientation.x = 0.0;
          pose_msg.pose.orientation.y = 0.0;
          pose_msg.pose.orientation.z = sin(theta/2.0);
          pose_msg.pose.orientation.w = cos(theta/2.0);
          pose_pub_->publish(pose_msg);
          
          auto landmarks = heaviest.getLandmarks(); // Assumes vector<Vector2d>
          visualization_msgs::msg::MarkerArray landmark_markers;
          int landmark_id = 0;
          for (const auto& lm : landmarks) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = this->now();
            marker.ns = "heaviest_particle_landmarks";
            marker.id = landmark_id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = lm[0];
            marker.pose.position.y = lm[1];
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            landmark_markers.markers.push_back(marker);
          }
          landmark_pub_->publish(landmark_markers);
        }
      }
    }

    void controlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    u_t(0) = msg->linear.x*1.5;
    {
      u_t(1) = msg->angular.z;
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_sub_;
    std::shared_ptr<ParticleFilter> particleFilter;
    std::vector<Particle> result;
    Vector2d u_t;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterMiddleware>());
    rclcpp::shutdown();
    return 0;
  }