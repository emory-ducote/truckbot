#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "ParticleFilter.h"
#include <fstream>


std::ofstream file("particles_log.csv");

using namespace LocalizationHelpers;

class ParticleFilterMiddleware : public rclcpp::Node {
  public:
    ParticleFilterMiddleware(std::shared_ptr<ParticleFilter> particleFilter) : 
                              Node("particle_filter_middleware"),
                              particleFilter(particleFilter)
    {
      cluster_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/cluster_markers",
                                                                                     10,
                                                                                    std::bind(&ParticleFilterMiddleware::clusterCallback, this, std::placeholders::_1));
      control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/odom",
                                                                          10,
                                                                          std::bind(&ParticleFilterMiddleware::controlCallback, this, std::placeholders::_1));
      resampled = particleFilter->getParticles();    
      u_t << 0.0, 0.0;    
      step = 0;

      file << "step,type,x,y,theta,particle_id,landmark_id,weight\n";  // CSV header
                                                                      
    }
  
  private:
    void clusterCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
      particleFilter->particleMotionUpdate(resampled, u_t);
      for (const auto& marker : msg->markers)
      {
        double q = pow(marker.pose.position.x, 2) + pow(marker.pose.position.y, 2);
        double r = std::sqrt(q);
        double theta1 = wrapAngle(atan2(marker.pose.position.y, marker.pose.position.x) + M_PI);
        if (r <= 3.0) {

        Vector2d z_t(r, theta1);
        particleFilter->particleWeightUpdate(resampled, z_t);
        }
      }
      std::cout << "Done through markers" << std::endl;
      // particleFilter->particlePurgeLandmarks(resampled);
      resampled = particleFilter->particleWeightResampling(resampled);

      int particle_id = 0;
      for (auto &p : resampled) {
            file << step << ",particle," << p.getState()[0] << "," << p.getState()[1] << "," << p.getState()[2] << "," << particle_id << ",," << p.getWeight() << "\n";
            // Log landmark estimates for this particle
            // auto landmark_estimates = p.getLandmarks(); // Assumes vector<Vector2d>
            // int landmark_id = 0;
            // for (auto& est : landmark_estimates) {
            //     file << step << ",particle_landmark," << est.getState()[0] << "," << est.getState()[1] << ",0," << particle_id << "," << landmark_id  << "," << p.getWeight() << "\n";
            //     landmark_id++;
            // }
            particle_id++;
            // std::cout << "particle: " << particle_id << " landmakrs: " << p.getLandmarks().size() << std::endl;
      }
      step++;
  

    }

    void controlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      u_t(0) = msg->linear.x;
      u_t(1) = msg->angular.z;
      // RCLCPP_INFO(this->get_logger(), "Got control x: %f, z: %f", u_t(0), u_t(1));
    }
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_sub_;
    std::shared_ptr<ParticleFilter> particleFilter;
    std::vector<Particle> resampled;
    Vector2d u_t;
    int step;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto particleFilter = std::make_shared<ParticleFilter>(100, 7, 1);
    rclcpp::spin(std::make_shared<ParticleFilterMiddleware>(particleFilter));
    rclcpp::shutdown();
    file.close();
    return 0;
  }