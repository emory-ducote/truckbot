#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "ParticleFilter.h"
#include <fstream>


std::ofstream file("particles_log.csv");

using namespace LocalizationHelpers;

class ParticleFilterMiddleware : public rclcpp::Node {
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
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
      landmark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/heaviest_particle_landmarks", 10);
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
      particleFilter->particlePurgeLandmarks(resampled);
      std::cout << "Purged" << std::endl;
      resampled = particleFilter->particleWeightResampling(resampled);
      std::cout << "Resampled" << std::endl;

      // Find the heaviest weighted particle
      if (!resampled.empty()) {
        auto max_it = std::max_element(resampled.begin(), resampled.end(), [](const Particle& a, const Particle& b) {
          return a.getWeight() < b.getWeight();
        });
        if (max_it != resampled.end()) {
          Particle& heaviest = *max_it;
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
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            landmark_markers.markers.push_back(marker);
          }
          landmark_pub_->publish(landmark_markers);
        }
      }
      
      int particle_id = 0;
      // for (auto &p : resampled) {
      //       p.landmarksInRange(3.0);
      //       file << step << ",particle," << p.getState()[0] << "," << p.getState()[1] << "," << p.getState()[2] << "," << particle_id << ",," << p.getWeight() << "\n";
      //       // Log landmark estimates for this particle
      //       auto landmark_estimates = p.getLandmarks(); // Assumes vector<Vector2d>
      //       int landmark_id = 0;
      //       for (auto& est : landmark_estimates) {
      //           file << step << ",particle_landmark," << est[0] << "," << est[1] << ",0," << particle_id << "," << landmark_id  << "," << p.getWeight() << "\n";
      //           landmark_id++;
      //       }
      //       particle_id++;
      //       std::cout << "particle: " << particle_id << " landmakrs: " << p.getLandmarks().size() << std::endl;
      // }
      // step++;
  

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

// int main()
// {
//     double points[][k] = {{3.0, 5.0}, {3.1, 5.1}, {-3.0, -5.0}};
//     std::shared_ptr<const Node> root = nullptr;

//     int n = sizeof(points)/sizeof(points[0]);

//     Eigen::Matrix2d cov;
//     cov << 0.1, 0.0, 0.1, 0.0;

//     for (int i=0; i<n; i++)
//         root = insert(root, points[i], cov);

//     // Delete (30, 40);
//     std::shared_ptr<const Node> root2 = deleteNode(root, points[0]);

//     printKDTree(root);
//     std::cout << "\n\n\n" << std::endl;
//     const double target[2] = {3, 5};
//     const double targetNew[2] = {9, 11};
//     printKDTree(root2);
//     std::cout << "\n\n\n" << std::endl;

//     std::vector<Vector2d> results;

//     std::cout << findNearest(root, target)->point[0] << std::endl;

//     findNodesWithinThreshold(root, target, 100.0, results);

//     for (int r = 0; r < results.size(); r++) 
//     {
//       std::cout << "RESULT" << r << " " << results[r] << std::endl; 
//     }


//     return 0;
// }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto particleFilter = std::make_shared<ParticleFilter>(10, 7, 1);
    rclcpp::spin(std::make_shared<ParticleFilterMiddleware>(particleFilter));
    rclcpp::shutdown();
    file.close();
    return 0;
  }