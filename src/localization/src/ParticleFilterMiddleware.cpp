#include "rclcpp/rclcpp.hpp"
#include "ParticleFilter.h"



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ParticleFilter filter(30);
    Vector2d z_t(2, 1);
    Vector2d u_t(0, 0);
    std::vector<Particle> resampled = filter.particleUpdate(filter.getParticles(), z_t, u_t);
    Vector2d z_t_2(2.01, 1);
    Vector2d u_t_2(0, 0);
    filter.particleUpdate(resampled, z_t_2, u_t_2);
    std::cout << resampled.size() << std::endl;


    // auto state = std::make_shared<EKF>();
    // rclcpp::spin(std::make_shared<EKFMiddleware>(state));
    rclcpp::shutdown();
    return 0;
  }