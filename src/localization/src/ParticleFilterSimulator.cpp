#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ParticleFilter.h"
#include <fstream>

using namespace Eigen;

int main() {
    ParticleFilter filter(100, 1);

    Vector2d landmarkPosition1(2, 0);
    Vector2d landmarkPosition2(2, 1);
    Vector2d landmarkPosition3(2, 2);
    Vector2d landmarkPosition4(0, 2);
    Vector2d landmarkPosition5(0, 1);
    std::vector<Vector2d> landmarks;
    landmarks.push_back(landmarkPosition1);
    // landmarks.push_back(landmarkPosition2);
    // landmarks.push_back(landmarkPosition3);
    // landmarks.push_back(landmarkPosition4);
    // landmarks.push_back(landmarkPosition5);


    Vector3d vehiclePosition(0.0, 0.0, 0.0);

    std::vector<Particle> resampled = filter.getParticles();

    std::ofstream file("particles_log.csv");
    file << "step,type,x,y,theta,particle_id,landmark_id\n";  // CSV header
    

    for (int step = 0; step < 20; step++) {
            double deltaX = landmarks[0][0] - vehiclePosition[0];
            double deltaY = landmarks[0][1] - vehiclePosition[1];
            double q = pow(deltaX, 2) + pow(deltaY, 2);
            double r = std::sqrt(q);
            double theta1 = atan2(deltaY, deltaX) - vehiclePosition[2];

            Vector2d u_t(0.1, 0.0000000001);
            if ((step > 1) && (step < 3)) {
                u_t(1) = 0.1;
            }
            Vector2d z_t(r, theta1);

            resampled = filter.particleUpdate(resampled, z_t, u_t);

            double v_t = u_t[0];
            double w_t = u_t[1];
            double theta = vehiclePosition[2];
            vehiclePosition[0] += (v_t / w_t) * (sin(theta + w_t) - sin(theta));
            vehiclePosition[1] += (v_t / w_t) * (cos(theta + w_t) - cos(theta));
            vehiclePosition[2] = wrapAngle(theta + w_t);

        // Log particles and their landmark estimates
        int particle_id = 0;
        std::cout << "TOTAL PARTICLE: " << resampled.size() << std::endl;
        for (auto &p : resampled) {
            file << step << ",particle," << p.getState()[0] << "," << p.getState()[1] << "," << p.getState()[2] << "," << particle_id << "," << "\n";
            // Log landmark estimates for this particle
            auto landmark_estimates = p.getLandmarks(); // Assumes vector<Vector2d>
            int landmark_id = 0;
            for (auto& est : landmark_estimates) {
                file << step << ",particle_landmark," << est.getState()[0] << "," << est.getState()[1] << ",0," << particle_id << "," << landmark_id  << "\n";
                landmark_id++;
            }
            particle_id++;
        }

        // Log vehicle
        file << step << ",vehicle," << vehiclePosition[0] << "," << vehiclePosition[1] << "," << vehiclePosition[2] << ",," << "\n";

        // Log landmark (no theta)
        file << step << ",true_landmark," << landmarks[0][0] << "," << landmarks[0][1] << ",0,,0\n";
    }
        file.close();
        return 0;
}
