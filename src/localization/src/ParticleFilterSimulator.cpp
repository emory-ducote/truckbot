#include "rclcpp/rclcpp.hpp"

#include "ParticleFilter.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <random>

using namespace Eigen;
using namespace LocalizationHelpers;


int main() {
    int num_landmarks = 15;
    ParticleFilter filter(250, 10, 1);
    std::vector<Vector2d> landmarks;

    // Place landmarks in a non-symmetrical L shape: 10 along x-axis (y=0), 5 along y-axis (x=0, y from 0 to 10)
    int num_landmarks_x = 10;
    int num_landmarks_y = 5;
    double x_min = -10.0, x_max = 10.0;
    double y_min = 0.0, y_max = 10.0;
    double dx = (num_landmarks_x > 1) ? (x_max - x_min) / (num_landmarks_x - 1) : 0.0;
    double dy = (num_landmarks_y > 1) ? (y_max - y_min) / (num_landmarks_y - 1) : 0.0;
    int idx = 1;
    // x-axis (y=0)
    for (int i = 0; i < num_landmarks_x; ++i, ++idx) {
        double x = x_min + i * dx;
        double y = 0.0;
        landmarks.emplace_back(Vector2d(x, y));
        std::cout << "Vector2d landmarkPosition" << idx 
                  << "(" << x << ", " << y << ");" << std::endl;
    }
    // y-axis (x=0, y>0), skip origin (i=0) to avoid duplicate (0,0)
    for (int i = 1; i < num_landmarks_y; ++i, ++idx) {
        double x = 0.0;
        double y = y_min + i * dy;
        landmarks.emplace_back(Vector2d(x, y));
        std::cout << "Vector2d landmarkPosition" << idx 
                  << "(" << x << ", " << y << ");" << std::endl;
    }



    Vector3d vehiclePosition(0.0, 0.0, 0.0);
    Vector2d u_t(2.0, 0.0);

    std::vector<Particle> resampled = filter.getParticles();

    std::ofstream file("particles_log.csv");
    file << "step,type,x,y,theta,particle_id,landmark_id,weight\n";  // CSV header
    

    for (int step = 0; step < 500; step++) {
        double weightSum;
        filter.particleMotionUpdate(resampled, u_t);
        std::cout << "STEP " << step << " vehicle: " << vehiclePosition[0] <<"," << vehiclePosition[1] << std::endl;
        if (step % 8) {
                u_t(1) = 0.1;
        }
        if (step % 3) {
                u_t(1) = -0.2;
        }
        if (step % 5) {
                u_t(1) = -0.3;
        }
        for (int j = 0; j < landmarks.size(); j++) {
            double deltaX = landmarks[j][0] - vehiclePosition[0];
            double deltaY = landmarks[j][1] - vehiclePosition[1];
            double q = pow(deltaX, 2) + pow(deltaY, 2);
            double r = std::sqrt(q);
            double theta1 = wrapAngle(atan2(deltaY, deltaX) - vehiclePosition[2]);

            

    
            Vector2d z_t(r, theta1);

            if ( r <= 5.0) {
                filter.particleWeightUpdate(resampled, z_t);
            }

            
        }
        double v_t = u_t[0] * 0.1;
            double w_t = u_t[1] * 0.1;
            double theta = vehiclePosition[2];
            if (std::abs(w_t) < 1e-6) {
                vehiclePosition[0] += v_t * std::cos(theta);
                vehiclePosition[1] += v_t * std::sin(theta);
                // theta unchanged for straight line
            } else {
                vehiclePosition[0] += (v_t / w_t) * (std::sin(theta + w_t) - std::sin(theta));
                vehiclePosition[1] += (v_t / w_t) * (std::cos(theta + w_t) - std::cos(theta));
                vehiclePosition[2] = wrapAngle(theta + w_t);
            }
        filter.particlePurgeLandmarks(resampled);
        resampled = filter.particleWeightResampling(resampled);
        // Log particles and their landmark estimates
        int particle_id = 0;
        std::cout << "TOTAL PARTICLE: " << resampled.size() << std::endl;
        for (auto &p : resampled) {
            file << step << ",particle," << p.getState()[0] << "," << p.getState()[1] << "," << p.getState()[2] << "," << particle_id << ",," << p.getWeight() << "\n";
            // Log landmark estimates for this particle
            auto landmark_estimates = p.getLandmarks(); // Assumes vector<Vector2d>
            int landmark_id = 0;
            for (auto& est : landmark_estimates) {
                file << step << ",particle_landmark," << est.getState()[0] << "," << est.getState()[1] << ",0," << particle_id << "," << landmark_id  << "," << p.getWeight() << "\n";
                landmark_id++;
            }
            particle_id++;
            // std::cout << "particle: " << particle_id << " landmakrs: " << p.getLandmarks().size() << std::endl;
        }

        // Log vehicle
        file << step << ",vehicle," << vehiclePosition[0] << "," << vehiclePosition[1] << "," << vehiclePosition[2] << ",,," << "\n";

        // Log landmark (no theta)
        for (int j = 0; j < landmarks.size(); j++) {
        file << step << ",true_landmark," << landmarks[j][0] << "," << landmarks[j][1] << ",0,,0,\n";
        }
        
    }
        file.close();
        return 0;
}
