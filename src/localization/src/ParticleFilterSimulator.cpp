#include "rclcpp/rclcpp.hpp"

#include "ParticleFilter.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <random>

using namespace Eigen;
using namespace LocalizationHelpers;


int main() {
    ParticleFilter filter(1, 10, 1);
    std::vector<Vector2d> landmarks;
    // Place N landmarks forming two corners at (5, 5) and (-5, 5), and a V shape at (0, -5)
    int num_landmarks = 50; // You can change this as needed
    int quarter = num_landmarks / 4;
    int half = num_landmarks / 2;
    int eighth = num_landmarks / 8;
    int v_count = num_landmarks / 4; // Number of points for the V shape
    // First quarter: vertical line at x=5, y from 0 to 5
    for (int i = 0; i < quarter; ++i) {
        double y = 5.0 * i / (quarter - 1);
        double x = 5.0;
        landmarks.emplace_back(Vector2d(x, y));
        std::cout << "Vector2d landmarkPosition" << (i+1)
                  << "(" << x << ", " << y << ");" << std::endl;
    }
    // Second quarter: horizontal line at y=5, x from 5 to 10
    for (int i = quarter; i < half; ++i) {
        double x = 5.0 + 5.0 * (i - quarter) / (half - quarter - 1);
        double y = 5.0;
        landmarks.emplace_back(Vector2d(x, y));
        std::cout << "Vector2d landmarkPosition" << (i+1)
                  << "(" << x << ", " << y << ");" << std::endl;
    }
    // Third quarter: vertical line at x=-5, y from 0 to 5
    for (int i = half; i < half + quarter; ++i) {
        double y = 5.0 * (i - half) / (quarter - 1);
        double x = -5.0;
        landmarks.emplace_back(Vector2d(x, y));
        std::cout << "Vector2d landmarkPosition" << (i+1)
                  << "(" << x << ", " << y << ");" << std::endl;
    }
    // Fourth quarter: horizontal line at y=5, x from -5 to -10
    for (int i = half + quarter; i < num_landmarks - v_count; ++i) {
        double x = -5.0 - 5.0 * (i - (half + quarter)) / (num_landmarks - v_count - (half + quarter) - 1);
        double y = 5.0;
        landmarks.emplace_back(Vector2d(x, y));
        std::cout << "Vector2d landmarkPosition" << (i+1)
                  << "(" << x << ", " << y << ");" << std::endl;
    }
    // V shape at (0, -5): two lines meeting at (0, -5)
    for (int i = 0; i < v_count; ++i) {
        double frac = (double)i / (v_count - 1);
        // Left arm: from (-5, -10) to (0, -5)
        double x_left = -5.0 + 5.0 * frac;
        double y_left = -10.0 + 5.0 * frac;
        landmarks.emplace_back(Vector2d(x_left, y_left));
        std::cout << "Vector2d V_left" << (i+1)
                  << "(" << x_left << ", " << y_left << ");" << std::endl;
        // Right arm: from (5, -10) to (0, -5)
        double x_right = 5.0 - 5.0 * frac;
        double y_right = -10.0 + 5.0 * frac;
        landmarks.emplace_back(Vector2d(x_right, y_right));
        std::cout << "Vector2d V_right" << (i+1)
                  << "(" << x_right << ", " << y_right << ");" << std::endl;
    }



    Vector3d vehiclePosition(0.0, 0.0, 0.0);
    Vector2d u_t(0.2, 0.0);

    std::vector<Particle> resampled = filter.getParticles();

    std::ofstream file("particles_log.csv");
    file << "step,type,x,y,theta,particle_id,landmark_id,weight\n";  // CSV header
    

    for (int step = 0; step < 50; step++) {
        double weightSum;
        std::cout << "STEP " << step << " vehicle: " << vehiclePosition[0] <<"," << vehiclePosition[1] << std::endl;
        if (step % 8) {
            u_t(1) = 0.1;
        }
        if (step % 2) {
            u_t(1) = -0.5;
        }
        if (step % 5) {
            u_t(1) = -0.3;
        }
        filter.particleMotionUpdate(resampled, u_t);
        bool resample = false;
        for (int j = 0; j < landmarks.size(); j++) {
            double deltaX = landmarks[j][0] - vehiclePosition[0];
            double deltaY = landmarks[j][1] - vehiclePosition[1];
            double q = pow(deltaX, 2) + pow(deltaY, 2);
            double r = std::sqrt(q);
            double theta1 = wrapAngle(atan2(deltaY, deltaX) - vehiclePosition[2]);

            

    
            Vector2d z_t(r, theta1);

            if ( r <= 10.0) {
                filter.particleWeightUpdate(resampled, z_t);
                resample = true;
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
                double theta_new = wrapAngle(theta + w_t);
                vehiclePosition[0] += (v_t / w_t) * (std::sin(theta_new) - std::sin(theta));
                vehiclePosition[1] += (v_t / w_t) * (std::cos(theta_new) - std::cos(theta));
                vehiclePosition[2] = theta_new;
            }
        // filter.particlePurgeLandmarks(resampled);
        if (resample) {
            resampled = filter.particleWeightResampling(resampled);
        }
        // Log particles and their landmark estimates
        int particle_id = 0;
        std::cout << "TOTAL PARTICLE: " << resampled.size() << std::endl;
        for (auto &p : resampled) {
            std::vector<Vector2d> l_i_r = p.landmarksInRange(10.0);
            std::cout << "IN RANGE: " <<  (l_i_r.size()) << std::endl;
            file << step << ",particle," << p.getState()[0] << "," << p.getState()[1] << "," << p.getState()[2] << "," << particle_id << ",," << p.getWeight() << "\n";
            // Log landmark estimates for this particle
            auto landmark_estimates = p.getLandmarks(); // Assumes vector<Vector2d>
            int landmark_id = 0;
            for (auto& est : landmark_estimates) {
                file << step << ",particle_landmark," << est[0] << "," << est[1] << ",0," << particle_id << "," << landmark_id  << "," << p.getWeight() << "\n";
                landmark_id++;
            }
            particle_id++;
            // printKDTree(p.tree);
            // std::cout << std::endl;
            // std::cout << "particle: " << particle_id << " landmakrs: " << p.getLandmarks().size() << std::endl;
        }

        // Log vehicle
        file << step << ",vehicle," << vehiclePosition[0] << "," << vehiclePosition[1] << "," << vehiclePosition[2] << ",,," << "\n";

        // Log landmark (no theta)
        for (int j = 0; j < landmarks.size(); j++) {
        file << step << ",true_landmark," << landmarks[j][0] << "," << landmarks[j][1] << ",0,,0,\n";
        }
        // std::cout << "-----------------------------------------------------------------" << std::endl;
        // std::cout << "-----------------------------------------------------------------" << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
        
    }
        file.close();
        return 0;
}
