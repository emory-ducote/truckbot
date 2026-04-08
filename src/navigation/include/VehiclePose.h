#ifndef VEHICLE_POSE_H
#define VEHICLE_POSE_H

#include <cmath>

namespace navigation {

struct VehiclePose {
    double x;
    double y;
    double theta;

    VehiclePose() : x(0.0), y(0.0), theta(0.0) {}
    VehiclePose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

    double euclideanDistanceTo(const VehiclePose& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double angleDifferenceBetween(const VehiclePose& other) const {
        double dtheta = theta - other.theta;
        // Normalize to [-pi, pi]
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;
        return dtheta;
    }
};

} 

#endif // VEHICLE_POSE_H