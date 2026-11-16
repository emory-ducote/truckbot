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

    VehiclePose differenceBetween(const VehiclePose& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dtheta = theta - other.theta;
        return VehiclePose(dx, dy, dtheta);
    }

    // Transform a world-frame point into this pose's body frame
    VehiclePose toBodyFrame(const VehiclePose& world) const {
        double dx = world.x - x;
        double dy = world.y - y;
        return VehiclePose(
             dx * std::cos(theta) + dy * std::sin(theta),
            -dx * std::sin(theta) + dy * std::cos(theta),
            0.0
        );
    }

    // Express a pose given in this pose's frame in the parent frame (e.g. map->odom composed with odom->base)
    VehiclePose compose(const VehiclePose& local) const {
        double c = std::cos(theta);
        double s = std::sin(theta);
        return VehiclePose(
            x + c * local.x - s * local.y,
            y + s * local.x + c * local.y,
            std::remainder(theta + local.theta, 2.0 * M_PI)
        );
    }
};

} 

#endif // VEHICLE_POSE_H