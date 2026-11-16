#ifndef TF_POSE_LOOKUP_H_
#define TF_POSE_LOOKUP_H_

#include <memory>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "VehiclePose.h"

namespace navigation {

struct TfPoseLookupConfig {
    std::string globalFrame = "map";
    std::string odomFrame = "odom";
    std::string baseFrame = "base_link";
    double maxCorrectionAge = 3.0;  // seconds a global->odom correction stays valid
};

// Vehicle pose in the global frame from TF: global->odom (particle filter) composed
// with odom->base (EKF), so it moves smoothly between particle filter updates.
class TfPoseLookup {
    public:
        TfPoseLookup(rclcpp::Node& node, const TfPoseLookupConfig& config);
        // nullopt if either link is unavailable or the global->odom correction is stale
        std::optional<VehiclePose> lookup();
        // e.g. "map->base_link", for health details
        std::string chainName() const;

    private:
        rclcpp::Node& node;
        TfPoseLookupConfig config;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
};

}

#endif
