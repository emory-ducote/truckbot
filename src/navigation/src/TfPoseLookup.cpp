#include "TfPoseLookup.h"

#include <cmath>

namespace navigation {

namespace {

VehiclePose toPlanarPose(const geometry_msgs::msg::Transform& t)
{
    const auto& q = t.rotation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return VehiclePose(t.translation.x, t.translation.y, yaw);
}

}

TfPoseLookup::TfPoseLookup(rclcpp::Node& node, const TfPoseLookupConfig& config)
    : node(node), config(config)
{
    tfBuffer = std::make_shared<tf2_ros::Buffer>(node.get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
}

std::optional<VehiclePose> TfPoseLookup::lookup()
{
    // Look up each link at its own latest time and compose them. A single global->base
    // lookup at TimePointZero would evaluate odom->base at the (old) particle filter
    // stamp, freezing the pose between particle filter updates.
    geometry_msgs::msg::TransformStamped globalOdom, odomBase;
    try {
        globalOdom = tfBuffer->lookupTransform(config.globalFrame, config.odomFrame, tf2::TimePointZero);
        odomBase = tfBuffer->lookupTransform(config.odomFrame, config.baseFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                             "%s unavailable: %s", chainName().c_str(), e.what());
        return std::nullopt;
    }

    // Per-link lookups never reject an old correction, so check that localization is still alive
    double correctionAge = (node.now() - rclcpp::Time(globalOdom.header.stamp)).seconds();
    if (correctionAge > config.maxCorrectionAge) {
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 2000,
                             "%s->%s is %.1fs old", config.globalFrame.c_str(), config.odomFrame.c_str(), correctionAge);
        return std::nullopt;
    }

    return toPlanarPose(globalOdom.transform).compose(toPlanarPose(odomBase.transform));
}

std::string TfPoseLookup::chainName() const
{
    return config.globalFrame + "->" + config.baseFrame;
}

}
