#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "CornerDetector.h"

class CornerDetectorMiddleware : public rclcpp::Node
{
public:
    CornerDetectorMiddleware() : Node("corner_detector_middleware")
    {
        double mapResolution     = declare_parameter<double>("map_resolution",      0.05);
        double mapRange          = declare_parameter<double>("map_range",           6.0);
        int    maxCorners        = declare_parameter<int>   ("max_corners",         100);
        double qualityLevel      = declare_parameter<double>("quality_level",       0.15);
        double minDistance       = declare_parameter<double>("min_distance",        0.3);
        int    blockSize         = declare_parameter<int>   ("block_size",          3);
        bool   useHarrisDetector = declare_parameter<bool>  ("use_harris_detector", false);
        double harrisK           = declare_parameter<double>("harris_k",            0.04);
        int    wallThickness     = declare_parameter<int>   ("wall_thickness",      2);
        double minCornerDistance = declare_parameter<double>("min_corner_distance", 0.5);

        cornerDetector = std::make_shared<CornerDetector>(mapResolution,
                                                      mapRange,
                                                      maxCorners,
                                                      qualityLevel,
                                                      minDistance,
                                                      blockSize,
                                                      useHarrisDetector,
                                                      harrisK,
                                                      wallThickness,
                                                      minCornerDistance);

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&CornerDetectorMiddleware::scanCallback, this, std::placeholders::_1));

        corner_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10);

        RCLCPP_INFO(get_logger(), "Corner detector node ready");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto corners = cornerDetector->extractCorners(*msg);

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto& corner : corners) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "laser";
            m.header.stamp    = msg->header.stamp;
            m.ns              = "corners";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::SPHERE;
            m.action          = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = corner.x();
            m.pose.position.y = corner.y();
            m.pose.position.z = 0.0;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;
            m.color.r = 1.0;
            m.color.g = 0.5;
            m.color.b = 0.0;
            m.color.a = 1.0;
            m.lifetime = rclcpp::Duration::from_seconds(0.15);
            marker_array.markers.push_back(m);
        }

        RCLCPP_DEBUG(get_logger(), "Publishing %zu corners", corners.size());
        corner_pub_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corner_pub_;
    std::shared_ptr<CornerDetector> cornerDetector;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CornerDetectorMiddleware>());
    rclcpp::shutdown();
    return 0;
}
