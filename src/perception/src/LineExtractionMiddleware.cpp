#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "LineExtraction.h"

class LineExtractionMiddleware : public rclcpp::Node
{
public:
    LineExtractionMiddleware() : Node("line_extraction_middleware")
    {
        double splitThreshold    = declare_parameter<double>("split_threshold",      0.05);
        int    minPointsPerLine  = declare_parameter<int>   ("min_points_per_line",  6);
        double minLineLength     = declare_parameter<double>("min_line_length",      0.3);
        double maxExtrapolation  = declare_parameter<double>("max_extrapolation",    0.8);
        double minCornerAngleDeg = declare_parameter<double>("min_corner_angle",     30.0);
        double gapThreshold      = declare_parameter<double>("gap_threshold",        0.5);
        double minCornerDistance = declare_parameter<double>("min_corner_distance",  0.5);

        extractor_ = std::make_shared<LineExtraction>(splitThreshold,
                                                      minPointsPerLine,
                                                      minLineLength,
                                                      maxExtrapolation,
                                                      minCornerAngleDeg,
                                                      gapThreshold,
                                                      minCornerDistance);

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LineExtractionMiddleware::scanCallback, this, std::placeholders::_1));

        corner_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10);

        RCLCPP_INFO(get_logger(), "Line extraction node ready");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto corners = extractor_->extractCorners(*msg);

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto& corner : corners) {
            visualization_msgs::msg::Marker m;
            // Use the scan timestamp so the particle filter's dt computation is correct
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
            marker_array.markers.push_back(m);
        }

        RCLCPP_DEBUG(get_logger(), "Publishing %zu corners", corners.size());
        corner_pub_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corner_pub_;
    std::shared_ptr<LineExtraction> extractor_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineExtractionMiddleware>());
    rclcpp::shutdown();
    return 0;
}
