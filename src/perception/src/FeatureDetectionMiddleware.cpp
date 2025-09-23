#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "FeatureDetection.h"


class FeatureDetectionMiddleware : public rclcpp::Node
{
public:
  FeatureDetectionMiddleware(std::shared_ptr<FeatureDetection> featureDetection)
  : Node("feature_detection_middleware"), featureDetection(featureDetection)
  {
    // Create a subscription to /scan topic
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
                                                                           10,                           
                                                                          std::bind(&FeatureDetectionMiddleware::scanCallback, this, std::placeholders::_1));
    cluster_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10);
  }

  void publishClusters(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::vector<pcl::PointIndices>& clusters)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto& indices : clusters) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "laser";  // set your frame
      marker.header.stamp = this->now();
      marker.ns = "clusters";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Scale of points
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;

      // Random color
      marker.color.r = static_cast<float>(rand()) / RAND_MAX;
      marker.color.g = static_cast<float>(rand()) / RAND_MAX;
      marker.color.b = static_cast<float>(rand()) / RAND_MAX;
      marker.color.a = 1.0;

      // Fill points
      for (int idx : indices.indices) {
        geometry_msgs::msg::Point pt;
        pt.x = cloud->points[idx].x;
        pt.y = cloud->points[idx].y;
        pt.z = cloud->points[idx].z;
        marker.points.push_back(pt);
      }

      marker_array.markers.push_back(marker);
    }

    cluster_pub_->publish(marker_array);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Convert LaserScan → PointCloud2
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*msg, cloud_msg);

    // Convert PointCloud2 → PCL
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(cloud_msg, cloud);

    RCLCPP_INFO(this->get_logger(), "Converted LaserScan to PCL cloud with %zu points", cloud.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    std::vector<pcl::PointIndices> clusters = featureDetection->euclideanCluster(cloudPtr);

    publishClusters(cloudPtr, clusters);
  }

  laser_geometry::LaserProjection projector_;  // LaserScan to PointCloud2
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;
  std::shared_ptr<FeatureDetection> featureDetection;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto featureDetection = std::make_shared<FeatureDetection>();
  rclcpp::spin(std::make_shared<FeatureDetectionMiddleware>(featureDetection));
  rclcpp::shutdown();
  return 0;
}
