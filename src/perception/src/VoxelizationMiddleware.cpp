#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Voxelization.h"


class VoxelizationMiddleware : public rclcpp::Node
{
public:
  VoxelizationMiddleware() : Node("voxelization_middleware")
  {
    double voxelSize = this->declare_parameter<double>("voxel_size", 0.05);
    // Create a subscription to /scan topic
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
                                                                           10,                           
                                                                          std::bind(&VoxelizationMiddleware::scanCallback, this, std::placeholders::_1));
    cluster_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cluster_markers", 10);

    RCLCPP_INFO(this->get_logger(), "Using voxel size of %f meters", voxelSize);
  
    voxelization = std::make_shared<Voxelization>(voxelSize);
  }

  
void publishClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;

  for (const auto& point : cloud->points) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser";  // set your frame
    marker.header.stamp = this->now();
    marker.ns = "cloud_points";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Scale of each sphere (size of point)
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // Random color per point
    marker.color.r = static_cast<float>(rand()) / RAND_MAX;
    marker.color.g = static_cast<float>(rand()) / RAND_MAX;
    marker.color.b = static_cast<float>(rand()) / RAND_MAX;
    marker.color.a = 1.0;

    // Position of point
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;

    marker_array.markers.push_back(marker);
  }
  RCLCPP_INFO(this->get_logger(), "Sending %d points", marker_array.markers.size());

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
    auto cloudOut = voxelization->voxelizeCloud(cloudPtr);

    publishClusters(cloudOut);
  }

  laser_geometry::LaserProjection projector_;  // LaserScan to PointCloud2
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_pub_;
  std::shared_ptr<Voxelization> voxelization;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelizationMiddleware>());
  rclcpp::shutdown();
  return 0;
}
