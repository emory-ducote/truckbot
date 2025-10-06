#include "Voxelization.h"


Voxelization::Voxelization(const double voxelSize) :
                                   voxelSize(voxelSize) {};

Voxelization::~Voxelization() {};

pcl::PointCloud<pcl::PointXYZ>::Ptr Voxelization::voxelizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(cloudIn);
    voxelFilter.setLeafSize(voxelSize, voxelSize, voxelSize);  // adjust voxel size (meters)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
    voxelFilter.filter(*cloudFiltered);

    return cloudFiltered;
}