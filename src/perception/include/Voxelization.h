#ifndef VOXELIZATION_H_
#define VOXELIZATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>


class Voxelization {
    public:
        Voxelization(const double voxelSize = 0.25);
        ~Voxelization();
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxelizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn);
    private:
        const double voxelSize;      
};

#endif
