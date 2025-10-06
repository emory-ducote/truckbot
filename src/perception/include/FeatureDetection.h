#ifndef FEATUREDETECTION_H_
#define FEATUREDETECTION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>

class FeatureDetection {
    public:
        FeatureDetection(const double clusterTolerance = 0.05,
                         const double minClusterSize = 2.0,
                         const double maxClusterSize = 10.0 );
        ~FeatureDetection();
        std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn);
    private:
        const double clusterTolerance;
        const double minClusterSize;
        const double maxClusterSize;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> clusterExtractor;
            
        
};

#endif
