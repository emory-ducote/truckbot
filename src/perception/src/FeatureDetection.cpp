#include "FeatureDetection.h"


FeatureDetection::FeatureDetection(const double clusterTolerance,
                                   const double minClusterSize,
                                   const double maxClusterSize) :
                                   clusterTolerance(clusterTolerance),
                                   minClusterSize(minClusterSize),
                                   maxClusterSize(maxClusterSize) 
{
    clusterExtractor.setClusterTolerance(clusterTolerance);
    clusterExtractor.setMinClusterSize(minClusterSize);
    clusterExtractor.setMaxClusterSize(maxClusterSize);
}

FeatureDetection::~FeatureDetection() {};

std::vector<pcl::PointIndices> FeatureDetection::euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn) 
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloudIn);

    std::vector<pcl::PointIndices> cluster_indices;

    clusterExtractor.setSearchMethod(tree);
    clusterExtractor.setInputCloud(cloudIn);
    clusterExtractor.extract(cluster_indices);

    // Print results
    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        std::cout << "Cluster " << cluster_id
                  << " has " << indices.indices.size() << " points." << std::endl;
        cluster_id++;
    }

    return cluster_indices;
}