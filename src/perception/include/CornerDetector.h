#pragma once
#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

// Corner extractor backed by OpenCV's goodFeaturesToTrack (Shi-Tomasi / Harris)
// corner detector. The laser scan is rasterized into an occupancy image, corner
// features are detected on that image, then mapped back into the laser frame.
//
// Defaults are tuned for an RPLIDAR A1 (~1 deg angular resolution, 0.15-12 m
// range, 360 deg FOV). Because the A1's returns get sparse with range (~10 cm
// between points at 6 m), wallThickness must be large enough to bridge those
// gaps so walls form continuous structure in the rasterized image.
class CornerDetector {
public:
    CornerDetector(double mapResolution     = 0.05,
                   double mapRange          = 6.0,
                   int    maxCorners        = 100,
                   double qualityLevel      = 0.15,
                   double minDistance       = 0.3,
                   int    blockSize         = 3,
                   bool   useHarrisDetector = false,
                   double harrisK           = 0.04,
                   int    wallThickness     = 2,
                   double minCornerDistance = 0.5);

    // Returns corner positions in the laser frame
    std::vector<Eigen::Vector2d> extractCorners(const sensor_msgs::msg::LaserScan& scan);

private:
    // Side length (px) of the square occupancy image
    int imageSize() const;

    // Rasterize valid scan returns into a thickened occupancy image
    cv::Mat rasterizeScan(const sensor_msgs::msg::LaserScan& scan) const;

    // Run goodFeaturesToTrack on the occupancy image
    std::vector<cv::Point2f> detectFeatures(const cv::Mat& occupancy) const;

    // Map feature pixels back to the laser frame, dropping ones near the origin
    std::vector<Eigen::Vector2d> toLaserFrame(const std::vector<cv::Point2f>& features) const;

    double mapResolution;      // metres per pixel
    double mapRange;           // half-extent (m) of the square rasterized region
    int    maxCorners;         // max corners goodFeaturesToTrack returns
    double qualityLevel;       // minimal accepted quality of corners (0..1)
    double minDistance;        // minimum spacing (m) between returned corners
    int    blockSize;          // neighbourhood size for the corner covariation matrix
    bool   useHarrisDetector;  // use Harris response instead of Shi-Tomasi minimal eigenvalue
    double harrisK;            // Harris free parameter (only used when useHarrisDetector)
    int    wallThickness;      // dilation radius (px) used to thicken rasterized walls
    double minCornerDistance;  // discard corners closer than this to the sensor origin
};
