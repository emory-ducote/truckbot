#include "LineExtraction.h"
#include <cmath>
#include <opencv2/imgproc.hpp>

LineExtraction::LineExtraction(double mapResolution,
                               double mapRange,
                               int    maxCorners,
                               double qualityLevel,
                               double minDistance,
                               int    blockSize,
                               bool   useHarrisDetector,
                               double harrisK,
                               int    wallThickness,
                               double minCornerDistance)
    : mapResolution(mapResolution)
    , mapRange(mapRange)
    , maxCorners(maxCorners)
    , qualityLevel(qualityLevel)
    , minDistance(minDistance)
    , blockSize(blockSize)
    , useHarrisDetector(useHarrisDetector)
    , harrisK(harrisK)
    , wallThickness(wallThickness)
    , minCornerDistance(minCornerDistance)
{}

int LineExtraction::imageSize() const
{
    return std::max(1, static_cast<int>(std::round(2.0 * mapRange / mapResolution)));
}

cv::Mat LineExtraction::rasterizeScan(const sensor_msgs::msg::LaserScan& scan) const
{
    const int    size   = imageSize();
    const double center = size / 2.0;

    cv::Mat occupancy = cv::Mat::zeros(size, size, CV_8UC1);

    float angle = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
        float r = scan.ranges[i];
        bool valid = !std::isnan(r) && !std::isinf(r)
                     && r >= scan.range_min && r <= scan.range_max;
        if (!valid) continue;

        // Image y grows downward, so flip the laser-frame y axis
        int px = static_cast<int>(std::round(center + r * std::cos(angle) / mapResolution));
        int py = static_cast<int>(std::round(center - r * std::sin(angle) / mapResolution));
        if (px >= 0 && px < size && py >= 0 && py < size)
            occupancy.at<uint8_t>(py, px) = 255;
    }

    if (wallThickness > 0) {
        int k = 2 * wallThickness + 1;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
        cv::dilate(occupancy, occupancy, kernel);
    }

    return occupancy;
}

std::vector<cv::Point2f> LineExtraction::detectFeatures(const cv::Mat& occupancy) const
{
    std::vector<cv::Point2f> features;
    cv::goodFeaturesToTrack(occupancy,
                            features,
                            maxCorners,
                            qualityLevel,
                            minDistance / mapResolution,
                            cv::noArray(),
                            blockSize,
                            useHarrisDetector,
                            harrisK);
    return features;
}

std::vector<Vector2d> LineExtraction::toLaserFrame(const std::vector<cv::Point2f>& features) const
{
    const double center = imageSize() / 2.0;

    std::vector<Vector2d> corners;
    corners.reserve(features.size());
    for (const auto& f : features) {
        Vector2d c((f.x - center) * mapResolution,
                   (center - f.y) * mapResolution);
        if (c.norm() >= minCornerDistance)
            corners.push_back(c);
    }
    return corners;
}

std::vector<Vector2d> LineExtraction::extractCorners(const sensor_msgs::msg::LaserScan& scan)
{
    cv::Mat occupancy              = rasterizeScan(scan);
    std::vector<cv::Point2f> features = detectFeatures(occupancy);
    return toLaserFrame(features);
}
