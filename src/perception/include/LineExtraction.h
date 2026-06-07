#pragma once
#include <vector>
#include <optional>
#include <Eigen/Dense>
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace Eigen;

struct LineSegment {
    Vector2d start;
    Vector2d end;
};

class LineExtraction {
public:
    LineExtraction(double splitThreshold    = 0.05,
                   int    minPointsPerLine  = 6,
                   double minLineLength     = 0.3,
                   double maxExtrapolation  = 0.8,
                   double minCornerAngleDeg = 30.0,
                   double gapThreshold      = 0.5,
                   double minCornerDistance = 0.5);

    // Returns corner positions in the laser frame
    std::vector<Vector2d> extractCorners(const sensor_msgs::msg::LaserScan& scan);

private:
    double splitThreshold_;
    int    minPointsPerLine_;
    double minLineLength_;
    double maxExtrapolation_;
    double minCornerAngle_;     // radians
    double gapThreshold_;
    double minCornerDistance_;  // discard corners closer than this to the sensor origin

    // Break scan into contiguous point arcs (splitting on gaps and invalid returns)
    std::vector<std::vector<Vector2d>> segmentScan(const sensor_msgs::msg::LaserScan& scan);

    // Recursive split phase; appends to result
    void splitRecursive(const std::vector<Vector2d>& pts, int s, int e,
                        std::vector<LineSegment>& result);

    std::vector<LineSegment> splitAndMerge(const std::vector<Vector2d>& pts);

    double perpendicularDistance(const Vector2d& p,
                                 const Vector2d& lineA,
                                 const Vector2d& lineB);

    // Returns intersection if it forms a valid corner, nullopt otherwise
    std::optional<Vector2d> computeCorner(const LineSegment& a, const LineSegment& b);

    // Remove corners closer than dedupRadius to an earlier corner
    std::vector<Vector2d> deduplicate(const std::vector<Vector2d>& corners, double dedupRadius);
};
