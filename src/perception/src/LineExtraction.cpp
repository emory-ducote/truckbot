#include "LineExtraction.h"
#include <cmath>
#include <algorithm>

LineExtraction::LineExtraction(double splitThreshold,
                               int    minPointsPerLine,
                               double minLineLength,
                               double maxExtrapolation,
                               double minCornerAngleDeg,
                               double gapThreshold,
                               double minCornerDistance)
    : splitThreshold_(splitThreshold)
    , minPointsPerLine_(minPointsPerLine)
    , minLineLength_(minLineLength)
    , maxExtrapolation_(maxExtrapolation)
    , minCornerAngle_(minCornerAngleDeg * M_PI / 180.0)
    , gapThreshold_(gapThreshold)
    , minCornerDistance_(minCornerDistance)
{}

std::vector<std::vector<Vector2d>> LineExtraction::segmentScan(const sensor_msgs::msg::LaserScan& scan)
{
    std::vector<std::vector<Vector2d>> arcs;
    std::vector<Vector2d> current;

    float angle = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
        float r = scan.ranges[i];
        bool valid = !std::isnan(r) && !std::isinf(r)
                     && r >= scan.range_min && r <= scan.range_max;

        if (!valid) {
            if (!current.empty()) { arcs.push_back(current); current.clear(); }
            continue;
        }

        Vector2d pt(r * std::cos(angle), r * std::sin(angle));

        if (!current.empty() && (pt - current.back()).norm() > gapThreshold_) {
            arcs.push_back(current);
            current.clear();
        }
        current.push_back(pt);
    }
    if (!current.empty()) arcs.push_back(current);
    return arcs;
}

double LineExtraction::perpendicularDistance(const Vector2d& p,
                                             const Vector2d& a,
                                             const Vector2d& b)
{
    Vector2d ab = b - a;
    double len = ab.norm();
    if (len < 1e-9) return (p - a).norm();
    // |cross(ab, ap)| / len
    return std::abs(ab.x() * (a.y() - p.y()) - (a.x() - p.x()) * ab.y()) / len;
}

void LineExtraction::splitRecursive(const std::vector<Vector2d>& pts, int s, int e,
                                    std::vector<LineSegment>& result)
{
    if (e - s < minPointsPerLine_ - 1) return;

    double maxDist = 0.0;
    int    maxIdx  = s + 1;
    for (int i = s + 1; i < e; ++i) {
        double d = perpendicularDistance(pts[i], pts[s], pts[e]);
        if (d > maxDist) { maxDist = d; maxIdx = i; }
    }

    if (maxDist > splitThreshold_) {
        splitRecursive(pts, s, maxIdx, result);
        splitRecursive(pts, maxIdx, e, result);
    } else {
        double len = (pts[e] - pts[s]).norm();
        if (len >= minLineLength_)
            result.push_back({pts[s], pts[e]});
    }
}

std::vector<LineSegment> LineExtraction::splitAndMerge(const std::vector<Vector2d>& pts)
{
    std::vector<LineSegment> result;
    if (static_cast<int>(pts.size()) < minPointsPerLine_) return result;
    splitRecursive(pts, 0, static_cast<int>(pts.size()) - 1, result);
    return result;
}

std::optional<Vector2d> LineExtraction::computeCorner(const LineSegment& a, const LineSegment& b)
{
    Vector2d d1 = a.end - a.start;
    Vector2d d2 = b.end - b.start;

    double denom = d1.x() * d2.y() - d1.y() * d2.x();
    if (std::abs(denom) < 1e-9) return std::nullopt;  // parallel

    // Angle between the two lines — reject near-parallel
    double cosAngle = std::abs(d1.normalized().dot(d2.normalized()));
    cosAngle = std::min(1.0, cosAngle);
    double acuteAngle = M_PI / 2.0 - std::acos(cosAngle);  // angle from 90°
    if ((M_PI / 2.0 - acuteAngle) < minCornerAngle_) return std::nullopt;

    Vector2d diff = b.start - a.start;
    double t = (diff.x() * d2.y() - diff.y() * d2.x()) / denom;
    Vector2d intersection = a.start + t * d1;

    // Both segments must have an endpoint close to the intersection (no wild extrapolation)
    double distA = std::min((intersection - a.start).norm(), (intersection - a.end).norm());
    double distB = std::min((intersection - b.start).norm(), (intersection - b.end).norm());
    if (distA > maxExtrapolation_ || distB > maxExtrapolation_) return std::nullopt;

    return intersection;
}

std::vector<Vector2d> LineExtraction::deduplicate(const std::vector<Vector2d>& corners,
                                                   double dedupRadius)
{
    std::vector<Vector2d> kept;
    for (const auto& c : corners) {
        bool duplicate = false;
        for (const auto& k : kept) {
            if ((c - k).norm() < dedupRadius) { duplicate = true; break; }
        }
        if (!duplicate) kept.push_back(c);
    }
    return kept;
}

std::vector<Vector2d> LineExtraction::extractCorners(const sensor_msgs::msg::LaserScan& scan)
{
    auto arcs = segmentScan(scan);

    std::vector<LineSegment> allSegments;
    for (const auto& arc : arcs) {
        for (auto& seg : splitAndMerge(arc))
            allSegments.push_back(seg);
    }

    std::vector<Vector2d> corners;
    for (size_t i = 0; i < allSegments.size(); ++i) {
        for (size_t j = i + 1; j < allSegments.size(); ++j) {
            auto corner = computeCorner(allSegments[i], allSegments[j]);
            if (corner && corner->norm() >= minCornerDistance_)
                corners.push_back(*corner);
        }
    }

    return deduplicate(corners, 0.15);
}
