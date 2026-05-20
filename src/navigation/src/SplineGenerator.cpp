#include "SplineGenerator.h"
#include <iostream>


SplineGenerator::SplineGenerator() {};

SplineGenerator::~SplineGenerator() {};

void SplineGenerator::setGlobalPath(const std::vector<navigation::VehiclePose>& path)
{
    this->globalPath = path;
}

const std::vector<navigation::VehiclePose>& SplineGenerator::getGlobalPath() const
{
    return this->globalPath;
}

void SplineGenerator::setNearestPoseIndex(const int poseIndex)
{
    this->nearestPoseIndex = poseIndex;
}

int SplineGenerator::getNearestPoseIndex() const
{
    return this->nearestPoseIndex;
}

navigation::VehiclePose getPoseClamped(const std::vector<navigation::VehiclePose>& points, int index)
{
    if (index < 0)
        return points[0];
    else if (index >= int(points.size()))
        return points.back();
    else
        return points[index];
}

double cubicHermite(double p0, double p1, double p2, double p3, double t)
{
    double a = -p0/2.0f + (3.0f*p1)/2.0f - (3.0f*p2)/2.0f + p3/2.0f;
    double b = p0 - (5.0f*p1)/2.0f + 2.0f*p2 - p3 / 2.0f;
    double c = -p0/2.0f + p2/2.0f;
    double d = p1;
  
    return a*t*t*t + b*t*t + c*t + d;
}

std::vector<navigation::VehiclePose> SplineGenerator::generateSpline()
{
    std::vector<navigation::VehiclePose> splinePoses;
    if (this->globalPath.empty()) {
        return splinePoses;
    }

    int clampedIndex = this->nearestPoseIndex;
    if (clampedIndex < 0) {
        clampedIndex = 0;
    }
    if (clampedIndex >= int(this->globalPath.size())) {
        clampedIndex = int(this->globalPath.size()) - 1;
    }

    int sampleCount = int(this->globalPath[clampedIndex].euclideanDistanceTo(globalPath.back()) * sampleScale);
    if (sampleCount < 2) {
        sampleCount = 2;
    }

    std::vector<navigation::VehiclePose> relevantPoints(this->globalPath.begin() + clampedIndex, globalPath.end());
    for (int i = 0; i < sampleCount; ++i)
    {
        double percent = ((double)i) / (double(sampleCount - 1));
        double tx = (relevantPoints.size() -1) * percent;
        int index = int(tx);
        double t = tx - floor(tx);

        navigation::VehiclePose a = getPoseClamped(relevantPoints, index - 1);
        navigation::VehiclePose b = getPoseClamped(relevantPoints, index + 0);
        navigation::VehiclePose c = getPoseClamped(relevantPoints, index + 1);
        navigation::VehiclePose d = getPoseClamped(relevantPoints, index + 2);
        double x = cubicHermite(a.x, b.x, c.x, d.x, t);
        double y = cubicHermite(a.y, b.y, c.y, d.y, t);
        splinePoses.push_back(navigation::VehiclePose(x, y, 0.0));
    }
    return splinePoses;
}