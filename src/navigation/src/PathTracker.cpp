#include "PathTracker.h"


PathTracker::PathTracker() {

};

PathTracker::~PathTracker() {};

void PathTracker::setVehiclePose(const navigation::VehiclePose& pose)
{
    vehiclePose = pose;
}
        
void PathTracker::setGlobalPath(const std::vector<navigation::VehiclePose>& path)
{
    this->globalPath = path;
}

const navigation::VehiclePose& PathTracker::getVehiclePose() const
{
    return this->vehiclePose;
}

const std::vector<navigation::VehiclePose>& PathTracker::getGlobalPath() const
{
    return this->globalPath;
}

int PathTracker::findNearestPose() const
{
    double minDistance = 1e9; int minIndex = -1;
    for (size_t i = 0; i < globalPath.size(); i++)
    {
        double currentDistance = globalPath[i].euclideanDistanceTo(vehiclePose);
        if (currentDistance < minDistance)
        {
            minDistance = currentDistance;
            minIndex = i;
        }
    } 
    return minIndex;
}
