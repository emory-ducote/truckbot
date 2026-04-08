#ifndef PATHTRACKER_H_
#define PATHTRACKER_H_

#include <vector>
#include "VehiclePose.h"

class PathTracker {
    public:
        PathTracker();
        ~PathTracker();
        void setVehiclePose(const navigation::VehiclePose& pose);
        void setGlobalPath(const std::vector<navigation::VehiclePose>& path);
        const navigation::VehiclePose& getVehiclePose() const;
        const std::vector<navigation::VehiclePose>& getGlobalPath() const;
        int findNearestPose() const;

    private:
        navigation::VehiclePose vehiclePose;
        std::vector<navigation::VehiclePose> globalPath;
};

#endif
