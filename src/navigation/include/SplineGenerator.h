#ifndef SPLINEGENERATOR_H_
#define SPLINEGENERATOR_H_

#include <vector>
#include "VehiclePose.h"

class SplineGenerator {
    public:
        SplineGenerator();
        ~SplineGenerator();
        void setGlobalPath(const std::vector<navigation::VehiclePose>& path);
        const std::vector<navigation::VehiclePose>& getGlobalPath() const;
        void setNearestPoseIndex(const int poseIndex);
        int getNearestPoseIndex() const;
        std::vector<navigation::VehiclePose> generateSpline();
    private:
        int nearestPoseIndex;
        std::vector<navigation::VehiclePose> globalPath;
        double sampleScale = 2.0;
};

#endif
