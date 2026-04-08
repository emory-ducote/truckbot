#ifndef SPLINEGENERATOR_H_
#define SPLINEGENERATOR_H_

#include <vector>
#include "VehiclePose.h"

class SplineGenerator {
    public:
        SplineGenerator();
        ~SplineGenerator();
        std::vector<navigation::VehiclePose> generateSpline();
    private:
        int nearestPose;
        std::vector<navigation::VehiclePose> globalPath;
        double sampleScale = 2.0;
};

#endif
