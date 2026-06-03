#ifndef PUREPURSUITCONTROLLER_H_
#define PUREPURSUITCONTROLLER_H_

#include <cmath>
#include <vector>
#include "VehiclePose.h"

class PurePursuitController {
    public:
        PurePursuitController(double lookaheadDistance);
        ~PurePursuitController();
        void setLookaheadDistance(const double distance);
        double getLookaheadDistance() const;
        void setGlobalPath(const std::vector<navigation::VehiclePose>& path);
        void setVehiclePose(const navigation::VehiclePose& pose);
        const navigation::VehiclePose& getVehiclePose() const;
        void setLocalPath(const std::vector<navigation::VehiclePose>& path);
        const std::vector<navigation::VehiclePose>& getLocalPath() const;
        double computeControl(const navigation::VehiclePose localLookahead, const double v);
        const navigation::VehiclePose& calculateLookaheadPoint();
        navigation::VehiclePose localLookahead(const navigation::VehiclePose& globalLookahead);
    private:
        double lookaheadDistance;
        navigation::VehiclePose vehiclePose;
        std::vector<navigation::VehiclePose> localPath;

};

#endif // PUREPURSUITCONTROLLER_H_