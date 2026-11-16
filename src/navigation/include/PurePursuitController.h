#ifndef PUREPURSUITCONTROLLER_H_
#define PUREPURSUITCONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <vector>
#include "VehiclePose.h"

struct PurePursuitCommand {
    double linear;
    double angular;
};

struct PurePursuitTuning {
    double slowdownAngle = 0.52;       // heading error (rad) where slowing begins
    double minSpeedScale = 0.3;        // fraction of nominal speed at the turn-in-place threshold
    double turnInPlaceAngle = 1.5708;  // heading error (rad) above which we rotate in place
    double turnInPlaceExitAngle = 0.5; // heading error (rad) below which turn in place ends
    double turnInPlaceOmega = 0.6;     // rad/s while rotating in place
};

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
        PurePursuitCommand computeCommand(const navigation::VehiclePose localLookahead, const double v);
        void setTuning(const PurePursuitTuning& tuning);
        const navigation::VehiclePose& calculateLookaheadPoint();
        navigation::VehiclePose localLookahead(const navigation::VehiclePose& globalLookahead);
    private:
        double lookaheadDistance;
        PurePursuitTuning tuning;
        bool turningInPlace = false;
        navigation::VehiclePose vehiclePose;
        std::vector<navigation::VehiclePose> localPath;

};

#endif // PUREPURSUITCONTROLLER_H_