#include "PurePursuitController.h"


PurePursuitController::PurePursuitController(double lookaheadDistance) : lookaheadDistance(lookaheadDistance) {};

PurePursuitController::~PurePursuitController() {};

void PurePursuitController::setLookaheadDistance(const double distance)
{
    this->lookaheadDistance = distance;
}

double PurePursuitController::getLookaheadDistance() const
{
    return this->lookaheadDistance;
}

void PurePursuitController::setVehiclePose(const navigation::VehiclePose& pose)
{
    this->vehiclePose = pose;
}

const navigation::VehiclePose& PurePursuitController::getVehiclePose() const
{
    return this->vehiclePose;
}

void PurePursuitController::setLocalPath(const std::vector<navigation::VehiclePose>& path)
{
    this->localPath = path;
}

const std::vector<navigation::VehiclePose>& PurePursuitController::getLocalPath() const
{
    return this->localPath;
}

const navigation::VehiclePose& PurePursuitController::calculateLookaheadPoint()
{
    double accumulated = 0.0;
    for (size_t i = 1; i < localPath.size(); ++i) {
        const auto& prev = localPath[i - 1];
        const auto& curr = localPath[i];
        double dx = curr.x - prev.x;
        double dy = curr.y - prev.y;
        accumulated += std::sqrt(dx * dx + dy * dy);
        if (accumulated >= lookaheadDistance) {
            return curr;
        }
    }
    // If lookaheadDistance is longer than the path, return the last point
    return localPath.back();
}

navigation::VehiclePose PurePursuitController::localLookahead(const navigation::VehiclePose& globalLookahead)
{
    return this->getVehiclePose().toBodyFrame(globalLookahead);
}

double PurePursuitController::computeControl(const navigation::VehiclePose localLookahead, const double v)
    {
        double Ld2 = localLookahead.x * localLookahead.x + localLookahead.y * localLookahead.y;

        if (Ld2 == 0.0) {
            return 0.0;
        }

        // curvature
        double kappa = (2.0 * localLookahead.y) / Ld2;

        // angular velocity (unicycle model)
        return v * kappa;
    }