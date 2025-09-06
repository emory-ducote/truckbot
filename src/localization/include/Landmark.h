#ifndef LANDMARK_H_
#define LANDMARK_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

class Landmark {
    public:
        Landmark(Vector2d x=Vector2d::Zero(),
                 MatrixXd P=MatrixXd::Identity(2, 2) * 1e-3,
                 int counter=1) :
                 x(x),
                 P(P),
                 counter(counter) {}
                 
        ~Landmark() = default;

        inline Vector2d getState() {
            return x;
        }
        inline Matrix2d getCovariance() {
            return P;
        }
        inline void sawLandmark() {
            counter += 1;
        }
        inline void missedLandmark() {
            counter -= 1;
        }
        inline int landmarkCounter() {
            return counter;
        }
        inline void updateLandmark(Vector2d newX, MatrixXd newP, int newCounter) {
            x = newX;
            P = newP;
            counter = newCounter;
        }

    private:
        Vector2d x;
        MatrixXd P;
        int counter;
};

#endif