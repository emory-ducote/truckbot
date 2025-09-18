#ifndef LANDMARK_H_
#define LANDMARK_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

class Landmark {
    public:
        Landmark(Vector2d x=Vector2d::Zero(),
                 MatrixXd P=Matrix2d::Identity(2, 2) * 1e-3,
                 int count=1) :
                 x(x),
                 P(P),
                 count(count) {}
                 
        ~Landmark() = default;

        const Vector2d& getState() { return x; }

        void setState(const Vector2d& newX) { x = newX; }

        const Matrix2d& getCovariance() { return P; }

        void setCovariance(const MatrixXd newP) { P = newP; }
        
        void sawLandmark() { count += 1; }

        void missedLandmark() { count -= 1; }

        int getCount() const { return count; }

        void setCount(const int newCount) { count = newCount; }
    private:
        Vector2d x;
        Matrix2d P;
        int count;
};

#endif