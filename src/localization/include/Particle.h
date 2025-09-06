#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Landmark.h"

using namespace Eigen;

class Particle {
    public:
        Particle(Vector3d x=Vector3d::Zero(),
                 MatrixXd P=MatrixXd::Identity(3, 3) * 1e-3) :
                 x(x),
                 P(P) {};
        ~Particle() = default;

        inline Vector3d getState() {
            return x;
        }
        inline MatrixXd getCovariance() {
            return P;
        }
        inline size_t landmarkCount() const {
            return landmarks.size();
        }
        inline std::vector<Landmark> getLandmarks() {
            return landmarks;
        }
        inline void removeLandmark(int index) {
            landmarks.erase(landmarks.begin() + index);
        }
        inline void addLandmark(Landmark landmark) {
            landmarks.push_back(landmark);
        }
        inline void setWeight(double newWeight) {
            weight = newWeight;
        }
        inline double getWeight() {
            return weight;
        }

    private:
        Vector3d x;
        Matrix3d P;
        std::vector<Landmark> landmarks;
        double weight = 1;
};

#endif