#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Landmark.h"
#include "PersistentKDTree.h"

using namespace Eigen;

class Particle {
    public:
        Particle(Vector3d x=Vector3d::Zero(),
                 Matrix3d P=Matrix3d::Identity(3, 3) * 1e-3) :
                 x(x),
                 P(P) {};

        ~Particle() = default;

        const Vector3d& getState() { return x; }
        
        void setState(const Vector3d newX) { x = newX; }
        
        const Matrix3d& getCovariance() { return P; }

        void setCovariance(const Matrix3d newP) { P = newP; }

        // size_t getLandmarkCount() const { return landmarks.size(); }
        
        // const std::vector<Landmark> getLandmarks() { return landmarks; }
        
        void removeLandmark(Landmark& landmark) 
        {
            Vector2d point = landmark.getState();
            double points[2] = {point(0), point(1)};
            tree = deleteNode(tree, points);
            // landmarks.erase(landmarks.begin() + index);
        }

        void addLandmark(const Landmark& landmark) { 
            Vector2d point = landmark.getState();
            Matrix2d cov = landmark.getCovariance();
            double points[2] = {point(0), point(1)};
            tree = insert(tree, points, cov); 
        }

        void updateLandmark(Landmark& oldLandmark, Landmark& newLandmark) 
        {
            removeLandmark(oldLandmark);
            addLandmark(newLandmark);
        }

        double[2] searchLandmark(Landmark& landmark)
        {
            Vector2d point = landmark.getState();
            double points[2] = {point(0), point(1)};
            const Node * best = findNearest(tree, points);
            if (best == nullptr) {
                return 
            }
        }

        double getWeight() const { return weight; }

        void setWeight(double newWeight) { weight = newWeight; }

        // std::vector<int>& getSeenLandmarks() { return seenLandmarks; }

        // void addSeenLandmark(const int index) { seenLandmarks.push_back(index) ; }

        // void clearSeenLandmarks() { seenLandmarks.clear(); }
    private:
        Vector3d x;
        Matrix3d P;
        std::shared_ptr<const Node> tree = nullptr;
        // std::vector<Landmark> landmarks;
        // std::vector<int> seenLandmarks;
        double weight = 1.0;
};

#endif