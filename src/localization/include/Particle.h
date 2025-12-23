#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Landmark.h"
#include "PersistentKDTree.h"

using namespace Eigen;

struct Particle {
        Particle(Vector3d x=Vector3d::Zero(),
                 Matrix3d P=Matrix3d::Identity(3, 3) * 1e-3) :
                 x(x),
                 P(P) {};

        const std::vector<Vector2d> getLandmarks()
        { 
            std::vector<Vector2d> landmarks;
            collectKDTreeLandmarks(tree, landmarks);
            return landmarks;
        }
        
        void removeLandmark(const Vector2d& point) 
        {
            double points[2] = {point(0), point(1)};
            tree = deleteNode(tree, points);
        }

        void addLandmark(const Landmark& landmark) { 
            Vector2d point = landmark.x;
            Matrix2d cov = landmark.P;
            double points[2] = {point(0), point(1)};
            tree = insert(tree, points, cov); 
        }

        void updateLandmark(Landmark& oldLandmark, Landmark& newLandmark) 
        {
            removeLandmark(oldLandmark.x);
            addLandmark(newLandmark);
        }

        const Node * searchLandmark(double points[2])
        {
            const Node * best = findNearest(tree, points);
            return best;
        }

        std::vector<Vector2d> landmarksInRange(const double maxRange, const double maxAngle)
        {
            std::vector<Vector2d> nearbyLandmarks;
            double target[2] = {x(0), x(1)};
            findNodesWithinThreshold(tree, 
                                     target, 
                                     maxRange, 
                                     nearbyLandmarks, 
                                     x(2), 
                                     maxAngle, // swath in radians (default 180 deg)
                                     0);
            return nearbyLandmarks;
        }

        std::shared_ptr<const Node> tree = nullptr;
        std::vector<Vector2d> seenLandmarks;
        Vector3d x;
        Matrix3d P;
        double weight = 1.0;
};
