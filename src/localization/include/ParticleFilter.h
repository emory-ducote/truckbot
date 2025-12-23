#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include "Particle.h"
#include "Landmark.h"
#include "LocalizationHelpers.h"


using namespace Eigen;

class ParticleFilter {
    public:
        ParticleFilter(const int numParticles = 100, 
                       const int newParticleIncrease = 1,
                       const double maxRange = 5.0,
                       const double maxAngle = 180.0,
                       const double newParticleThreshold = 0.8,
                       const double neffThreshold = 0.6,
                       const double measurementNoiseRange = 0.1,
                       const double measurementNoiseBearing = 0.01,
                       const double linearVelocityAlpha1 = 0.2,
                       const double linearVelocityAlpha2 = 0.05,
                       const double angularVelocityAlpha1 = 0.05,
                       const double angularVelocityAlpha2 = 0.2);
        ~ParticleFilter();

        struct LikelihoodResult {
            double weight;
            Eigen::Vector2d z_hat;
            Eigen::Matrix2d H;
            Eigen::Matrix2d Q;
            Eigen::Vector2d prevX;
            Eigen::Matrix2d prevP;
        };

        void sampleNewParticlePose(Particle& particle, const Vector2d& u_t, const double dt);
        LikelihoodResult updateLikelihoodCorrespondence(Particle& particle, const Vector2d& z_t);
        void landmarkUpdate(Particle& particle, const Vector2d& z_t);
        std::vector<int> systematicResample(const std::vector<double>& weights);
        void particleMotionUpdate(const Vector2d& u_t, const double dt);
        void particleWeightUpdate(const Vector2d& z_t);
        void particlePurgeLandmarks();
        std::vector<Particle> particleWeightResampling();
        std::vector<Particle> particleFilterLoop(const Vector2d& u_t, std::vector<Vector2d> z_t_s, const double dt); 
    private:
        const int numParticles;
        std::vector<Particle> particles;
        const int newParticleIncrease;
        const double maxRange;
        const double maxAngle;
        const double newParticleThreshold;
        const double neffThreshold;
        const double measurementNoiseRange;
        const double measurementNoiseBearing;
        const double linearVelocityAlpha1;
        const double linearVelocityAlpha2;
        const double angularVelocityAlpha1;
        const double angularVelocityAlpha2;
        Matrix2d Q_t; 
        Vector3d initialSigmas;
};

#endif