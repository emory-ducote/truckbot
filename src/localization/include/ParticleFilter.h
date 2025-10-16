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
                       const double frequency = 20,
                       const int newParticleIncrease = 1);
        ~ParticleFilter();

        struct LikelihoodResult {
            double weight;
            Eigen::Vector2d z_hat;
            Eigen::Matrix2d H;
            Eigen::Matrix2d Q;
            Eigen::Vector2d prevX;
            Eigen::Matrix2d prevP;
        };

        std::vector<Particle> getParticles() { return particles;}

        void sampleNewParticlePose(Particle& particle, const Vector2d& u_t, double dt);
        bool landmarkInRange(const Vector3d& state, const Vector2d& landmarkState);
        LikelihoodResult updateLikelihoodCorrespondence(Particle& particle, const Vector2d& z_t);
        void landmarkUpdate(Particle& particle, const Vector2d& z_t);
        std::vector<int> systematicResample(const std::vector<double>& weights);
        void particleMotionUpdate(std::vector<Particle>& particles, const Vector2d& u_t);
        void particleWeightUpdate(std::vector<Particle>& particles, const Vector2d& z_t);
        void particlePurgeLandmarks(std::vector<Particle>& particles);
        std::vector<Particle> particleWeightResampling(std::vector<Particle>& particles);
    private:
        const int numParticles;
        std::vector<Particle> particles;

        const double frequency;
        const double dt = 1 / frequency;
        Matrix2d Q_t; 
        const double p_0 = 1e-2;
        const int newParticleIncrease;
        Vector3d initialSigmas;
        
        
};

#endif