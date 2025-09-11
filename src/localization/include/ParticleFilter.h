#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include "Particle.h"
#include "Landmark.h"


using namespace Eigen;


inline double wrapAngle(double angle) {
    angle = std::fmod(angle + M_PI, 2 * M_PI);  // shift by +π, wrap modulo 2π
    if (angle < 0)
        angle += 2 * M_PI;                       // ensure it's positive
    return angle - M_PI;                          // shift back to [-π, π]
}

class ParticleFilter {
    public:
        ParticleFilter(const int numParticles = 100, const int frequency = 20);
        ~ParticleFilter();
        std::vector<Particle> getParticles() { return particles;}

        void sampleNewParticlePose(Particle& particle, Vector2d& u_t, double dt);
        bool landmarkInRange(const Vector3d& state, const Vector2d& landmarkState);
        void updateLikelihoodCorrespondence(Particle& particle, 
                                            Vector2d& z_t, 
                                            std::vector<double>& weights,
                                            std::vector<Vector2d>& z_hats,
                                            std::vector<Matrix2d>& Hs,
                                            std::vector<MatrixXd>& Qs);
        void landmarkUpdate(Particle& particle, Vector2d& z_t, Vector2d& u_t);
        std::vector<int> systematicResample(const std::vector<double>& weights);
        std::vector<Particle> particleUpdate(std::vector<Particle> particles, Vector2d& z_t, Vector2d& u_t);
    private:
        const int numParticles;
        std::vector<Particle> particles;
        const int frequency;
        const double dt = 1 / frequency;
        Matrix2d Q_t; 
        const double p_0 = 0.0001;
        const int newParticleIncrease = 1;
        Vector3d initial_sigmas;

};

#endif