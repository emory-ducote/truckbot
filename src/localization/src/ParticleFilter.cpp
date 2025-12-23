#include <numeric>   
#include <random>
#include <iostream>
#include <spdlog/spdlog.h>
#include <execution>
#include "spdlog/fmt/ostr.h"
#include "ParticleFilter.h"

using namespace LocalizationHelpers;


ParticleFilter::ParticleFilter(const int numParticles, 
                               const int newParticleIncrease,
                               const double maxRange,
                               const double maxAngle,
                               const double newParticleThreshold,
                               const double neffThreshold,
                               const double measurementNoiseRange,
                               const double measurementNoiseBearing,
                               const double linearVelocityAlpha1,
                               const double linearVelocityAlpha2,
                               const double angularVelocityAlpha1,
                               const double angularVelocityAlpha2) : 
                               numParticles(numParticles),
                               particles(numParticles),
                               newParticleIncrease(newParticleIncrease),
                               maxRange(maxRange),
                               maxAngle(maxAngle),
                               newParticleThreshold(newParticleThreshold),
                               neffThreshold(neffThreshold),
                               measurementNoiseRange(measurementNoiseRange),
                               measurementNoiseBearing(measurementNoiseBearing),
                               linearVelocityAlpha1(linearVelocityAlpha1),
                               linearVelocityAlpha2(linearVelocityAlpha2),
                               angularVelocityAlpha1(angularVelocityAlpha1),
                               angularVelocityAlpha2(angularVelocityAlpha2) {
    spdlog::set_level(spdlog::level::debug);
    initialSigmas << 0.0, 0.0, 0.0;

    Q_t << measurementNoiseRange, 0, 
           0, measurementNoiseBearing;
    for (int m = 0; m < numParticles; m++)
    {
        std::random_device rd;
        std::mt19937 gen(rd());

        std::normal_distribution<> dist_x(0.0, initialSigmas[0]);
        std::normal_distribution<> dist_y(0.0, initialSigmas[1]);
        std::normal_distribution<> dist_theta(0.0, initialSigmas[2]);

        Vector3d noisy_state(dist_x(gen), dist_y(gen), dist_theta(gen));

        particles[m] = Particle(noisy_state);
    }

}

ParticleFilter::~ParticleFilter(){}

void ParticleFilter::sampleNewParticlePose(Particle& particle, const Vector2d& u_t, const double dt) 
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> standardNormal(0.0, 1.0);

    double linearVelocity = u_t[0];     // control input linear velocity (m/s)
    double angularVelocity = u_t[1];    // control input angular velocity (rad/s)
    
    // Apply 4-alpha noise model
    double linearVelStd = std::sqrt(linearVelocityAlpha1 * std::abs(linearVelocity) + linearVelocityAlpha2 * std::abs(angularVelocity));
    double angularVelStd = std::sqrt(angularVelocityAlpha1 * std::abs(linearVelocity) + angularVelocityAlpha2 * std::abs(angularVelocity));
    
    double noisyLinearVel = linearVelocity + linearVelStd * standardNormal(gen);
    double noisyAngularVel = angularVelocity + angularVelStd * standardNormal(gen);
    
    double theta = particle.x[2];
    double x, y, theta_new;

    if (std::abs(noisyAngularVel) < 1e-6) {
        // Straight line motion
        x = particle.x[0] + noisyLinearVel * dt * std::cos(theta);
        y = particle.x[1] + noisyLinearVel * dt * std::sin(theta);
        theta_new = theta;
    } else {
        // Rotation + translation (unicycle model)
        theta_new = (wrapAngle(theta + noisyAngularVel * dt));
        x = particle.x[0] + (noisyLinearVel / noisyAngularVel) * (std::sin(theta_new) - std::sin(theta));
        y = particle.x[1] - (noisyLinearVel / noisyAngularVel) * (std::cos(theta_new) - std::cos(theta)); 
    }
    particle.x = Vector3d(x, y, theta_new);
}

ParticleFilter::LikelihoodResult ParticleFilter::updateLikelihoodCorrespondence(Particle& particle, const Vector2d& z_t)
{
    LikelihoodResult result;
    double global_angle = wrapAngle(particle.x[2] + z_t[1]);
    double mapX = particle.x[0] + z_t[0] * cos(global_angle);
    double mapY = particle.x[1] + z_t[0] * sin(global_angle);
    double points[2] = {mapX, mapY};
    const Node * nearest = particle.searchLandmark(points);
    if (nearest == nullptr) {
        spdlog::debug("Measurement {} No nearest!", z_t);
        result.weight = 1e-9; 
        result.z_hat.setZero();
        result.H.setZero();
        result.Q.setIdentity();
        result.prevX.setZero();
        result.prevP.setIdentity();
        return result;
    }
    
    double deltaX = nearest->point[0] - particle.x[0] + 1e-9;
    double deltaY = nearest->point[1] - particle.x[1] + 1e-9;
    double q = pow(deltaX, 2) + pow(deltaY, 2);
    double r = std::sqrt(q);
    double theta = wrapAngle(atan2(deltaY, deltaX) - particle.x[2]);
    
    // measurement prediction
    Vector2d z_hat_j(r, theta);
    
    // measurement jacobian
    Matrix2d H_j;
    H_j << deltaX / r, deltaY / r,
            - deltaY / q, deltaX / q;

    Vector2d innovation = z_t - z_hat_j;
    innovation(1) = wrapAngle(innovation(1));
    Matrix2d Q_j;
    double w_j;
    
    if (std::abs(innovation(0)) > 1.0 || std::abs(innovation(1)) > 0.3) {
        w_j = 1e-9;
        Q_j = Matrix2d::Identity() * 10;
    }
    else {
        // measurement covariance
        Q_j = H_j * nearest->P * H_j.transpose() + Q_t;

        // small perturbation may make things more stable
        Q_j += 1e-9 * Matrix2d::Identity();

        double exponent = -0.5 * innovation.transpose() * Q_j.inverse() * innovation;
        double normalizer = std::pow(2 * M_PI, z_t.size() / 2.0) * std::sqrt(Q_j.determinant());
        w_j = std::exp(exponent) / normalizer;
    }
    
    result.weight = w_j;
    result.z_hat = z_hat_j;
    result.H = H_j;
    result.Q = Q_j;
    result.prevX = Vector2d(nearest->point[0], nearest->point[1]);
    Matrix2d P = nearest->P;
    result.prevP = P;

    return result;
}


void ParticleFilter::landmarkUpdate(Particle& particle, const Vector2d& z_t) 
{

    auto [w, z_hat, H, Q, prevX, prevP] = this->updateLikelihoodCorrespondence(particle, z_t);

    bool newFeature = (newParticleThreshold > w);
    // spdlog::debug("New Feature {}", newFeature);

    if (newFeature) 
    {
        particle.weight = newParticleThreshold*particle.weight;
        double global_angle = wrapAngle(particle.x[2] + z_t[1]);
        double mapX = particle.x[0] + z_t[0] * cos(global_angle);
        double mapY = particle.x[1] + z_t[0] * sin(global_angle);
        Vector2d mu_j_t(mapX, mapY);
        Matrix2d H_j;
        H_j << cos(particle.x[2] + z_t[1]), -z_t[0] * sin(particle.x[2] + z_t[1]),
            sin(particle.x[2] + z_t[1]),  z_t[0] * cos(particle.x[2] + z_t[1]);
        MatrixXd sigma_j_t = H_j * Q_t * H_j.transpose();
        particle.addLandmark(Landmark(mu_j_t, sigma_j_t));
        particle.seenLandmarks.push_back(mu_j_t);
    }
    else 
    {
        particle.weight = w*particle.weight;
        Landmark oldLandmark(prevX, prevP);
        MatrixXd K = oldLandmark.P * H.transpose() * Q.inverse();
        Vector2d innovation = z_t - z_hat;
        innovation(1) = wrapAngle(innovation(1));
        Vector2d mu_j_t = oldLandmark.x + K * innovation;
        MatrixXd sigma_j_t = (MatrixXd::Identity(2,2) - K * H) * oldLandmark.P;
        Landmark newLandmark(mu_j_t, sigma_j_t);
        particle.updateLandmark(oldLandmark, newLandmark);
        particle.seenLandmarks.push_back(mu_j_t);
    }

}

std::vector<int> ParticleFilter::systematicResample(const std::vector<double>& weights) {
    int M = weights.size();
    std::vector<int> indexes(M);

    // Build cumulative distribution (CDF)
    std::vector<double> cdf(M);
    cdf[0] = weights[0];
    for (int i = 1; i < M; i++) {
        cdf[i] = cdf[i - 1] + weights[i];
    }

    // Random offset in [0, 1/M)
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0 / M);
    double u0 = dist(gen);

    // Walk along CDF with equally spaced thresholds
    int i = 0;
    for (int m = 0; m < M; m++) {
        double u = u0 + (double)m / M;
        while (u > cdf[i]) {
            i++;
        }
        indexes[m] = i;
    }

    return indexes;
}

void ParticleFilter::particleMotionUpdate(const Vector2d& u_t, const double dt)
{
    std::for_each(std::execution::par, particles.begin(), particles.end(),
        [&](Particle& particle) {
            this->sampleNewParticlePose(particle, u_t, dt);
        });
}

void ParticleFilter::particleWeightUpdate(const Vector2d& z_t)
{
    std::for_each(std::execution::par, particles.begin(), particles.end(),
        [&](Particle& particle) {
            this->landmarkUpdate(particle, z_t);
        });
}   

void ParticleFilter::particlePurgeLandmarks()
{
    std::for_each(std::execution::par, particles.begin(), particles.end(),
        [&](Particle& particle) {
            std::vector<Vector2d> landmarksInRange = particle.landmarksInRange(maxRange, maxAngle);
            std::vector<Vector2d>& seenLandmarks = particle.seenLandmarks;
            std::vector<Vector2d> inRangeButNotSeen;
            for (const auto& lm : landmarksInRange) {
                bool seen = false;
                for (const auto& seenLm : seenLandmarks) {
                    if ((lm - seenLm).norm() < 1e-6) { 
                        seen = true;
                        break;
                    }
                }
                if (!seen) {
                    inRangeButNotSeen.push_back(lm);
                }
            }
            for (auto& nS: inRangeButNotSeen) 
            {
                particle.removeLandmark(nS);
            }
            particle.seenLandmarks.clear();
        });
}

std::vector<Particle> ParticleFilter::particleWeightResampling()
{
    spdlog::debug("Started Resampling");

    double weightSum;
    for (int p = 0; p < numParticles; p++) {
        weightSum += particles[p].weight;
    }

    // collect and normalize weights
    std::vector<double> normalizedWeights(numParticles);
    for (int p = 0; p < numParticles; p++) {
        normalizedWeights[p] = particles[p].weight / weightSum;
    }

    // Compute Neff (effective number of particles)
    double neff = 0.0;
    double sum_sq = 0.0;
    for (int p = 0; p < numParticles; p++) {
        sum_sq += normalizedWeights[p] * normalizedWeights[p];
    }
    if (sum_sq > 0.0) {
        neff = 1.0 / sum_sq;
    } else {
        neff = 0.0;
    }

    spdlog::info("NEFF: {}", neff);

    spdlog::debug("Started Resampling");


    // Set Neff threshold (e.g., half the number of particles)
    double neff_threshold = neffThreshold * numParticles;
    std::vector<Particle> resampledParticles;
    if (neff < neff_threshold) {
        spdlog::debug("NEFF BELOW THRESHOLD");
        std::vector<int> resampledIndices = systematicResample(normalizedWeights);
        spdlog::debug("SYSTEMATIC RESAMPLED");
        resampledParticles.resize(numParticles);
        spdlog::debug("RESIZED");
        for (int m = 0; m < numParticles; m++) {
            spdlog::debug("ABOUT TO RESAMPLE {}", m);
            spdlog::debug("PICKED: {}", particles[resampledIndices[m]].x);
            resampledParticles[m] = particles[resampledIndices[m]];
            spdlog::debug("RESAMPLED {}", m);
        }

    } else {
        // No resampling, just return current particles
        resampledParticles = particles;
    }

    particles = resampledParticles;


    spdlog::debug("Started Resampling");

    return particles;

}
    
std::vector<Particle> ParticleFilter::particleFilterLoop(const Vector2d& u_t, std::vector<Vector2d> z_t_s, const double dt)
{
    spdlog::debug("Loop Start");
    particleMotionUpdate(u_t, dt);
    spdlog::debug("Motion Updated");
    for (const auto& z_t: z_t_s)
    {
        if (z_t(0) <= maxRange) {
            particleWeightUpdate(z_t);
        }
    }
    spdlog::debug("Measurements Updated");
    particlePurgeLandmarks();
    spdlog::debug("Landmarks Purged");
    return particleWeightResampling();
}