#include <numeric>   
#include <random>
#include <iostream>
#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"
#include <execution>
#include "ParticleFilter.h"

using namespace LocalizationHelpers;


ParticleFilter::ParticleFilter(const int numParticles, 
                               const double frequency,
                               const int newParticleIncrease) : 
                               numParticles(numParticles),
                               particles(numParticles),
                               frequency(frequency),
                               newParticleIncrease(newParticleIncrease) {
    spdlog::set_level(spdlog::level::info);
    initialSigmas << 5.0, 5.0, 1.0;
    Q_t << 1e-2, 0, 
            0, 1e-3;
    for (int m = 0; m < numParticles; m++)
    {
        // Random number generator
        std::random_device rd;
        std::mt19937 gen(rd());

        std::normal_distribution<> dist_x(0.0, initialSigmas[0]);
        std::normal_distribution<> dist_y(0.0, initialSigmas[1]);
        std::normal_distribution<> dist_theta(0.0, initialSigmas[2]);

        // Add noise

        Vector3d noisy_state(dist_x(gen), dist_y(gen), dist_theta(gen));
        particles[m] = Particle(noisy_state);
    }
}

ParticleFilter::~ParticleFilter(){}

void ParticleFilter::sampleNewParticlePose(Particle& particle, const Vector2d& u_t, double dt) 
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::normal_distribution<> dist_v(0.0, 0.5);
    std::normal_distribution<> dist_w(0.0, 0.5);


    double v_t = u_t[0] + dist_v(gen);   // linear velocity (m/s)
    double w_t = u_t[1] + dist_w(gen);   // angular velocity (rad/s)
    double theta = particle.getState()[2];

    double x, y, theta_new;


    if (std::abs(w_t) < 1e-6) {
        // Straight line motion
        x = particle.getState()[0] + v_t * dt * std::cos(theta);
        y = particle.getState()[1] + v_t * dt * std::sin(theta);
        theta_new = theta;
    } else {
        // Rotation + translation (unicycle model)
        x = particle.getState()[0] + (v_t / w_t) * (std::sin(theta + w_t * dt) - std::sin(theta));
        y = particle.getState()[1] - (v_t / w_t) * (std::cos(theta + w_t * dt) - std::cos(theta)); // <-- minus sign
        theta_new = wrapAngle(theta + w_t * dt);
    }

    particle.setState(Vector3d(x, y, theta_new));
}



bool ParticleFilter::landmarkInRange(const Vector3d& state, const Vector2d& landmarkState)
{
    const double maxRange = 3.0;
    const double maxAngle = 110.0 * M_PI / 180.0; // radians

    double dx = landmarkState[0] - state[0];
    double dy = landmarkState[1] - state[1];
    double r = std::sqrt(dx * dx + dy * dy);
    if (r > maxRange) return false;

    double bearing_global = std::atan2(dy, dx);
    double relative_bearing = wrapAngle(bearing_global - state[2]);

    return (std::abs(relative_bearing) <= maxAngle);
}

ParticleFilter::LikelihoodResult ParticleFilter::updateLikelihoodCorrespondence(Particle& particle, const Vector2d& z_t)
{
    LikelihoodResult result;

    double mapX = particle.getState()[0] + z_t[0] * cos(particle.getState()[2] + z_t[1]);
    double mapY = particle.getState()[1] + z_t[0] * sin(particle.getState()[2] + z_t[1]);
    double points[2] = {mapX, mapY};
    const Node * nearest = particle.searchLandmark(points);
    if (nearest == nullptr) {
        spdlog::debug("No nearest!");
        result.weight = 1e-9; 
        result.z_hat.setZero();
        result.H.setZero();
        result.Q.setIdentity();
        result.prevX.setZero();
        result.prevP.setIdentity();
        return result;
    }
    
    double deltaX = mapX - particle.getState()[0];
    double deltaY = mapY - particle.getState()[1];
    double q = pow(deltaX, 2) + pow(deltaY, 2);
    double r = std::sqrt(q);
    double theta = wrapAngle(atan2(deltaY, deltaX) - particle.getState()[2]);
    
    // measurement prediction
    Vector2d z_hat_j(r, theta);
    
    // measurement jacobian
    Matrix2d H_j;
    H_j << deltaX / r, deltaY / r,
            - deltaY / q, deltaX / q;

    Vector2d innovation = z_t - z_hat_j;
    Matrix2d Q_j;
    double w_j;
    
    if ((innovation(0) > 1.0) || (innovation(1) > 0.3)) {
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

    spdlog::debug("Weight {}", w_j);
    
    
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

    bool newFeature = (p_0 > w);

    if (newFeature) 
    {
        particle.setWeight(w*particle.getWeight());
        spdlog::debug("Adding new landmark");
        double mapX = particle.getState()[0] + z_t[0] * cos(particle.getState()[2] + z_t[1]);
        double mapY = particle.getState()[1] + z_t[0] * sin(particle.getState()[2] + z_t[1]);
        Vector2d mu_j_t(mapX, mapY);
        Matrix2d H_j;
        H_j << cos(particle.getState()[2] + z_t[1]), -z_t[0] * sin(particle.getState()[2] + z_t[1]),
            sin(particle.getState()[2] + z_t[1]),  z_t[0] * cos(particle.getState()[2] + z_t[1]);
        MatrixXd sigma_j_t = H_j * Q_t * H_j.transpose();
        particle.addLandmark(Landmark(mu_j_t, sigma_j_t));
        // particle.addSeenLandmark(j);
    }
    else 
    {
        particle.setWeight(p_0*particle.getWeight());
        spdlog::debug("Updating landmark");
        Landmark oldLandmark(prevX, prevP);
        
        MatrixXd K = oldLandmark.P * H.transpose() * Q.inverse();

        Vector2d mu_j_t = oldLandmark.x + K * (z_t - z_hat);

        MatrixXd sigma_j_t = (MatrixXd::Identity(2,2) - K * H) * oldLandmark.P;
        Landmark newLandmark(mu_j_t, sigma_j_t);

        particle.updateLandmark(oldLandmark, newLandmark);
        // particle.addSeenLandmark(j);
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

void ParticleFilter::particleMotionUpdate(std::vector<Particle>& particles, const Vector2d& u_t)
{
     // update particle poses in parallel
    std::for_each(std::execution::par, particles.begin(), particles.end(),
        [&](Particle& particle) {
            this->sampleNewParticlePose(particle, u_t, dt);
        });
}

void ParticleFilter::particleWeightUpdate(std::vector<Particle>& particles, const Vector2d& z_t)
{
    // main landmark update and weight sum in parallel
    std::for_each(std::execution::par, particles.begin(), particles.end(),
        [&](Particle& particle) {
            this->landmarkUpdate(particle, z_t);
        });
}   

// void ParticleFilter::particlePurgeLandmarks(std::vector<Particle>& particles)
// {
//     std::for_each(std::execution::par, particles.begin(), particles.end(),
//         [&](Particle& particle) {
//             for (int j = 0; j < particle.getLandmarkCount(); j++)
//             {
//                 Landmark landmark = particle.getLandmarks()[j];
//                 bool inRange = landmarkInRange(particle.getState(), landmark.getState());
//                 bool wasSeen = std::find(particle.getSeenLandmarks().begin(),
//                                          particle.getSeenLandmarks().end(), j) 
//                                != particle.getSeenLandmarks().end();

//                 if ((inRange) && (!wasSeen))
//                 {
//                     landmark.missedLandmark();
//                     particle.updateLandmark(j, landmark);
//                     if (particle.getLandmarks()[j].getCount() <= 0) {
//                         particle.removeLandmark(j);
//                         j--;
//                     }
//                 }
//             }
//             particle.clearSeenLandmarks();
//         });
// }

std::vector<Particle> ParticleFilter::particleWeightResampling(std::vector<Particle>& particles)
{
    double weightSum;
    for (int p = 0; p < numParticles; p++) {
        weightSum += particles[p].getWeight();
    }

    // collect and normalize weights
    std::vector<double> normalizedWeights(numParticles);
    for (int p = 0; p < numParticles; p++) {
        normalizedWeights[p] = particles[p].getWeight() / weightSum;
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

    // Set Neff threshold (e.g., half the number of particles)
    double neff_threshold = 0.5 * numParticles;
    std::vector<Particle> resampledParticles;
    if (neff < neff_threshold) {
        std::vector<int> resampledIndices = systematicResample(normalizedWeights);
        resampledParticles.resize(numParticles);
        for (int m = 0; m < numParticles; m++) {
            resampledParticles[m] = particles[resampledIndices[m]];
        }
    } else {
        // No resampling, just return current particles
        resampledParticles = particles;
    }

    // Compute average particle location after resampling
    double avg_x = 0.0, avg_y = 0.0, avg_theta = 0.0;
    for (auto& p : resampledParticles) {
        avg_x += p.getState()[0];
        avg_y += p.getState()[1];
        avg_theta += p.getState()[2];
    }
    int n = resampledParticles.size();
    if (n > 0) {
        avg_x /= n;
        avg_y /= n;
        avg_theta /= n;
    }
    // std::cout << "Average particle location after resampling: x=" << avg_x << ", y=" << avg_y << ", theta=" << avg_theta << std::endl;

    return resampledParticles;

}
    



