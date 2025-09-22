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
    initialSigmas <<5.0, 5.0, 0.5;
    Q_t << 1e-3, 0, 
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
    double v_t = u_t[0];
    double w_t = u_t[1];
    double theta = particle.getState()[2];

    double x, y, theta_new;
    if (std::abs(w_t) < 1e-6) {
        x = particle.getState()[0] + v_t * dt * std::cos(theta);
        y = particle.getState()[1] + v_t * dt * std::sin(theta);
        theta_new = particle.getState()[2];
    } else {
        x = particle.getState()[0] + ((v_t / w_t) * (std::sin(theta + w_t * dt) - std::sin(theta)));
        y = particle.getState()[1] + ((v_t / w_t) * (std::cos(theta + w_t * dt) - std::cos(theta)));
        theta_new = wrapAngle(particle.getState()[2] + wrapAngle(w_t * dt));
    }
    particle.setState(Vector3d(x, y, theta_new));
}


bool ParticleFilter::landmarkInRange(const Vector3d& state, const Vector2d& landmarkState)
{
    // TODO add range and bearing gate
    double deltaX = landmarkState[0] - state[0];
    double deltaY = landmarkState[1] - state[1];
    double q = pow(deltaX, 2) + pow(deltaY, 2);
    double r = std::sqrt(q);
    return (r <= 5.0);
}

void ParticleFilter::updateLikelihoodCorrespondence(Particle& particle, 
                                                    const Vector2d& z_t, 
                                                    std::vector<double>& weights,
                                                    std::vector<Vector2d>& z_hats,
                                                    std::vector<Matrix2d>& Hs,
                                                    std::vector<MatrixXd>& Qs)
{
    size_t landmarkCount = particle.getLandmarkCount();
    spdlog::debug("Updating likelihood of {} landmarks", landmarkCount);

    for (size_t j = 0; j < particle.getLandmarkCount(); j++)
    {
        Landmark landmark = particle.getLandmarks()[j];
        // some useful calculations for forming our prediction
        // and jacobian matrices
        double deltaX = landmark.getState()[0] - particle.getState()[0];
        double deltaY = landmark.getState()[1] - particle.getState()[1];
        double q = pow(deltaX, 2) + pow(deltaY, 2);
        double r = std::sqrt(q);
        double theta = wrapAngle(atan2(deltaY, deltaX) - particle.getState()[2]);
        
        // measurement prediction
        Vector2d z_hat_j(r, theta);
        
        // measurement jacobian
        Matrix2d H_j;
        H_j << deltaX / r, deltaY / r,
             - deltaY / q, deltaX / q;
       
        // measurement covariance
        MatrixXd Q_j = H_j * landmark.getCovariance() * H_j.transpose() + Q_t;

        // small perturbation may make things more stable
        Q_j += 1e-9 * Matrix2d::Identity();

        // likelihood of correspondence
        Vector2d innovation = z_t - z_hat_j;
        // std::cout << "INNOVATION: "<< innovation << std::endl;
        
        double exponent = -0.5 * innovation.transpose() * Q_j.inverse() * innovation;
        double normalizer = std::pow(2 * M_PI, z_t.size() / 2.0) * std::sqrt(Q_j.determinant());
        double w_j = std::exp(exponent) / normalizer;
        
        spdlog::debug("landmark covariance {}", landmark.getCovariance());
        spdlog::debug("landmark innovation \n{}", innovation);
        spdlog::debug("H_j {}", H_j);
        spdlog::debug("z_hat_j {}", z_hat_j);
        spdlog::debug("Exponent: {}, Normalizer: {}", exponent, normalizer);
        spdlog::debug("Calculated weight {}", w_j);
        
        weights[j] = w_j;
        z_hats[j] = z_hat_j;
        Hs[j] = H_j;
        Qs[j] = Q_j;
    }

}


void ParticleFilter::landmarkUpdate(Particle& particle, const Vector2d& z_t) 
{
    size_t landmarkCount = particle.getLandmarkCount();
    std::vector<double> weights(landmarkCount);
    std::vector<Vector2d> z_hats(landmarkCount);
    std::vector<Matrix2d> Hs(landmarkCount);
    std::vector<MatrixXd> Qs(landmarkCount);

    this->updateLikelihoodCorrespondence(particle, z_t, weights, z_hats, Hs, Qs);

    weights.push_back(p_0); // importance factor new feature    
    size_t heaviestFeature = std::distance(weights.begin(),
    std::max_element(weights.begin(), weights.end()));
    particle.setWeight(weights[heaviestFeature]);

    // Loop over both existing landmarks AND the "new landmark slot"
    for (size_t j = 0; j <= particle.getLandmarkCount(); j++) {
        
        if (j == particle.getLandmarkCount()) {
            // --- Handle NEW landmark hypothesis ---
            if (heaviestFeature == j) {
                spdlog::debug("Adding new landmark");
                double mapX = particle.getState()[0] + z_t[0] * cos(particle.getState()[2] + z_t[1]);
                double mapY = particle.getState()[1] + z_t[0] * sin(particle.getState()[2] + z_t[1]);
                Vector2d mu_j_t(mapX, mapY);
                Matrix2d H_j;
                H_j << cos(particle.getState()[2] + z_t[1]), -z_t[0] * sin(particle.getState()[2] + z_t[1]),
                    sin(particle.getState()[2] + z_t[1]),  z_t[0] * cos(particle.getState()[2] + z_t[1]);
                MatrixXd sigma_j_t = H_j * Q_t * H_j.transpose();
                particle.addLandmark(Landmark(mu_j_t, sigma_j_t, newParticleIncrease));
                particle.addSeenLandmark(j);
            }
        } else {
            // --- Handle EXISTING landmarks ---
            if (heaviestFeature == j) {
                spdlog::debug("Updating landmark {}", j);
                Landmark landmark = particle.getLandmarks()[j];
                MatrixXd K = landmark.getCovariance() * Hs[j].transpose() * Qs[j].inverse();
                Vector2d mu_j_t = landmark.getState() + K * (z_t - z_hats[j]);
                MatrixXd sigma_j_t = (MatrixXd::Identity(2,2) - K * Hs[j]) * landmark.getCovariance();
                Landmark newLandmark(mu_j_t, sigma_j_t, landmark.getCount() + 1);
                particle.updateLandmark(j, newLandmark);
                particle.addSeenLandmark(j);
            }
        }
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

void ParticleFilter::particlePurgeLandmarks(std::vector<Particle>& particles)
{
    for (auto& particle: particles) 
    {
        for (int j = 0; j < particle.getLandmarkCount(); j++)
        {
            Landmark landmark = particle.getLandmarks()[j];
            bool inRange = landmarkInRange(particle.getState(), landmark.getState());
            bool wasSeen = std::find(particle.getSeenLandmarks().begin(), particle.getSeenLandmarks().end(), j) != particle.getSeenLandmarks().end();
            
            if ((inRange) && (!wasSeen))
            {
                landmark.missedLandmark();
                particle.updateLandmark(j, landmark);
                if (particle.getLandmarks()[j].getCount() <= 0) {
                    particle.removeLandmark(j);
                    j--;
                }
            }
        }
        particle.clearSeenLandmarks();

    }
}

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
    double neff_threshold = 0.666 * numParticles;
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
    std::cout << "Average particle location after resampling: x=" << avg_x << ", y=" << avg_y << ", theta=" << avg_theta << std::endl;

    return resampledParticles;

}
    



