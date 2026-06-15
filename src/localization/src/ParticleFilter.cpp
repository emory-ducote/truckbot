#include <numeric>
#include <random>
#include <iostream>
#include <spdlog/spdlog.h>
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
                               const double angularVelocityAlpha2,
                               const double p0) :
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
                               angularVelocityAlpha2(angularVelocityAlpha2),
                               p0(p0) {
    spdlog::set_level(spdlog::level::info);
    initialSigmas << 0.0, 0.0, 0.0;

    Q_t << measurementNoiseRange, 0, 
           0, measurementNoiseBearing;
    // Single shared RNG (constructing random_device/mt19937 per particle was
    // wasteful). With initialSigmas == 0 every particle starts at the origin;
    // raise the sigmas to seed initial pose diversity.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dist_x(0.0, initialSigmas[0]);
    std::normal_distribution<> dist_y(0.0, initialSigmas[1]);
    std::normal_distribution<> dist_theta(0.0, initialSigmas[2]);

    for (int m = 0; m < numParticles; m++)
    {
        Vector3d noisy_state(dist_x(gen), dist_y(gen), dist_theta(gen));
        particles[m] = Particle(noisy_state);
    }

}

ParticleFilter::~ParticleFilter(){}

void ParticleFilter::sampleNewParticlePose(Particle& particle, const Vector2d& u_t, const double dt)
{
    static thread_local std::mt19937 gen(std::random_device{}());
    std::normal_distribution<> standardNormal(0.0, 1.0);

    double linearVelocity = u_t[0];     // control input linear velocity (m/s)
    double angularVelocity = u_t[1];    // control input angular velocity (rad/s)
    
    // Apply 4-alpha noise model
    double linearVelStd = std::sqrt(linearVelocityAlpha1 * linearVelocity*linearVelocity + linearVelocityAlpha2 * angularVelocity*angularVelocity);
    double angularVelStd = std::sqrt(angularVelocityAlpha1 * linearVelocity*linearVelocity + angularVelocityAlpha2 * angularVelocity*angularVelocity);
    
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
        result.matched = false;
        result.z_hat.setZero();
        result.H.setZero();
        result.Q.setIdentity();
        result.prevX.setZero();
        result.prevP.setIdentity();
        return result;
    }
    
    double deltaX = nearest->point[0] - particle.x[0];
    double deltaY = nearest->point[1] - particle.x[1];
    double q = deltaX * deltaX + deltaY * deltaY;
    // Guard against a landmark coinciding with the robot (division by r/q below).
    if (q < 1e-12) q = 1e-12;
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

    // measurement covariance
    Q_j = H_j * nearest->P * H_j.transpose() + Q_t;
    Q_j += 1e-9 * Matrix2d::Identity();

    // chi-squared gate: 2-DOF at 99% confidence = 9.21. The gate (not the raw
    // likelihood density) decides association: inside the gate the measurement
    // updates the nearest landmark; outside it is treated as a new feature.
    double mahalanobis_sq = innovation.transpose() * Q_j.inverse() * innovation;
    result.matched = (mahalanobis_sq <= 9.21);
    if (!result.matched) {
        w_j = 1e-9;
    } else {
        double exponent = -0.5 * mahalanobis_sq;
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

    auto [w, matched, z_hat, H, Q, prevX, prevP] = this->updateLikelihoodCorrespondence(particle, z_t);

    bool newFeature = !matched;
    // spdlog::debug("New Feature {}", newFeature);

    if (newFeature)
    {
        // New / unassociated feature: weight by a fixed default importance
        // weight p0 so new features contribute on the same scale as matched
        // ones (rather than the inconsistent x1.0 it used to apply).
        particle.weight = p0 * particle.weight;
        double global_angle = wrapAngle(particle.x[2] + z_t[1]);
        double mapX = particle.x[0] + z_t[0] * cos(global_angle);
        double mapY = particle.x[1] + z_t[0] * sin(global_angle);
        Vector2d mu_j_t(mapX, mapY);
        Matrix2d H_j;
        H_j << cos(particle.x[2] + z_t[1]), -z_t[0] * sin(particle.x[2] + z_t[1]),
            sin(particle.x[2] + z_t[1]),  z_t[0] * cos(particle.x[2] + z_t[1]);
        MatrixXd sigma_j_t = H_j * Q_t * H_j.transpose();
        particle.addLandmark(Landmark(mu_j_t, sigma_j_t));
        particle.seenLandmarksThisCycle.insert({mu_j_t(0), mu_j_t(1)});
    }
    else 
    {
        particle.weight = w*particle.weight;
        Landmark oldLandmark(prevX, prevP);
        Matrix2d K = oldLandmark.P * H.transpose() * Q.inverse();
        Vector2d innovation = z_t - z_hat;
        innovation(1) = wrapAngle(innovation(1));
        Vector2d mu_j_t = oldLandmark.x + K * innovation;
        // Joseph form: keeps the covariance symmetric and positive-definite over
        // many updates (Q_t is the measurement noise R). More stable than (I-KH)P.
        Matrix2d IKH = Matrix2d::Identity() - K * H;
        Matrix2d sigma_j_t = IKH * oldLandmark.P * IKH.transpose() + K * Q_t * K.transpose();
        Landmark newLandmark(mu_j_t, sigma_j_t);
        particle.updateLandmark(oldLandmark, newLandmark);
        particle.seenLandmarksThisCycle.insert({mu_j_t(0), mu_j_t(1)});
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
        // Bound to M-1: cdf[M-1] is the (normalized) weight sum, which can be
        // slightly below u due to floating-point accumulation, so an unbounded
        // walk could index past the end.
        while (i < M - 1 && u > cdf[i]) {
            i++;
        }
        indexes[m] = i;
    }

    return indexes;
}

void ParticleFilter::particleMotionUpdate(const Vector2d& u_t, const double dt)
{
    for (auto& particle : particles)
        sampleNewParticlePose(particle, u_t, dt);
}

void ParticleFilter::particleWeightUpdate(const Vector2d& z_t)
{
    for (auto& particle : particles)
        landmarkUpdate(particle, z_t);
}

void ParticleFilter::particlePurgeLandmarks()
{
    for (auto& particle : particles) {
            std::vector<Vector2d> landmarksInRange = particle.landmarksInRange(maxRange, maxAngle);
            for (const auto& lm : landmarksInRange) {
                auto key = std::make_pair(lm(0), lm(1));
                if (particle.seenLandmarksThisCycle.count(key)) continue;
                if (particle.seenLandmarkCounts[key] <= 0) {
                    particle.removeLandmark(lm);
                } else {
                    particle.seenLandmarkCounts[key] -= 1;
                }
            }
            particle.seenLandmarksThisCycle.clear();
    }
}

std::vector<Particle> ParticleFilter::particleWeightResampling()
{
    spdlog::debug("Started Resampling");

    double weightSum = 0.0;
    for (int p = 0; p < numParticles; p++) {
        weightSum += particles[p].weight;
    }

    // Guard against total weight collapse (all measurements gated out, or
    // underflow from multiplying many small likelihoods). Normalizing by a
    // zero / non-finite sum would yield inf/NaN weights and a garbage resample,
    // so fall back to a uniform distribution and skip resampling this cycle.
    if (!std::isfinite(weightSum) || weightSum <= 0.0) {
        spdlog::warn("Degenerate weight sum ({}); skipping resample, resetting to uniform.", weightSum);
        bestParticle = particles[0];  // all weights equal; any particle is representative
        for (auto& particle : particles) {
            particle.weight = 1.0 / numParticles;
        }
        return particles;
    }

    // collect and normalize weights
    std::vector<double> normalizedWeights(numParticles);
    for (int p = 0; p < numParticles; p++) {
        normalizedWeights[p] = particles[p].weight / weightSum;
    }

    // Capture the best (max-weight) particle BEFORE any resample resets weights
    // to uniform, so the published estimate keeps a coherent pose+map.
    int bestIdx = 0;
    for (int p = 1; p < numParticles; p++) {
        if (normalizedWeights[p] > normalizedWeights[bestIdx]) bestIdx = p;
    }
    bestParticle = particles[bestIdx];

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

    spdlog::debug("NEFF: {}", neff);

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
            // After resampling the population represents the posterior uniformly,
            // so reset weights to 1/N.
            resampledParticles[m].weight = 1.0 / numParticles;
            spdlog::debug("RESAMPLED {}", m);
        }

    } else {
        // No resampling: keep particles but store the normalized accumulated
        // weights back so they stay bounded and continue accumulating next cycle.
        resampledParticles = particles;
        for (int p = 0; p < numParticles; p++) {
            resampledParticles[p].weight = normalizedWeights[p];
        }
    }

    particles = resampledParticles;


    spdlog::debug("Started Resampling");

    return particles;

}
    
std::vector<Particle> ParticleFilter::particleFilterLoop(const Vector2d& u_t, std::vector<Vector2d> z_t_s, const double dt)
{
    spdlog::debug("Loop Start");
    // Weights are NOT reset here: they accumulate across cycles and are reset to
    // uniform only immediately after a resample (see particleWeightResampling).
    // Resetting every cycle would discard the measurement evidence from any
    // cycle that did not trigger a resample.
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
