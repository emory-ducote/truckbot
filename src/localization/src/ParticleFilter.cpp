#include <numeric>   
#include <random>
#include "ParticleFilter.h"

ParticleFilter::ParticleFilter(const int numParticles, 
                               const int frequency) : 
                               numParticles(numParticles),
                               particles(numParticles),
                               frequency(frequency) {
    Q_t << 0, 1e-3, 
           0, 1e-3;
}

ParticleFilter::~ParticleFilter(){}

void ParticleFilter::sampleNewParticlePose(Particle& particle, Vector2d& u_t, double dt) 
{
    double v_t = u_t[0];
    double w_t = u_t[1];
    double theta = particle.getState()[2];
    particle.getState()[0] += (v_t / w_t) * (sin(theta + w_t * dt) - sin(theta));
    particle.getState()[1] += (v_t / w_t) * (cos(theta + w_t * dt) - cos(theta));
    particle.getState()[2] = wrapAngle(theta + w_t * dt);
}

bool ParticleFilter::landmarkInRange(const Vector3d& state, const Vector2d& landmarkState)
{
    // TODO add range and bearing gate
    return true;
}

void ParticleFilter::updateLikelihoodCorrespondence(Particle& particle, 
                                                    Vector2d& z_t, 
                                                    std::vector<double>& weights,
                                                    std::vector<Vector2d>& z_hats,
                                                    std::vector<Matrix2d>& Hs,
                                                    std::vector<MatrixXd>& Qs)
{
    size_t landmarkCount = particle.landmarkCount();
    for (int j = 0; j < particle.landmarkCount(); j++)
    {
        Landmark landmark = particle.getLandmarks()[j];
        // some useful calculations for forming our prediction
        // and jacobian matrices
        double deltaX = landmark.getState()[0] - particle.getState()[0];
        double deltaY = landmark.getState()[1] - particle.getState()[1];
        double q = pow(deltaX, 2) + pow(deltaY, 2);
        double r = std::sqrt(q);
        double theta = atan2(deltaY, deltaX) - particle.getState()[2];

        // measurement prediction
        Vector2d z_hat_j(r, theta);
        
        // measurement jacobian
        Matrix2d H_j;
        H_j << deltaX / r, deltaY / r,
             - deltaY / q, deltaX / q;

        // measurement covariance
        MatrixXd Q_j = H_j * landmark.getCovariance() * H_j.transpose() + Q_t;

        // likelihood of correspondence
        Vector2d innovation = z_t - z_hat_j;
        double exponent = -0.5 * innovation.transpose() * Q_j.inverse() * innovation;
        double normalizer = std::pow(2 * M_PI, z_t.size() / 2.0) * std::sqrt(Q_j.determinant());
        double w_j = std::exp(exponent) / normalizer;
        
        weights[j] = w_j;
        z_hats[j] = z_hat_j;
        Hs[j] = H_j;
        Qs[j] = Q_j;
    }

}


void ParticleFilter::landmarkUpdate(Particle& particle, Vector2d& z_t, Vector2d& u_t) 
{
    size_t landmarkCount = particle.landmarkCount();
    std::vector<double> weights(landmarkCount);
    std::vector<Vector2d> z_hats(landmarkCount);
    std::vector<Matrix2d> Hs(landmarkCount);
    std::vector<MatrixXd> Qs(landmarkCount);

    this->updateLikelihoodCorrespondence(particle, z_t, weights, z_hats, Hs, Qs);

    weights.push_back(p_0); // importance factor new feature
    size_t newFeatureIndex = weights.size() - 1;
    int heaviestFeature = std::distance(weights.begin(), std::max_element(weights.begin(), weights.end()));
    particle.setWeight(weights[heaviestFeature]);

    for (int j = 0; j < particle.landmarkCount(); j++)
    {
        Landmark landmark = particle.getLandmarks()[j];
        if (heaviestFeature == newFeatureIndex)
        {
            double mapX = particle.getState()[0] + z_t[0] * cos(particle.getState()[2] + z_t[1]);
            double mapY = particle.getState()[1] + z_t[0] * cos(particle.getState()[2] + z_t[1]);
            Vector2d mu_j_t(mapX, mapY);
            MatrixXd sigma_j_t = (Hs[j].inverse().transpose()) * Q_t * Hs[j].inverse();
            particle.addLandmark(Landmark(mu_j_t, sigma_j_t));
        }
        else if (heaviestFeature == j) 
        {
            MatrixXd K = landmark.getCovariance() * Hs[j].transpose() * Qs[j].inverse();
            Vector2d mu_j_t = landmark.getState() + K * (z_t - z_hats[j]);
            MatrixXd sigma_j_t = (MatrixXd::Identity(2,2) - K * Hs[j]) * landmark.getCovariance();
            int updatedCounter = landmark.landmarkCounter() + 1;
            landmark.updateLandmark(mu_j_t, sigma_j_t, updatedCounter);
        }
        else
        {
            // dont change previous mean and covariance
            if (this->landmarkInRange(particle.getState(), landmark.getState())) // and did not see it
            {
                landmark.missedLandmark();
                if (landmark.landmarkCounter() < 0)
                {
                    particle.removeLandmark(j);
                }
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

std::vector<Particle> ParticleFilter::particleUpdate(std::vector<Particle>& particles, Vector2d& z_t, Vector2d& u_t)
{
    double weightSum = 0;
    // main landmark update
    for (Particle particle : particles)
    {
        this->landmarkUpdate(particle, z_t, u_t);
        weightSum += particle.getWeight();
    }

    // collect and normalize weights
    std::vector<double> normalizedWeights(numParticles);
    for (int p = 0; p < numParticles; p++) {
        normalizedWeights[p] = particles[p].getWeight() / weightSum;
    }

    std::vector<int> resampledIndices = systematicResample(normalizedWeights);
    
    std::vector<Particle> resampledParticles(numParticles);
    for (int m = 0; m < numParticles; m++)
    {
        resampledParticles[m] = particles[resampledIndices[m]];
    }

    return resampledParticles;

}
    



