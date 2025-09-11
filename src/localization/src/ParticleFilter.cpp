#include <numeric>   
#include <random>
#include <iostream>
#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"
#include "ParticleFilter.h"

template <typename T>
struct fmt::formatter<T, char, std::enable_if_t<
    std::is_base_of_v<Eigen::EigenBase<T>, T>>> {
    
    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const T& mat, FormatContext& ctx) {
        std::ostringstream oss;
        oss << mat;
        return fmt::format_to(ctx.out(), "{}", oss.str());
    }
};

ParticleFilter::ParticleFilter(const int numParticles, 
                               const int frequency) : 
                               numParticles(numParticles),
                               particles(numParticles),
                               frequency(frequency) {
    spdlog::set_level(spdlog::level::debug);
    initial_sigmas << 1.0, 1.0, 0.00;
    // initial_sigmas << 0.0, 0.0, 0.00;
    for (int m = 0; m < numParticles; m++)
    {
        // Random number generator
        std::random_device rd;
        std::mt19937 gen(rd());

        std::normal_distribution<> dist_x(0.0, initial_sigmas[0]);
        std::normal_distribution<> dist_y(0.0, initial_sigmas[1]);
        std::normal_distribution<> dist_theta(0.0, initial_sigmas[2]);

        // Add noise

        Vector3d noisy_state(dist_x(gen), dist_y(gen), dist_theta(gen));
        particles[m] = Particle(noisy_state);
    }
    Q_t << 1e-3, 0, 
            0, 1e-3;
}

ParticleFilter::~ParticleFilter(){}

void ParticleFilter::sampleNewParticlePose(Particle& particle, Vector2d& u_t, double dt) 
{
    double v_t = u_t[0];
    double w_t = u_t[1];
    double theta = particle.getState()[2];
    double x = particle.getState()[0] + ((v_t / w_t) * (sin(theta + w_t * dt) - sin(theta)));
    double y = particle.getState()[1]  +((v_t / w_t) * (cos(theta + w_t * dt) - cos(theta)));
    double theta_new = particle.getState()[2]  +wrapAngle(theta + w_t * dt);
    particle.setState(Vector3d(x, y, theta_new));

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
    spdlog::debug("Updating likelihood of {} landmarks", landmarkCount);

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
        // Q_j += 1e-9 * Matrix2d::Identity();
        // likelihood of correspondence
        Vector2d innovation = z_t - z_hat_j;
        
        double exponent = -0.5 * innovation.transpose() * Q_j.inverse() * innovation;
        double normalizer = std::pow(2 * M_PI, z_t.size() / 2.0) * std::sqrt(Q_j.determinant());
        double w_j = std::exp(exponent) / normalizer;
        
        spdlog::debug("Q_J: {}", Q_j);
        spdlog::debug("landmark covariance {}", landmark.getCovariance());
        spdlog::debug("landmark innovation {}", innovation);
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

    int heaviestFeature = std::distance(weights.begin(),
                                    std::max_element(weights.begin(), weights.end()));
    std::cout << "HEAVIEST: " << heaviestFeature << " " << weights[heaviestFeature]<< std::endl;
    particle.setWeight(weights[heaviestFeature]);

    // Loop over both existing landmarks AND the "new landmark slot"
    for (int j = 0; j <= particle.landmarkCount(); j++) {
        
        if (j == particle.landmarkCount()) {
            // --- Handle NEW landmark hypothesis ---
            if (heaviestFeature == j) {
                spdlog::debug("Adding new landmark");

                double mapX = particle.getState()[0] + z_t[0] * cos(particle.getState()[2] + z_t[1]);
                double mapY = particle.getState()[1] + z_t[0] * sin(particle.getState()[2] + z_t[1]);
                Vector2d mu_j_t(mapX, mapY);

                double deltaX = mapX - particle.getState()[0];
                double deltaY = mapY - particle.getState()[1];
                double q = pow(deltaX, 2) + pow(deltaY, 2);

                Matrix2d H_j;
                H_j << cos(particle.getState()[2] + z_t[1]), -z_t[0] * sin(particle.getState()[2] + z_t[1]),
                    sin(particle.getState()[2] + z_t[1]),  z_t[0] * cos(particle.getState()[2] + z_t[1]);

                MatrixXd sigma_j_t = (H_j.inverse().transpose()) * Q_t * H_j.inverse();

                particle.addLandmark(Landmark(mu_j_t, sigma_j_t, newParticleIncrease));
            }
        } else {
            // --- Handle EXISTING landmarks ---
            if (heaviestFeature == j) {
                spdlog::debug("Updating landmark {}", j);

                Landmark landmark = particle.getLandmarks()[j];
                MatrixXd K = landmark.getCovariance() * Hs[j].transpose() * Qs[j].inverse();
                Vector2d mu_j_t = landmark.getState() + K * (z_t - z_hats[j]);
                MatrixXd sigma_j_t = (MatrixXd::Identity(2,2) - K * Hs[j]) * landmark.getCovariance();

                landmark.updateLandmark(mu_j_t, sigma_j_t, landmark.landmarkCounter() + newParticleIncrease);

            } else {
                spdlog::debug("Unseen landmark {}", j);

                Landmark landmark = particle.getLandmarks()[j];
                if (this->landmarkInRange(particle.getState(), landmark.getState())) {
                    landmark.missedLandmark();
                    if (landmark.landmarkCounter() < 0) {
                        particle.removeLandmark(j);
                    }
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

std::vector<Particle> ParticleFilter::particleUpdate(std::vector<Particle> particles, Vector2d& z_t, Vector2d& u_t)
{
    for (Particle& particle: particles) 
    {
        this->sampleNewParticlePose(particle, u_t, dt);
    }

    double weightSum = 0;
    // main landmark update
    for (Particle& particle : particles)
    {
        spdlog::debug("Updating Particle");
        this->landmarkUpdate(particle, z_t, u_t);
        spdlog::debug("Updated Particle, new landmark count: {}\n\n\n", particle.landmarkCount());
        weightSum += particle.getWeight();
    }
    
    // collect and normalize weights
    std::vector<double> normalizedWeights(numParticles);
    for (int p = 0; p < numParticles; p++) {
        normalizedWeights[p] = particles[p].getWeight() / weightSum;
    }
    
    std::vector<int> resampledIndices = systematicResample(normalizedWeights);
    for (int i =0; i < resampledIndices.size() ; i++) {
        
    }
    
    std::vector<Particle> resampledParticles(numParticles);
    for (int m = 0; m < numParticles; m++)
    {
        resampledParticles[m] = particles[resampledIndices[m]];
    }

    return resampledParticles;

}
    



