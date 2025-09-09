#include <cmath>
#include "ParticleFilter.h"

int main(int argc, char** argv) {

    ParticleFilter filter(100, 1);

    Vector2d landmarkPosition(2, 0);
    Vector2d vehiclePosition(0, 0);

    std::vector<Particle> resampled = filter.getParticles();

    for (int i = 0; i < 50; i++) 
    {
        double deltaX = landmarkPosition[0] - vehiclePosition[0];
        double deltaY = landmarkPosition[1] - vehiclePosition[1];
        double q = pow(deltaX, 2) + pow(deltaY, 2);
        double r = std::sqrt(q);
        double theta = atan2(deltaY, deltaX) - vehiclePosition[2];

        Vector2d u_t(0.1, 0.0);
        Vector2d z_t(r, theta);

        std::vector<Particle> resampled = filter.particleUpdate(resampled, z_t, u_t);

        vehiclePosition += u_t;
    }

    return 0;
  }