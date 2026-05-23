#ifndef PUREPURSUIT_H_
#define PUREPURSUIT_H_

#include <cmath>

class PurePursuit {
public:
    PurePursuit(double wheelbase) : L(wheelbase) {}

    double computeControl(double x_l, double y_l,
                        double v)
    {
        double Ld2 = x_l * x_l + y_l * y_l;

        if (Ld2 == 0.0) {
            return 0.0;
        }

        // curvature
        double kappa = (2.0 * y_l) / Ld2;

        // angular velocity (unicycle model)
        return v * kappa;
    }

private:
    double L; // wheelbase
};

#endif