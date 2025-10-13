#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

struct Landmark {
        Landmark(Vector2d x=Vector2d::Zero(),
                 MatrixXd P=Matrix2d::Identity(2, 2) * 1e-3) :
                 x(x),
                 P(P) {}
                 
        Vector2d x;
        Matrix2d P;
};        