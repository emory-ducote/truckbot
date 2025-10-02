#include <vector>
#include <iostream>
#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"
#include "EKF.h"

using namespace Eigen;
using namespace LocalizationHelpers;
using namespace std;

EKF::EKF(const double frequency, Vector3d x, MatrixXd P) :
            frequency(frequency), x(x), P(P) {
        spdlog::set_level(spdlog::level::info);
            }

EKF::~EKF() {}

MatrixXd EKF::generate_Gt(const Vector3d& u_t_1, const float& v_t, const float& w_t, const double& dt) {
    Matrix3d Gt = Matrix3d::Identity();
    double prev_theta = u_t_1[2];

    Gt(0, 2) = ((-v_t / w_t) * cos(prev_theta)) + ((v_t / w_t) * cos(prev_theta + w_t * dt));
    Gt(1, 2) = ((-v_t / w_t) * cos(prev_theta)) + ((v_t / w_t) * cos(prev_theta + w_t * dt));

    spdlog::debug("Gt matrix:\n {}\n", Gt);
    return Gt;
}

MatrixXd EKF::generate_Vt(const Vector3d& u_t_1, const float& v_t, const float& w_t, const double& dt) {
    MatrixXd Vt = MatrixXd::Zero(3, 2);
    double prev_theta = u_t_1[2];

    Vt(0, 0) = (-sin(prev_theta) + sin(prev_theta + w_t * dt)) / w_t;
    Vt(0, 1) = (( v_t * (sin(prev_theta) - sin(prev_theta + w_t * dt))) / pow(w_t,2)) + 
               (( v_t * cos(prev_theta + w_t * dt) * dt) / w_t);
    Vt(1, 0) = (cos(prev_theta) - cos(prev_theta + w_t * dt)) / w_t;
    Vt(1, 1) = (( - v_t * (cos(prev_theta) - cos(prev_theta + w_t * dt))) / pow(w_t,2)) + 
               (( v_t * sin(prev_theta + w_t * dt) * dt) / w_t);
    Vt(2, 0) = 0;
    Vt(2, 1) = dt;

    spdlog::debug("Vt matrix:\n {}\n", Vt);
    return Vt;
}

MatrixXd EKF::generate_Mt(const float& v_t, const float& w_t) {
    MatrixXd Mt = MatrixXd::Zero(2, 2);
    Mt(0, 0) = (alpha_1 * (pow(v_t, 2))) + (alpha_2 * (pow(w_t, 2)));
    Mt(1, 1) = (alpha_3 * (pow(v_t, 2))) + (alpha_4 * (pow(w_t, 2)));

    spdlog::debug("Mt matrix:\n {}\n", Mt);
    return Mt;
}

void EKF::predict(const Vector3d& u_t_1, const float& v_t, const float& w_t, const double& dt) {
    double prev_theta = u_t_1[2];
    MatrixXd Gt = generate_Gt(u_t_1, v_t, w_t, dt);
    MatrixXd Vt = generate_Vt(u_t_1, v_t, w_t, dt);
    MatrixXd Mt = generate_Mt(v_t, w_t);

    Vector3d position_prediction(((-v_t / w_t) * sin(prev_theta)) + ((v_t / w_t) * sin(prev_theta + w_t * dt)), 
                                 ((v_t / w_t) * cos(prev_theta)) - ((v_t / w_t) * cos(prev_theta + w_t * dt)),
                                  w_t * dt); 
    this->x = x + position_prediction;
    this->x[2] = wrapAngle(this->x[2]);
    this->P = Gt * this->P * Gt.transpose() + Vt * Mt * Vt.transpose(); 

    spdlog::debug("Predicted position: {}\n", this->x.transpose());
    spdlog::debug("Predicted covariance P: \n {}\n", this->P);

}

void EKF::ekf_loop(Vector3d& u_t_1, const float& v_t, const float& w_t) {
    if (fabs(w_t) < 1e-6)
    {
        predict(u_t_1, v_t, 0.000001, dt);
    }
    else 
    {
        predict(u_t_1, v_t, w_t, dt);
    }

}
