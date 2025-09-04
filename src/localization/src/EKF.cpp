#include <vector>
#include <iostream>
#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"
#include "EKF.h"

using namespace Eigen;
using namespace std;

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

EKF::EKF(const double frequency, Vector3d x, MatrixXd P) :
            frequency(frequency), x(x), P(P) {
        spdlog::set_level(spdlog::level::debug);
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
    this->P = Gt * this->P * Gt.transpose() + Vt * Mt * Vt.transpose(); 

    spdlog::debug("Predicted position: {}\n", this->x.transpose());
    spdlog::debug("Predicted covariance P: \n {}\n", this->P);

}




// void EKF::update(Vector3d& acc_data, double dt, const Matrix3d& Sigma_a) {

//     Vector3d gravity_n(0, 0, 0);                    
//     VectorXd y(3);
//     y = -acc_data;
    
//     MatrixXd H = compute_H(gravity_n);

//     VectorXd y_pred(3);
//     Matrix3d Rnb = this->orientation.toRotationMatrix();
//     y_pred = Rnb.transpose() * gravity_n;
    
//     VectorXd e = y - y_pred;
    
//     spdlog::debug("Combined measurement y: {}\n", y.transpose());
//     spdlog::debug("Prediction y_pred: {}\n", y_pred.transpose());
//     spdlog::debug("Measurement error e: {}\n", e.transpose());
    
//     MatrixXd R = compute_R(Sigma_a);
    
//     MatrixXd S = H * this->P * H.transpose() + R;
//     MatrixXd K = this->P * H.transpose() * S.inverse();
    
//     spdlog::debug("Kalman gain K:\n {}\n", K);
    
//     VectorXd dx = K * e;
//     this->position += dx.segment<3>(0);
//     this->velocity += dx.segment<3>(3);
//     Vector4d dq = dx.segment<4>(6);
//     Quaterniond dq_quat(1, 0.5 * dq(0), 0.5 * dq(1), 0.5 * dq(2));
//     this->orientation = (this->orientation * dq_quat).normalized();
//     this->P = (MatrixXd::Identity(10, 10) - K * H) * this->P;

//     // Quaternion norm and squared norm
//     Vector4d q_vec = dq_quat.coeffs();
//     double norm_q = q_vec.norm();
//     double norm_q_sq = norm_q * norm_q;

//     // Identity matrix
//     Matrix4d I = Matrix4d::Identity();

//     // Outer product: q̃ * q̃ᵀ
//     Matrix4d outer = q_vec * q_vec.transpose();

//     // Compute Jacobian Jt
//     Matrix4d P_quat_tilde = this->P.block<4, 4>(6, 6);

//     Matrix4d Jt = (1.0 / norm_q) * (I - outer / norm_q_sq);

//     // Renormalize quaternion covariance
//     Matrix4d P_quat = Jt * P_quat_tilde * Jt.transpose();

//     // Store updated quaternion covariance back into P
//     this->P.block<4, 4>(6, 6) = P_quat;
    
//     spdlog::debug("Updated this position: {}\n", this->position.transpose());
//     spdlog::debug("Updated this velocity: {}\n", this->velocity.transpose());
//     spdlog::debug("Updated this orientation (quaternion): {}\n", this->orientation.coeffs().transpose());
//     spdlog::debug("Updated covariance P:\n {}\n", this->P);
// }

void EKF::ekf_loop(Vector3d& u_t_1, const float& v_t, const float& w_t) {
    if (fabs(w_t) < 1e-6)
    {
        predict(u_t_1, v_t, 0.000001, dt);
    }
    else 
    {
        predict(u_t_1, v_t, w_t, dt);
    }
    // update(acc_data, dt, Sigma_a);

}
